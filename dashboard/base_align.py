"""Fixed camera + robot base calibration from one D435 view.

The checkerboard on the table gives the fixed camera's pose relative to the
table (PnP on the color image). The robot base pose is then found by
registering the UR3e surface model (posed by the live joint angles) to the
D435 depth cloud:
  1. coarse search over base x, y, yaw on the table plane (the base stands on it),
  2. ICP over all 6 DoF, using only model points facing the camera.
No gripper-camera samples are needed.
"""

import time

import cv2
import numpy as np
from scipy.spatial import cKDTree

from . import calibration as cal
from . import kinematics as kin
from .cameras import table_pose


def _deproject(depth, K, stride):
    d = depth[::stride, ::stride]
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    m = d > 0.1
    z = d[m]
    return np.c_[(u[m] - K[0, 2]) * z / K[0, 0], (v[m] - K[1, 2]) * z / K[1, 1], z]


def _xf(T, P):
    return P @ T[:3, :3].T + T[:3, 3]


def _kabsch(A, B):
    """Rigid T minimising |T A - B|."""
    ca, cb = A.mean(0), B.mean(0)
    U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    T = np.eye(4)
    T[:3, :3] = Vt.T @ D @ U.T
    T[:3, 3] = cb - T[:3, :3] @ ca
    return T


def _table_pose(x, y, yaw, z):
    T = np.eye(4)
    T[:2, :2] = [[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]]
    T[:3, 3] = [x, y, z]
    return T


def visible(P, N, W, K, cam_w, cell=4, tol=0.015):
    """Model points the camera can see: facing it, and not behind another link
    (z-buffer on cell x cell pixel bins)."""
    front = np.einsum('ij,ij->i', N, cam_w - P) > 0
    Pc = _xf(kin.inv(W), P)
    z = Pc[:, 2]
    ok = front & (z > 0.05)
    u = (K[0, 0] * Pc[:, 0] / np.maximum(z, 1e-6) + K[0, 2]) // cell
    v = (K[1, 1] * Pc[:, 1] / np.maximum(z, 1e-6) + K[1, 2]) // cell
    key = (v.astype(np.int64) * 100000 + u.astype(np.int64))
    zmin = {}
    for k_, z_ in zip(key[ok], z[ok]):
        if z_ < zmin.get(k_, np.inf):
            zmin[k_] = z_
    nearest = np.array([zmin.get(k_, np.inf) for k_ in key])
    return ok & (z <= nearest + tol)


def icp(T, Pm, Nm, tree, scene, cam_w, W, K, iters=40):
    """Refine T_world_base. Returns T and fit stats on the final visible set."""
    for it in range(iters):
        thr = 0.03 if it < 10 else 0.015 if it < 25 else 0.008
        P, N = _xf(T, Pm), Nm @ T[:3, :3].T
        vis = visible(P, N, W, K, cam_w)
        d, j = tree.query(P[vis], distance_upper_bound=thr)
        ok = np.isfinite(d)
        if ok.sum() < 50:
            break
        dT = _kabsch(P[vis][ok], scene[j[ok]])
        T = dT @ T
        if np.linalg.norm(dT[:3, 3]) < 1e-5 and np.abs(dT[:3, :3] - np.eye(3)).max() < 1e-6:
            break
    P, N = _xf(T, Pm), Nm @ T[:3, :3].T
    vis = visible(P, N, W, K, cam_w)
    d, _ = tree.query(P[vis])
    inl = d < 0.01
    return T, {'visible_pts': int(vis.sum()), 'inlier_frac': float(inl.mean()) if inl.size else 0.0,
               'inlier_rms_mm': float(np.sqrt(np.mean(d[inl] ** 2)) * 1000) if inl.any() else None}


def fmt_mm(v):
    return '—' if v is None else f'{v:.1f}'


def align(color, depth, K, dist, board, q, surface, log=print):
    t0 = time.time()
    corners = board.detect(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY))
    if corners is not None:
        T_fb, pnp_rms = cal.solve_pnp(board, corners, K, dist)
        W = T_fb @ board.T_board_world                 # world (board centre, z up) in fcam
        log(f'table plane from the board (pnp {pnp_rms:.2f}px)')
    else:
        # board hidden or a different size: the table plane comes from the depth;
        # the result is then expressed with the robot base as world
        T_fb, pnp_rms = None, None
        Tw = table_pose(_deproject(depth, K, 4).astype(np.float32))
        if Tw is None:
            raise RuntimeError('no full board in view and no table plane found in the depth')
        W = kin.inv(Tw)
        log(f'board not fully visible ({board.cols}x{board.rows}); table plane from the depth')
    cam_w = kin.inv(W)[:3, 3]

    P = _xf(kin.inv(W), _deproject(depth, K, 2))       # depth cloud in world
    near = (np.abs(P[:, 0]) < 0.3) & (np.abs(P[:, 1]) < 0.3) & (np.abs(P[:, 2]) < 0.03)
    z_table = float(np.median(P[near, 2])) if near.sum() > 100 else 0.0
    scene = P[(P[:, 2] > z_table + 0.015) & (P[:, 2] < 1.0) & (np.hypot(P[:, 0], P[:, 1]) < 1.5)]
    if len(scene) < 500:
        raise RuntimeError('too few depth points above the table — is the robot in view?')
    tree = cKDTree(scene)
    log(f'depth table offset {z_table * 1000:.1f}mm; {len(scene)} scene points')

    Pm, Nm = surface.points(q)
    # coarse: base upright on the table, search x, y, yaw
    rng = np.random.default_rng(0)
    sub = rng.choice(len(Pm), min(500, len(Pm)), replace=False)
    lo, hi = scene[:, :2].min(0), scene[:, :2].max(0)
    xs, ys = np.arange(lo[0], hi[0], 0.04), np.arange(lo[1], hi[1], 0.04)
    gx, gy = [a.ravel() for a in np.meshgrid(xs, ys)]
    best = []
    for yaw in np.radians(np.arange(0, 360, 15)):
        Pr = _xf(_table_pose(0, 0, yaw, z_table), Pm[sub])
        for c in range(0, len(gx), 200):
            X = Pr[None] + np.c_[gx[c:c + 200], gy[c:c + 200], np.zeros(len(gx[c:c + 200]))][:, None]
            d, _ = tree.query(X.reshape(-1, 3), distance_upper_bound=0.03)
            score = np.minimum(d, 0.03).reshape(len(X), -1).mean(1)
            for k in np.argsort(score)[:3]:
                best.append((score[k], gx[c + k], gy[c + k], yaw))
    best.sort(key=lambda b: b[0])
    log(f'coarse search: {len(gx)} positions x 24 yaws, best {best[0][0] * 1000:.1f}mm mean dist')

    # refine the best few distinct candidates
    cands, fits = [], []
    for s, x, y, yaw in best:
        if all(np.hypot(x - c[1], y - c[2]) > 0.08 or abs(np.angle(np.exp(1j * (yaw - c[3])))) > 0.3 for c in cands):
            cands.append((s, x, y, yaw))
        if len(cands) == 5:
            break
    for s, x, y, yaw in cands:
        T, st = icp(_table_pose(x, y, yaw, z_table), Pm, Nm, tree, scene, cam_w, W, K)
        fits.append((st['inlier_frac'], T, st))
    fits.sort(key=lambda f: -f[0])
    _, T_wb, st = fits[0]
    tilt = float(np.degrees(np.arccos(np.clip(T_wb[2, 2], -1, 1))))
    st.update({'tilt_deg': tilt, 'base_height_mm': float((T_wb[2, 3] - z_table) * 1000), 'pnp_rms_px': pnp_rms,
               'plane_source': 'board' if T_fb is not None else 'depth',
               'seconds': time.time() - t0,
               'runner_up_inlier_frac': fits[1][0] if len(fits) > 1 else None})
    log(f'ICP: {st["inlier_frac"] * 100:.0f}% of {st["visible_pts"]} visible model points within 1cm, '
        f'rms {fmt_mm(st["inlier_rms_mm"])}mm, tilt {tilt:.2f}deg, base z {T_wb[2, 3] * 1000:.1f}mm')

    # the board/table plane is the base XY plane: keep only x, y, yaw from ICP
    T_wb = _table_pose(T_wb[0, 3], T_wb[1, 3], np.arctan2(T_wb[1, 0], T_wb[0, 0]), z_table)
    log('base snapped upright onto the table plane (slide it in the base editor if x/y/yaw are off)')

    T_fbase = W @ T_wb                                 # base in fcam
    T_base_fcam = kin.inv(T_fbase)
    return {
        'T_base_fcam': T_base_fcam,
        'T_base_board': None if T_fb is None else T_base_fcam @ T_fb,
        'T_world_base': T_wb if T_fb is not None else np.eye(4),
        'stats': st,
    }
