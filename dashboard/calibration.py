"""Checkerboard detection + PnP (used by the dashboard), and the old joint
hand-eye / DH solver (only used by the porttest scripts and tests; the
dashboard no longer does hand-eye — see dashboard/server.py).

Frames
  base     UR controller base frame (DH frame 0)
  flange   UR tool flange (DH frame 6, the controller's "Tool_flange" TCP)
  gcam     gripper camera optical frame (OpenCV: x right, y down, z forward)
  fcam     fixed (scene) camera optical frame
  board    checkerboard frame (OpenCV: origin at first inner corner, x along
           columns, y along rows, z into the board)
  world    board centre, z up (board rotated 180 deg about x); the dashboard's
           3D scene is drawn in this frame

Unknowns of the old solver, jointly by bundle adjustment over all samples
  T_flange_gcam   hand-eye of the gripper camera
  T_base_board    where the board lies in the robot base frame
  T_base_fcam     fixed camera extrinsics
  dDH             optional corrections to the controller's DH table
The gripper camera's intrinsics can be estimated from the same images.

Residuals are pixel reprojection errors of the board corners in both cameras
(plus a Gaussian prior that keeps dDH near zero), so the result says directly
how well robot kinematics + calibration explain what the cameras see.
"""

import time

import cv2
import numpy as np
from scipy.optimize import least_squares

from . import kinematics as kin

DET_FLAGS = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY


class Board:
    def __init__(self, cols, rows, square):
        self.cols, self.rows, self.square = int(cols), int(rows), float(square)
        jj, ii = np.mgrid[0:self.rows, 0:self.cols]
        self.obj = np.c_[ii.ravel() * self.square, jj.ravel() * self.square,
                         np.zeros(ii.size)].astype(np.float64)
        # world = board centre with z up
        c = np.eye(4)
        c[:3, 3] = [(self.cols - 1) * self.square / 2, (self.rows - 1) * self.square / 2, 0]
        flip = np.diag([1.0, -1.0, -1.0, 1.0])
        self.T_board_world = c @ flip

    def describe(self):
        return {'cols': self.cols, 'rows': self.rows, 'square': self.square,
                'T_board_world': self.T_board_world.tolist()}

    def detect(self, gray, fast=False, max_width=640):
        """Inner corners (N, 2) in row-major order, or None.

        fast: detect on a downscaled copy without the exhaustive search, then
        refine at full resolution — cheap enough for the live preview. Samples
        use the slow, accurate path.
        """
        if not fast:
            # small or steep boards (640x480 D435 across the table) need upsampling
            for s in (1, 2, 3):
                g = cv2.resize(gray, None, fx=s, fy=s, interpolation=cv2.INTER_CUBIC) if s > 1 else gray
                ok, corners = cv2.findChessboardCornersSB(g, (self.cols, self.rows), DET_FLAGS)
                if ok:
                    return corners.reshape(-1, 2).astype(np.float64) / s
            return None
        scale = min(1.0, max_width / gray.shape[1])
        small = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA) if scale < 1 else gray
        ok, corners = cv2.findChessboardCornersSB(small, (self.cols, self.rows), 0)
        if not ok:
            return None
        corners = (corners / scale).astype(np.float32)
        cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                         (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 0.01))
        return corners.reshape(-1, 2).astype(np.float64)

    def detect_partial(self, gray, max_width=640):
        """Grid size of a partly visible board, for the UI hint; or None."""
        scale = min(1.0, max_width / gray.shape[1])
        if scale < 1:
            gray = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        try:
            ok, _, meta = cv2.findChessboardCornersSBWithMeta(gray, (3, 3), cv2.CALIB_CB_LARGER)
        except cv2.error:
            return None
        return [int(meta.shape[1]), int(meta.shape[0])] if ok else None


def solve_pnp(board, corners, K, dist):
    ok, rvec, tvec = cv2.solvePnP(board.obj, corners, K, dist, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        return None
    rvec, tvec = cv2.solvePnPRefineLM(board.obj, corners, K, dist, rvec, tvec)
    T = kin.pose_to_T(tvec.ravel(), rvec.ravel())
    proj, _ = cv2.projectPoints(board.obj, rvec, tvec, K, dist)
    rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - corners) ** 2, axis=1))))
    return T, rms


# ── board on an orthographic heightmap (SICK T Mini through Nova) ────────────
#
# Nova's grid: x = ox + col * px, y = oy + row * py, z = height above its fitted
# floor (the table), z up. The ToF intensity blurs the square edges and the
# black squares drop out, so corner detectors miss the board. The pixel scale is
# known, though: match the board pattern over yaw, then fit the 36 black-square
# centres (local darkness centroids) with a rigid x, y, yaw in Nova's floor.
# Tilt comes from Nova's floor (the board lies on the table), height from the
# table around the board + the board's thickness (its ToF height reads ~2 mm
# high). Fitting in the world frame (z up, like Nova) keeps the fit from
# mirroring: the pattern alone is symmetric under a flip along the columns.

def _board_world_pattern(board, xw, yw, margin=0.6):
    """1 on white, 0 on black, nan off the board, at world (board-centre) coords."""
    Tbw = board.T_board_world
    xo = Tbw[0, 0] * xw + Tbw[0, 1] * yw + Tbw[0, 3]
    yo = Tbw[1, 0] * xw + Tbw[1, 1] * yw + Tbw[1, 3]
    s = board.square
    i, j = np.floor(xo / s + 1).astype(int), np.floor(yo / s + 1).astype(int)
    inside = (i >= 0) & (i <= board.cols) & (j >= 0) & (j <= board.rows)
    plate = ((xo > -(1 + margin) * s) & (xo < (board.cols + margin) * s)
             & (yo > -(1 + margin) * s) & (yo < (board.rows + margin) * s))
    p = np.full(np.shape(xw), np.nan, np.float32)
    p[plate] = 1.0
    p[inside] = ((i[inside] + j[inside]) % 2 == 0)        # square (i, j) black when i + j is odd
    return p


def _board_world_points(board):
    """Black-square centres and inner corners, world xy."""
    s, Twb = board.square, kin.inv(board.T_board_world)
    ii, jj = np.meshgrid(np.arange(board.cols + 1), np.arange(board.rows + 1))
    k = (ii + jj) % 2 == 1
    black = np.c_[(ii[k] - 0.5) * s, (jj[k] - 0.5) * s, np.zeros(k.sum())]
    to_w = lambda P: (Twb[:3, :3] @ P.T).T[:, :2] + Twb[:2, 3]
    return to_w(black), to_w(board.obj)


def _rot2(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s], [s, c]])


def _ortho_coarse(board, g, px, py, step_deg=5.0):
    """(score, yaw, centre pixel) by normalized template matching over yaw."""
    r = int(np.ceil((board.cols + 2) * board.square * 0.8 / min(abs(px), abs(py))))
    vv, uu = np.mgrid[-r:r + 1, -r:r + 1]
    best = (-2.0, 0.0, 0.0, 0.0)
    for yaw in np.radians(np.arange(0, 360, step_deg)):
        c, s = np.cos(yaw), np.sin(yaw)
        X, Y = uu * px, vv * py
        p = _board_world_pattern(board, c * X + s * Y, -s * X + c * Y)       # world = R(-yaw) · offset
        m = ~np.isnan(p)
        ys, xs = np.where(m)
        y0, y1, x0, x1 = ys.min(), ys.max() + 1, xs.min(), xs.max() + 1
        t = cv2.GaussianBlur(np.nan_to_num(p, nan=0.5), (0, 0), 1.5)[y0:y1, x0:x1].astype(np.float32)
        if t.shape[0] > g.shape[0] or t.shape[1] > g.shape[1]:
            return best
        res = cv2.matchTemplate(g, t, cv2.TM_CCOEFF_NORMED, mask=m[y0:y1, x0:x1].astype(np.float32))
        res[~np.isfinite(res)] = -2
        _, mx, _, loc = cv2.minMaxLoc(res)
        if mx > best[0]:
            best = (mx, yaw, loc[0] + r - x0, loc[1] + r - y0)
    return best


def ortho_board(board, gray, height, grid, prev=None, thickness=0.0):
    """Board on an orthographic heightmap. gray: intensity, height: z in metres
    (0 = no reading), grid: ((ox, oy, oz), (px, py, pz)) in metres; prev: the
    last result's 'track' to refine from. Returns None or a dict:
      T        T_cam_board (cam = Nova's grid frame), as solve_pnp gives it
      rms_px   black-square centre residual in pixels; rms_mm the same in mm
      corners  inner corners (N, 2) in pixels, row-major like Board.detect
      n        black squares found (of 36); track: (yaw, x, y) for the next call"""
    (ox, oy, _), (px, py, _) = (np.asarray(v, float) for v in grid)
    valid = height != 0
    if valid.sum() < 1000:
        return None
    g = gray.astype(np.float32)
    g[~valid] = 0.0                                   # black squares drop out: dark
    g[cv2.dilate(valid.astype(np.uint8), np.ones((15, 15), np.uint8)) == 0] = np.median(g[valid])
    black, inner = _board_world_points(board)
    dark = np.clip(np.median(g[valid]) - g, 0, None)
    w = int(round(0.45 * board.square / abs(px)))
    vy, ux = np.mgrid[-w:w + 1, -w:w + 1]

    def centres(yaw, xc, yc):
        meas, keep = [], []
        for k, (x, y) in enumerate(black @ _rot2(yaw).T + [xc, yc]):
            u0, v0 = int(round((x - ox) / px)), int(round((y - oy) / py))
            if not (w <= u0 < g.shape[1] - w and w <= v0 < g.shape[0] - w):
                continue
            patch = dark[v0 - w:v0 + w + 1, u0 - w:u0 + w + 1]
            if patch.sum() > 0:
                meas.append([ox + (u0 + (patch * ux).sum() / patch.sum()) * px,
                             oy + (v0 + (patch * vy).sum() / patch.sum()) * py])
                keep.append(k)
        return np.array(meas), np.array(keep, int)

    def refine(yaw, xc, yc):
        for _ in range(5):
            meas, keep = centres(yaw, xc, yc)
            if len(keep) < 8:
                return None
            f = lambda q: (black[keep] @ _rot2(q[0]).T + q[1:] - meas).ravel()
            yaw, xc, yc = least_squares(f, [yaw, xc, yc], loss='soft_l1', f_scale=0.002).x
        r = f([yaw, xc, yc]).reshape(-1, 2)
        return yaw, xc, yc, float(np.sqrt(np.mean(np.sum(r ** 2, 1)))), len(keep)

    good = lambda f: f is not None and f[4] >= 0.8 * len(black) and f[3] < 0.002
    fit = refine(*prev) if prev is not None else None
    if not good(fit):
        gs = cv2.resize(g, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
        score, yaw, u, v = _ortho_coarse(board, gs, 2 * px, 2 * py)
        fit = refine(yaw, ox + 2 * u * px, oy + 2 * v * py) if score > 0.3 else None
        if not good(fit):
            return None
    yaw, xc, yc, rms, n = fit
    # board top = the table around the plate (Nova floor, ~0) + the plate's thickness
    vv, uu = np.mgrid[0:g.shape[0], 0:g.shape[1]]
    X, Y = ox + uu * px - xc, oy + vv * py - yc
    c, s = np.cos(yaw), np.sin(yaw)
    xw, yw = c * X + s * Y, -s * X + c * Y
    hx, hy = (board.cols + 1) * board.square / 2, (board.rows + 1) * board.square / 2
    ring = valid & (np.abs(xw) < hx + 0.08) & (np.abs(yw) < hy + 0.08) & ~((np.abs(xw) < hx + 0.04) & (np.abs(yw) < hy + 0.04))
    ring &= np.abs(height) < 0.02
    z_table = float(np.median(height[ring])) if ring.sum() > 200 else 0.0
    T_cam_world = np.eye(4)
    T_cam_world[:2, :2] = _rot2(yaw)
    T_cam_world[:3, 3] = xc, yc, z_table + thickness
    uv = inner @ _rot2(yaw).T + [xc, yc]
    corners = np.c_[(uv[:, 0] - ox) / px, (uv[:, 1] - oy) / py]
    return {'T': T_cam_world @ kin.inv(board.T_board_world), 'rms_px': rms / abs(px), 'rms_mm': rms * 1000,
            'corners': corners, 'n': n, 'track': (yaw, xc, yc)}


# ── solver ──────────────────────────────────────────────────────────────────

# DH entries that cannot be told apart from the free transforms: theta1 and d1
# act like a rotation/shift of T_base_board, and all of link 6 is absorbed by
# T_flange_gcam. Everything else is refined against a prior.
DH_FREE = np.ones((6, 4), bool)
DH_FREE[0, 0] = DH_FREE[0, 2] = False
DH_FREE[5, :] = False
DH_OFFSETS_ONLY = np.zeros((6, 4), bool)
DH_OFFSETS_ONLY[1:5, 0] = True


def _project(P, K, dist):
    proj, _ = cv2.projectPoints(P, np.zeros(3), np.zeros(3), K, dist)
    return proj.reshape(-1, 2)


def _hand_eye(T_base_flange, T_gcam_board):
    """Pick the cv2 hand-eye method whose board estimates agree best."""
    Rg = [T[:3, :3] for T in T_base_flange]
    tg = [T[:3, 3] for T in T_base_flange]
    Rt = [T[:3, :3] for T in T_gcam_board]
    tt = [T[:3, 3] for T in T_gcam_board]
    best = None
    for name, m in [('tsai', cv2.CALIB_HAND_EYE_TSAI), ('park', cv2.CALIB_HAND_EYE_PARK),
                    ('horaud', cv2.CALIB_HAND_EYE_HORAUD), ('daniilidis', cv2.CALIB_HAND_EYE_DANIILIDIS)]:
        try:
            Rx, tx = cv2.calibrateHandEye(Rg, tg, Rt, tt, method=m)
        except cv2.error:
            continue
        X = np.eye(4)
        X[:3, :3], X[:3, 3] = Rx, tx.ravel()
        Ys = [Tf @ X @ Tb for Tf, Tb in zip(T_base_flange, T_gcam_board)]
        spread = float(np.median([np.linalg.norm(Y[:3, 3] - np.median([y[:3, 3] for y in Ys], 0)) for Y in Ys]))
        if np.all(np.isfinite(X)) and (best is None or spread < best[2]):
            best = (name, X, spread, Ys)
    if best is None:
        raise RuntimeError('hand-eye failed: need more varied poses (rotate the wrist between samples)')
    return best


def solve(samples, board, gcam, fcam, dh_ctrl, opts=None):
    """gcam/fcam: {'K': 3x3, 'dist': (5,), 'size': [w, h]} (fcam may be None).

    opts: intrinsics (bool, estimate gripper intrinsics), dh ('none'|'offsets'|'full'),
    dh_sigma_m, dh_sigma_rad.
    Returns a JSON-friendly result dict.
    """
    opts = {'intrinsics': True, 'dh': 'full', 'dh_sigma_m': 0.002, 'dh_sigma_rad': 0.002, **(opts or {})}
    log = []
    g = [s for s in samples if s.get('gripper_corners')]
    if len(g) < 4:
        raise RuntimeError(f'need at least 4 samples where the gripper camera sees the board (have {len(g)})')
    g_c = [np.asarray(s['gripper_corners'], np.float64) for s in g]
    q_g = [np.asarray(s['q']) for s in g]

    # 1. gripper camera intrinsics
    Kg, dg = np.asarray(gcam['K'], float), np.asarray(gcam['dist'], float).ravel()
    intr = None
    if opts['intrinsics'] and len(g) >= 6:
        rms, Kg, dg, _, _ = cv2.calibrateCamera(
            [board.obj.astype(np.float32)] * len(g), [c.astype(np.float32) for c in g_c],
            tuple(gcam['size']), Kg.copy(), dg.copy(),
            flags=cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_K3)
        dg = dg.ravel()
        intr = {'rms_px': float(rms), 'K': Kg.tolist(), 'dist': dg.tolist(), 'size': list(gcam['size'])}
        log.append(f'gripper intrinsics: rms {rms:.3f}px, fx {Kg[0, 0]:.1f} fy {Kg[1, 1]:.1f}')

    # 2. per-sample board poses and robot FK
    T_gb = [solve_pnp(board, c, Kg, dg)[0] for c in g_c]
    T_bf = [kin.fk(q, dh_ctrl) for q in q_g]

    # 3. hand-eye initial guess
    method, X0, spread, Ys = _hand_eye(T_bf, T_gb)
    Y0 = kin.mean_T(Ys)
    log.append(f'hand-eye init ({method}): board spread {spread * 1000:.2f}mm')

    # 4. fixed camera initial guess
    f = [s for s in samples if s.get('fixed_corners')] if fcam else []
    f_c = [np.asarray(s['fixed_corners'], np.float64) for s in f]
    Kf = np.asarray(fcam['K'], float) if fcam else None
    df = np.asarray(fcam['dist'], float).ravel() if fcam else None
    F0 = kin.mean_T([Y0 @ kin.inv(solve_pnp(board, c, Kf, df)[0]) for c in f_c]) if f else None

    # 5. bundle adjustment
    def run(dh_base, mask, sig):
        n_dh = int(mask.sum())
        x0 = np.r_[kin.T_to_vec(X0), kin.T_to_vec(Y0),
                   kin.T_to_vec(F0) if F0 is not None else [], np.zeros(n_dh)]
        sig_v = np.tile([sig[1], sig[0], sig[0], sig[1]], (6, 1))[mask]

        def unpack(x):
            X, Y = kin.vec_to_T(x[0:6]), kin.vec_to_T(x[6:12])
            k = 12
            F = None
            if F0 is not None:
                F, k = kin.vec_to_T(x[12:18]), 18
            dh = dh_base.copy()
            dh[mask] += x[k:k + n_dh]
            return X, Y, F, dh, x[k:k + n_dh]

        def residuals(x):
            X, Y, F, dh, d = unpack(x)
            P_base = (Y[:3, :3] @ board.obj.T).T + Y[:3, 3]
            r = []
            for q, c in zip(q_g, g_c):
                T = kin.inv(kin.fk(q, dh) @ X)
                r.append((_project((T[:3, :3] @ P_base.T).T + T[:3, 3], Kg, dg) - c).ravel())
            if F is not None:
                Fi = kin.inv(F)
                Pf = (Fi[:3, :3] @ P_base.T).T + Fi[:3, 3]
                pf = _project(Pf, Kf, df)
                for c in f_c:
                    r.append((pf - c).ravel())
            if n_dh:
                r.append(d / sig_v)
            return np.concatenate(r)

        sol = least_squares(residuals, x0, loss='soft_l1', f_scale=2.0, x_scale='jac', max_nfev=200)
        X, Y, F, dh, d = unpack(sol.x)
        P_base = (Y[:3, :3] @ board.obj.T).T + Y[:3, 3]
        per = []
        for s, q, c in zip(g, q_g, g_c):
            T = kin.inv(kin.fk(q, dh) @ X)
            e = np.linalg.norm(_project((T[:3, :3] @ P_base.T).T + T[:3, 3], Kg, dg) - c, axis=1)
            per.append({'id': s['id'], 'rms_px': float(np.sqrt(np.mean(e ** 2)))})
        g_rms = float(np.sqrt(np.mean([p['rms_px'] ** 2 for p in per])))
        f_rms = None
        if F is not None:
            Fi = kin.inv(F)
            pf = _project((Fi[:3, :3] @ P_base.T).T + Fi[:3, 3], Kf, df)
            f_rms = float(np.sqrt(np.mean([np.mean(np.sum((pf - c) ** 2, 1)) for c in f_c])))
        return {'X': X, 'Y': Y, 'F': F, 'dh': dh, 'gripper_rms_px': g_rms, 'fixed_rms_px': f_rms,
                'per_sample': per}

    sig = (opts['dh_sigma_m'], opts['dh_sigma_rad'])
    none = np.zeros((6, 4), bool)
    fits = {
        'nominal': run(kin.UR3E_NOMINAL_DH.copy(), none, sig),
        'controller': run(np.asarray(dh_ctrl, float), none, sig),
    }
    mask = {'none': None, 'offsets': DH_OFFSETS_ONLY, 'full': DH_FREE}[opts['dh']]
    if mask is not None:
        fits['refined'] = run(np.asarray(dh_ctrl, float), mask, sig)
    for k, v in fits.items():
        log.append(f'BA with {k} DH: gripper {v["gripper_rms_px"]:.3f}px'
                   + (f', fixed {v["fixed_rms_px"]:.3f}px' if v['fixed_rms_px'] is not None else ''))
    best_key = 'refined' if 'refined' in fits else 'controller'
    best = fits[best_key]

    # how far the refined FK moves the flange vs. the controller's FK over the sampled poses
    fk_shift = None
    if best_key == 'refined':
        d = [kin.pose_error(kin.fk(q, dh_ctrl), kin.fk(q, best['dh'])) for q in q_g]
        fk_shift = {'max_mm': max(t for t, _ in d) * 1000, 'mean_mm': float(np.mean([t for t, _ in d])) * 1000,
                    'max_mrad': max(r for _, r in d) * 1000}
    d_nom = [kin.pose_error(kin.fk(q, kin.UR3E_NOMINAL_DH), kin.fk(q, dh_ctrl))[0] for q in q_g]

    T_base_world = best['Y'] @ board.T_board_world
    return {
        'stamp': time.strftime('%Y-%m-%dT%H:%M:%S'),
        'n_samples': len(g), 'n_fixed': len(f),
        'board': board.describe(),
        'dh_used': best_key,
        'T_flange_gcam': best['X'].tolist(),
        'T_base_board': best['Y'].tolist(),
        'T_base_fcam': None if best['F'] is None else best['F'].tolist(),
        'T_world_base': kin.inv(T_base_world).tolist(),
        'gripper_intrinsics': intr or {'K': Kg.tolist(), 'dist': dg.tolist(), 'size': list(gcam['size'])},
        'dh_controller': np.asarray(dh_ctrl).tolist(),
        'dh_refined': best['dh'].tolist(),
        'dh_delta': (best['dh'] - np.asarray(dh_ctrl)).tolist(),
        'fits': {k: {'gripper_rms_px': v['gripper_rms_px'], 'fixed_rms_px': v['fixed_rms_px']}
                 for k, v in fits.items()},
        'per_sample': best['per_sample'],
        'fk_refined_vs_controller': fk_shift,
        'fk_nominal_vs_controller_mm': {'max': max(d_nom) * 1000, 'mean': float(np.mean(d_nom)) * 1000},
        'log': log,
    }
