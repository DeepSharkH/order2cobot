"""Register a CAD model in the robot base frame from the cameras (no robot motion).

    cloud:  fixed D435 depth -> table plane -> the object's top face at its CAD height ->
            min-area rectangle (centre, yaw mod 180 deg) -> disambiguate the 180 deg ->
            refine x / y / yaw on the colour image: project the CAD edges and pull them
            onto the image edges (distance transform). ~1 mm: a first guess, not a target.
    survey: the gripper-camera port survey (porttest/port_survey.py -> data/ports.yaml,
            rays agree to ~0.1 mm): fit the 8 CAD port centres with yaw/tilt/translation
            and the row pitch free.
"""
import io
import time
import urllib.request
import warnings
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.optimize import least_squares, minimize

from cadguide.model import ROOT, apply, pose, yaw_of

VISION = yaml.safe_load((ROOT / 'configs' / 'vision.yaml').read_text())
DASH = VISION['dashboard_url'].rstrip('/')
# rough work areas in base [m] (x, y): where to look for each object on the table
AREA = {'ls108g': {'x': [-0.46, -0.16], 'y': [0.06, 0.30]},
        'breadboard': {'x': [-0.48, -0.16], 'y': [-0.12, 0.12]}}
# the switch's port face points toward the robot base (-y in base), see rj45-insert skill
FACE_TOWARD = np.array([0.0, -1.0])


class RegistrationFailed(RuntimeError):
    pass


# ── capture ──────────────────────────────────────────────────────────────────

def _get(path, timeout=10):
    return urllib.request.urlopen(f'{DASH}{path}', timeout=timeout).read()


def capture(cam=0, n_depth=5):
    color = cv2.imdecode(np.frombuffer(_get(f'/api/frame/{cam}'), np.uint8), cv2.IMREAD_COLOR)
    stack = []
    for _ in range(n_depth):
        stack.append(np.load(io.BytesIO(_get(f'/api/depth/{cam}'))))
        time.sleep(0.25)
    d = np.stack(stack).astype(np.float32)
    d[d <= 0] = np.nan
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)
        depth = np.nan_to_num(np.nanmedian(d, axis=0), nan=0.0)
    return color, depth


def camera_in_base():
    """(T_base_cam, K) of the fixed camera from the dashboard's scene calibration."""
    sc = yaml.safe_load((ROOT / VISION['scene_file']).read_text())
    b = sc['base']
    T_world_base = pose(b['x'], b['y'], b.get('z', 0.0), np.radians(b['yaw_deg']))
    return np.linalg.inv(T_world_base) @ np.array(sc['T_world_fcam']), np.array(sc['fcam']['K'])


def cloud(depth, K, T):
    """Base-frame points (N,3) and pixels (N,2); depth-edge 'flying pixels' dropped."""
    k = np.ones((5, 5), np.uint8)
    far = cv2.dilate(depth, k)
    near = cv2.erode(np.where(depth > 0, depth, 10.0).astype(np.float32), k)
    v, u = np.nonzero((depth > 0) & (far - near < 0.004))
    z = depth[v, u]
    P = np.c_[(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z]
    return P @ T[:3, :3].T + T[:3, 3], np.c_[u, v]


def project(P, T_base_cam, K):
    Ti = np.linalg.inv(T_base_cam)
    q = P @ Ti[:3, :3].T + Ti[:3, 3]
    return (q[:, :2] / q[:, 2:]) * [K[0, 0], K[1, 1]] + [K[0, 2], K[1, 2]]


# ── geometry ─────────────────────────────────────────────────────────────────

def table_plane(P, tol=0.002, seed=0):
    """Dominant near-horizontal plane (point, upward normal), RANSAC + SVD refit."""
    rng = np.random.default_rng(seed)
    S = P[rng.choice(len(P), min(len(P), 40000), replace=False)]
    best, best_n = None, 0
    for _ in range(500):
        a, b, c = S[rng.choice(len(S), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-12:
            continue
        n /= np.linalg.norm(n)
        n *= np.sign(n[2] or 1)
        if n[2] < np.cos(np.radians(10)):
            continue
        k = int((np.abs((S - a) @ n) < tol).sum())
        if k > best_n:
            best, best_n = (a, n), k
    a, n = best
    inl = P[np.abs((P - a) @ n) < tol]
    c = inl.mean(0)
    n = np.linalg.svd(inl - c, full_matrices=False)[2][2]
    return c, n * np.sign(n[2])


def tilt(n):
    """Rotation taking base z onto n (smallest rotation)."""
    z = np.array([0.0, 0.0, 1.0])
    v, c = np.cross(z, n), float(z @ n)
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx / (1 + c)


def top_face_rect(model, P, c, n, area):
    """Points on the object's top face -> (centre xy, yaw mod pi, size, z of the top)."""
    h = (P - c) @ n
    height = model.top_z() - model.bottom_z()
    inxy = (P[:, 0] > area['x'][0]) & (P[:, 0] < area['x'][1]) & (P[:, 1] > area['y'][0]) & (P[:, 1] < area['y'][1])
    # wide band, then the object's own top plane: the table plane and D435 depth are not
    # flat to 1 mm over the work area (the far end of the breadboard read 3 mm high), and
    # cables crossing above the object must not join the top face
    band = inxy & (h > 0.5 * height) & (h < 1.5 * height + 0.006)
    if band.sum() < 300:
        raise RegistrationFailed(f'{model.name}: only {band.sum()} points near its top-face height '
                                 f'{height * 1000:.1f} mm above the table in the work area')
    c_top, n_top = table_plane(P[band], tol=0.0015)
    Q = P[band][np.abs((P[band] - c_top) @ n_top) < 0.0015]
    if len(Q) < 300:
        raise RegistrationFailed(f'{model.name}: no top-face plane ({len(Q)} points)')
    # largest connected blob on a 1 mm grid
    res = 0.001
    ij = np.floor((Q[:, :2] - Q[:, :2].min(0)) / res).astype(int)
    grid = np.zeros(ij.max(0) + 1, np.uint8)
    grid[ij[:, 0], ij[:, 1]] = 255
    grid = cv2.morphologyEx(grid, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    grid = cv2.morphologyEx(grid, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))   # cut thin streaks
    nlab, lab = cv2.connectedComponents(grid)
    if nlab < 2:
        raise RegistrationFailed(f'{model.name}: top face not found after cleaning')
    sizes = [(lab == k).sum() for k in range(1, nlab)]
    keep = lab[ij[:, 0], ij[:, 1]] == 1 + int(np.argmax(sizes))
    Q = Q[keep]
    (cx, cy), (w, d), ang = cv2.minAreaRect((Q[:, :2] * 1e4).astype(np.float32))
    cx, cy, w, d = cx / 1e4, cy / 1e4, w / 1e4, d / 1e4
    yaw = np.radians(ang)
    if (w < d) != (model.size[0] < model.size[1]):   # make yaw align model x with rect side w
        yaw += np.pi / 2
        w, d = d, w
    # the face is often partly occluded (cables, the arm, depth holes on white plastic):
    # place the known-size rectangle where it covers the most points and pokes out least
    W2, D2 = model.size[:2] / 2
    q = Q[:, :2][::max(1, len(Q) // 6000)]

    def cost(x):
        c_, s_ = np.cos(x[2]), np.sin(x[2])
        r = (q - x[:2]) @ np.array([[c_, -s_], [s_, c_]])
        out = np.maximum(np.abs(r) - [W2, D2], 0)
        return float(np.mean(np.minimum(np.hypot(*out.T), 0.01)))   # truncated: outliers cap

    best = min((minimize(cost, [cx + dx, cy + dy, yaw], method='Nelder-Mead',
                         options={'xatol': 1e-5, 'fatol': 1e-9, 'initial_simplex': np.array(
                             [[cx + dx, cy + dy, yaw], [cx + dx + 0.003, cy + dy, yaw],
                              [cx + dx, cy + dy + 0.003, yaw], [cx + dx, cy + dy, yaw + 0.02]])})
                for dx in (-0.01, 0, 0.01) for dy in (-0.01, 0, 0.01)), key=lambda r: r.fun)
    cx, cy, yaw = best.x
    return np.array([cx, cy]), yaw, np.array([w, d]), float(np.median(Q[:, 2])), len(Q), n_top


def rail_colour_score(model, T, color, Tc, K):
    """Breadboard 180 deg check: red (+) lines must be red, blue (-) lines blue."""
    rl = model.spec['rail_lines']
    ys = np.concatenate([np.linspace(a, b, 40) for a, b in rl['y']]) / 1000

    def redness(xs):
        P = np.array([[x / 1000, y, 0.0] for x in xs for y in ys])
        uv = np.round(project(apply(T, [1, 1], P), Tc, K)).astype(int)
        ok = (uv[:, 0] >= 0) & (uv[:, 1] >= 0) & (uv[:, 0] < color.shape[1]) & (uv[:, 1] < color.shape[0])
        px = color[uv[ok, 1], uv[ok, 0]].astype(float)
        return float(np.mean(px[:, 2] - px[:, 0])) if len(px) else 0.0
    return redness(rl['plus_red_x']) - redness(rl['minus_blue_x'])


def edge_points(model):
    """CAD edges visible from above, object frame (m), ~1 mm spacing."""
    C = model.outline_top()
    pts = [np.linspace(C[i], C[(i + 1) % 4], int(np.linalg.norm(C[(i + 1) % 4] - C[i]) * 1000))
           for i in range(4)]
    if model.kind == 'breadboard':
        rl = model.spec['rail_lines']
        for x in rl['plus_red_x'] + rl['minus_blue_x']:
            for a, b in rl['y']:
                pts.append(np.array([[x / 1000, y / 1000, 0.0] for y in np.arange(a, b, 1.0)]))
        for a, b in model.spec['strip_gaps_x'] + [model.spec['channel']['x']]:
            for x in (a, b):
                pts.append(np.array([[x / 1000, y / 1000, 0.0]
                                     for y in np.arange(-model.spec['size'][1] / 2, model.spec['size'][1] / 2, 1.0)]))
    else:   # the top front edge is also the port face's top: already in the outline
        pass
    return np.vstack(pts)


def refine_on_image(model, T, color, Tc, K, search_mm=6.0):
    """Shift / turn the model in its table plane until its edges sit on image edges."""
    g = cv2.GaussianBlur(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY), (3, 3), 0)
    edges = cv2.Canny(g, 30, 90)
    dt = cv2.distanceTransform(255 - edges, cv2.DIST_L2, 3)
    E = edge_points(model)
    n = T[:3, 2]

    def moved(x):
        dx, dy, dyaw = x[0] / 1000, x[1] / 1000, np.radians(x[2])
        c, s = np.cos(dyaw), np.sin(dyaw)
        R = tilt(n) @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ tilt(n).T
        T2 = T.copy()
        T2[:3, :3] = R @ T[:3, :3]
        T2[:3, 3] = T[:3, 3] + [dx, dy, 0]
        return T2

    def cost(x, trunc=8.0):
        uv = project(apply(moved(x), [1, 1], E), Tc, K)
        ok = (uv[:, 0] >= 0) & (uv[:, 1] >= 0) & (uv[:, 0] < dt.shape[1] - 1) & (uv[:, 1] < dt.shape[0] - 1)
        d = cv2.remap(dt, uv[ok, 0].astype(np.float32).reshape(-1, 1), uv[ok, 1].astype(np.float32).reshape(-1, 1),
                      cv2.INTER_LINEAR).ravel()
        return float(np.mean(np.minimum(d, trunc))) if ok.mean() > 0.8 else 99.0

    starts = [[x, y, 0.0] for x in (-search_mm / 2, 0, search_mm / 2) for y in (-search_mm / 2, 0, search_mm / 2)]
    best = min((minimize(cost, s, method='Nelder-Mead', options={'xatol': 0.02, 'fatol': 1e-3,
                                                                 'initial_simplex': np.array(s) + np.array(
                                                                     [[0, 0, 0], [1.5, 0, 0], [0, 1.5, 0], [0, 0, 1.0]])})
                for s in starts), key=lambda r: r.fun)
    uv = project(apply(moved(best.x), [1, 1], E), Tc, K)
    ok = (uv[:, 0] >= 0) & (uv[:, 1] >= 0) & (uv[:, 0] < dt.shape[1] - 1) & (uv[:, 1] < dt.shape[0] - 1)
    d = dt[uv[ok, 1].astype(int), uv[ok, 0].astype(int)]
    return moved(best.x), {'shift_mm': [round(float(v), 2) for v in best.x[:2]],
                           'turn_deg': round(float(best.x[2]), 3), 'edge_cost_px': round(best.fun, 2),
                           'edge_inliers': round(float(np.mean(d < 2.0)), 3)}


def overlay(model, T, scale, color, Tc, K, path):
    img = color.copy()
    E = project(model.place(T, scale, edge_points(model)), Tc, K)
    for u, v in E.astype(int):
        cv2.circle(img, (u, v), 1, (0, 255, 255), -1)
    for fid in model.features():
        u, v = project(model.place(T, scale, model.feature(fid).p)[None], Tc, K)[0].astype(int)
        cv2.circle(img, (u, v), 2, (255, 0, 255), -1)
    cv2.imwrite(str(path), img)


# ── entry points ─────────────────────────────────────────────────────────────

def from_cloud(model, out_dir=None):
    color, depth = capture(0)
    Tc, K = camera_in_base()
    P, _ = cloud(depth, K, Tc)
    area = AREA[model.name]
    near = (P[:, 0] > area['x'][0] - 0.1) & (P[:, 0] < area['x'][1] + 0.1) & \
           (P[:, 1] > area['y'][0] - 0.1) & (P[:, 1] < area['y'][1] + 0.1)
    c, n = table_plane(P[near])
    ctr, yaw, size, z_top, npts, n = top_face_rect(model, P, c, n, area)
    size_err = (size - model.size[:2]) * 1000
    hyps = []
    for k in (0, 1):
        T = np.eye(4)
        T[:3, :3] = tilt(n) @ pose(0, 0, 0, yaw + k * np.pi)[:3, :3]
        top = np.array([ctr[0], ctr[1], z_top])
        T[:3, 3] = top - T[:3, :3] @ [0, 0, model.top_z()]
        hyps.append(T)
    if model.kind == 'switch':
        face = [float((T[:3, :3] @ [0, -1, 0])[:2] @ FACE_TOWARD) for T in hyps]
        T, why = hyps[int(np.argmax(face))], f'port face toward base {FACE_TOWARD.tolist()}'
    else:
        sc = [rail_colour_score(model, T, color, Tc, K) for T in hyps]
        T, why = hyps[int(np.argmax(sc))], f'rail colours (red-blue score {max(sc):.0f} vs {min(sc):.0f})'
    T_depth = T
    T, img = refine_on_image(model, T, color, Tc, K)
    d_img = (T[:3, 3] - T_depth[:3, 3]) * 1000
    info = {'method': 'cloud+image (fixed D435)', 'points': npts,
            'rect_size_error_mm': [round(float(v), 1) for v in size_err],
            'orientation': why, 'image': img,
            'depth_vs_image_mm': [round(float(v), 2) for v in d_img[:2]],
            'table_normal': [round(float(v), 4) for v in n],
            'yaw_deg': round(np.degrees(yaw_of(T)), 2)}
    if out_dir:
        out_dir.mkdir(parents=True, exist_ok=True)
        overlay(model, T, [1, 1], color, Tc, K, out_dir / f'{model.name}_overlay.png')
        cv2.imwrite(str(out_dir / f'{model.name}_color.png'), color)
    return T, np.array([1.0, 1.0]), info


def from_survey(model, path):
    """Fit the CAD port row to the triangulated port centres (data/ports.yaml)."""
    if model.kind != 'switch':
        raise RegistrationFailed('the port survey only exists for the switch')
    sv = yaml.safe_load(Path(path).read_text())
    meas = np.array(sv['centre'])
    feats = np.array([model.feature(f'port:{k}').p for k in range(1, 9)])
    d = meas[-1] - meas[0]
    x0 = [*meas.mean(0) - feats.mean(0), np.arctan2(d[1], d[0]), 0.0, 0.0, 0.0]

    def T_of(x):
        return pose(*x[:3], x[3], x[4], x[5])

    def res(x):
        T = T_of(x)
        pred = model.place(T, [1 + x[6], 1], feats)
        return np.r_[(pred - meas).ravel() / 1e-4, x[4:6] / np.radians(2)]

    # translation must be solved with the rotation about the CAD origin, not the row centre
    x0[:3] = meas.mean(0) - pose(0, 0, 0, x0[3])[:3, :3] @ feats.mean(0)
    sol = least_squares(res, np.array(x0 + [0.0]))
    T, scale = T_of(sol.x), np.array([1 + sol.x[6], 1.0])
    pred = model.place(T, scale, feats)
    r = np.linalg.norm(pred - meas, axis=1) * 1000
    info = {'method': f'gripper-cam port survey ({path})', 'views': sv.get('views'),
            'measured': time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(Path(path).stat().st_mtime)),
            'residual_mm': [round(float(v), 2) for v in r], 'rms_mm': round(float(np.sqrt(np.mean(r ** 2))), 3),
            'pitch_mm': round(model.spec['pitch'] * scale[0], 3), 'yaw_deg': round(np.degrees(yaw_of(T)), 2)}
    return T, scale, info


def from_touch(model, face_y, top_z, left_x, yaw=None):
    """Switch pose from the plug-tip touch references (rj45_b touch_refs: face, top-edge pass,
    left-end pass; tip coordinates, metres). The pass points sit a tip half-width past the
    edges; the port-1 offsets measured once (configs/products/ls108g.yaml, run
    20261025-184457) carry that, so port 1 lands exactly where the touch formula puts it and
    the CAD gives the rest. The yaw comes from the previous registration (touches give none)."""
    if model.kind != 'switch':
        raise RegistrationFailed('touch registration exists for the switch only')
    prod = yaml.safe_load((ROOT / 'configs' / 'products' / 'ls108g.yaml').read_text())
    p1 = np.array([left_x + prod['port1_from_left_pass_x'], face_y, top_z - prod['port1_below_top_pass_z']])
    yaw = 0.0 if yaw is None else yaw
    T = pose(0, 0, 0, yaw)
    T[:3, 3] = p1 - T[:3, :3] @ model.feature('port:1').p
    info = {'method': 'plug-tip touches (face, top-edge pass, left-end pass) + port-1 offsets of 10-25',
            'touch_mm': [round(v * 1000, 2) for v in (face_y, top_z, left_x)],
            'yaw_deg': round(np.degrees(yaw), 3)}
    return T, np.array([1.0, 1.0]), info
