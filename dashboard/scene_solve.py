"""Gripper-cam mount + robot base from board views at several arm poses.

World = board centre, z up (the board is the reference, it does not move).
Unknowns: base (x, y, yaw) on the board plane (z = 0, upright) and the mount
T_flange_gcam (6). Residuals: pixel reprojection of the board corners in each
view, predicted through base · FK(q) · mount. Several starts (rotation from the
one-view mount, and the photo guess with both tilt signs and four image rolls)
— the best fit wins.

Run offline on a capture session:
  python -m dashboard.scene_solve data/calib/<session> [--save]
"""

import argparse
import json
from pathlib import Path

import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R

from . import calibration as cal
from . import kinematics as kin

ROOT = Path(__file__).resolve().parent.parent


def base_T(x, y, yaw):
    T = np.eye(4)
    c, s = np.cos(yaw), np.sin(yaw)
    T[:2, :2] = [[c, -s], [s, c]]
    T[:2, 3] = x, y
    return T


def unpack(p):
    return base_T(*p[:3]), kin.vec_to_T(p[3:9])


def residuals(p, views, board, K, dist, dh):
    Tb, M = unpack(p)
    Pw = (kin.inv(board.T_board_world) @ np.c_[board.obj, np.ones(len(board.obj))].T).T[:, :3]
    out = []
    for q, uv in views:
        Tc = kin.inv(Tb @ kin.fk(q, dh) @ M)
        P = (Tc[:3, :3] @ Pw.T).T + Tc[:3, 3]
        if np.any(P[:, 2] <= 0.01):
            out.append(np.full(uv.size, 50.0))
            continue
        out.append((cal._project(P, K, dist) - uv).ravel())
    return np.concatenate(out)


def starts(mount_view=None, base0=(0.0, 0.0, 0.0)):
    """Initial guesses. Photo guess: camera on the side bracket, ~80 mm out along
    flange +y (beside the Hand-E, perpendicular to the fingers), ~100 mm below the
    flange, optical axis tilted ~23 deg from the tool axis."""
    t0 = np.array([0.0, 0.08, 0.10])
    rots = []
    if mount_view is not None:
        rots.append(R.from_matrix(mount_view[:3, :3]))
    for tilt in (23, -23):
        for roll in (0, 90, 180, 270):
            rots.append(R.from_euler('x', tilt, degrees=True) * R.from_euler('z', roll, degrees=True))
    for r in rots:
        for t in ([t0] + ([mount_view[:3, 3]] if mount_view is not None else [])):
            for yaw in np.radians([base0[2], base0[2] + 90, base0[2] - 90, base0[2] + 180]):
                yield np.r_[base0[0], base0[1], yaw, r.as_rotvec(), t]


def solve(views, board, K, dist, dh, mount_view=None, base0=(0.0, 0.0, 0.0)):
    args = (views, board, K, dist, dh)
    # screen every start briefly, refine the three best
    short = sorted((least_squares(residuals, p0, args=args, loss='soft_l1', f_scale=5.0, max_nfev=25)
                    for p0 in starts(mount_view, base0)), key=lambda r: r.cost)[:3]
    best = min((least_squares(residuals, r.x, args=args, loss='soft_l1', f_scale=2.0, max_nfev=400) for r in short),
               key=lambda r: r.cost)
    r = least_squares(residuals, best.x, args=args, loss='soft_l1', f_scale=1.0)
    res = residuals(r.x, views, board, K, dist, dh).reshape(len(views), -1, 2)
    per = np.sqrt((res ** 2).sum(-1).mean(-1))
    return r.x, per


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('session')
    ap.add_argument('--save', action='store_true', help='write base + mount into configs/calibration/scene.yaml')
    ap.add_argument('--drop', type=float, default=3.0, help='drop views above this rms [px] and re-solve')
    a = ap.parse_args()
    cfg = yaml.safe_load((ROOT / 'configs/calib_dashboard.yaml').read_text())
    board = cal.Board(cfg['board']['cols'], cfg['board']['rows'], cfg['board']['square'])
    sess = Path(a.session)
    meta = json.loads((sess / 'meta.json').read_text())
    K, dist, dh = np.array(meta['K']), np.array(meta['dist']), np.array(meta['dh'])
    samples = json.loads((sess / 'samples.json').read_text())
    views = [(np.array(s['q']), np.array(s['gripper_corners'])) for s in samples if s.get('gripper_corners')]
    ids = [s['id'] for s in samples if s.get('gripper_corners')]
    scene_file = ROOT / cfg.get('scene_file', 'configs/calibration/scene.yaml')
    scene = yaml.safe_load(scene_file.read_text()) if scene_file.exists() else {}
    mv = np.array(scene['T_flange_gcam']) if scene.get('T_flange_gcam') else None
    b = scene.get('base', {'x': 0, 'y': 0, 'yaw_deg': 0})
    x, per = solve(views, board, K, dist, dh, mv, (b['x'], b['y'], np.radians(b['yaw_deg'])))
    keep = per < a.drop
    if not keep.all():
        print('dropping views', [i for i, k in zip(ids, keep) if not k], np.round(per[~keep], 1))
        views = [v for v, k in zip(views, keep) if k]
        ids = [i for i, k in zip(ids, keep) if k]
        x, per = solve(views, board, K, dist, dh, kin.vec_to_T(x[3:9]), (x[0], x[1], x[2]))
    Tb, M = unpack(x)
    print(f'{len(views)} views, rms {np.sqrt(np.mean(per ** 2)):.2f} px, worst {per.max():.2f} px')
    print('per view', dict(zip(ids, np.round(per, 2))))
    print(f'base x {x[0] * 1000:.1f} mm  y {x[1] * 1000:.1f} mm  yaw {np.degrees(x[2]):.2f} deg')
    eul = R.from_matrix(M[:3, :3]).as_euler('xyz', degrees=True)
    print(f'mount t {np.round(M[:3, 3] * 1000, 1)} mm  rot xyz {np.round(eul, 1)} deg  '
          f'optical axis vs tool axis {np.degrees(np.arccos(M[2, 2])):.1f} deg')
    if a.save:
        scene['base'] = {'x': float(x[0]), 'y': float(x[1]), 'yaw_deg': float(np.degrees(x[2]))}
        scene['T_flange_gcam'] = M.tolist()
        scene['gcam'] = {'stamp': meta.get('stamp'), 'session': str(sess), 'views': len(views),
                         'rms_px': float(np.sqrt(np.mean(per ** 2))), 'method': 'scene_solve (base + mount)'}
        scene_file.write_text(yaml.safe_dump(scene, sort_keys=False))
        print('saved', scene_file)


if __name__ == '__main__':
    main()
