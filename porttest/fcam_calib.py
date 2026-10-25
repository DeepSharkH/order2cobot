"""Calibrate the fixed D435 in robot coordinates through the calibrated gripper camera.

Per board placement: the gripper camera gives T_base_board (hand-eye + PnP), the D435
gives T_fcam_board (PnP), so T_base_fcam = T_base_board @ inv(T_fcam_board). Placements are
appended to data/fcam_samples.json; `solve` averages them and writes
configs/calibration/fcam_extrinsics.yaml.

Usage: python -m porttest.fcam_calib add      (robot must already see the board)
       python -m porttest.fcam_calib solve
"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

from dashboard import calibration as cal
from porttest import dash
from porttest.locate import K as KG
from porttest.locate import SQUARE, X, flange

ROOT = Path(__file__).resolve().parent.parent
SAMPLES = ROOT / 'data/fcam_samples.json'
OUT = ROOT / 'configs/calibration/fcam_extrinsics.yaml'
FLAGS = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
BOARD = cal.Board(8, 7, SQUARE)


def corners(img):
    ok, c = cv2.findChessboardCornersSB(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (8, 7), flags=FLAGS)
    return c.reshape(-1, 2).astype(np.float64) if ok else None


def pnp(c, K, dist):
    T = cal.solve_pnp(BOARD, c.reshape(-1, 1, 2), K, dist)[0]
    proj, _ = cv2.projectPoints(BOARD.obj, cv2.Rodrigues(T[:3, :3])[0], T[:3, 3], K, dist)
    return T, float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - c) ** 2, axis=1))))


def add():
    msg, _ = dash.action('capture', timeout=30)
    sess = sorted((ROOT / 'data/calib').iterdir())[-1]
    s = json.loads((sess / 'samples.json').read_text())[-1]
    F = flange()
    fc = next(c for c in dash.state()['cams'] if 'D435' in c['name'])
    Kf, df = np.array(fc['K']), np.array(fc.get('dist') or np.zeros(5))
    cg = corners(cv2.imread(str(sess / s['gripper_image'])))
    cf = corners(cv2.imread(str(sess / s['fixed_image'])))
    if cg is None or cf is None:
        raise SystemExit(f'board not found (gripper {cg is not None}, D435 {cf is not None})')
    Tgb, eg = pnp(cg, KG, np.zeros(5))
    Tbb = F @ X @ Tgb
    out = {'sample': s['id'], 'T_base_board': Tbb.tolist(), 'rms_gripper_px': eg, 'variants': []}
    for name, c in (('as_detected', cf), ('reversed', cf[::-1].copy())):   # corner order 0 or 180 deg
        Tfb, ef = pnp(c, Kf, df)
        Tbf = Tbb @ np.linalg.inv(Tfb)
        out['variants'].append({'order': name, 'T_base_fcam': Tbf.tolist(), 'rms_fixed_px': ef})
        print(f'{name:12s} D435 at {np.round(Tbf[:3, 3], 3)} looking {np.round(Tbf[:3, 2], 2)}  '
              f'PnP rms {ef:.2f}px (gripper {eg:.2f}px)')
    allS = json.loads(SAMPLES.read_text()) if SAMPLES.exists() else []
    allS.append(out)
    SAMPLES.write_text(json.dumps(allS))
    print(f'{msg}; placement {len(allS)} stored, board centre in base '
          f'{np.round((Tbb @ np.array([3.5 * SQUARE, 3 * SQUARE, 0, 1]))[:3], 4)}')


def solve(order=None):
    S = json.loads(SAMPLES.read_text())
    best = None
    for name in ([order] if order else ['as_detected', 'reversed']):
        Ts = [np.array(next(v for v in s['variants'] if v['order'] == name)['T_base_fcam']) for s in S]
        t = np.array([T[:3, 3] for T in Ts])
        rot = R.from_matrix([T[:3, :3] for T in Ts])
        mean_r = rot.mean()
        spread_t = np.linalg.norm(t - t.mean(0), axis=1)
        spread_r = np.degrees((rot * mean_r.inv()).magnitude())
        print(f'{name}: {len(S)} placements, position spread [mm] {np.round(spread_t * 1000, 1)}, '
              f'angle spread [deg] {np.round(spread_r, 2)}')
        if best is None or spread_t.max() < best[0]:
            T = np.eye(4)
            T[:3, :3] = mean_r.as_matrix()
            T[:3, 3] = t.mean(0)
            best = (spread_t.max(), name, T, spread_t, spread_r)
    _, name, T, st, sr = best
    OUT.write_text(yaml.safe_dump({'T_base_fcam': T.tolist(), 'order': name, 'placements': len(S),
                                   'spread_mm': (st * 1000).tolist(), 'spread_deg': sr.tolist()}, sort_keys=False))
    print('chosen', name, 'D435 at', np.round(T[:3, 3], 4), '-> saved', OUT)


if __name__ == '__main__':
    {'add': add, 'solve': solve}[sys.argv[1]](*sys.argv[2:])
