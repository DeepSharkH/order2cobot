"""Joint calibration from a dashboard session: gripper hand-eye, board, fixed D435 (= base in the
camera) and optionally DH joint offsets, with the thorough corner detection for both cameras.

Writes configs/calibration/result.yaml (the dashboard's result) and updates
configs/calibration/handeye_gripper.yaml (used by porttest.locate).
Usage: python -m porttest.joint_solve <session dir> [dh: none|offsets|full]
"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

from dashboard import calibration as cal
from dashboard import kinematics as kin
from porttest import dash
from porttest.fcam_calib import corners
from porttest.urexec import URBridge, call

ROOT = Path(__file__).resolve().parent.parent
SQUARE = 0.0152


def main(session, dh_mode='offsets', *drop):
    d = Path(session)
    S = json.loads((d / 'samples.json').read_text())
    for s in S:
        for cam in ('gripper', 'fixed'):
            c = corners(cv2.imread(str(d / s[f'{cam}_image']))) if s.get(f'{cam}_image') else None
            s[f'{cam}_corners'] = None if c is None else c.reshape(-1, 1, 2).tolist()
    S = [s for s in S if s['id'] not in {int(x) for x in drop}]
    # the D435 sees a static board: keep only detections that agree with the median (no flipped order)
    fc = [np.array(s['fixed_corners']).reshape(-1, 2) for s in S if s['fixed_corners']]
    med = np.median(np.array(fc), axis=0)
    for s in S:
        if s['fixed_corners'] and np.abs(np.array(s['fixed_corners']).reshape(-1, 2) - med).max() > 3.0:
            print(f"sample {s['id']}: D435 corners off the median, dropped")
            s['fixed_corners'] = None
    S = [s for s in S if s['gripper_corners'] or s['fixed_corners']]
    print(f"{len(S)} samples, gripper sees the board in {sum(1 for s in S if s['gripper_corners'])}, "
          f"D435 in {sum(1 for s in S if s['fixed_corners'])}")
    cams = {c['name']: c for c in dash.state()['cams']}
    g = cams['Gripper Gemini 335Lg']
    f = cams['Fixed D435']
    gcam = {'K': np.array(g['K']), 'dist': np.zeros(5), 'size': g['size']}
    fcam = {'K': np.array(f['K']), 'dist': np.array(f.get('dist') or np.zeros(5)), 'size': f['size']}
    dh = kin.dh_from_bridge(call(URBridge(), 'robot_get_kinematic_info')['data'])
    res = cal.solve(S, cal.Board(8, 7, SQUARE), gcam, fcam, dh, {'intrinsics': False, 'dh': dh_mode})
    res['session'] = str(d.relative_to(ROOT)) if d.is_relative_to(ROOT) else str(d)
    print('\n'.join(res['log'][-6:]))
    print('fits', json.dumps(res['fits']))
    print('refined FK vs controller', res.get('fk_refined_vs_controller'))
    X = np.array(res['T_flange_gcam'])
    F = np.array(res['T_base_fcam'])
    print('gripper cam in flange [mm]', np.round(X[:3, 3] * 1000, 1), 'axis', np.round(X[:3, 2], 3))
    print('D435 in base [m]', np.round(F[:3, 3], 4), 'looking', np.round(F[:3, 2], 3))
    (ROOT / 'configs/calibration/result.yaml').write_text(yaml.safe_dump(res, sort_keys=False))
    he = ROOT / 'configs/calibration/handeye_gripper.yaml'
    old = yaml.safe_load(he.read_text())
    old.update({'square': SQUARE, 'T_flange_gcam': X.tolist(), 'T_base_board': res['T_base_board'],
                'gripper_K': g['K'], 'session': res['session'], 'rms_px': res['fits'][res['dh_used']]['gripper_rms_px']})
    he.write_text(yaml.safe_dump(old, sort_keys=False))
    print('saved result.yaml and handeye_gripper.yaml')


if __name__ == '__main__':
    main(*sys.argv[1:])
