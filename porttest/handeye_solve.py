"""Offline hand-eye solve of the gripper camera with a sweep over the board square size.

The robot motion is metric, so a wrong square size shows up as a worse bundle-adjustment
fit; the sweep picks the size that fits best. Writes data/handeye_gripper.yaml.
Usage: python -m porttest.handeye_solve <session dir | samples json> [drop ids...]
"""
import json
import sys
from pathlib import Path

import numpy as np
import yaml

from dashboard import calibration as cal
from dashboard import kinematics as kin
from porttest import dash
from porttest.urexec import URBridge, call

OUT = Path(__file__).resolve().parent.parent / 'data' / 'handeye_gripper.yaml'


def main(session, drop=()):
    f = Path(session) if str(session).endswith('.json') else Path(session) / 'samples.json'
    samples = [s for s in json.loads(f.read_text())
               if s['id'] not in drop and s.get('gripper_corners')]
    for s in samples:                       # the fixed camera does not take part here
        s['fixed_corners'] = None
    cam = next(c for c in dash.state()['cams'] if 'Gemini' in c['name'])
    gcam = {'K': np.array(cam['K']), 'dist': np.zeros(5), 'size': cam['size']}
    dh = kin.dh_from_bridge(call(URBridge(), 'robot_get_kinematic_info')['data'])
    print(f'{len(samples)} samples: {[s["id"] for s in samples]}')
    fits = []
    for sq in np.arange(0.014, 0.0261, 0.0005):
        try:
            res = cal.solve(samples, cal.Board(8, 7, float(sq)), gcam, None, dh, {'intrinsics': False, 'dh': 'none'})
        except Exception as e:
            print(f'{sq * 1000:.1f} mm: {e}')
            continue
        rms = min(f['gripper_rms_px'] for f in res['fits'].values())
        fits.append((rms, sq, res))
        print(f'square {sq * 1000:5.1f} mm  rms {rms:.3f} px')
    rms, sq, res = min(fits, key=lambda f: f[0])
    # refine around the best value
    for s2 in np.arange(sq - 0.0005, sq + 0.00051, 0.0001):
        r2 = cal.solve(samples, cal.Board(8, 7, float(s2)), gcam, None, dh, {'intrinsics': False, 'dh': 'none'})
        rr = min(f['gripper_rms_px'] for f in r2['fits'].values())
        if rr < rms:
            rms, sq, res = rr, s2, r2
    print(f'best square {sq * 1000:.2f} mm, rms {rms:.3f} px')
    X = np.array(res['T_flange_gcam'])
    print('T_flange_gcam\n', np.round(X, 4))
    print('camera origin in flange [mm]', np.round(X[:3, 3] * 1000, 1),
          ' optical axis in flange', np.round(X[:3, 2], 3))
    print('\n'.join(res.get('log', [])[-6:]))
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(yaml.safe_dump({'square': float(sq), 'rms_px': float(rms), 'session': str(session),
                                   'samples': [s['id'] for s in samples],
                                   'T_flange_gcam': X.tolist(),
                                   'T_base_board': res.get('T_base_board'),
                                   'gripper_K': cam['K']}, sort_keys=False))
    print('saved', OUT)


if __name__ == '__main__':
    main(sys.argv[1], [int(a) for a in sys.argv[2:]])
