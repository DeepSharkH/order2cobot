"""Manual calibration on the robot (read-only on the bridge: it only reads the joints).

The operator seats a centre-gripped part by hand (freedrive / pendant jog) at known holes near
the board corners, and moves to the workspace poses; this records each one.

  python -m breadboard.teach corner A1 A3          # the part's holes (solver ids or CAD names)
  python -m breadboard.teach pose storage_s1       # a named workspace pose
  python -m breadboard.teach fit                   # board pose from 3+ corners, residuals
  python -m breadboard.teach show

Poses are stored as flange poses + joints, so a later TCP measurement re-fits without
re-teaching. Teach all corners at the insertion orientation (see breadboard/calibration.py).
"""
import argparse
import sys
import time

import numpy as np

from breadboard import calibration as cal
from dashboard import kinematics as kin
from urcore.urexec import URBridge, call

CLIENT = 'breadboard-teach'


def read_robot():
    """Joints and flange pose (FK with the controller's own DH, independent of the active TCP)."""
    b = URBridge(client_name=CLIENT)
    q = [float(call(b, 'robot_get_joint_positions')['data'][k]) for k in kin.JOINT_NAMES]
    dh = kin.dh_from_bridge(call(b, 'robot_get_kinematic_info')['data'])
    return q, cal.pose_of(kin.fk(q, dh))


def main():
    ap = argparse.ArgumentParser(prog='python -m breadboard.teach')
    sub = ap.add_subparsers(dest='cmd', required=True)
    c = sub.add_parser('corner', help='record a seated part at these holes')
    c.add_argument('holes', nargs='+')
    c.add_argument('--note', default='')
    p = sub.add_parser('pose', help='record a named workspace pose')
    p.add_argument('name')
    sub.add_parser('fit', help='fit the board pose from the corners')
    sub.add_parser('show')
    a = ap.parse_args()
    calib = cal.load()
    stamp = time.strftime('%Y-%m-%dT%H:%M:%S')
    if a.cmd == 'corner':
        cal.Board().part(a.holes)                       # unknown hole -> KeyError before touching the file
        q, F = read_robot()
        calib['corners'].append({'holes': a.holes, 'flange': F, 'q': q, 'stamp': stamp, 'note': a.note})
        cal.save(calib)
        print(f"corner {len(calib['corners'])}: {a.holes} flange {np.round(F[:3], 4)}")
    elif a.cmd == 'pose':
        q, F = read_robot()
        calib['workspace'][a.name] = {'flange': F, 'q': q, 'stamp': stamp}
        cal.save(calib)
        print(f'pose {a.name}: flange {np.round(F[:3], 4)}')
    elif a.cmd == 'fit':
        g = cal.load_gripper()
        T, res = cal.fit(calib, g['tcp_pins'])
        calib['fit'] = {'T_base_board': T.round(6).tolist(), 'residuals_mm': res.round(3).tolist(),
                        'tcp_pins': g['tcp_pins'], 'stamp': stamp}
        cal.save(calib)
        print('residuals mm:', res.round(3), '' if res.max() < cal.RESIDUAL_WARN_MM else
              f'-> above {cal.RESIDUAL_WARN_MM} mm: check the board model (rows, e-f channel) and the corners')
    else:
        print(cal.yaml.safe_dump(calib, sort_keys=False) if calib['corners'] or calib['workspace'] else 'nothing taught yet')


if __name__ == '__main__':
    sys.exit(main())
