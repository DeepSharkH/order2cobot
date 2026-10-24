"""Capture hand-eye samples: orbit the gripper camera around the board and capture in the dashboard.

The orbit centre is the board, assumed ~`dist` m out along the tool axis from the flange.
Usage: python -m porttest.handeye_capture [dist]
"""
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

from porttest import dash
from porttest.geometry import urpose
from porttest.urexec import URExec

# (rotation axis in the tool frame, degrees) pairs applied in order; '' = shift along tool z
POSES_ALL = [
    [('x', 15)], [('x', -15)], [('y', 15)], [('y', -15)],
    [('x', 12), ('y', 12)], [('x', -12), ('y', 12)], [('x', 12), ('y', -12)], [('x', -12), ('y', -12)],
    [('z', 40)], [('z', -40)], [('z', 35), ('x', 12)], [('z', -35), ('y', 12)],
    [('dz', -0.05)], [('dz', 0.04), ('x', -10)],
]

POSES = POSES_ALL


def orbit(F, P, steps):
    """Flange 4x4 F moved by rotations about axes through P (tool-frame axes) and shifts."""
    T = F.copy()
    for ax, v in steps:
        if ax == 'dz':
            T[:3, 3] += T[:3, 2] * v
            continue
        axis = T[:3, 'xyz'.index(ax)]
        Rr = R.from_rotvec(axis * np.radians(v)).as_matrix()
        T[:3, :3] = Rr @ T[:3, :3]
        T[:3, 3] = Rr @ (T[:3, 3] - P) + P
    return T


def main(dist=0.41, only=None):
    ex = URExec()
    pose, _ = ex.pose()
    F = np.eye(4)
    F[:3, :3] = R.from_rotvec(pose[3:]).as_matrix()
    F[:3, 3] = pose[:3]
    P = F[:3, 3] + F[:3, 2] * dist
    print('start', np.round(pose, 4), 'orbit centre', np.round(P, 3))
    home = urpose(pose)
    if not only:
        print('capture start:', dash.action('capture', timeout=30)[0])
    for i, steps in enumerate(only or POSES):
        T = orbit(F, P, steps)
        target = list(T[:3, 3]) + list(R.from_matrix(T[:3, :3]).as_rotvec())
        try:
            msgs, _ = ex.run(f'movej(get_inverse_kin({urpose(target)}, qnear=get_actual_joint_positions()), a=0.6, v=0.4)\n'
                             'sleep(0.5)', timeout=60)
        except Exception as e:                      # unreachable or a stop: skip this pose
            print(i, steps, 'move failed:', str(e)[:160])
            continue
        time.sleep(2.0)
        try:
            print(i, steps, dash.action('capture', timeout=30)[0], msgs)
        except Exception as e:
            print(i, steps, 'capture failed:', e)
    ex.run(f'movej(get_inverse_kin({home}, qnear=get_actual_joint_positions()), a=0.6, v=0.4)', timeout=60)
    print('samples:', len(dash.state()['samples']))


if __name__ == '__main__':
    main(*[float(a) for a in sys.argv[1:]])
