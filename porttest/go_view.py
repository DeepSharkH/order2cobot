"""Move to a camera view of a target (checked joint move), or just print the plan.

Usage: python -m porttest.go_view tx ty tz dist deg roll [go]
The camera looks along +y, `deg` down, from `dist`; `roll` turns the image about the
view axis. Prints the flange pose and IK joints next to the current ones; with 'go'
moves there through URExec.move_checked (elbow, wrist-3 cable and plate clearance checks).
"""
import sys

import numpy as np

from porttest.locate import flange
from porttest.urexec import URExec, call
from porttest.view_pose import flange_for_view

JOINTS = ('base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3')
W3_REF = -4.97      # cable relaxed here (operator, 2026-10-25 after the camera remount)


def main(tx, ty, tz, dist, deg, roll, go=False):
    ex = URExec()
    a = np.radians(deg)
    pose, _ = flange_for_view([tx, ty, tz], [0.0, np.cos(a), -np.sin(a)], dist, flange(ex.b), (1.0,), roll=roll)
    q0 = [call(ex.b, 'robot_get_joint_positions')['data'][k] for k in JOINTS]
    q = call(ex.b, 'robot_inverse_kinematics', pose={'position': pose[:3], 'orientation': pose[3:]}, qNear=q0)['data']
    q = [q[k] for k in JOINTS]
    print('pose  ', np.round(pose, 4).tolist())
    print('q now ', np.round(q0, 3).tolist())
    print('q goal', np.round(q, 3).tolist(), 'max joint change', round(float(np.max(np.abs(np.subtract(q, q0)))), 2), 'rad')
    if go:
        ex.move_checked(pose, w3_ref=W3_REF)
        print('arrived', np.round(ex.pose()[0], 4).tolist())


if __name__ == '__main__':
    v = [float(x) for x in sys.argv[1:7]]
    main(*v, go='go' in sys.argv)
