"""Hand-eye capture around a pose where the gripper camera sees the board: tilts about base x/y,
wrist turns and shifts, each through URExec.move_checked (IK, bent elbow, wrist-3 cable limit,
plug clearance). Captures land in the dashboard's current session.
Usage: python -m porttest.handeye_capture2 x y z
"""
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

from porttest.look import look
from porttest.urexec import BridgeError, URExec

# (rotation vector in base [deg], shift [m]) relative to the anchor pose
VARIANTS = [
    ((12, 0, 0), (0, 0, 0)), ((-12, 0, 0), (0, 0, 0)), ((0, 12, 0), (0, 0, 0)), ((0, -12, 0), (0, 0, 0)),
    ((0, 0, 25), (0, 0, 0)), ((0, 0, -25), (0, 0, 0)),
    ((9, 9, 0), (0, 0, 0)), ((-9, 9, 0), (0, 0, 0)), ((9, -9, 0), (0, 0, 0)), ((-9, -9, 0), (0, 0, 0)),
    ((8, 0, 20), (0.03, 0, 0)), ((-8, 0, -20), (-0.03, 0, 0)), ((0, 8, -20), (0, 0.03, 0)), ((0, -8, 20), (0, -0.03, 0)),
    ((0, 0, 0), (0, 0, -0.05)), ((6, -6, 15), (0, 0, 0.03)),
]


def main(x, y, z, variants=None, w3_ref=None):
    ex = URExec()
    R0 = R.from_rotvec([0, np.pi, 0])
    good = []
    for rot, sh in variants or VARIANTS:
        Rn = R.from_rotvec(np.radians(rot)) * R0
        pose = [x + sh[0], y + sh[1], z + sh[2]] + list(Rn.as_rotvec())
        try:
            r = look(ex, pose, w3_ref)
        except BridgeError as e:
            print(rot, sh, 'skipped:', str(e)[:120])
            continue
        print(rot, sh, r)
        if r['gripper']:
            good.append(r['sample'])
    ex.move_checked([x, y, z, 0, np.pi, 0], w3_ref=w3_ref)
    print('samples with the board in the gripper camera:', good)


if __name__ == '__main__':
    main(*[float(a) for a in sys.argv[1:4]])
