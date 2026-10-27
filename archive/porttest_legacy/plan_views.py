"""Plan gripper-camera views of the port face that keep the plug and its cable off the table.

For each candidate (distance, downward angle, image-x sign) and the three survey shifts
(0, +-5 cm along x) print the flange pose, the plug tip, the shell rear (where the cable
leaves and droops) and the IK joints. Usage: python -m porttest.plan_views tx ty tz
"""
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

from porttest.locate import flange
from porttest.urexec import URBridge, call
from porttest.view_pose import flange_for_view

PLATE_Z = 0.015
MIN_REAR = 0.10          # shell rear at least this far above the plate
TIP = np.array([0.0, 0.035, 0.19])        # plug tip in the flange frame (this grip)
REAR = np.array([0.0, 0.035 - 0.057, 0.19])
JOINTS = ('base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3')


def plan(target, b, w3=None, w3_tol=0.4, rolls=range(0, 360, 30)):
    F0 = flange(b)
    q0 = [call(b, 'robot_get_joint_positions')['data'][k] for k in JOINTS]
    out = []
    w3 = q0[5] if w3 is None else w3
    for dist in (0.28, 0.32):
        for deg in (25, 30, 35):
            a = np.radians(deg)
            view = [0.0, np.cos(a), -np.sin(a)]
            for xs in rolls:
                rows, ok = [], True
                for dx in (0.0, 0.05, -0.05):
                    t = np.array(target) + [dx, 0, 0]
                    pose, _ = flange_for_view(t, view, dist, F0, (1.0,), roll=xs)
                    Rm = R.from_rotvec(pose[3:]).as_matrix()
                    tip = np.array(pose[:3]) + Rm @ TIP
                    rear = np.array(pose[:3]) + Rm @ REAR
                    low = min(tip[2], rear[2]) - PLATE_Z
                    try:
                        q = call(b, 'robot_inverse_kinematics', pose={'position': pose[:3], 'orientation': pose[3:]},
                                 qNear=q0)['data']
                        q = [q[k] for k in JOINTS]
                    except Exception:
                        q = None
                    ok &= q is not None and low >= MIN_REAR and abs(q[5] - w3) <= w3_tol
                    rows.append((dx, pose, tip, rear, low, q))
                out.append((ok, dist, deg, xs, rows))
    return out


if __name__ == '__main__':
    tgt = [float(v) for v in sys.argv[1:4]]
    for ok, dist, deg, xs, rows in plan(tgt, URBridge()):
        lows = [round(r[4] * 100, 1) for r in rows]
        w3 = [None if r[5] is None else round(r[5][5], 2) for r in rows]
        if not ok:
            continue
        print(f'OK dist {dist:.2f} angle {deg} roll {xs:3d}: lowest point above plate [cm] {lows}, '
              f'flange z {[round(r[1][2], 3) for r in rows]}, wrist3 {w3}')
