"""Flange pose that puts the gripper camera at `dist` from a target, looking along `view`.

Usage: python -m porttest.view_pose tx ty tz vx vy vz dist [go]
Prints the flange pose (and with 'go' moves there with a joint move).
"""
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

from urcore.urexec import urpose
from porttest.locate import X, camera, flange
from porttest.urexec import URExec, call


def flange_for_view(target, view, dist, F_now, xsigns=(1.0, -1.0), roll=0.0):
    z = np.asarray(view, float)
    z /= np.linalg.norm(z)
    C_now = camera(F_now)
    best = None
    for xs in xsigns:                            # image x along +-base x; keep the one closest to now
        x = np.array([xs, 0.0, 0.0])
        x = x - z * (x @ z)
        x /= np.linalg.norm(x)
        x = R.from_rotvec(z * np.radians(roll)).apply(x)     # roll the image about the view axis
        y = np.cross(z, x)
        C = np.eye(4)
        C[:3, :3] = np.c_[x, y, z]
        C[:3, 3] = np.asarray(target) - z * dist
        ang = np.degrees(R.from_matrix(C[:3, :3] @ C_now[:3, :3].T).magnitude())
        if best is None or ang < best[0]:
            best = (ang, C)
    F = best[1] @ np.linalg.inv(X)
    return list(F[:3, 3]) + list(R.from_matrix(F[:3, :3]).as_rotvec()), best[0]


if __name__ == '__main__':
    a = [float(v) for v in sys.argv[1:8]]
    ex = URExec()
    xs = (1.0,) if '+x' in sys.argv else (-1.0,) if '-x' in sys.argv else (1.0, -1.0)
    pose, turn = flange_for_view(a[:3], a[3:6], a[6], flange(ex.b), xs)
    q = call(ex.b, 'robot_inverse_kinematics', pose={'position': pose[:3], 'orientation': pose[3:]},
             qNear=list(ex.pose()[1].values())[:6])['data']
    print('IK', {k: round(v, 3) for k, v in q.items() if isinstance(v, float)})
    print('flange pose', np.round(pose, 4), f'(camera turns {turn:.0f} deg)')
    if 'go' in sys.argv[8:]:
        print(ex.run(f'movej(get_inverse_kin({urpose(pose)}, qnear=get_actual_joint_positions()), a=0.6, v=0.4)',
                     timeout=90))
