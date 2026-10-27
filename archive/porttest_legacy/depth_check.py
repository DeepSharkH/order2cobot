"""Compare the gripper camera's depth cloud with the board plane known from the hand-eye solve."""
import asyncio
import sys

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

from porttest import dash
from porttest.square_from_depth import _grab
from porttest.urexec import URBridge, call


def main():
    he = yaml.safe_load(open('data/handeye_gripper.yaml'))
    X, Y = np.array(he['T_flange_gcam']), np.array(he['T_base_board'])
    p = call(URBridge(), 'robot_get_pose')['data']['pose']
    F = np.eye(4)
    F[:3, :3] = R.from_rotvec(p['orientation']).as_matrix()
    F[:3, 3] = p['position']
    C = F @ X                                      # camera in base
    Cinv = np.linalg.inv(C)
    n = Cinv[:3, :3] @ Y[:3, 2]                    # board plane in the camera frame
    o = Cinv[:3, :3] @ Y[:3, 3] + Cinv[:3, 3]
    color, cloud = asyncio.run(_grab())
    rays = cloud / cloud[:, 2:3]
    z_exp = (o @ n) / (rays @ n)                   # expected depth if the point lies on the board plane
    near = np.abs(cloud[:, 2] / z_exp - np.median(cloud[:, 2] / z_exp)) < 0.1
    ratio = cloud[near, 2] / z_exp[near]
    print(f'camera {np.round(C[:3, 3], 3)}; board plane at {np.median(z_exp):.3f} m along the axis')
    print(f'measured/expected depth: median {np.median(ratio):.3f}, IQR {np.percentile(ratio, 25):.3f}-{np.percentile(ratio, 75):.3f} '
          f'({near.sum()} of {len(cloud)} points)')


if __name__ == '__main__':
    sys.exit(main())
