"""UR3e forward kinematics (standard DH) and small SE(3) helpers.

Two parameter sets are used:
  * UR3E_NOMINAL_DH: the published UR3e DH table (universal-robots.com, "DH
    parameters for calculations of kinematics and dynamics"), identical to
    Universal_Robots_ROS2_Description/config/ur3e/default_kinematics.yaml.
  * the controller's factory-calibrated table, read from the robot through
    the AI Agent Bridge (robot_get_kinematic_info). Rows are
    [theta_offset, a, d, alpha] in standard DH; joint 2-4 rows carry large
    compensating d/theta values, which is normal for UR's calibration of the
    three parallel axes.

Joint order everywhere: base, shoulder, elbow, wrist1, wrist2, wrist3.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

JOINT_NAMES = ['base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3']

# rows: theta_offset [rad], a [m], d [m], alpha [rad]
UR3E_NOMINAL_DH = np.array([
    [0.0, 0.0, 0.15185, np.pi / 2],
    [0.0, -0.24355, 0.0, 0.0],
    [0.0, -0.21320, 0.0, 0.0],
    [0.0, 0.0, 0.13105, np.pi / 2],
    [0.0, 0.0, 0.08535, -np.pi / 2],
    [0.0, 0.0, 0.09210, 0.0],
])


def dh_from_bridge(data):
    """robot_get_kinematic_info data -> (6, 4) array [theta, a, d, alpha]."""
    return np.array([[data[str(i)][k] for k in ('DHTheta', 'DHa', 'DHd', 'DHAlpha')]
                     for i in range(6)], dtype=float)


def dh_link(theta, a, d, alpha):
    ct, st, ca, sa = np.cos(theta), np.sin(theta), np.cos(alpha), np.sin(alpha)
    return np.array([[ct, -st * ca, st * sa, a * ct],
                     [st, ct * ca, -ct * sa, a * st],
                     [0.0, sa, ca, d],
                     [0.0, 0.0, 0.0, 1.0]])


def fk_frames(q, dh):
    """All DH frames in the base frame: [T_base_0 (=I), T_base_1, ..., T_base_6]."""
    out = [np.eye(4)]
    T = np.eye(4)
    for i in range(6):
        th, a, d, al = dh[i]
        T = T @ dh_link(q[i] + th, a, d, al)
        out.append(T)
    return out


def fk(q, dh):
    """Flange pose T_base_flange."""
    return fk_frames(q, dh)[-1]


def pose_to_T(pos, rotvec):
    T = np.eye(4)
    T[:3, :3] = R.from_rotvec(rotvec).as_matrix()
    T[:3, 3] = pos
    return T


def T_to_pose(T):
    return T[:3, 3].copy(), R.from_matrix(T[:3, :3]).as_rotvec()


def vec_to_T(v):
    """[rx, ry, rz, x, y, z] -> 4x4."""
    return pose_to_T(v[3:6], v[0:3])


def T_to_vec(T):
    p, r = T_to_pose(T)
    return np.r_[r, p]


def inv(T):
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


def pose_error(Ta, Tb):
    """(translation error [m], rotation error [rad]) between two poses."""
    dT = inv(Ta) @ Tb
    return float(np.linalg.norm(dT[:3, 3])), float(np.linalg.norm(R.from_matrix(dT[:3, :3]).as_rotvec()))


def mean_T(Ts):
    """Average of rigid transforms (chordal rotation mean, arithmetic translation)."""
    Ts = list(Ts)
    T = np.eye(4)
    T[:3, :3] = R.from_matrix([t[:3, :3] for t in Ts]).mean().as_matrix()
    T[:3, 3] = np.mean([t[:3, 3] for t in Ts], axis=0)
    return T
