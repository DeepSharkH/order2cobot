"""Gripper-camera geometry with the hand-eye calibration: pixel -> ray / point in robot base.

The camera pose comes from the current flange pose (bridge) and T_flange_gcam from
configs/calibration/scene.yaml (multi-view solve, dashboard/scene_solve.py; the camera
sits on the side bracket, optical axis ~24 deg off the tool axis). Intrinsics are the
factory ones (no distortion).
"""
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

from porttest.urexec import URBridge, call

ROOT = Path(__file__).resolve().parent.parent
SCENE = yaml.safe_load((ROOT / 'configs/calibration/scene.yaml').read_text())
CFG = yaml.safe_load((ROOT / 'configs/calib_dashboard.yaml').read_text())
X = np.array(SCENE['T_flange_gcam'])
K = np.array([[610.1984252929688, 0.0, 635.2977905273438], [0.0, 610.2023315429688, 364.4780578613281], [0.0, 0.0, 1.0]])
SQUARE = CFG['board']['square']


def flange(b=None):
    p = call(b or URBridge(), 'robot_get_pose')['data']['pose']
    F = np.eye(4)
    F[:3, :3] = R.from_rotvec(p['orientation']).as_matrix()
    F[:3, 3] = p['position']
    return F


def camera(F):
    return F @ X


def ray(C, u, v):
    """Origin and unit direction (base) of the ray through pixel (u, v) at 1280x720."""
    d = C[:3, :3] @ np.array([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], 1.0])
    return C[:3, 3], d / np.linalg.norm(d)


def on_z(C, u, v, z):
    o, d = ray(C, u, v)
    return o + d * (z - o[2]) / d[2]

