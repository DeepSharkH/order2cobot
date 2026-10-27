"""Checked move to a flange pose, then capture and run the thorough board detection on both cameras.
Usage: python -m porttest.look x y z [rx ry rz]   (orientation defaults to tool down, [0, pi, 0])"""
import json
import sys
from pathlib import Path

import cv2

from porttest.fcam_calib import corners
from porttest.port_survey import capture_png
from porttest.urexec import URExec

ROOT = Path(__file__).resolve().parent.parent


def look(ex, pose, w3_ref=None):
    q = ex.move_checked(pose, w3_ref=w3_ref)
    img, msg = capture_png()
    sess = sorted((ROOT / 'data/calib').iterdir())[-1]
    s = json.loads((sess / 'samples.json').read_text())[-1]
    g = corners(img) is not None
    f = corners(cv2.imread(str(sess / s['fixed_image']))) is not None
    return {'sample': s['id'], 'q': [round(v, 3) for v in q], 'gripper': g, 'fixed': f}


if __name__ == '__main__':
    a = [float(v) for v in sys.argv[1:]]
    pose = a[:3] + (a[3:6] if len(a) >= 6 else [0.0, 3.14159265, 0.0])
    print(look(URExec(), pose))
