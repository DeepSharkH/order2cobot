"""Triangulate the 8 switch ports from several gripper-camera views (camera shifted along base x).

Usage: python -m porttest.port_survey [shift_m ...]   (default 0 0.05 -0.05)
Writes data/ports.yaml with the 3D centres / bottom-centres in robot base.
"""
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

from porttest import dash, ports
from porttest.geometry import urpose
from porttest.locate import camera, flange
from porttest.urexec import URExec

ROOT = Path(__file__).resolve().parent.parent
ROI = (300, 300, 1050, 420)


def capture_png(cam=1):
    """Latest full-resolution colour frame from the dashboard (GET /api/frame/{cam})."""
    import urllib.request
    png = urllib.request.urlopen(f'http://localhost:8090/api/frame/{cam}', timeout=10).read()
    return cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_COLOR), 'frame'


def main(shifts=(0.0, 0.05, -0.05)):
    ex = URExec()
    p0, _ = ex.pose()
    views = []
    for dx in shifts:
        target = list(p0)
        target[0] += dx
        ex.run(f'movel({urpose(target)}, a=0.2, v=0.04)\nsleep(0.5)', timeout=60)
        time.sleep(1.0)
        img, msg = capture_png()
        F = flange(ex.b)
        det = ports.detect(img, ROI, debug=str(ROOT / f'data/ports_view_{dx:+.2f}.png'))
        print(f'shift {dx:+.2f}: {len(det)} ports, u {[round(d["centre"][0]) for d in det]}')
        if len(det) == 8:
            views.append((camera(F), det))
    ex.run(f'movel({urpose(p0)}, a=0.2, v=0.04)', timeout=60)
    if len(views) < 2:
        raise SystemExit('need >= 2 views with all 8 ports')
    out = {}
    for key in ('centre', 'bottom'):
        P, rms = ports.triangulate(views, key)
        out[key] = P.tolist()
        print(f'\n{key}: ray rms per port [mm] {np.round(rms, 2)}')
        for i, p in enumerate(P):
            print(f'  port {i + 1}: {np.round(p * 1000, 1)} mm')
    C = np.array(out['centre'])
    d = np.diff(C, axis=0)
    pitch = np.linalg.norm(d, axis=1)
    line = C[-1] - C[0]
    print(f'\npitch [mm] {np.round(pitch * 1000, 2)}  mean {pitch.mean() * 1000:.2f}')
    print(f'row direction {np.round(line / np.linalg.norm(line), 4)}  '
          f'yaw {np.degrees(np.arctan2(line[1], line[0])):.2f} deg, slope in z {np.degrees(np.arcsin(line[2] / np.linalg.norm(line))):.2f} deg')
    out['views'] = len(views)
    (ROOT / 'data/ports.yaml').write_text(yaml.safe_dump(out, sort_keys=False))
    print('saved data/ports.yaml')


if __name__ == '__main__':
    main(tuple(float(a) for a in sys.argv[1:]) or (0.0, 0.05, -0.05))
