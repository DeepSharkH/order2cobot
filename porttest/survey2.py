"""Port survey with a rolled camera: views at the current pose and shifted +-dx along base x.

The port row is detected in the image rotated by `angle` (it is tilted by the camera roll);
the detection window follows the row as the camera shifts. Writes data/ports.yaml.
Usage: python -m porttest.survey2 angle cu cv row_du row_dv px_per_m
"""
import sys
import time
from pathlib import Path

import numpy as np
import yaml

from porttest import ports
from porttest.geometry import urpose
from porttest.locate import camera, flange
from porttest.port_survey import capture_png
from porttest.urexec import URExec

ROOT = Path(__file__).resolve().parent.parent


def main(angle, cu, cv, du, dv, px_per_m, shifts=(0.0, 0.05, -0.05)):
    ex = URExec()
    p0, _ = ex.pose()
    row = np.array([du, dv]) / np.hypot(du, dv)          # image direction of base +x along the row
    views = []
    for dx in shifts:
        t = list(p0)
        t[0] += dx
        ex.run(f'movel({urpose(t)}, a=0.2, v=0.03)\nsleep(0.5)', timeout=90)
        time.sleep(1.0)
        img, _ = capture_png()
        c = np.array([cu, cv]) - row * dx * px_per_m     # camera +x -> the scene slides toward -x
        det = ports.detect_rotated(img, angle, tuple(c), half=(230, 60),
                                   debug=str(ROOT / f'data/survey2_{dx:+.2f}.png'))
        print(f'shift {dx:+.2f}: {len(det)} ports', [[round(v) for v in d['centre']] for d in det])
        if len(det) == 8:
            views.append((camera(flange(ex.b)), det))
    ex.run(f'movel({urpose(p0)}, a=0.2, v=0.03)', timeout=90)
    out = {}
    for key in ('centre', 'bottom'):
        P, rms = ports.triangulate(views, key)
        out[key] = P.tolist()
        print(f'{key}: ray rms [mm] {np.round(rms, 2)}')
        for i, p in enumerate(P):
            print(f'  port {i + 1}: {np.round(p * 1000, 1)} mm')
    C = np.array(out['centre'])
    pitch = np.linalg.norm(np.diff(C, axis=0), axis=1)
    line = C[-1] - C[0]
    print(f'pitch [mm] {np.round(pitch * 1000, 2)} mean {pitch.mean() * 1000:.2f}; '
          f'yaw {np.degrees(np.arctan2(line[1], line[0])):.2f} deg')
    out['views'] = len(views)
    (ROOT / 'data/ports.yaml').write_text(yaml.safe_dump(out, sort_keys=False))
    print('saved data/ports.yaml')


if __name__ == '__main__':
    main(*[float(a) for a in sys.argv[1:7]])
