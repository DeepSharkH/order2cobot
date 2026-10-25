"""Find and seat port 1 from the camera survey (data/ports.yaml) and the measured grip.

Grip (2026-10-25 pm, regripped by the operator so the gripper cam looks at the insertion
from the side): the plug points along flange -x. Measured in the gripper-cam image: tip
41.7 mm out along -x, ~5 mm (±4) to flange -y; plug centre line ~193 mm under the flange
(breadboard touch). For insertion the tool is turned +90° about the vertical (ROT), so the
plug points base +y and the tip sits at flange + TIP in base. The port-1 centre from the
survey gives the flange pose with the tip on the face; a wide peck grid (x ±4 mm, z ±2 mm,
10 mm standoff) finds the opening, force mode seats it, the dashboard checks the link.
Usage: python -m porttest.find_port1 [go]
"""
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import yaml

from porttest.go_view import W3_REF
from porttest.urexec import URExec

ROOT = Path(__file__).resolve().parent.parent
TIP = np.array([0.0053, 0.0417, -0.193])   # plug tip from the flange, base frame, insertion orientation
ROT = [-2.2214, 2.2214, 0.0]                # tool down, turned +90 deg about z: flange -x = base +y


def qa(port, phase):
    out = subprocess.run(['curl', '-s', '-m', '30', '-X', 'POST', f'http://localhost:8090/api/qa/{port}/{phase}'],
                         capture_output=True, text=True).stdout
    try:
        return json.loads(out)
    except ValueError:
        return {'ok': False, 'reason': out[:200]}


def main(go=False):
    C = np.array(yaml.safe_load((ROOT / 'data/ports.yaml').read_text())['centre'])
    x, yf, z = C[0]
    c = list(np.array([x, yf, z]) - TIP)                   # flange with the tip on the face at port 1
    high = [c[0], c[1] - 0.03, c[2] + 0.10]                 # in front of and above it (plug >= 10 cm up)
    print('port 1 centre [mm]', np.round(C[0] * 1000, 1))
    print('face pose (flange)', np.round(c, 4).tolist(), '| approach', np.round(high, 4).tolist())
    if not go:
        return
    ex = URExec()
    ex.move_checked(high + ROT, w3_ref=W3_REF, v=0.2)
    rot = ', '.join(f'{v:.6f}' for v in ROT)
    s = f'''movel(p[{c[0]:.5f}, {c[1] - 0.015:.5f}, {c[2]:.5f}, {rot}], a=0.2, v=0.02)
if grid_search(p[{c[0]:.5f}, {c[1]:.5f}, {c[2]:.5f}, {rot}], [1, 0, 0], [0, 1, 0], 17, 0.0005, -0.002, 11, 0.0004, 0.010, 0.008, 20.0):
  d = insert_fm(25.0, 0.012, 6.0)
  textmsg(str_cat("seated ", d))
  textmsg("seatpose ", get_actual_tcp_pose())
end'''
    msgs, pose = ex.run(s, timeout=1500)
    print([m for m in msgs if not m.startswith('col')])
    if not any(m.startswith('found') for m in msgs):
        print('port 1 not found')
        return
    res = qa(1, 'insert')
    print('link check:', res.get('ok'), res.get('reason'))


if __name__ == '__main__':
    main('go' in sys.argv)
