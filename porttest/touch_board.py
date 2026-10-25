"""Plug height below the flange: guarded descent of the gripped plug onto the checkerboard.

The board top is world z = 0 (configs/calibration/scene.yaml gives the base pose in the
world), so the flange z at contact gives the plug's lowest point below the flange.
Moves (checked) 5 cm above the board centre with the tool down, pecks down at 5 mm/s
until 15 N reaction, records, backs off 2 cm. Usage: python -m porttest.touch_board [go]
"""
import sys
from pathlib import Path

import numpy as np
import yaml

from dashboard import kinematics as kin
from porttest.go_view import W3_REF
from porttest.urexec import URExec

ROOT = Path(__file__).resolve().parent.parent
ROT = [0.0, np.pi, 0.0]             # tool down, as in qa_run (plug along base +y)
PLUG_BELOW = 0.190                  # rough lowest plug point under the flange (session 4: centre 185.4 mm)


def board_in_base():
    b = yaml.safe_load((ROOT / 'configs/calibration/scene.yaml').read_text())['base']
    c, s = np.cos(np.radians(b['yaw_deg'])), np.sin(np.radians(b['yaw_deg']))
    T = np.eye(4)
    T[:2, :2] = [[c, -s], [s, c]]
    T[:3, 3] = b['x'], b['y'], b.get('z', 0.0)
    return kin.inv(T)               # T_base_world: world origin = board centre, top surface


def main(go=False):
    W = board_in_base()
    top = W[:3, 3]
    print('board centre / top in base [mm]', np.round(top * 1000, 1))
    start = [top[0], top[1], top[2] + PLUG_BELOW + 0.05] + ROT
    print('approach flange pose', np.round(start, 4).tolist())
    if not go:
        return
    ex = URExec()
    ex.move_checked(start, w3_ref=W3_REF, min_clear=0.04, v=0.2)
    msgs, pose = ex.run('d = peck([0, 0, -1], 0.07, 15.0, 0.005)\n'
                        'textmsg(str_cat("touch ", d))\n'
                        'textmsg(str_cat("tcp ", get_actual_tcp_pose()))\n'
                        'p = get_actual_tcp_pose()\nmovel(pose_add(p, p[0, 0, 0.02, 0, 0, 0]), a=0.2, v=0.01)',
                        timeout=180)
    print([m for m in msgs if m.startswith(('touch', 'tcp'))])
    tcp = next(m for m in msgs if m.startswith('tcp '))
    z = float(tcp.split('[')[1].split(',')[2])
    print(f'contact flange z {z * 1000:.1f} mm -> plug lowest point {(z - top[2]) * 1000:.1f} mm below the flange')


if __name__ == '__main__':
    main('go' in sys.argv)
