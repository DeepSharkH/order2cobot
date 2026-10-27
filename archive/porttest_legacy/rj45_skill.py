"""Steps of the rj45-insert skill: tip-pose moves and one-program searches.

Usage: python -m porttest.rj45_skill <cmd> [args]   (see the functions below)
Tip poses are base-frame plug-tip positions in the insertion orientation (plug along base +y).
"""
import json
import sys

import numpy as np

from porttest.go_view import W3_REF
from porttest.urexec import GRIP, URExec

ROT = [-2.2214, 2.2214, 0.0]         # tool down, turned +90 deg about z: flange -x = base +y
N, A, B = [0, 1, 0], [1, 0, 0], [0, 0, 1]   # face normal into the switch, along the row, up


def flange(tip):
    """Flange pose for a tip position (ROT maps flange (x, y, z) to base (-y, -x, -z))."""
    t = GRIP['tcp_tip']
    return [tip[0] + t[1], tip[1] + t[0], tip[2] + t[2]] + ROT


def v(x):
    return '[' + ', '.join(f'{c:.5f}' for c in x) + ']'


def tp(x):
    return 'p' + v(list(x) + ROT)


def show(msgs):
    for m in msgs:
        print('  ', m)


def goto(ex, tip, high=0.10):
    """Checked joint move above the tip target, then a straight vertical movel down (tip TCP)."""
    above = [tip[0], tip[1], tip[2] + high]
    ex.move_checked(flange(above), w3_ref=W3_REF, v=0.2)
    msgs, pose = ex.run(f'movel({tp(tip)}, a=0.2, v=0.03)\ntextmsg("at ", get_actual_tcp_pose())', timeout=60)
    show(msgs)
    return msgs


def up(ex, dy=0.01, dz=0.10):
    """Back away from the face and lift (recovery path: straight back, then up)."""
    s = (f'q = get_actual_tcp_pose()\nmovel(pose_add(q, p[0, {-dy}, 0, 0, 0, 0]), a=0.1, v=0.01)\n'
         f'q = get_actual_tcp_pose()\nmovel(pose_add(q, p[0, 0, {dz}, 0, 0, 0]), a=0.2, v=0.03)\n'
         'textmsg("up ", get_actual_tcp_pose())')
    msgs, _ = ex.run(s, timeout=60)
    show(msgs)


def run(ex, body, timeout=120):
    print(body)
    msgs, pose = ex.run(body, timeout=timeout)
    show(msgs)
    return msgs


if __name__ == '__main__':
    ex = URExec()
    cmd, args = sys.argv[1], [float(a) for a in sys.argv[2:]]
    if cmd == 'goto':
        goto(ex, args[:3], *(args[3:4] or [0.10]))
    elif cmd == 'up':
        up(ex, *args)
    elif cmd == 'run':        # URScript body from stdin
        run(ex, sys.stdin.read(), timeout=args[0] if args else 120)
    print(json.dumps({'flange': np.round(ex.pose()[0], 5).tolist()}))
