"""Put a jumper (2 legs, adjacent holes, in a casing held by the custom fingers) into the board
where the model says two holes are; the operator then reports which holes it went into. That
answer is the ground truth for the board pose + grasp offset (explore mode).

One program: (1) the leg tips touch the hole-free strip between row a and the rail -> leg length
below the fingertip; (2) above the target pair, pecks (centre, rings of 0.7 / 1.4 mm) until the
legs drop > 1.5 mm below the surface; (3) seat straight down to the casing, capped at 6 N.
Low forces all the way (the legs bend): touch 2 N, seat 6 N. Pure position control, straight z.
The program ends holding still (no push). `--pull` pulls it out straight up afterwards.

  python -m breadboard.jumper_probe j10 j11          # plan: program + IK check, no motion
  python -m breadboard.jumper_probe j10 j11 --run
  python -m breadboard.jumper_probe --pull --run
"""
import argparse
import json
import re
import time
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

from breadboard.touch_locate import GRIPPER, tool_down
from urcore.urexec import URExec, call
from urcore.urlib import LIB
from urcore.urscript_lint import lint

ROOT = Path(__file__).resolve().parent.parent
APP, CLIENT = 'breadboard_explore', 'breadboard-explore'
J = ('base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3')
F_TOUCH, F_SEAT = 2.0, 6.0
SAFE = 0.020          # transit: fingertip this far above the top (low approach, user 10-27)
DROP = 0.0015         # legs this far below the surface = they found the holes
RINGS = (0.0007, 0.0014)


def hole(name, pose):
    """Base x, y (m) of a hole like 'j10' from board_pose.yaml (rows along -x)."""
    h = pose['holes_mm']
    col, row = name[0].lower(), int(name[1:])
    ys = {c: h['a_y'] - i * 2.54 for i, c in enumerate('abcde')}
    ys.update({c: h['f_y'] - i * 2.54 for i, c in enumerate('fghij')})
    return np.array([h['row1_x'] - (row - 1) * 2.54, ys[col]]) / 1000


def pattern():
    out = [(0.0, 0.0)]
    for r in RINGS:
        out += [(r * np.cos(a), r * np.sin(a)) for a in np.arange(8) * np.pi / 4]
    return out


def program(rv, strip_xy, target_xy, top):
    o = ', '.join(f'{v:.5f}' for v in rv)
    P = lambda x, y, z: f'p[{x:.5f}, {y:.5f}, {z}, {o}]'  # noqa: E731
    sx, sy = strip_xy
    tx, ty = target_xy
    s = ['textmsg("jumper_probe start")',
         # (1) leg length on the hole-free strip
         f'movel({P(sx, sy, f"{top + SAFE:.5f}")}, a=0.3, v=0.03)',
         f'h = guarded_axial([0, 0, -1], {SAFE + 0.004:.4f}, {F_TOUCH}, 0.005)',
         'if not h:', '  textmsg("ABORT no strip contact")', f'  movel({P(sx, sy, f"{top + SAFE:.5f}")}, a=0.3, v=0.03)',
         'else:',
         'zs = get_actual_tcp_pose()[2]',
         f'textmsg("strip ", zs)',
         'back_off([0, 0, -1], 0.003)',
         f'movel({P(sx, sy, f"zs + {SAFE}")}, a=0.3, v=0.05)',
         # (2) pecks above the target pair
         f'movel({P(tx, ty, f"zs + {SAFE}")}, a=0.3, v=0.05)',
         'found = False', 'zf = 0']
    for i, (dx, dy) in enumerate(pattern()):
        s += ['if not found:',
              f'  movel({P(tx + dx, ty + dy, f"zs + {DROP}")}, a=0.2, v=0.01)',
              f'  h = guarded_axial([0, 0, -1], 0.006, {F_TOUCH}, 0.003)',
              '  z = get_actual_tcp_pose()[2]',
              f'  if zs - z > {DROP}:',
              f'    found = True', '    zf = z', f'    textmsg("found {i} ", get_actual_tcp_pose())',
              '  else:',
              f'    textmsg("peck {i} ", z)',
              f'    movel({P(tx + dx, ty + dy, f"zs + {DROP}")}, a=0.2, v=0.01)',
              '  end',
              'end']
    # (3) seat down to the casing (fingertip ~ casing bottom at the top), capped at F_SEAT
    s += ['if found:',
          f'  d = seat_z(zf - {top:.5f} + 0.001, {F_SEAT}, 0.003)',
          '  textmsg("hold ", get_actual_tcp_pose())',
          'else:',
          '  textmsg("NOT FOUND")',
          f'  movel({P(tx, ty, f"zs + {SAFE}")}, a=0.3, v=0.02)',
          'end',
          'end',                     # strip contact
          'textmsg("jumper_probe done")']
    return '\n'.join(s)


def pull_program(rv, top):
    return '\n'.join(['textmsg("pull start")',
                      'pull_out([0, 0, -1], 20.0, 0.012, 0.004)',
                      'q = get_actual_tcp_pose()',
                      f'movel(p[q[0], q[1], {top + 0.045:.5f}, q[3], q[4], q[5]], a=0.3, v=0.02)',
                      'textmsg("pull done")'])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('holes', nargs='*', default=['j10', 'j11'])
    ap.add_argument('--run', action='store_true')
    ap.add_argument('--pull', action='store_true')
    a = ap.parse_args()
    g = yaml.safe_load(GRIPPER.read_text())
    pose = yaml.safe_load((ROOT / 'configs/breadboard/board_pose.yaml').read_text())
    top = pose['top_z_mm'] / 1000
    grip = {'tcp_tip': [0, 0, g['fingertip_z'], 0, 0, 0], 'collision_points': g['collision_points']}
    ex = URExec(APP, CLIENT, grip, tcp_key='tcp_tip')
    q0 = [call(ex.b, 'robot_get_joint_positions')['data'][k] for k in J]
    if a.pull:
        msgs, _ = ex.run(pull_program(None, top), timeout=120) if a.run else (['plan only'], None)
        print('\n'.join(msgs))
        return
    # legs side by side along the finger face = flange y: turn flange y along base x (the rows)
    best = None
    for yaw in (np.pi / 2, -np.pi / 2):
        rv = tool_down(yaw)
        fl = R.from_rotvec(rv).as_matrix() @ -np.array(grip['tcp_tip'][:3]) + [-0.40, -0.04, top + 0.04]
        ik = ex._in_active_tcp(list(fl) + rv, q0)
        r = call(ex.b, 'robot_inverse_kinematics', pose={'position': ik[:3], 'orientation': ik[3:]}, qNear=q0)
        if 'data' not in r:         # the host IK times out (status gap) where it finds no solution
            continue
        d = abs(float(r['data']['wrist3']) - q0[5])
        if best is None or d < best[0]:
            best = (d, yaw)
    yaw = best[1]
    rv = tool_down(yaw)
    pair = [hole(h, pose) for h in a.holes]
    target = np.mean(pair, axis=0)
    h = pose['holes_mm']
    strip = (target[0], (h['a_y'] + 6.43 / 2) / 1000)      # between row a and the inner rail (6.43 mm)
    prog = program(rv, strip, target, top)
    problems = lint(LIB + prog)
    print(prog, '\n---\nlint:', problems or 'ok', '| yaw deg', round(np.degrees(yaw)), '| target mm',
          np.round(target * 1000, 2).tolist(), '| strip mm', np.round(np.array(strip) * 1000, 2).tolist())
    bad = []
    for x, y, z in [(*strip, top + 0.04), (*strip, top), (*target, top + SAFE), (*target, top)]:
        fl = R.from_rotvec(rv).as_matrix() @ -np.array(grip['tcp_tip'][:3]) + [x, y, z]
        ik = ex._in_active_tcp(list(fl) + rv, q0)       # the bridge IK works in the active TCP
        try:
            r = call(ex.b, 'robot_inverse_kinematics', pose={'position': ik[:3], 'orientation': ik[3:]}, qNear=q0)['data']
            q = [float(r[k]) for k in J]
            if abs(q[5] - q0[5]) > 0.8 or abs(q[2]) < 0.5:
                bad.append(((x, y, z), 'w3/elbow', np.round(q, 2).tolist()))
        except Exception as e:  # noqa: BLE001
            bad.append(((x, y, z), str(e)[:100]))
    print('IK:', 'all reachable' if not bad else bad)
    if not a.run or problems or bad:
        return
    start = R.from_rotvec(rv).as_matrix() @ -np.array(grip['tcp_tip'][:3]) + [strip[0], strip[1], top + 0.04]
    ex.move_checked(list(start) + rv, v=0.2, floor=top, min_clear=0.02)   # then down to 2 cm in the program
    t0 = time.time()
    msgs, _ = ex.run(prog, timeout=300)
    res = {'stamp': time.strftime('%Y-%m-%dT%H:%M:%S'), 'secs': round(time.time() - t0, 1), 'holes': a.holes,
           'target_mm': np.round(target * 1000, 2).tolist(), 'yaw_deg': round(float(np.degrees(yaw))), 'msgs': msgs}
    zs = next((float(m.split()[1]) for m in msgs if m.startswith('strip ')), None)
    if zs is not None:
        res['leg_length_mm'] = round((zs - top) * 1000, 2)
    print(json.dumps(res, indent=1))
    d = ROOT / 'data' / 'runs' / time.strftime('jumper_probe_%Y%m%d-%H%M%S')
    d.mkdir(parents=True, exist_ok=True)
    (d / 'result.json').write_text(json.dumps(res, indent=1))
    print('saved', d)


if __name__ == '__main__':
    main()
