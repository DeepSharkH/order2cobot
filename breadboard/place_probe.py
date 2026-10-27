"""Place a 2-leg casing (jumper, LED, ... from casings.json) into two holes where the model says
they are; the operator reports the holes it went into (explore mode, ground truth).

One program per placement:
  (0) a part left in the board by the last run is pulled out first (state file);
  (1) the leg tips touch the hole-free strip on the target's half -> leg length below the fingertip;
  (2) low approach (2 cm) to the pair + the last found offset; pecks (centre, rings 0.7 / 1.4 mm)
      at 2 N until the legs drop > 1.5 mm below the surface;
  (3) seat straight down, capped at 6 N (legs bend), release the push;
  (4) lift 5 mm (legs still ~1.5 mm in the holes) so the operator can read them; hold.
The found offset (hole found - model target) is saved: it is the next search's centre and, over
several rows, tells a grasp offset (constant) from a pitch error (grows with the row).

  python -m breadboard.place_probe j3 j4                       # plan, no motion
  python -m breadboard.place_probe j3 j4 --part jumper_s1 --run
  python -m breadboard.place_probe --pull --run                # take the part out, lift
  python -m breadboard.place_probe --confirm j3 j4             # operator: it is in these holes
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

from breadboard import grasp_calib
from breadboard.touch_locate import GRIPPER, tool_down
from urcore.urexec import URExec, call
from urcore.urlib import LIB
from urcore.urscript_lint import lint

ROOT = Path(__file__).resolve().parent.parent
APP, CLIENT = 'breadboard_explore', 'breadboard-explore'
STATE = ROOT / 'data' / 'breadboard_state.json'
CASINGS = ROOT / 'assets/models/component_casings/casings.json'
J = ('base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3')
PITCH = 0.00254
F_TOUCH, F_SEAT, F_PULL = 2.0, 6.0, 20.0
SAFE = 0.020          # low approach: fingertip this far above the top (user 10-27)
DROP = 0.0015         # legs this far below the surface = they found the holes
LIFT = 0.005          # after seating, legs stay ~1.5 mm in the holes for reading
RINGS = (0.0007, 0.0014)
X_MIN = -0.460        # reach: further out the elbow straightens (singularity)
CALIB_Y = -0.045      # grasp calibration at the row-1 end, near half (the white cube sits on the a-side rail)
BOARD_H = 0.0085      # board top above the plate (board.yaml)
LEGS_RANGE_MM = (5.6, 8.5)   # leg tips below the fingertips (strip; 6.2-6.6 with the usual grip, 7.8 with a 1 mm deeper
                              # one, 10-27); bent legs read 3.9, legs pulled out of the casing 13.1
GRASP_MAX = 0.006     # grasp calibration beyond this = suspect bent legs (-4.56 placed fine on 10-27; the bent case read -4.34 with 3.9 mm legs: leg length is the better sign)


def casing(name):
    return next(c for c in json.loads(CASINGS.read_text())['casings'] if c['casing'] == name)


def hole(name, pose):
    """Base x, y (m) of a hole like 'j10' from board_pose.yaml (rows along -x)."""
    h = pose['holes_mm']
    col, row = name[0].lower(), int(name[1:])
    ys = {c: h['a_y'] - i * 2.54 for i, c in enumerate('abcde')}
    ys.update({c: h['f_y'] - i * 2.54 for i, c in enumerate('fghij')})
    return np.array([h['row1_x'] - (row - 1) * 2.54, ys[col]]) / 1000


def strip_y(col, pose):
    """The hole-free strip between the outer hole row and the rail, on the target's half."""
    h = pose['holes_mm']
    return (h['a_y'] + 6.43 / 2) / 1000 if col in 'abcde' else (h['j_y'] - 6.43 / 2) / 1000


def pattern(c):
    out = [tuple(c)]
    for r in RINGS:
        out += [(c[0] + r * np.cos(a), c[1] + r * np.sin(a)) for a in np.arange(8) * np.pi / 4]
    return out


def load_state():
    return json.loads(STATE.read_text()) if STATE.exists() else {}


def save_state(st):
    STATE.parent.mkdir(parents=True, exist_ok=True)
    STATE.write_text(json.dumps(st, indent=1))


def pull_lines(top):
    return ['textmsg("pull start")', f'pull_out([0, 0, -1], {F_PULL}, 0.010, 0.004)',
            'q = get_actual_tcp_pose()',
            f'movel(p[q[0], q[1], {top + SAFE:.5f}, q[3], q[4], q[5]], a=0.3, v=0.02)']


def program(rv, strip_xy, target, offset, top, pull_first, drop=DROP):
    o = ', '.join(f'{v:.5f}' for v in rv)
    P = lambda x, y, z: f'p[{x:.5f}, {y:.5f}, {z}, {o}]'  # noqa: E731
    sx, sy = strip_xy
    tx, ty = target
    s = ['textmsg("place_probe start")'] + (pull_lines(top) if pull_first else [])
    s += [f'movel({P(sx, sy, f"{top + SAFE:.5f}")}, a=0.3, v=0.03)',
          f'h = guarded_axial([0, 0, -1], {SAFE + 0.004:.4f}, {F_TOUCH}, 0.005)',
          'if not h:', '  textmsg("ABORT no strip contact")', f'  movel({P(sx, sy, f"{top + SAFE:.5f}")}, a=0.3, v=0.03)',
          'else:',
          'zs = get_actual_tcp_pose()[2]', 'textmsg("strip ", zs)',
          'back_off([0, 0, -1], 0.003)',
          f'movel({P(sx, sy, f"zs + {SAFE}")}, a=0.3, v=0.03)',
          f'movel({P(tx + offset[0], ty + offset[1], f"zs + {SAFE}")}, a=0.3, v=0.03)',
          'found = False', 'zf = 0']
    for i, (x, y) in enumerate(pattern((tx + offset[0], ty + offset[1]))):
        s += ['if not found:',
              f'  movel({P(x, y, f"zs + {DROP}")}, a=0.2, v=0.01)',
              f'  h = guarded_axial([0, 0, -1], 0.006, {F_TOUCH}, 0.003)',
              '  z = get_actual_tcp_pose()[2]',
              f'  if zs - z > {drop}:',
              '    found = True', '    zf = z', f'    textmsg("found {i} ", get_actual_tcp_pose())',
              '  else:', f'    textmsg("peck {i} ", z)', f'    movel({P(x, y, f"zs + {DROP}")}, a=0.2, v=0.01)', '  end',
              'end']
    s += ['if found:',
          f'  d = seat_z(zf - {top:.5f} + 0.001, {F_SEAT}, 0.003)',
          '  textmsg("seated ", get_actual_tcp_pose())',
          f'  pull_out([0, 0, -1], {F_PULL}, {LIFT}, 0.003)',
          '  textmsg("hold ", get_actual_tcp_pose())',
          'else:',
          '  textmsg("NOT FOUND")', f'  movel({P(tx, ty, f"zs + {SAFE}")}, a=0.3, v=0.02)',
          'end',
          'end',                     # strip contact
          'textmsg("place_probe done")']
    return '\n'.join(s)


def snapshot(ex):
    """Fixed cam + gripper cam frames from the dashboard (PNG bytes), for the run record."""
    import urllib.request
    out = {}
    for cam, name in ((0, 'grasp_fixed.png'), (1, 'grasp_gripper.png')):
        try:
            out[name] = urllib.request.urlopen(f'http://127.0.0.1:8090/api/frame/{cam}', timeout=5).read()
        except OSError:
            pass
    return out


def grip_record(ex, note):
    """How the part is held: the bridge's Gripper Close settings (no position / force readout exists)
    and the operator's note. The Hand-E is closed on the pendant for now."""
    rec = {'note': note}
    try:
        sk = next(s for s in call(ex.b, 'smartskill_list')['data'] if s.get('type') == 'robotiq-gripper-close-smart-skill')
        rec['close_skill'] = {k: sk['parameters']['dao'].get(k) for k in ('force', 'speed', 'targetPosition')}
    except Exception as e:  # noqa: BLE001
        rec['close_skill'] = f'unavailable: {str(e)[:80]}'
    return rec


def ik_ok(ex, grip, rv, q0, pts, w3_tol=0.8):
    bad = []
    for x, y, z in pts:
        fl = R.from_rotvec(rv).as_matrix() @ -np.array(grip['tcp_tip'][:3]) + [x, y, z]
        ik = ex._in_active_tcp(list(fl) + rv, q0)       # the bridge IK works in the active TCP
        r = call(ex.b, 'robot_inverse_kinematics', pose={'position': ik[:3], 'orientation': ik[3:]}, qNear=q0)
        if 'data' not in r:                           # host IK times out (gap) where there is no solution
            bad.append(((x, y, z), 'no IK'))
            continue
        q = [float(r['data'][k]) for k in J]
        if abs(q[5] - q0[5]) > w3_tol or abs(q[2]) < 0.5:
            bad.append(((x, y, z), 'w3/elbow', np.round(q, 2).tolist()))
    return bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('holes', nargs='*')
    ap.add_argument('--part', default='jumper_s1')
    ap.add_argument('--run', action='store_true')
    ap.add_argument('--pull', action='store_true')
    ap.add_argument('--confirm', action='store_true', help='record the holes the operator read')
    ap.add_argument('--calib', action='store_true', help='grasp calibration (casing against the row-1 end) first')
    ap.add_argument('--grasp-dx', type=float, help='insertion only: the grasp (legs - fingertip along x, mm) of an earlier --calib with the same grip')
    ap.add_argument('--shift', type=float, nargs=2, default=[0.0, 0.0], metavar=('DX', 'DY'),
                    help='search centre shift for a retry, mm (from the last pecks\' drops; logged, never learned as is)')
    ap.add_argument('--drop', type=float, default=DROP * 1000, help='legs this far below the strip = found, mm')
    ap.add_argument('--note', default='', help='operator note on the grasp (how it was closed, tilt, ...)')
    a = ap.parse_args()
    st = load_state()
    if a.confirm:
        last = st.get('last', {})
        last['operator_holes'] = [h.lower() for h in a.holes]
        last['hit'] = sorted(last['operator_holes']) == sorted(last.get('holes', []))
        st['last'] = last
        st.setdefault('history', []).append(dict(last))
        save_state(st)
        print(json.dumps(last, indent=1))
        return
    g = yaml.safe_load(GRIPPER.read_text())
    pose = yaml.safe_load((ROOT / 'configs/breadboard/board_pose.yaml').read_text())
    top = pose['top_z_mm'] / 1000
    grip = {'tcp_tip': [0, 0, g['fingertip_z'], 0, 0, 0], 'collision_points': g['collision_points']}
    ex = URExec(APP, CLIENT, grip, tcp_key='tcp_tip')
    q0 = [call(ex.b, 'robot_get_joint_positions')['data'][k] for k in J]
    if a.pull:
        msgs, _ = ex.run('\n'.join(pull_lines(top)), timeout=120) if a.run else (['plan only'], None)
        print('\n'.join(msgs))
        if a.run:
            st['inserted'] = None
            save_state(st)
        return
    spec = casing(a.part)
    pair = [hole(h, pose) for h in a.holes]
    span = abs(pair[1][0] - pair[0][0]) if len(pair) == 2 else 0
    want = spec['pin_spacing'] * PITCH
    if len(pair) != 2 or abs(pair[1][1] - pair[0][1]) > 1e-6 or abs(span - want) > 1e-6:
        raise SystemExit(f'{a.part}: give 2 holes in one column, {spec["pin_spacing"]} rows apart')
    target = np.mean(pair, axis=0)
    if min(pair[0][0], pair[1][0]) < X_MIN:
        raise SystemExit(f'beyond the reach limit (x < {X_MIN}): rows past ~41')
    board_off = np.array(st.get('offset_mm', [0, 0])) / 1000      # board pose error (legs - model)
    offset = board_off.copy()
    # legs side by side along the finger face = flange y: turn flange y along base x (the rows)
    yaw = next((y for y in (-np.pi / 2, np.pi / 2) if not ik_ok(ex, grip, tool_down(y), q0, [(*target, top + 0.04)])), None)
    if yaw is None:
        raise SystemExit('no tool yaw reachable (wrist 3 / elbow / IK)')
    rv = tool_down(yaw)
    strip = (target[0], strip_y(a.holes[0][0].lower(), pose))
    pull_first = bool(st.get('inserted'))
    g = 0.0
    if a.calib:
        leg_len = (st.get('last', {}).get('leg_length_mm') or 6.5) / 1000
        face_x = pose['row1_end_x_mm'] / 1000 + board_off[0]   # same frame as the holes: model + learned board offset
        cprog, cz, cx0 = grasp_calib.program(rv, face_x, CALIB_Y, top,
                                             top - BOARD_H, spec, leg_len, strip=strip)   # legs measured first
        print(cprog, f'\n--- calib: casing bottom z {cz * 1000:.1f} mm (top {top * 1000:.1f}), start x {cx0 * 1000:.1f}, '
              f'legs {leg_len * 1000:.2f} mm\nlint:', lint(LIB + cprog) or 'ok')
        bad = ik_ok(ex, grip, rv, q0, [(cx0, CALIB_Y, top + 0.04), (cx0, CALIB_Y, cz), (cx0 - 0.020, CALIB_Y, cz)])
        print('IK calib:', 'all reachable' if not bad else bad)
        if not a.run or bad:
            return
        if pull_first:
            msgs, _ = ex.run('\n'.join(pull_lines(top)), timeout=120)
            print('\n'.join(msgs))
            st['inserted'] = None
            save_state(st)
            pull_first = False
        start = R.from_rotvec(rv).as_matrix() @ -np.array(grip['tcp_tip'][:3]) + [cx0, CALIB_Y, top + 0.04]
        ex.move_checked(list(start) + rv, v=0.2, floor=top, min_clear=0.02)
        cmsgs, _ = ex.run(cprog, timeout=180)
        print('\n'.join(m for m in cmsgs if not m.startswith(('pose ', 'force ', 'contact '))))
        shots = snapshot(ex)          # how the part sits in the fingers, at the calibration pose
        g = grasp_calib.parse(cmsgs, face_x, spec)
        if g is None:
            raise SystemExit('grasp calibration: no side contact')
        print(f'grasp: legs {g * 1000:+.2f} mm from the fingertip along x')
        # bent legs peck at 2 N and find nothing (lesson rack-pick-bent-legs): stop before the first peck
        zl = next((float(m.split()[1]) for m in cmsgs if m.startswith('strip ')), None)
        legs = None if zl is None else (zl - top) * 1000
        dc = next((float(m.split()[-1]) for m in cmsgs if m.startswith('casing bottom below tips ')), None)
        if legs is None and dc:      # the strip line can be lost in the log read-back (10-27); dc = legs - LEG_BELOW_CASING
            legs = (dc + grasp_calib.LEG_BELOW_CASING) * 1000
        if legs is None or not LEGS_RANGE_MM[0] <= legs <= LEGS_RANGE_MM[1] or abs(g) > GRASP_MAX:
            raise SystemExit(f'suspect bent legs: leg length {legs} mm (normal {LEGS_RANGE_MM}), '
                             f'grasp {g * 1000:+.2f} mm (max {GRASP_MAX * 1000:.0f}); no pecks')
        offset = board_off - [g, 0.0]
        q0 = [call(ex.b, 'robot_get_joint_positions')['data'][k] for k in J]   # the arm moved: fresh joints
    if a.grasp_dx is not None and not a.calib:
        g = a.grasp_dx / 1000
        offset = board_off - [g, 0.0]
    offset = offset + np.array(a.shift) / 1000
    prog = program(rv, strip, target, offset, top, pull_first, a.drop / 1000)
    problems = lint(LIB + prog)
    print(prog, '\n---\nlint:', problems or 'ok', '| part', a.part, '| yaw deg', round(np.degrees(yaw)),
          '| target mm', np.round(target * 1000, 2).tolist(), '| offset mm', np.round(offset * 1000, 2).tolist(),
          '| strip mm', np.round(np.array(strip) * 1000, 2).tolist(), '| pull first', st.get('inserted'))
    bad = ik_ok(ex, grip, rv, q0, [(*strip, top + 0.04), (*strip, top), (*target, top + SAFE), (*target, top)])
    print('IK:', 'all reachable' if not bad else bad)
    if not a.run or problems or bad:
        return
    start = R.from_rotvec(rv).as_matrix() @ -np.array(grip['tcp_tip'][:3]) + [strip[0], strip[1], top + 0.04]
    if not pull_first and not a.calib:   # a part in the board: the program pulls it out before any sideways move
        ex.move_checked(list(start) + rv, v=0.2, floor=top, min_clear=0.02)
    t0 = time.time()
    if not a.calib:
        shots = {}
    msgs, _ = ex.run(prog, timeout=300)
    res = {'grip': grip_record(ex, a.note), 'shots': list(shots),
           'stamp': time.strftime('%Y-%m-%dT%H:%M:%S'), 'secs': round(time.time() - t0, 1), 'part': a.part,
           'holes': [h.lower() for h in a.holes], 'target_mm': np.round(target * 1000, 2).tolist(),
           'offset_used_mm': np.round(offset * 1000, 2).tolist(), 'yaw_deg': round(float(np.degrees(yaw))), 'msgs': msgs,
           'shift_mm': a.shift, 'drop_mm': a.drop, 'grasp_given_mm': a.grasp_dx}
    zs = next((float(m.split()[1]) for m in msgs if m.startswith('strip ')), None)
    if zs is not None:
        res['leg_length_mm'] = round((zs - top) * 1000, 2)
    found = next((m for m in msgs if m.startswith('found ')), None)
    if found:
        p = [float(v) for v in found.split('p[')[1].rstrip(']').split(',')]
        res['found_offset_mm'] = np.round((np.array(p[:2]) - target) * 1000, 2).tolist()      # TCP - target
        legs = np.array(p[:2]) + [g, 0.0]
        res['legs_offset_mm'] = np.round((legs - target) * 1000, 2).tolist()                   # legs - target
        st['inserted'] = res['holes']
        if a.calib or a.grasp_dx is not None:
            ok, dev = grasp_calib.legs_offset_ok(p[0], target[0], board_off[0], g)
            res['grasp_dx_mm'] = round(g * 1000, 2)
            res['pair_check'] = 'ok' if ok else f'ONE HOLE OFF suspected: legs {dev * 1000:+.2f} mm from the expected pair'
            # calibration (casing bottom) vs drop (leg tips, ~6-7 mm lower): grows with the part's tilt
            res['calib_residual_mm'] = round(dev * 1000, 2)
            if ok:      # only a confirmed pair teaches the board offset
                st['offset_mm'] = res['legs_offset_mm']
        else:
            st['offset_mm'] = res['found_offset_mm']   # jumper runs: hand-centred grasp, legs = TCP
    st['last'] = {k: res[k] for k in ('stamp', 'part', 'holes', 'target_mm', 'offset_used_mm') if k in res} | \
                 {k: res.get(k) for k in ('found_offset_mm', 'legs_offset_mm', 'leg_length_mm', 'grasp_dx_mm',
                                          'pair_check', 'calib_residual_mm', 'grip')}
    save_state(st)
    print(json.dumps({k: v for k, v in res.items() if k != 'msgs'}, indent=1))
    print('\n'.join(m for m in msgs if not m.startswith(('pose ', 'force ', 'contact '))))
    d = ROOT / 'data' / 'runs' / time.strftime('place_probe_%Y%m%d-%H%M%S')
    d.mkdir(parents=True, exist_ok=True)
    for name, img in shots.items():
        (d / name).write_bytes(img)
    (d / 'result.json').write_text(json.dumps(res, indent=1))
    print('saved', d)


if __name__ == '__main__':
    main()
