"""Find the breadboard by touch (explore mode): 3 guarded touches on the top surface, 2 on the
long edge on the far side (+y in base) and 1 on the free end (+x), all in ONE program.

The rough board box comes from the fixed camera (ArUco cube + depth, 2026-10-27). The fingertip
z in configs/breadboard/gripper.yaml is provisional, so the first touch descends guarded from
high up, and every later height is relative to the measured top. Contacts come back through
textmsg(); this script fits the board top plane, the edge line and the end, and prints the
board corner in base. Straight guarded moves only, 4 N stop, 5 mm/s, back off before the end.

  python -m breadboard.touch_locate            # plan: program + IK/reach checks, no motion
  python -m breadboard.touch_locate --run      # play it on the robot (move tier + Remote)
"""
import argparse
import ast
import json
import re
import time
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

from urcore.urexec import URExec, call, urpose
from urcore.urscript_lint import lint
from urcore.urlib import LIB

ROOT = Path(__file__).resolve().parent.parent
APP, CLIENT = 'breadboard_explore', 'breadboard-explore'

# rough board box in base (m), fixed cam 2026-10-27: long axis along base x
BOX = dict(x_row1=-0.530, x_end=-0.365, y_far=-0.008, y_near=-0.064, z_top=0.013)
SAFE = 0.035          # transit height above the top
F_STOP, V_TOUCH = 4.0, 0.005
OUT = 0.012           # edge touches start this far outside the rough edge
UNDER = 0.003         # edge touches run this far below the measured top

# (name, x, y): top touches on the hole fields (not the channel at y ~ -0.035), away from
# the wires (x < -0.505) and the white cube (far-right corner)
TOP = [('top1', -0.465, -0.025), ('top2', -0.400, -0.025), ('top3', -0.440, -0.046)]
# x > -0.47: further out the UR3 elbow straightens (< 0.5 rad) at these heights
# (name, x, y, direction): edge touches start outside and move inwards
EDGE = [('far1', -0.465, BOX['y_far'] + OUT, (0, -1, 0)),
        ('far2', -0.415, BOX['y_far'] + OUT, (0, -1, 0)),
        ('end1', BOX['x_end'] + OUT, -0.048, (-1, 0, 0))]


def tool_down(yaw=0.0):
    """Rotation vector of a tool pointing straight down, jaw (flange x) turned `yaw` from base x."""
    return list((R.from_euler('z', yaw) * R.from_euler('x', np.pi)).as_rotvec())


def pick_yaw(ex, grip, q0, xy=(-0.440, -0.036), z=0.06):
    """The tool yaw whose IK keeps wrist 3 closest to now (camera cable, +-0.8 rad)."""
    best = None
    for yaw in np.arange(-np.pi, np.pi, np.pi / 12):
        rv = tool_down(yaw)
        fl = (R.from_rotvec(rv).as_matrix() @ -np.array(grip['tcp_tip'][:3])) + [xy[0], xy[1], z]
        try:
            r = call(ex.b, 'robot_inverse_kinematics', pose={'position': list(fl), 'orientation': rv}, qNear=q0)['data']
        except Exception:  # noqa: BLE001
            continue
        d = abs(float(r['wrist3']) - q0[5])
        if best is None or d < best[0]:
            best = (d, float(yaw))
    return best[1]


def program(rv):
    o = ', '.join(f'{v:.5f}' for v in rv)
    p = lambda x, y, z: f'p[{x:.5f}, {y:.5f}, {z:.5f}, {o}]'  # noqa: E731
    zs = BOX['z_top'] + SAFE
    s = ['textmsg("touch_locate start")']
    # first touch: guarded from high up (fingertip z provisional)
    n, x, y = TOP[0]
    s += [f'movel({p(x, y, zs)}, a=0.3, v=0.05)',
          f'h = guarded_axial([0, 0, -1], 0.10, {F_STOP}, {V_TOUCH})',
          'if not h:', '  textmsg("ABORT no top contact")', f'  movel({p(x, y, zs)}, a=0.3, v=0.05)', '  halt',
          'end', f'textmsg("C {n} ", get_actual_tcp_pose())',
          'back_off([0, 0, -1], 0.003)', 'zt = get_actual_tcp_pose()[2] - 0.003']
    for n, x, y in TOP[1:]:
        s += [f'movel(p[{x:.5f}, {y:.5f}, zt + {SAFE:.3f}, {o}], a=0.3, v=0.05)',
              f'movel(p[{x:.5f}, {y:.5f}, zt + 0.006, {o}], a=0.3, v=0.02)',
              f'h = guarded_axial([0, 0, -1], 0.015, {F_STOP}, {V_TOUCH})',
              f'if h:', f'  textmsg("C {n} ", get_actual_tcp_pose())', 'else:', f'  textmsg("MISS {n}")', 'end',
              'back_off([0, 0, -1], 0.003)']
    for n, x, y, d in EDGE:
        dx, dy, dz = d
        s += [f'movel(p[{x:.5f}, {y:.5f}, zt + {SAFE:.3f}, {o}], a=0.3, v=0.05)',
              # down beside the edge, guarded: a contact here means the rough edge is wrong
              f'h = guarded_axial([0, 0, -1], {SAFE + UNDER:.4f}, {F_STOP}, 0.01)',
              'if h:', f'  textmsg("BLOCKED {n}")', 'else:',
              f'  h = guarded_axial([{dx}, {dy}, {dz}], {OUT + 0.012:.4f}, {F_STOP}, {V_TOUCH})',
              '  if h:', f'    textmsg("C {n} ", get_actual_tcp_pose())', '  else:', f'    textmsg("MISS {n}")', '  end',
              f'  back_off([{dx}, {dy}, {dz}], 0.003)',
              'end',
              'q = get_actual_tcp_pose()',
              f'movel(p[q[0], q[1], zt + {SAFE:.3f}, {o}], a=0.3, v=0.02)']
    s += ['textmsg("touch_locate done")']
    return '\n'.join(s)


def targets(rv, ztop_guess):
    """Every pose the program visits (tip TCP), for the IK pre-check."""
    zs = ztop_guess + SAFE
    pts = [(TOP[0][1], TOP[0][2], zs)]
    pts += [(x, y, zs) for _, x, y in TOP[1:]] + [(x, y, ztop_guess + 0.006) for _, x, y in TOP[1:]]
    pts += [(x, y, zs) for _, x, y, _ in EDGE] + [(x, y, ztop_guess - UNDER) for _, x, y, _ in EDGE]
    pts += [(x + d[0] * (OUT + 0.012), y + d[1] * (OUT + 0.012), ztop_guess - UNDER) for _, x, y, d in EDGE]
    return [list(p) + rv for p in pts]


def parse(msgs):
    C = {}
    for m in msgs:
        k = re.match(r'C (\w+) p(\[.*\])', m)
        if k:
            C[k.group(1)] = np.array(ast.literal_eval(k.group(2))[:3])
    return C


def fit(C, yaw):
    """Top plane from the top contacts, far edge line, end point. The closed tip is 4 mm along the
    jaw (flange x) x 5 mm across: its half size along base x / y moves a side contact from the
    tip centre to the touched face."""
    c, s = abs(np.cos(yaw)), abs(np.sin(yaw))
    half = (0.002 * c + 0.0025 * s, 0.002 * s + 0.0025 * c)
    out = {'tip_half_mm': [round(h * 1000, 2) for h in half]}
    T = np.array([C[k] for k in ('top1', 'top2', 'top3') if k in C])
    if len(T) == 3:
        nrm = np.cross(T[1] - T[0], T[2] - T[0])
        nrm /= np.linalg.norm(nrm) * np.sign(nrm[2])
        out['top_z_mm'] = round(float(T[:, 2].mean() * 1000), 2)
        out['tilt_deg'] = round(float(np.degrees(np.arccos(nrm[2]))), 3)
    if 'far1' in C and 'far2' in C:
        a, b = C['far1'] - [0, half[1], 0], C['far2'] - [0, half[1], 0]
        out['far_edge_y_mm'] = round(float((a[1] + b[1]) / 2 * 1000), 2)
        out['yaw_deg'] = round(float(np.degrees(np.arctan2(b[1] - a[1], b[0] - a[0]))), 3)
    if 'end1' in C:
        out['end_x_mm'] = round(float((C['end1'][0] - half[0]) * 1000), 2)
    return out


GRIPPER = ROOT / 'configs/breadboard/gripper.yaml'


def top_from_scene():
    """Board top z in base (m) from the calibrated scene: plate top (world, objects.yaml) +
    board height (board.yaml) - base z in the world (scene.yaml)."""
    scene = yaml.safe_load((ROOT / 'configs/calibration/scene.yaml').read_text())
    plate = yaml.safe_load((ROOT / 'configs/wire/objects.yaml').read_text())['objects']['protoboard']['plate_z_mm'] / 1000
    board = yaml.safe_load((ROOT / 'configs/breadboard/board.yaml').read_text())['size_mm']['height'] / 1000
    return plate + board - scene['base']['z']


def fingertip_from_top(C, used):
    """The real flange -> fingertip length: the tip touched the top at z_c with TCP `used`, so the
    tip is (top - z_c) longer than modelled."""
    zc = np.mean([C[k][2] for k in ('top1', 'top2', 'top3') if k in C])
    return used + (top_from_scene() - zc)


def update_gripper(est, run, tol=0.001):
    """Write a measured fingertip length into gripper.yaml (comments kept): fingertip_z, tcp_pins z
    and every collision point move by the same delta. Only when it changed by more than `tol`."""
    import re
    txt = GRIPPER.read_text()
    old = yaml.safe_load(txt)['fingertip_z']
    delta = est - old
    if abs(delta) <= tol:
        print(f'fingertip {old:.4f} m confirmed ({delta * 1000:+.1f} mm)')
        return False

    def shift(m):
        return f'{m.group(1)}{float(m.group(2)) + delta:.4f}{m.group(3)}'
    txt = re.sub(r'^(fingertip_z: )([\d.]+)()', shift, txt, flags=re.M)
    txt = re.sub(r'^(tcp_pins: \[[^,]+, [^,]+, )([\d.]+)(,)', shift, txt, flags=re.M)
    txt = re.sub(r'^(  \w+: \[[^,]+, [^,]+, )([\d.]+)(\])', shift, txt, flags=re.M)     # collision points
    txt = re.sub(r"^stamp: .*$", f"stamp: '{time.strftime('%Y-%m-%dT%H:%M')}'", txt, flags=re.M)
    txt = re.sub(r'^source: .*$', f'source: fingertip_z measured by touch ({run}); grasp_h still PROVISIONAL', txt, flags=re.M)
    GRIPPER.write_text(txt)
    print(f'gripper.yaml: fingertip {old:.4f} -> {est:.4f} m ({delta * 1000:+.1f} mm)')
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', action='store_true')
    ap.add_argument('--set-fingertip', metavar='RUN_DIR', help='apply the fingertip length of a saved run')
    a = ap.parse_args()
    if a.set_fingertip:
        res = json.loads((Path(a.set_fingertip) / 'result.json').read_text())
        C = {k: np.array(v) / 1000 for k, v in res['contacts_mm'].items()}
        update_gripper(fingertip_from_top(C, res.get('fingertip_z_used', 0.2545)), Path(a.set_fingertip).name)
        return
    g = yaml.safe_load(GRIPPER.read_text())
    grip = {'tcp_tip': [0, 0, g['fingertip_z'], 0, 0, 0], 'collision_points': g['collision_points']}
    ex = URExec(APP, CLIENT, grip, tcp_key='tcp_tip')
    q0 = [call(ex.b, 'robot_get_joint_positions')['data'][k] for k in ('base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3')]
    yaw = pick_yaw(ex, grip, q0)
    rv = tool_down(yaw)
    prog = program(rv)
    problems = lint(LIB + prog)
    print(prog, '\n---\nlint:', problems or 'ok', '| tool yaw deg', round(np.degrees(yaw), 1))
    bad = []
    for t in targets(rv, BOX['z_top']):
        fl = (R.from_rotvec(t[3:]).as_matrix() @ -np.array(grip['tcp_tip'][:3])) + t[:3]   # flange pose of the tip target
        try:
            r = call(ex.b, 'robot_inverse_kinematics', pose={'position': list(fl), 'orientation': t[3:]}, qNear=q0)['data']
            q = [float(r[k]) for k in ('base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3')]
            if abs(q[5] - q0[5]) > 0.8 or abs(q[2]) < 0.5:
                bad.append((np.round(t[:3], 3).tolist(), 'w3/elbow', np.round(q, 2).tolist()))
        except Exception as e:  # noqa: BLE001
            bad.append((np.round(t[:3], 3).tolist(), str(e)[:120]))
    print('IK:', 'all reachable' if not bad else bad)
    if not a.run or problems or bad:
        return
    start = BOX['z_top'] + 0.065            # higher is out of the UR3's reach at x -0.44 (IK times out)
    fl = (R.from_rotvec(rv).as_matrix() @ -np.array(grip['tcp_tip'][:3])) + [-0.420, -0.036, start]
    ex.move_checked(list(fl) + rv, v=0.2, floor=BOX['z_top'] - 0.009, min_clear=0.05)
    t0 = time.time()
    msgs, pose = ex.run(prog, timeout=240)
    C = parse(msgs)
    res = {'stamp': time.strftime('%Y-%m-%dT%H:%M:%S'), 'secs': round(time.time() - t0, 1), 'msgs': msgs,
           'tool_yaw_deg': round(float(np.degrees(yaw)), 1), 'fingertip_z_used': g['fingertip_z'],
           'contacts_mm': {k: np.round(v * 1000, 2).tolist() for k, v in C.items()}, 'fit': fit(C, yaw)}
    if all(k in C for k in ('top1', 'top2', 'top3')):
        res['fingertip_z_est'] = round(float(fingertip_from_top(C, g['fingertip_z'])), 4)
    print(json.dumps(res, indent=1))
    d = ROOT / 'data' / 'runs' / time.strftime('touch_locate_%Y%m%d-%H%M%S')
    d.mkdir(parents=True, exist_ok=True)
    (d / 'result.json').write_text(json.dumps(res, indent=1))
    print('saved', d)
    if 'fingertip_z_est' in res:      # the tool is measured every run; the file follows the hardware
        update_gripper(res['fingertip_z_est'], d.name)


if __name__ == '__main__':
    main()
