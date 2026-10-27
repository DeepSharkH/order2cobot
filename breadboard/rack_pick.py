"""Pick a 2-leg casing out of the casing rack and hold it the way breadboard-place needs it
(skill rack-pick): legs straight down, leg pair along the finger faces, casing bottom at the tips.

  locate: the rack pose by touch, from the camera box in configs/breadboard/rack.yaml.
    P1  2 touches on the +y long edge and 1 on the breadboard end (1.5 N side push, 4 mm below
        the top), then the first edge again (> 0.3 mm = the rack moved). The end touch runs in
        the ~2 cm gap to the board, on the -y half (the white cube sits on the +y rail); the
        robot end folds the arm back over its base (shoulder -2.7, wrist 3 -4.4).
    P2  3 touches on the solid strips between the rows (2 N, 5 mm/s), then the first again.
  pick R<k>: (A) height calibration next to the slot (2 N) and on the casing top (1 N): the CAD
    height must come out, then Gripper Open and a guarded descent (2 N) to 1 mm above the
    surface; the robot holds there. (B, --close, after the operator's go) Gripper Close
    (100 % force, the only setting), pull straight up (stop at 3 N = a leg is caught), lift.

Grasp, pull and lift: tool yaw = rack yaw -/+ 90 deg (jaw along rack y', across the rows), whichever keeps wrist 3
nearer the cable reference; the legs then lie along the finger faces as placing needs (placing turns the tool
itself). With the rack along base x that is -90 deg, the placing yaw. Touches and transit (closed tips, any yaw): the
multiple of 90 deg off the rack nearest the reference. Wrist 3 limits from rack.yaml: `wrist3_tol_rad` (0.8) for
touches / transit, `wrist3_grasp_tol_rad` for the grasp (the user accepted 1.4 for the rack turned 90 deg, 10-27);
the wrist turns between the two by a joint move at transit height. Never halt; aborts go through if/else.

  python -m breadboard.rack_pick locate                      # plan: programs + IK, no motion
  python -m breadboard.rack_pick locate --run
  python -m breadboard.rack_pick find [--run]                # which slots hold a casing (fixed cam); --run parks the arm first
  python -m breadboard.rack_pick pick R4 --part led_s2       # plan
  python -m breadboard.rack_pick pick R4 --part led_s2 --run             # (A), holds at the grasp
  python -m breadboard.rack_pick pick R4 --part led_s2 --close --run     # (B)
  python -m breadboard.rack_pick --confirm R4 "<what the operator / a check saw>"
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

from breadboard.place_probe import casing, grip_record, ik_ok, snapshot
from breadboard.touch_locate import GRIPPER, tool_down
from dashboard.ur_bridge import URBridge
from urcore.urexec import URExec, call
from urcore.urlib import LIB
from urcore.urscript_lint import lint

ROOT = Path(__file__).resolve().parent.parent
APP, CLIENT = 'breadboard_pick', 'breadboard-pick'
RACK = ROOT / 'configs/breadboard/rack.yaml'
STATE = ROOT / 'data' / 'rack_state.json'
J = ('base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3')
F_TOP, F_CASE, F_SIDE, F_HIT = 2.0, 1.0, 1.5, 2.0
F_CAUGHT = 8.0       # pull stop: the post-it holds the legs with ~4 N (lesson post-it-holds-legs; 3 N stopped a clean pick)
V_TOUCH, V_SLOW = 0.005, 0.002
V_TRANSIT, V_APPROACH, V_LIFT = 0.15, 0.05, 0.03   # free moves; only contacts are guarded and slow
PULL_SLOW = 0.010     # the legs (~6.5 mm) leave paper and slit within this, pulled at V_SLOW; the rest at V_LIFT
SAFE = 0.045          # transit: tips this far above the rack top (25 mm over a 20 mm casing)
OUT = 0.015           # side touches start this far outside the rough face
REACH = 0.015         # ... and push this far past it (the camera box was 13+ mm off in y, 10-27)
UNDER = 0.004         # side touches run this far below the top (plate face z' 10-18)
HALF = (0.002, 0.0025)   # closed tip (4 mm jaw x 5 mm) half size along the jaw (flange x) / across it
CASE_TOL = 0.001      # casing top - surface = CAD height +- this
MOVED = 0.0003
# touch spots in rack mm, chosen on 10-27's camera image: bare blue between the tape straps
# (x' ~26-46, ~146-167) and short of the post-it edge (x' ~105)
EDGE_X = (60.0, 95.0)
TOP = [(73.1, 19.3), (73.1, 51.9), (95.0, 35.6)]   # on the strips between rows S1/S2, S3/S4, S2/S3
END_Y = 10.0          # end touch this far from the -y long edge (rack mm)
W3_REF = -3.78        # wrist 3 where the camera cable is relaxed (memory ur3-cable-limits)


def w3_tol(grasp=False):
    cfg = load_rack()
    t = float(cfg.get('wrist3_tol_rad', 0.8))
    return float(cfg.get('wrist3_grasp_tol_rad', t)) if grasp else t


def half_along(d, rv):
    """Closed tip half size along the unit base-xy direction d, for the tool orientation rv."""
    c = abs(np.asarray(d, float) @ R.from_rotvec(rv).as_matrix()[:2, 0])
    return c * HALF[0] + np.sqrt(max(0.0, 1 - c * c)) * HALF[1]


def pushes(rough, cfg):
    """Base-xy push directions of the edge and the end touches: into the rack through the touched faces."""
    _, ux, uy = frame(rough)
    s = -1 if flipped(cfg) else 1
    return -s * uy, s * ux


def choose_rv(ex, grip, xy, z, across=True):
    """Tool down, the yaw whose IK wrist 3 is nearest the cable reference at xy: across=True (grasp) the jaw across
    the rows (rack y', two yaws 180 deg apart); across=False (closed-tip touches, transit) any multiple of 90 deg."""
    cfg = load_rack()
    th = np.radians((cfg.get('pose') or cfg['rough'])['yaw_deg'])
    q0 = [call(ex.b, 'robot_get_joint_positions')['data'][k] for k in J]
    best = None
    # one IK (the old placing yaw, reachable on this table); tool straight down, wrist 3 moves one-for-one with the
    # tool yaw, so the other yaws follow without IK (an unreachable IK only returns after a timeout)
    rv0 = tool_down(-np.pi / 2)
    fl = R.from_rotvec(rv0).as_matrix() @ -np.array(grip['tcp_tip'][:3]) + [xy[0], xy[1], z]
    r = call(ex.b, 'robot_inverse_kinematics', pose={'position': list(fl), 'orientation': list(rv0)}, qNear=q0[:5] + [W3_REF])
    if 'data' not in r:
        raise SystemExit(f'no IK at {np.round(np.array(xy) * 1000, 1).tolist()}')
    w0 = float(r['data']['wrist3'])
    for yaw in (th - np.pi / 2, th + np.pi / 2) + (() if across else (th, th + np.pi)):
        w = w0 - np.angle(np.exp(1j * (yaw + np.pi / 2)))          # IK 10-27: yaw +90 -> wrist 3 -1.57
        w += 2 * np.pi * np.round((W3_REF - w) / (2 * np.pi))      # the turn nearest the reference
        if best is None or abs(w - W3_REF) < best[0]:
            best = (abs(w - W3_REF), tool_down(yaw), yaw)
    print(f'{"grasp" if across else "touch"} tool yaw {np.degrees(best[2]):.0f} deg, wrist 3 {best[0]:.2f} rad off the cable '
          f'reference (tol {w3_tol(across)})')
    return best[1]


def load_rack():
    return yaml.safe_load(RACK.read_text())


def frame(pose):
    """Origin (m) and unit x', y' of the rack frame in base."""
    th = np.radians(pose['yaw_deg'])
    return np.array(pose['origin_mm']) / 1000, np.array([np.cos(th), np.sin(th)]), np.array([-np.sin(th), np.cos(th)])


def at(pose, xp, yp):
    o, ux, uy = frame(pose)
    return o + ux * xp / 1000 + uy * yp / 1000


def slot(cfg, k):
    g = cfg['geometry']
    n = g.get('cols', 5)            # slots per row along x' (the original slit print: 5; Jan's 10-27 holder: 4)
    c, r = (k - 1) % n, (k - 1) // n
    return g['slot_x0'] + g['slot_dx'] * c, g['slot_y0'] + g['slot_dy'] * r


def flipped(cfg):
    """Slot 1 at the robot end: x' runs along -x base, the +y edge is y' 0 and the robot end x' 0."""
    return cfg['orientation']['slot1_end'] == 'robot'


def touched_faces(cfg):
    """Rack coordinates of the faces locate touches: the +y-base long edge (y') and the
    breadboard end (-x base, x'), and the y' of the end touch."""
    W, L = cfg['geometry']['size'][1], cfg['geometry']['size'][0]
    return (0.0, L, W - END_Y) if flipped(cfg) else (W, 0.0, END_Y)


def load_state():
    return json.loads(STATE.read_text()) if STATE.exists() else {'slots': {}, 'history': []}


def save_state(st):
    STATE.parent.mkdir(parents=True, exist_ok=True)
    STATE.write_text(json.dumps(st, indent=1))


def open_ex(grip, run):
    """URExec for a run; for a plan only the bridge (IK, joints): creating the app needs the move tier."""
    if run:
        return URExec(APP, CLIENT, grip, tcp_key='tcp_tip')
    ex = URExec.__new__(URExec)
    ex.b, ex.grip, ex.tcp = URBridge(client_name=CLIENT), grip, grip['tcp_tip']
    return ex


def gripper(ex, which, st=None):
    """Hand-E through the MCP smart skill Gripper Open / Close (no parameters; never changed). It runs with our
    app loaded (10-27), so no app switching. No position / force readout: the controller log's warnings are the
    only check (xmlrpc "Invalid request" code 3 = the Hand-E is not activated). Records the finger state in st."""
    base = max(int(l['ts']) for l in call(ex.b, 'psx_logs')['data']['lines'])
    t0 = time.time()
    call(ex.b, 'smartskill_run', type=f'robotiq-gripper-{which}-smart-skill', confirm=True)
    while time.time() - t0 < 20:
        time.sleep(0.3)
        if call(ex.b, 'smartskill_status')['data'].get('active') is None:
            break
    warn = [l['message'] for l in call(ex.b, 'psx_logs')['data']['lines']
            if int(l['ts']) > base and l['level'] in ('WARNING', 'ERROR', 'FAULT')]
    if st is not None:
        st['fingers'] = 'unknown' if warn else ('open' if which == 'open' else 'closed')
        save_state(st)
    return {'skill': which, 'secs': round(time.time() - t0, 1), 'warnings': warn}


def P(rv):
    o = ', '.join(f'{v:.5f}' for v in rv)
    return o, (lambda x, y, z: f'p[{x:.5f}, {y:.5f}, {z:.5f}, {o}]')


def side_touch(p, o, name, xy, d, z_tr, z_side):
    """Down beside the face (a contact there = the rough face is > OUT off), then push in at F_SIDE."""
    dx, dy = d
    return [f'movel({p(*xy, z_tr)}, a=0.3, v=0.05)',
            f'h = guarded_axial([0, 0, -1], {z_tr - z_side:.4f}, {F_TOP}, 0.01)',
            'if h:', f'  textmsg("BLOCKED {name}")', 'else:',
            f'  h = guarded_axial([{dx}, {dy}, 0], {OUT + REACH:.4f}, {F_SIDE}, {V_TOUCH})',
            '  if h:', f'    textmsg("C {name} ", get_actual_tcp_pose())', '  else:', f'    textmsg("MISS {name}")', '  end',
            f'  back_off([{dx}, {dy}, 0], 0.003)', 'end',
            'q = get_actual_tcp_pose()', f'movel(p[q[0], q[1], {z_tr:.5f}, {o}], a=0.3, v=0.02)']


def down_touch(p, o, name, xy, z_from, maxd, f, v, z_tr):
    return [f'movel({p(*xy, z_tr)}, a=0.5, v={V_TRANSIT})', f'movel({p(*xy, z_from)}, a=0.3, v={V_APPROACH})',
            f'h = guarded_axial([0, 0, -1], {maxd:.4f}, {f}, {v})',
            'if h:', f'  textmsg("C {name} ", get_actual_tcp_pose())', 'else:', f'  textmsg("MISS {name}")', 'end',
            'back_off([0, 0, -1], 0.003)', f'movel({p(*xy, z_tr)}, a=0.5, v={V_APPROACH})']


def locate_p1(cfg, rv):
    """Edges + end from the camera box: a list of (name, start xy, push direction) and the program."""
    rough, (yE, xE, yEnd) = cfg['rough'], touched_faces(cfg)
    top = rough['top_z_mm'] / 1000
    o, p = P(rv)
    s = 1 if not flipped(cfg) else -1      # outward normals of the touched faces in rack directions
    de, dn = (tuple(np.round(v, 4).tolist()) for v in pushes(rough, cfg))
    pts = [(f'edge{i + 1}', at(rough, x, yE + s * OUT * 1000), de) for i, x in enumerate(EDGE_X)]
    pts.append(('end', at(rough, xE - s * OUT * 1000, yEnd), dn))
    pts.append(('edge1b', pts[0][1], de))
    lines = ['textmsg("rack locate P1 start")']
    for n, xy, d in pts:
        lines += side_touch(p, o, n, xy, d, top + SAFE, top - UNDER)
    return pts, '\n'.join(lines + ['textmsg("rack locate P1 done")'])


def locate_p2(pose, top, rv):
    o, p = P(rv)
    pts = [(f'top{i + 1}', at(pose, x, y)) for i, (x, y) in enumerate(TOP)] + [('top1b', at(pose, *TOP[0]))]
    lines = ['textmsg("rack locate P2 start")']
    for n, xy in pts:
        lines += down_touch(p, o, n, xy, top + 0.006, 0.012, F_TOP, V_TOUCH, top + SAFE)
    return pts, '\n'.join(lines + ['textmsg("rack locate P2 done")'])


def parse(msgs):
    C = {}
    for m in msgs:
        k = re.match(r'C (\w+) p(\[.*\])', m)
        if k:
            C[k.group(1)] = np.array(ast.literal_eval(k.group(2))[:3])
    return C


def fit_pose(cfg, C, rv):
    """Rack pose from the touched long edge (2 points) and end: tip centre -> face by the tip half size."""
    de, dn = pushes(cfg['rough'], cfg)
    a, b = C['edge1'][:2] + de * half_along(de, rv), C['edge2'][:2] + de * half_along(de, rv)
    e = C['end'][:2] + dn * half_along(dn, rv)
    ux = (b - a) / np.linalg.norm(b - a)
    ux *= np.sign(ux @ frame(cfg['rough'])[1])
    uy = np.array([-ux[1], ux[0]])
    yE, xE, _ = touched_faces(cfg)
    o = ux * (e @ ux - xE / 1000) + uy * ((a + b) / 2 @ uy - yE / 1000)
    return {'origin_mm': np.round(o * 1000, 2).tolist(), 'yaw_deg': round(float(np.degrees(np.arctan2(ux[1], ux[0]))), 3)}


def check_ik(ex, grip, rv, pts, grasp=False):
    q0 = [call(ex.b, 'robot_get_joint_positions')['data'][k] for k in J]
    return q0, ik_ok(ex, grip, rv, q0[:5] + [W3_REF], pts, w3_tol(grasp))     # wrist 3 judged against the cable


def start_move(ex, rv, grip, xy, z, floor):
    fl = R.from_rotvec(rv).as_matrix() @ -np.array(grip['tcp_tip'][:3]) + [xy[0], xy[1], z]
    ex.move_checked(list(fl) + list(rv), w3_ref=W3_REF, w3_tol=w3_tol(), v=0.2, floor=floor, min_clear=0.03)


def start_q(ex, rv, grip, xy, z, floor):
    """Joints for a tip pose, with move_checked's checks (bent elbow, wrist 3 vs the cable, clearance), so the
    joint move can open the next program instead of costing a step of its own."""
    fl = R.from_rotvec(rv).as_matrix() @ -np.array(grip['tcp_tip'][:3]) + [xy[0], xy[1], z]
    q0 = [call(ex.b, 'robot_get_joint_positions')['data'][k] for k in J]
    ik = ex._in_active_tcp(list(fl) + rv, q0)
    r = call(ex.b, 'robot_inverse_kinematics', pose={'position': list(ik[:3]), 'orientation': list(ik[3:])}, qNear=q0)['data']
    q = [float(r[k]) for k in J]
    low = min((fl + R.from_rotvec(rv).as_matrix() @ np.array(c))[2] for c in grip['collision_points'].values()) - 0.005 - floor
    if abs(q[2]) < 0.5 or abs(q[5] - W3_REF) > w3_tol() or low < 0.03:
        raise SystemExit(f'start pose rejected: elbow {q[2]:.2f}, wrist3 {q[5]:.2f} (ref {W3_REF}), clearance {low * 1000:.0f} mm')
    return q


def run_dir(kind):
    d = ROOT / 'data' / 'runs' / time.strftime(f'{kind}_%Y%m%d-%H%M%S')
    d.mkdir(parents=True, exist_ok=True)
    return d


def locate(a, grip, rv):
    cfg = load_rack()
    top = cfg['rough']['top_z_mm'] / 1000
    pts1, prog1 = locate_p1(cfg, rv)
    print(prog1, '\n---\nlint P1:', lint(LIB + prog1) or 'ok')
    ex = open_ex(grip, a.run)
    ik = [(*xy, z) for _, xy, d in pts1 for z in (top + SAFE, top - UNDER)]
    ik += [(xy[0] + d[0] * (OUT + REACH), xy[1] + d[1] * (OUT + REACH), top - UNDER) for _, xy, d in pts1]
    ik += [(*at(cfg['rough'], x, y), z) for x, y in TOP for z in (top + SAFE, top + 0.006)]
    q0, bad = check_ik(ex, grip, rv, ik)
    print('touch spots (base mm):', {n: np.round(xy * 1000, 1).tolist() for n, xy, _ in pts1},
          '\nIK:', bad or 'all reachable', '| wrist3 now', round(q0[5], 2))
    if not a.run or bad or lint(LIB + prog1):
        return
    d = run_dir('rack_locate')
    start_move(ex, rv, grip, pts1[0][1], top + 0.06, top)
    t0 = time.time()
    msgs1, _ = ex.run(prog1, timeout=300)
    C = parse(msgs1)
    res = {'stamp': time.strftime('%Y-%m-%dT%H:%M:%S'), 'rough': cfg['rough'], 'msgs_p1': msgs1}
    if not all(k in C for k in ('edge1', 'edge2', 'end')):
        res['miss_reason'] = 'P1: ' + '; '.join(m for m in msgs1 if m.startswith(('BLOCKED', 'MISS')))
    else:
        pose = fit_pose(cfg, C, rv)
        res['pose_p1'] = pose
        if 'edge1b' in C:
            res['edge_moved_mm'] = round(float(abs(C['edge1b'][1] - C['edge1'][1])) * 1000, 3)
        pts2, prog2 = locate_p2(pose, top, rv)
        msgs2, _ = ex.run(prog2, timeout=300)
        res['msgs_p2'] = msgs2
        C.update(parse(msgs2))
        T = np.array([C[k] for k in ('top1', 'top2', 'top3') if k in C])
        if len(T) == 3:
            nrm = np.cross(T[1] - T[0], T[2] - T[0])
            nrm /= np.linalg.norm(nrm) * np.sign(nrm[2])
            pose['top_z_mm'] = round(float(T[:, 2].mean() * 1000), 2)
            pose['tilt_deg'] = round(float(np.degrees(np.arccos(nrm[2]))), 3)
            pose['top_spread_mm'] = round(float(np.ptp(T[:, 2]) * 1000), 2)
            # the rough top is the scene plate + 18 (CAD): a contact below it = tool shorter or rack lower
            res['top_vs_cad_mm'] = round(pose['top_z_mm'] - cfg['rough']['top_z_mm'], 2)
        if 'top1b' in C and 'top1' in C:
            res['top_moved_mm'] = round(float(np.linalg.norm(C['top1b'] - C['top1'])) * 1000, 3)
        moved = max(res.get('edge_moved_mm', 0), res.get('top_moved_mm', 0)) > MOVED * 1000
        if 'top_z_mm' not in pose or pose['top_spread_mm'] > 1.0 or moved:
            res['miss_reason'] = ('rack moved' if moved else 'top touches missing or > 1 mm apart (a pocket?)')
        res['pose'] = pose
    res['contacts_mm'] = {k: np.round(v * 1000, 2).tolist() for k, v in C.items()}
    res['secs'] = round(time.time() - t0, 1)
    (d / 'result.json').write_text(json.dumps(res, indent=1))
    print(json.dumps({k: v for k, v in res.items() if not k.startswith('msgs')}, indent=1), '\nsaved', d)
    if 'miss_reason' not in res:
        txt = RACK.read_text()
        pose |= {'source': f'touch ({d.name}): 2 edge + 1 end + 3 top', 'stamp': res['stamp']}
        txt = re.sub(r'^pose: .*$', 'pose: ' + json.dumps(pose), txt, flags=re.M)
        RACK.write_text(txt)
        print('rack.yaml pose updated')


def pick_points(cfg, k):
    pose = cfg['pose']
    top = pose['top_z_mm'] / 1000
    xs, ys = slot(cfg, k)
    centre = cfg['geometry']['size'][0] / 2
    xn = xs + (17.5 if xs < centre else -17.5)      # solid rack between the pockets, towards the centre
    return pose, top, at(pose, xs, ys), at(pose, xn, ys)


def grip_depth(z_tips, z_top, z_surface, height):
    """How deep the casing sits in the fingers at the moment of the close: its bottom below the fingertips (mm,
    + = below), from the casing-top touch minus the CAD height and, independently, from the surface it stands on.
    Valid only until the casing slips; nothing re-measures it after the grip (10-27)."""
    from_top = (z_tips - (z_top - height)) * 1000
    from_surface = (z_tips - z_surface) * 1000
    return {'casing_bottom_below_tips_mm': round(from_top, 2), 'from_surface_mm': round(from_surface, 2),
            'disagree_mm': round(from_top - from_surface, 2), 'source': 'casing-top touch - CAD height, at the close'}


def prog_grasp(p, sxy, z_tr, z_top, z_grasp):
    """Open fingers: turn the wrist to the grasp yaw above the casing (joint move, tips in place), fast to 5 mm above
    the casing top (the neighbours' tops are as high), guarded below."""
    return '\n'.join(['textmsg("rack pick grasp start")', f'movej({p(*sxy, z_tr)}, a=1.0, v=0.8)',
                      f'movel({p(*sxy, z_top + 0.005)}, a=0.5, v={V_APPROACH})',
                      f'h = guarded_axial([0, 0, -1], {z_top + 0.005 - z_grasp:.4f}, {F_HIT}, {V_TOUCH})',
                      'if h:', '  textmsg("ABORT finger hit something above the grasp height")',
                      f'  movel({p(*sxy, z_tr)}, a=0.3, v={V_APPROACH})',
                      'else:', '  textmsg("grasp ", get_actual_tcp_pose())', 'end'])


def prog_pull(o, stop, z_lift, o_t=None):
    """Straight up: the first PULL_SLOW slow with the force stop (legs in paper / slit), then lift fast; at the top
    turn the wrist back to the touch yaw o_t (joint move, tips in place)."""
    return '\n'.join(['textmsg("rack pick B start")', f'ok = pull_out([0, 0, -1], {stop}, {PULL_SLOW:.4f}, {V_SLOW})',
                      'if ok:', '  q = get_actual_tcp_pose()', f'  movel(p[q[0], q[1], {z_lift:.5f}, {o}], a=0.5, v={V_LIFT})',
                      '  textmsg("lifted ", get_actual_tcp_pose())']
                     + ([f'  movej(p[q[0], q[1], {z_lift:.5f}, {o_t}], a=1.0, v=0.8)'] if o_t and o_t != o else [])
                     + ['else:', '  textmsg("CAUGHT: holding")', 'end',
                      'textmsg("rack pick B done")'])


def pick(a, grip, rv, rv_t):
    """One command: [close] -> A (joint move + surface / casing-top touches) -> height check -> open -> G (grasp
    descent) -> close -> B (pull, lift). Every check stops the run (height, gripper warning, finger contact,
    pull limit). --close resumes at the close from the pending grasp (--closed: pull only)."""
    cfg, st = load_rack(), load_state()
    if not cfg.get('pose'):
        raise SystemExit('no touched rack pose: run locate first')
    k = int(a.slot.upper().lstrip('R'))
    spec = casing(a.part)
    H = spec['height'] / 1000
    pose, top, sxy, nxy = pick_points(cfg, k)
    sxy = sxy + frame(pose)[1] * a.slit_dx / 1000     # the casing along the slit, from find (post-it: anywhere in it)
    z_tr = top + SAFE
    o, p = P(rv)            # grasp, pull, lift
    o_t, p_t = P(rv_t)      # touches, transit
    ex = open_ex(grip, a.run)
    tm, t0 = {}, time.time()
    mark = lambda name: tm.__setitem__(name, round(time.time() - t0, 1))  # noqa: E731

    def finish(d, res, msgs):
        """After B: snapshot, state, result."""
        res['msgs_b'] = msgs
        pk = next((m for m in msgs if m.startswith('pulled ')), None)
        res['pull_peak_n'] = float(pk.split('peak ')[1]) if pk else None
        res['caught'] = any(m.startswith(('CAUGHT', 'pull limit')) for m in msgs)
        for name, img in snapshot(ex).items():
            (d / name.replace('grasp_', 'picked_')).write_bytes(img)
        res['result'] = 'caught' if res['caught'] else 'in the fingers (check the image, then --confirm)'
        if res['caught']:
            res['miss_reason'] = f'pull > {a.pull_stop} N: the paper / slit holds a leg; holding, image saved'
        else:
            st['slots'].setdefault(f'R{k}', {})['part'] = None
            st['in_gripper'] = {'part': a.part, 'from': f'R{k}', 'pin1': st['pending'].get('pin1'), 'run': d.name,
                                'grip_depth': st['pending'].get('grip_depth')}
            st['pending'] = None
        st['history'].append({'stamp': time.strftime('%Y-%m-%dT%H:%M:%S'), 'slot': f'R{k}', 'part': a.part, 'run': d.name,
                              'result': res['result'], 'pull_peak_n': res['pull_peak_n'],
                              **({'miss_reason': res['miss_reason']} if 'miss_reason' in res else {})})
        mark('done')
        res['timing_s'] = tm
        save_state(st)
        (d / 'result.json').write_text(json.dumps(res, indent=1))
        print(json.dumps({k_: v for k_, v in res.items() if not k_.startswith('msgs')}, indent=1), '\nsaved', d)

    def close_and_pull(d, res, zs):
        if a.closed:    # already closed on the casing (an earlier close whose pull stopped)
            res['close'] = {'skill': 'close', 'by': 'already closed', 'warnings': []}
        else:
            res['close'] = gripper(ex, 'close', st) | {'grip': grip_record(ex, 'robot grasp in the rack, Gripper Close smart skill')}
            mark('closed')
        if res['close']['warnings']:
            res['miss_reason'] = f'Gripper Close failed ({res["close"]["warnings"][0][:90]}): not pulled; fingers open around the casing'
            (d / 'result.json').write_text(json.dumps(res, indent=1))
            raise SystemExit(res['miss_reason'])
        msgs, _ = ex.run(prog_pull(o, a.pull_stop, zs + H + 0.030, o_t), timeout=120)
        mark('B pulled')
        finish(d, res, msgs)

    if a.close:     # resume at the close: the fingers must still be where the grasp descent left them
        pend = st.get('pending')
        if not pend or pend['slot'] != f'R{k}':
            raise SystemExit(f'no grasp pending at R{k}: run the pick first')
        q0, bad = check_ik(ex, grip, rv, [(*sxy, pend['z_surface'] + H + 0.030)], grasp=True)
        here = np.array(call(ex.b, 'robot_get_pose')['data']['pose']['position'])
        want = np.array(pend['grasp_mm']) / 1000 + R.from_rotvec(rv).as_matrix() @ -np.array(grip['tcp_tip'][:3])
        off = float(np.linalg.norm(here - want))
        print('IK:', bad or 'ok', f'| flange {off * 1000:.1f} mm from the grasp pose')
        if off > 0.002:
            bad = bad + [('not at the grasp pose', round(off * 1000, 1))]
        if not a.run or bad:
            return
        d = ROOT / 'data' / 'runs' / pend['run']
        res = json.loads((d / 'result.json').read_text())
        res.pop('miss_reason', None)
        return close_and_pull(d, res, pend['z_surface'])

    # plan: every program (grasp / pull with the nominal surface) linted, every point through IK
    progs = {'A': '\n'.join(['textmsg("rack pick A start")', 'movej(q_start, a=0.8, v=0.5)']
                            + down_touch(p_t, o_t, 'surf', nxy, top + 0.006, 0.012, F_TOP, V_SLOW, z_tr)
                            + down_touch(p_t, o_t, 'ctop', sxy, top + H + 0.006, 0.012, F_CASE, V_SLOW, z_tr)
                            + ['textmsg("rack pick A done")']),
             'G': prog_grasp(p, sxy, z_tr, top + H, top + 0.001), 'B': prog_pull(o, a.pull_stop, top + H + 0.030, o_t)}
    problems = {n: lint(LIB + ('q_start = [0, 0, 0, 0, 0, 0]\n' if n == 'A' else '') + s) for n, s in progs.items()}
    q0, bad = check_ik(ex, grip, rv_t, [(*nxy, z_tr), (*nxy, top), (*sxy, z_tr), (*sxy, top + H), (*sxy, top + H + 0.030)])
    bad += check_ik(ex, grip, rv, [(*sxy, z_tr), (*sxy, top + H), (*sxy, top + 0.001), (*sxy, top + H + 0.030)], grasp=True)[1]
    print(progs['A'], '\n---\nlint:', {n: v or 'ok' for n, v in problems.items()})
    print(f'R{k} {a.part}: slot (base mm)', np.round(sxy * 1000, 1).tolist(), '| surface touch', np.round(nxy * 1000, 1).tolist(),
          '| rack top', round(top * 1000, 2), '| fingers', st.get('fingers', 'unknown'),
          '\nIK:', bad or 'all reachable', '| wrist3 now', round(q0[5], 2))
    if not a.run or bad or any(problems.values()):
        return
    if st.get('in_gripper'):
        raise SystemExit(f'the gripper holds {st["in_gripper"]}: place or release it first')
    d = run_dir('rack_pick')
    res = {'stamp': time.strftime('%Y-%m-%dT%H:%M:%S'), 'slot': f'R{k}', 'part': a.part,
           'rack_pose': pose.get('source'), 'slot_mm': np.round(sxy * 1000, 2).tolist(), 'slit_dx_mm': a.slit_dx}

    def stop(reason):
        res['miss_reason'] = reason
        res['timing_s'] = tm
        st['history'].append({'stamp': res['stamp'], 'slot': f'R{k}', 'part': a.part, 'run': d.name, 'miss_reason': reason})
        save_state(st)
        (d / 'result.json').write_text(json.dumps(res, indent=1))
        print(json.dumps({k_: v for k_, v in res.items() if not k_.startswith('msgs')}, indent=1), '\nMISS:', reason, '\nsaved', d)

    if st.get('fingers') != 'closed':       # the touches need closed tips
        res['close_empty'] = gripper(ex, 'close', st)
        mark('closed empty')
        if res['close_empty']['warnings']:
            return stop(f'Gripper Close failed ({res["close_empty"]["warnings"][0][:90]}): activate the Hand-E')
    q = start_q(ex, rv_t, grip, nxy, top + 0.06, top)
    msgs, _ = ex.run(f'q_start = {[round(v, 5) for v in q]}\n' + progs['A'], timeout=180)
    mark('A touched')
    res['msgs_a'] = msgs
    C = parse(msgs)
    if 'surf' not in C or 'ctop' not in C:
        return stop('no surface / casing-top contact: ' + '; '.join(m for m in msgs if m.startswith('MISS')))
    zs, zt = C['surf'][2], C['ctop'][2]
    res |= {'z_surface_mm': round(zs * 1000, 2), 'z_top_mm': round(zt * 1000, 2),
            'height_mm': round((zt - zs) * 1000, 2), 'cad_height_mm': spec['height']}
    if abs(zt - zs - H) > a.height_tol / 1000:
        return stop(f'casing top - surface {res["height_mm"]} mm vs CAD {spec["height"]} +- {a.height_tol}: '
                    'wrong casing, sunk into the pocket, standing proud or tilted; not grasped')
    res['open'] = gripper(ex, 'open', st)
    mark('opened')
    if res['open']['warnings']:     # xmlrpc "Invalid request" (code 3) = the Hand-E is not activated (10-27)
        return stop(f'Gripper Open failed ({res["open"]["warnings"][0][:90]}): activate the Hand-E; no descent')
    msgs, _ = ex.run(prog_grasp(p, sxy, z_tr, zt, zs + 0.001), timeout=120)
    mark('G at grasp')
    res['msgs_grasp'] = msgs
    gp = next((m for m in msgs if m.startswith('grasp ')), None)
    if gp is None:
        return stop('open fingers stopped above the grasp height (hit a neighbour, the post-it or the casing); not closed')
    grasp = np.array(ast.literal_eval(gp.split(' p', 1)[1])[:3])
    res['grip_depth'] = grip_depth(grasp[2], zt, zs, H)
    st['pending'] = {'slot': f'R{k}', 'part': a.part, 'run': d.name, 'z_surface': zs,
                     'grasp_mm': np.round(grasp * 1000, 2).tolist(), 'grip_depth': res['grip_depth'],
                     'pin1': st['slots'].get(f'R{k}', {}).get('pin1')}
    save_state(st)
    close_and_pull(d, res, zs)


SCENE = ROOT / 'configs/calibration/scene.yaml'
FRAME_URL = 'http://127.0.0.1:8090/api/frame/0'
VIEW, VIEW_DZ = (-0.150, -0.150), 0.130   # arm parked out of the fixed cam's view of the rack (10-27, rack turned)


def fcam():
    """Fixed cam (configs/calibration/scene.yaml): pixel -> base xy on a base-z plane, and base xyz -> pixel."""
    sc = yaml.safe_load(SCENE.read_text())
    K, T, b = np.array(sc['fcam']['K']), np.array(sc['T_world_fcam']), sc['base']
    Rb, tb = R.from_euler('z', b['yaw_deg'], degrees=True).as_matrix(), np.array([b['x'], b['y'], b['z']])

    def onto(uv, zb):
        d, o = T[:3, :3] @ np.linalg.solve(K, [uv[0], uv[1], 1.0]), T[:3, 3]
        return (Rb.T @ (o + (zb + b['z'] - o[2]) / d[2] * d - tb))[:2]

    def project(p):
        c = np.linalg.inv(T) @ np.r_[Rb @ np.asarray(p) + tb, 1]
        return (K @ c[:3])[:2] / c[2]
    return onto, project


def to_rack(pose, xy):
    """Base xy (m) -> rack x', y' (mm)."""
    o, ux, uy = frame(pose)
    r = np.asarray(xy) - o
    return np.array([r @ ux, r @ uy]) * 1000


def nearest_slot(cfg, xr):
    k = min(range(1, 21), key=lambda k: np.hypot(*(np.array(slot(cfg, k)) - xr)))
    return k, np.asarray(xr) - slot(cfg, k)


def find(cfg):
    """Casings standing in the rack, from one fixed-cam frame: the touched rack pose, refined on the visible
    slits (the camera's own view of the rack, so a camera bias cancels), then each white blob inside the rack
    top (always blue / pink / yellow there) is a casing whose lowest pixel is its foot on the rack top."""
    import urllib.request
    import cv2
    cam, (L, W) = dict(cfg['pose']), cfg['geometry']['size']
    zt = cam['top_z_mm'] / 1000
    onto, project = fcam()
    im = cv2.imdecode(np.frombuffer(urllib.request.urlopen(FRAME_URL, timeout=5).read(), np.uint8), cv2.IMREAD_COLOR)
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

    def top_mask(m):        # the rack top, m mm inside its edges, in the image
        poly = [project([*at(cam, x, y), zt]) for x, y in ((m, m), (L - m, m), (L - m, W - m), (m, W - m))]
        mask = np.zeros(hsv.shape[:2], np.uint8)
        cv2.fillConvexPoly(mask, np.round(poly).astype(np.int32), 255)
        return mask > 0

    # slits: dark blobs on the top -> nearest slot centre within 6 mm -> refit the camera's rack frame
    dark = (top_mask(3) & (hsv[:, :, 2] < 120) & ~((hsv[:, :, 0] > 15) & (hsv[:, :, 0] < 45))).astype(np.uint8)
    n, _, st, cen = cv2.connectedComponentsWithStats(dark)
    pairs = []
    for i in range(1, n):
        if 30 < st[i, 4] < 120:
            b = onto(cen[i], zt)
            k, dev = nearest_slot(cfg, to_rack(cam, b))
            if np.hypot(*dev) < 6:
                pairs.append((np.array(slot(cfg, k)) / 1000, b))
    if len(pairs) >= 4:
        src, dst = np.array([s for s, _ in pairs]), np.array([b for _, b in pairs])
        ms, md = src.mean(0), dst.mean(0)
        U, _, Vt = np.linalg.svd((src - ms).T @ (dst - md))
        Rm = Vt.T @ np.diag([1, np.linalg.det(Vt.T @ U.T)]) @ U.T
        cam |= {'origin_mm': np.round((md - Rm @ ms) * 1000, 2).tolist(), 'yaw_deg': round(float(np.degrees(np.arctan2(Rm[1, 0], Rm[0, 0]))), 3)}
    # casings: white on the top
    white = (top_mask(1) & (hsv[:, :, 1] < 50) & (hsv[:, :, 2] > 140)).astype(np.uint8)
    n, lab, st, _ = cv2.connectedComponentsWithStats(white)
    found = []
    for i in range(1, n):
        if st[i, 4] < 20 or st[i, 3] < 6:
            continue
        ys, xs = np.nonzero(lab == i)
        foot = (xs[ys >= ys.max() - 2].mean(), ys.max() + 0.5)
        xr = to_rack(cam, onto(foot, zt))
        k, dev = nearest_slot(cfg, xr)
        found.append({'slot': f'R{k}', 'rack_mm': np.round(xr, 1).tolist(), 'off_slot_mm': np.round(dev, 1).tolist(),
                      'px': [int(foot[0]), int(foot[1])], 'area_px': int(st[i, 4]),
                      'in_slot': bool(abs(dev[0]) < 10 and abs(dev[1]) < 6)})
    return {'stamp': time.strftime('%Y-%m-%dT%H:%M:%S'), 'slits_matched': len(pairs), 'camera_fit': cam, 'casings': found}, im


def confirm(slot_name, text):
    st = load_state()
    rec = {'stamp': time.strftime('%Y-%m-%dT%H:%M:%S'), 'slot': slot_name.upper(), 'confirm': text}
    st['history'].append(rec)
    save_state(st)
    last = next((h for h in reversed(st['history'][:-1]) if h.get('slot') == rec['slot'] and h.get('run')), None)
    if last:
        f = ROOT / 'data' / 'runs' / last['run'] / 'result.json'
        r = json.loads(f.read_text())
        r['confirm'] = rec
        f.write_text(json.dumps(r, indent=1))
    print(json.dumps(rec, indent=1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', nargs='?', choices=('locate', 'find', 'pick'))
    ap.add_argument('slot', nargs='?')
    ap.add_argument('--part')
    ap.add_argument('--run', action='store_true')
    ap.add_argument('--close', action='store_true', help='(B) close on the casing and pull it out')
    ap.add_argument('--closed', action='store_true', help='(B) the fingers are already closed on the casing: pull only')
    ap.add_argument('--pull-stop', type=float, default=F_CAUGHT, help='(B) stop the pull above this force, N (cap 20)')
    ap.add_argument('--height-tol', type=float, default=CASE_TOL * 1000,
                    help='casing top - surface = CAD height +- this, mm (user 10-27: 1.0; a wider one only by the user)')
    ap.add_argument('--slit-dx', type=float, default=0.0, help="casing centre along the slit (x') from the slot centre, mm (find's off_slot)")
    ap.add_argument('--confirm', nargs=2, metavar=('SLOT', 'TEXT'))
    a = ap.parse_args()
    if a.confirm:
        return confirm(*a.confirm)
    g = yaml.safe_load(GRIPPER.read_text())
    grip = {'tcp_tip': [0, 0, g['fingertip_z'], 0, 0, 0], 'collision_points': g['collision_points']}
    cfg = load_rack()
    if a.cmd in ('locate', 'find', 'pick'):
        g_ = cfg.get('pose') or cfg['rough']
        xy = at(g_, *slot(cfg, int(a.slot.upper().lstrip('R')))) if a.cmd == 'pick' and a.slot \
            else at(g_, cfg['geometry']['size'][0] / 2, cfg['geometry']['size'][1] / 2)
        ex_ = open_ex(grip, False)
        z_ = g_['top_z_mm'] / 1000 + SAFE
        rv = list(choose_rv(ex_, grip, xy, z_, across=a.cmd == 'pick'))
        rv_t = list(choose_rv(ex_, grip, xy, z_, across=False)) if a.cmd == 'pick' else rv
    if a.cmd == 'locate':
        locate(a, grip, rv)
    elif a.cmd == 'find':
        cfg = load_rack()
        if a.run:       # the arm over the rack hides it from the fixed cam: park beside it first
            o, p = P(rv)
            z = cfg['pose']['top_z_mm'] / 1000 + VIEW_DZ
            ex = open_ex(grip, True)
            q0, bad = check_ik(ex, grip, rv, [(*VIEW, z)])
            if bad:
                raise SystemExit(f'view pose not reachable: {bad}')
            ex.run('\n'.join(['q = get_actual_tcp_pose()', f'if q[2] < {z:.5f}:', f'  movel(p[q[0], q[1], {z:.5f}, {o}], a=0.3, v=0.03)',
                              'end', f'movel({p(*VIEW, z)}, a=0.3, v=0.05)']), timeout=90)
        res, im = find(cfg)
        d = run_dir('rack_find')
        import cv2
        cv2.imwrite(str(d / 'fixed.png'), im)
        (d / 'result.json').write_text(json.dumps(res, indent=1))
        print(json.dumps(res, indent=1), '\nsaved', d)
    elif a.cmd == 'pick':
        if not a.slot or not a.part:
            raise SystemExit('pick R<k> --part <casing>')
        pick(a, grip, rv, rv_t)
    else:
        ap.print_help()


if __name__ == '__main__':
    main()
