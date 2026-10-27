"""Grasp calibration: after every grasp, measure where the part's legs sit under the fingertip.

Why (2026-10-27): an LED held ~2 mm off-centre went into j19/j21 instead of j20/j22. The peck
search accepts ANY hole pair, so a grasp offset over half a pitch (1.27 mm) lands one hole off
and still reports success (skills_lib/breadboard/grasp-offset-half-pitch). Error sources: the
part sits imprecisely in the fingers, and the custom finger is not snug on the Hand-E (tilt).

Measurement (coarse is enough, < +-1.27 mm; the hole drop gives the fine position): the
casing's leading face touches a known vertical face at <= 1.5 N, 2 mm/s. Reference = the
board's row-1 end face (configs/breadboard/board_pose.yaml row1_end_x_mm), approached from +x
with the casing bottom ~1 mm below the board top, legs hanging free outside the board.
The legs' midpoint in the casing comes from casings.json (exit_x, symmetric), so
    legs_x - tcp_x = face_x + width_x / 2 - tcp_x_at_contact.
Only x (the leg direction, along the rows) is measured for now; y is the board offset.
TODO: two heights for the finger tilt; y against the long edge; the camera as a cross-check.
"""
import numpy as np

F_SIDE, V_SIDE = 1.5, 0.002
GAP = 0.010            # start this far from the casing face
PLATE_CLEAR = 0.0008   # leg tips at least this far above the plate
OVERLAP_MIN = 0.0005   # casing bottom at least this far below the board top


def program(rv, face_x, y, top, plate, spec, leg_len, safe=0.020, strip=None):
    """URScript: casing face against the end face from +x. Returns (script, z, start_x); z is the
    planning estimate. With `strip` (x, y of a hole-free strip) the legs are measured there first
    and the descent height is computed in the program (a new grasp: leg length unknown)."""
    o = ', '.join(f'{v:.5f}' for v in rv)
    Pa = lambda x, yy, z: f'p[{x if isinstance(x, str) else f"{x:.5f}"}, {yy:.5f}, {z}, {o}]'  # noqa: E731
    P = lambda x, z: Pa(x, y, z)  # noqa: E731
    half = spec['width_x'] / 2000
    z = max(top - 0.001, plate + leg_len + PLATE_CLEAR)
    if strip is None and top - z < OVERLAP_MIN:
        raise ValueError(f'legs too long ({leg_len * 1000:.1f} mm) to touch the end face above the plate')
    x0 = face_x + half + 0.003 + GAP          # +3 mm for a grasp offset up to -3 mm
    side = [f'movel({P(x0, f"{top + safe:.5f}")}, a=0.3, v=0.03)',
            # down beside the board end, guarded: a contact here means something is in the way
            f'h = guarded_axial([0, 0, -1], {top + safe:.5f} - zc, 2.0, 0.005)',
            'if h:', '  textmsg("BLOCKED down")', f'  movel({P(x0, f"{top + safe:.5f}")}, a=0.3, v=0.02)',
            'else:',
            f'  h = guarded_axial([-1, 0, 0], {GAP + 0.010:.4f}, {F_SIDE}, {V_SIDE})',
            '  if h:', '    textmsg("C side ", get_actual_tcp_pose())', '  else:', '    textmsg("MISS side")', '  end',
            '  back_off([-1, 0, 0], 0.002)',
            '  q = get_actual_tcp_pose()', f'  movel({P("q[0]", f"{top + safe:.5f}")}, a=0.3, v=0.02)',
            'end']
    s = ['textmsg("grasp_calib start")']
    if strip is None:
        s += [f'zc = {z:.5f}'] + side
    else:
        sx, sy = strip
        s += [f'movel({Pa(sx, sy, f"{top + safe:.5f}")}, a=0.3, v=0.03)',
              f'h = guarded_axial([0, 0, -1], {safe + 0.004:.4f}, 2.0, 0.005)',
              'if not h:', '  textmsg("ABORT no strip contact")', f'  movel({Pa(sx, sy, f"{top + safe:.5f}")}, a=0.3, v=0.03)',
              'else:',
              'zs = get_actual_tcp_pose()[2]', 'textmsg("strip ", zs)',
              'back_off([0, 0, -1], 0.003)',
              f'movel({Pa(sx, sy, f"{top + safe:.5f}")}, a=0.3, v=0.03)',
              # casing bottom 1 mm below the top, unless the legs would come closer than PLATE_CLEAR to the plate
              f'zc = {top - 0.001:.5f}',
              f'if {plate + PLATE_CLEAR:.5f} + (zs - {top:.5f}) > zc:', f'  zc = {plate + PLATE_CLEAR:.5f} + (zs - {top:.5f})', 'end',
              f'if {top:.5f} - zc < {OVERLAP_MIN}:', '  textmsg("ABORT legs too long for the end face")',
              'else:'] + side + ['end', 'end']
    s += ['textmsg("grasp_calib done")']
    return '\n'.join(s), z, x0


def parse(msgs, face_x, spec):
    """legs_x - tcp_x (m, base x) from the side contact, or None."""
    m = next((m for m in msgs if m.startswith('C side ')), None)
    if m is None:
        return None
    tcp_x = float(m.split('p[')[1].split(',')[0])
    return face_x + spec['width_x'] / 2000 - tcp_x


def measure(ex, gripper, part):
    """Kept from the planned API; the board-end touch is driven from breadboard.place_probe --calib."""
    raise NotImplementedError('use python -m breadboard.place_probe <holes> --part <casing> --calib')


def corrected_tcp(gripper, offset):
    """gripper['tcp_pins'] shifted by the measured offset (flange frame)."""
    t = list(gripper['tcp_pins'])
    return [t[0] + offset[0], t[1] + offset[1], t[2] + offset[2]] + t[3:]


def legs_offset_ok(found_tcp_x, target_x, board_dx, g, pitch=0.00254):
    """After the drop: the legs are within half a pitch of the target pair (else one hole off)."""
    legs = found_tcp_x + g
    return abs(legs - (target_x + board_dx)) < pitch / 2, float(legs - (target_x + board_dx))
