#!/usr/bin/env python3
"""Component storage table for the UR3e + Hand-E: a base fixed to the table and square modules
standing on four toes, one module per pin spacing (1, 2 and 4) plus one that cradles the RJ45 grip.

Frames (mm). Base frame: origin at the front-left corner of the base top, x along the front
edge, y away from it, z up; slots.json gives every pick in this frame. Module frame: origin at
the centre of the plate top, axes as the base.

Every module has the same four toes and every cell the same four sockets:
  A  front left   6 mm pin in a round hole    -> x, y
  B  front right  6 mm pin in a slot along x  -> yaw
  C, D  back      8 mm pins in loose holes    -> nothing; the M3 hold-down screws run through them
  all four shoulders sit on the base top      -> z, roll, pitch
The 8 mm pins don't fit A or B, so a module only goes in one way round.

Parts stand in straight rows of holes: pin_spacing s puts the holes s * 2.54 mm apart, one row
per part, centred on its slot. Leads run along x and the fingertips close along y, so every pick
uses the same wrist orientation and the same height; only x and y change. Bodies rest on the
plate top. Jumpers go into blind holes, so their bar stands clear of the plate for the tips.
The fingertips are the Hand-E ones the RJ45 grip was made for (24 x 8 mm), so one set of
fingers picks both.

Needs manifold3d (installed into ./.deps):
    uv pip install --python python3 --target .deps manifold3d numpy
Usage: python3 component_table.py   -> *.stl in print orientation, slots.json
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".deps"))
sys.path.insert(1, os.path.join(HERE, "..", "rj45_grip"))
from manifold3d import CrossSection, JoinType, Manifold, OpType  # noqa: E402

import rj45_grip as rj  # noqa: E402  the cradle is cut from the grip's own outline

COMPONENTS = os.path.join(HERE, "..", "..", "..", "src", "circuit_mapper", "example_components.json")

# ---- layout ----
PITCH = 2.54         # protoboard grid
CELL = 84.0          # module grid pitch on the base
MOD = 82.0           # module plate size (1 mm gap to each neighbour)
NX = NY = 2          # cells
BORDER = 4.0         # base rim outside the cells
CELLS = {"S1": (0, 0), "S2": (1, 0), "S4": (0, 1), "RJ45": (1, 1)}  # RJ45 on the right: cable leaves the board
CORNER_R = 3.0

# ---- base ----
BASE_T = 10.0        # thickness
WIN = 50.0           # open window under each module
M4_D, M4_HEAD = 4.5, 9.0   # countersunk screws into the table

# ---- module toes and sockets ----
PLATE_T = 8.0        # plate thickness = lead guide length; 4 mm let parts lean up to 15 degrees
LEAD_MAX = 10.0      # longest lead below a part's body
H_MOD = 14.0         # plate top above the base top; leaves the lead ends 4 mm clear of the base
TOE_U = 35.0         # toes at (+-TOE_U, +-TOE_U) from the module centre
TOE_D = 12.0         # toe column; its shoulder ring must reach past every socket lead-in
PIN_L = 5.0          # pin length below the shoulder
PIN_AB = 6.0         # locating pins A and B
PIN_CD = 8.0         # key pins C and D, hollow for the M3 screws
PIN_CLR = 0.1        # A in its hole, B across its slot (radial); tune with the coupon
CD_CLR = 0.6         # C and D never touch their holes
SLOT_EXTRA = 4.0     # slot B is this much longer than its pin, along x
PIN_CHAMFER = 0.8    # lead-in on the pin tips
SOCKET_D = PIN_L + 0.8
SOCKET_CHAMFER = 0.6
SKIRT_T, SKIRT_H = 2.0, 0.0          # rim under the plate edge; the 8 mm plate is stiff enough without
M3_D, M3_HEAD_D, M3_HEAD_H = 3.4, 6.2, 3.2
M3_NUT_AF, M3_NUT_H = 5.8, 2.6
TOES = {"A": (-TOE_U, -TOE_U), "B": (TOE_U, -TOE_U), "C": (TOE_U, TOE_U), "D": (-TOE_U, TOE_U)}
PIN_DIA = {"A": PIN_AB, "B": PIN_AB, "C": PIN_CD, "D": PIN_CD}

# ---- lead holes ----
HOLE_D = 1.1         # leads are 0.45-0.65 mm; small FDM holes print ~0.2 mm under size
CSK_D = 1.7          # 45 degree countersink at the top, kept shallow so the bore stays long
BLIND_D = 4.5        # jumper hole depth: the legs bottom out and the bar stands clear (legs >= 7 mm)
ROW_Y0, ROW_PITCH = -30.0, 20.0      # slot rows along y; the open 8 mm tips fit between them
ROW_X0 = 1.0         # rows sit 1 mm right of centre, clear of the title on the left edge
ENGRAVE, STROKE = 0.4, 0.8

# ---- Hand-E fingertips, the same ones the RJ45 grip is made for ----
TIP_T = rj.FINGERTIP_T   # thickness along the closing direction (y)
TIP_W = rj.FINGERTIP_W   # width along the leads (x)
HANDE_STROKE = 50.0      # widest opening between the tips
BODY_W = 7.0         # widest body across a row (6.3 mm electrolytic, 5.8 mm LED flange)
TIP_OPEN = 2.5       # free gap on each side of a body while the tips come down
TIP_Z = 2.0          # tip bottom above the plate top: above a 5 mm LED's 1-1.5 mm flange
APPROACH = 30.0      # straight-down approach from this far above the grip

# ---- RJ45 module ----
RJ_T = 10.0          # thick plate: the cradle is cut into it, so it prints top-down too
RJ_X = -37.0         # plug tip x in the module frame; the cable leaves at +x
RJ_CLR = 1.0         # in-plane pocket clearance: the H and its flares centre the grip on the tips, the cradle only sets z and tilt
RJ_LEADIN = 1.2      # 45 degree chamfer at the pocket top, so a plug set down a bit off slides in
CABLE_D = 6.0        # cable diameter (measure); the clip at the edge snaps over it
CLIP_L = 8.0
CLIP_Z = -1.5        # cable centre in the clip, 1.5 mm below the plate top
CABLE_ZC = CABLE_D / 2   # cable centre above the shell bottom, lying on the cover
RJ_OPEN = 2 * (rj.H_W / 2 + rj.FLARE_DY) + 2   # tips open outside the flares, then close
RJ_Z = CLIP_Z - CABLE_ZC   # leg bottoms (grip z = 0) below the plate top: the cable runs level into the clip

# rows front to back: (x pitch, kinds); a kind is a pin count, or "J" for a jumper
LAYOUT = {
# in-row pitch >= TIP_W / 2 + neighbour half-length + 1: the 24 mm tips close over the row
    "S1": (1, [(19, [3, 3, 3, 3]), (20, [4, 2, 2, 2]), (19, [2, 2, 2, 2]), (19, [2, 2, 2, "J"])]),
    "S2": (2, [(24, [4, 4, 4]), (19, [2] * 4), (19, [2] * 4), (19, ["J"] * 4)]),
    "S4": (4, [(22, [2] * 3), (22, [2] * 3), (22, ["J"] * 3), (22, ["J"] * 3)]),
}

BASE_W = 2 * BORDER + NX * CELL

# stroke font on a 4 x 6 grid
FONT = {
    "0": [[(0, 0), (4, 0), (4, 6), (0, 6), (0, 0)]],
    "1": [[(1, 5), (2, 6), (2, 0)], [(1, 0), (3, 0)]],
    "2": [[(0, 6), (4, 6), (4, 3), (0, 3), (0, 0), (4, 0)]],
    "3": [[(0, 6), (4, 6), (4, 0), (0, 0)], [(1, 3), (4, 3)]],
    "4": [[(0, 6), (0, 3), (4, 3)], [(3, 6), (3, 0)]],
    "5": [[(4, 6), (0, 6), (0, 3), (4, 3), (4, 0), (0, 0)]],
    "6": [[(4, 6), (0, 6), (0, 0), (4, 0), (4, 3), (0, 3)]],
    "7": [[(0, 6), (4, 6), (1.5, 0)]],
    "8": [[(0, 0), (4, 0), (4, 6), (0, 6), (0, 0)], [(0, 3), (4, 3)]],
    "9": [[(4, 3), (0, 3), (0, 6), (4, 6), (4, 0), (0, 0)]],
    "S": [[(4, 5), (3, 6), (1, 6), (0, 5), (0, 4), (1, 3), (3, 3), (4, 2), (4, 1), (3, 0), (1, 0), (0, 1)]],
    "J": [[(1, 6), (4, 6)], [(3, 6), (3, 1), (2, 0), (1, 0), (0, 1)]],
    "R": [[(0, 0), (0, 6), (3, 6), (4, 5), (4, 4), (3, 3), (0, 3)], [(2, 3), (4, 0)]],
    ".": [[(2, 0), (2, 0.01)]],
    " ": [],
}


def seg(r):
    return int(min(64, max(16, math.ceil(2 * math.pi * r / 0.6))))


def box(x0, x1, y0, y1, z0, z1):
    return Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def cyl(x, y, r, z0, z1, r1=None):
    """Vertical cylinder at (x, y), or a cone with top radius r1."""
    r1 = r if r1 is None else r1
    return Manifold.cylinder(z1 - z0, r, r1, seg(max(r, r1))).translate([x, y, z0])


def prism(cs, z0, z1):
    return cs.extrude(z1 - z0).translate([0, 0, z0])


def union(parts):
    return Manifold.batch_boolean(parts, OpType.Add)


def rrect(x0, x1, y0, y1, r):
    c = CrossSection.circle(r, seg(r))
    return CrossSection.batch_hull([c.translate([x, y]) for x in (x0 + r, x1 - r) for y in (y0 + r, y1 - r)])


def stroke(p, q, w=STROKE):
    c = CrossSection.circle(w / 2, 12)
    return CrossSection.batch_hull([c.translate(p), c.translate(q)])


def text(s, x, y, h):
    """Stroke text centred on (x, y), cap height h."""
    k = h / 6
    adv = 5.6 * k
    x0, y0 = x - (adv * len(s) - 1.6 * k) / 2, y - h / 2
    caps = []
    for i, ch in enumerate(s):
        for line in FONT[ch]:
            pts = [(x0 + i * adv + px * k, y0 + py * k) for px, py in line]
            caps += [stroke(p, q) for p, q in zip(pts, pts[1:])]
    return CrossSection.batch_boolean(caps, OpType.Add)


def engrave(cs):
    return prism(cs, -ENGRAVE, 0.2)


def cell_xy(i, j):
    return BORDER + CELL * (i + 0.5), BORDER + CELL * (j + 0.5)


def slots():
    """Every slot in its module frame: id, spacing, pin count, jumper flag, centre and holes."""
    out = []
    for name, (s, rows) in LAYOUT.items():
        p, k = s * PITCH, 0
        for r, (dx, kinds) in enumerate(rows):
            y = ROW_Y0 + r * ROW_PITCH
            for i, kind in enumerate(kinds):
                k += 1
                x = ROW_X0 + (i - (len(kinds) - 1) / 2) * dx
                n = 2 if kind == "J" else kind
                holes = [(x + (j - (n - 1) / 2) * p, y) for j in range(n)]
                out.append({"id": f"{name}-{k:02d}", "module": name, "spacing": s, "pins": n,
                            "jumper": kind == "J", "x": x, "y": y, "holes": holes})
    return out


def lead_hole(x, y, depth):
    r, rc = HOLE_D / 2, CSK_D / 2
    return [cyl(x, y, r, -depth, 0.5), cyl(x, y, r, -(rc - r), 0.2, rc + 0.2)]


def module_body(t, skirt):
    """Plate of thickness t on the four toes; returns the solid and the hold-down screw cuts."""
    parts = [prism(rrect(-MOD / 2, MOD / 2, -MOD / 2, MOD / 2, CORNER_R), -t, 0)]
    if skirt:
        ring = rrect(-MOD / 2, MOD / 2, -MOD / 2, MOD / 2, CORNER_R) - rrect(
            -MOD / 2 + SKIRT_T, MOD / 2 - SKIRT_T, -MOD / 2 + SKIRT_T, MOD / 2 - SKIRT_T, 1.0)
        parts.append(prism(ring, -t - SKIRT_H, -t + 0.01))
    for k, (x, y) in TOES.items():
        r = PIN_DIA[k] / 2
        parts += [cyl(x, y, TOE_D / 2, -H_MOD, -t + 0.01),
                  cyl(x, y, r, -H_MOD - PIN_L + PIN_CHAMFER, -H_MOD + 0.01),
                  cyl(x, y, r - PIN_CHAMFER, -H_MOD - PIN_L, -H_MOD - PIN_L + PIN_CHAMFER, r)]
    cuts = []
    for k in "CD":
        x, y = TOES[k]
        cuts += [cyl(x, y, M3_D / 2, -H_MOD - PIN_L - 1, 1), cyl(x, y, M3_HEAD_D / 2, -M3_HEAD_H, 1)]
    return union(parts), cuts


def title(name, s=None):
    """Module name: along the left edge on the spacing modules (the front row needs the front
    band), across the front on the RJ45 module, whose plug tip reaches the left edge."""
    if s is None:
        return engrave(text(name, 0, -MOD / 2 + 5.5, 5.0))
    return engrave(text(f"{name} {s * PITCH:.2f}", 0, 0, 4.0).rotate(90).translate([-MOD / 2 + 3.5, 0]))


def module_s(name, slot_list):
    s = LAYOUT[name][0]
    body, cuts = module_body(PLATE_T, skirt=True)
    marks = []
    for sl in slot_list:
        depth = BLIND_D if sl["jumper"] else PLATE_T + 1
        for x, y in sl["holes"]:
            cuts += lead_hole(x, y, depth)
        x1, y1 = sl["holes"][0]                       # pin 1 sits at -x, marked by a triangle
        marks.append(CrossSection([[(x1 - 3.3, y1 - 0.9), (x1 - 1.6, y1), (x1 - 3.3, y1 + 0.9)]]))
        marks.append(text(sl["id"].split("-")[1].lstrip("0"), sl["x"], sl["y"] - 6.5, 3.0))
    cuts += [engrave(CrossSection.batch_boolean(marks, OpType.Add)), title(name, s)]
    return body - union(cuts)


def module_rj45():
    """Plate with the grip cradle cut in: the leg bottoms sit on the pocket floor and the legs,
    flares and cover locate it; the plug tip and the cable lie in channels, the cable end in a clip."""
    body, cuts = module_body(RJ_T, skirt=False)
    xs0, xs1, ya = rj.SHELL_X0, rj.SHELL_X1, rj.YA
    outline = CrossSection.batch_boolean([CrossSection([p]) for p in rj.handle_outline()]
                                         + [CrossSection.square([xs1 - xs0, 2 * ya]).translate([xs0, -ya])],
                                         OpType.Add).translate([RJ_X, 0])
    pieces = [CrossSection([p]) for p in rj.handle_outline()] + [CrossSection.square([xs1 - xs0, 2 * ya]).translate([xs0, -ya])]
    lead = [Manifold.batch_hull([prism(pc.translate([RJ_X, 0]).offset(RJ_CLR, JoinType.Round), -RJ_LEADIN, -RJ_LEADIN + 0.01),
                                 prism(pc.translate([RJ_X, 0]).offset(RJ_CLR + RJ_LEADIN, JoinType.Round), 0, 1)]) for pc in pieces]
    cuts += [prism(outline.offset(RJ_CLR, JoinType.Round), RJ_Z, 1)] + lead + [   # 45 degree lead-in on each convex piece
             box(xs0 + RJ_X - RJ_CLR, xs1 + RJ_X + RJ_CLR, -ya - RJ_CLR, ya + RJ_CLR, -RJ_T - 1, RJ_Z + 0.01),  # cover, cut through
             box(rj.ZIP_X0 - 1.5 + RJ_X, rj.ZIP_X0 + rj.ZIP_W + 1.5 + RJ_X, -13, 13, -RJ_T - 1, 1),          # zip-tie head anywhere
             box(-1 + RJ_X, xs0 + RJ_X, -rj.PLUG_W / 2 - 0.6, rj.PLUG_W / 2 + 0.6, -RJ_T - 1, 1)]          # plug tip, cut through
    rc, clip_x0 = CABLE_D / 2 + 0.1, MOD / 2 - CLIP_L
    cuts += [box(xs1 + RJ_X - 1, clip_x0 + 0.01, -CABLE_D / 2 - 0.5, CABLE_D / 2 + 0.5, CLIP_Z - rc, 1),
             Manifold.cylinder(CLIP_L + 1, rc, rc, seg(rc)).rotate([0, 90, 0]).translate([clip_x0, 0, CLIP_Z]),
             title("RJ45")]
    return body - union(cuts)


def socket(points, r, depth):
    """Hole (or slot, for two points) with a 45 degree lead-in at the top."""
    hole = Manifold.batch_hull([cyl(x, y, r, -depth, 1) for x, y in points])
    lead = Manifold.batch_hull([cyl(x, y, r, -SOCKET_CHAMFER, 0.01, r + SOCKET_CHAMFER + 0.01) for x, y in points])
    return [hole, lead]


def base():
    m = prism(rrect(0, BASE_W, 0, BASE_W, CORNER_R), -BASE_T, 0)
    cuts = []
    for i in range(NX):
        for j in range(NY):
            cx, cy = cell_xy(i, j)
            cuts.append(prism(rrect(cx - WIN / 2, cx + WIN / 2, cy - WIN / 2, cy + WIN / 2, 4), -BASE_T - 1, 1))
            for k, (u, v) in TOES.items():
                x, y = cx + u, cy + v
                if k == "A":
                    cuts += socket([(x, y)], PIN_AB / 2 + PIN_CLR, SOCKET_D)
                elif k == "B":
                    cuts += socket([(x - SLOT_EXTRA / 2, y), (x + SLOT_EXTRA / 2, y)], PIN_AB / 2 + PIN_CLR, SOCKET_D)
                else:
                    cuts += socket([(x, y)], PIN_CD / 2 + CD_CLR, SOCKET_D)
                    nut = CrossSection.circle(M3_NUT_AF / math.sqrt(3), 6).translate([x, y])
                    cuts += [cyl(x, y, M3_D / 2, -BASE_T - 1, 0), prism(nut, -BASE_T - 1, -BASE_T + M3_NUT_H)]
    for x, y in base_screws():
        cuts += [cyl(x, y, M4_D / 2, -BASE_T - 1, 1), cyl(x, y, M4_D / 2, -(M4_HEAD - M4_D) / 2, 0.2, M4_HEAD / 2 + 0.2)]
    cuts.append(engrave(stroke((2.2, 2.2), (12, 2.2)) + stroke((2.2, 2.2), (2.2, 12))))  # datum corner
    return m - union(cuts)


def base_screws():
    """Countersunk fixing screws: in the frame in front of and behind the windows, and in the cross
    ribs at the cell centre lines, away from the sockets that crowd the cell corners."""
    rim, mid = (BORDER + (CELL - WIN) / 2) / 2, BORDER + CELL
    xs = [cell_xy(i, 0)[0] for i in range(NX)]
    ys = [cell_xy(0, j)[1] for j in range(NY)]
    return ([(x, rim) for x in xs] + [(x, BASE_W - rim) for x in xs]
            + [(x, mid) for x in xs] + [(mid, y) for y in ys])


def coupons():
    """Two quick prints to tune the fits: lead-hole pairs 0.9-1.3 mm with one A pin, and a block
    with A sockets at four clearances."""
    plate = prism(rrect(-30, 30, -9, 9, 2), -PLATE_T, 0)
    r = PIN_AB / 2
    plate += union([cyl(22, 0, TOE_D / 2, -PLATE_T - 4, -PLATE_T + 0.01),
                    cyl(22, 0, r, -PLATE_T - 4 - PIN_L + PIN_CHAMFER, -PLATE_T - 4 + 0.01),
                    cyl(22, 0, r - PIN_CHAMFER, -PLATE_T - 4 - PIN_L, -PLATE_T - 4 - PIN_L + PIN_CHAMFER, r)])
    cuts, labels = [], []
    for i, d in enumerate((0.9, 1.0, 1.1, 1.2, 1.3)):
        x = -24 + i * 9
        for dx in (-PITCH / 2, PITCH / 2):
            cuts.append(cyl(x + dx, 2.5, d / 2, -PLATE_T - 1, 0.5))
            cuts.append(cyl(x + dx, 2.5, d / 2, -(CSK_D - d) / 2, 0.2, CSK_D / 2 + 0.2))
        labels.append(text(str(round(d * 10)), x, -4.5, 3.0))
    plate -= union(cuts + [engrave(CrossSection.batch_boolean(labels, OpType.Add))])
    block = prism(rrect(-30, 30, -9, 9, 2), -BASE_T, 0)
    cuts, labels = [], []
    for i, c in enumerate((0.10, 0.15, 0.20, 0.25, 0.30)):
        x = -24 + i * 12
        cuts += socket([(x, 2)], r + c, SOCKET_D)
        labels.append(text(str(round(c * 100)), x, -5.5, 3.0))
    block -= union(cuts + [engrave(CrossSection.batch_boolean(labels, OpType.Add))])
    return plate, block


def part_kind(c):
    s = c.get("pin_spacing", 1)
    s = 3 if s == "center_gap" else int(s)      # E to F across the channel is 7.62 mm = 3 pitches
    return s, "J" if c["type"] == "jumper" else len(c["pins"])


# leads -x to +x as they sit in the row (flat face of a TO-92 toward -y); parts not listed keep the JSON order
PIN_ORDER = {
    "bjt": ["emitter", "base", "collector"],                                    # generic EBC, e.g. 2N3904 / 2N3906
    "rgb_led": ["red_anode", "common_cathode", "green_anode", "blue_anode"],    # common 5 mm, longest lead 2nd
    "switch": {"SPDT": ["throw_1", "common", "throw_2"]},
    "potentiometer": ["end_1", "wiper", "end_2"],
}


def pin_order(c):
    o = PIN_ORDER.get(c["type"])
    if isinstance(o, dict):
        o = o.get(c["parameters"].get("contact_form"))
    return o if o and sorted(o) == sorted(c["pins"]) else c["pins"]


def assign(slot_list):
    """Default loading plan: each example component in the first free slot that fits it."""
    with open(COMPONENTS) as f:
        comps = json.load(f)["components"]
    unplaced = []
    for c in comps:
        s, kind = part_kind(c)
        for sl in slot_list:
            if sl.get("part") is None and sl["module"] == f"S{s}" and (sl["jumper"] if kind == "J" else
                                                                         not sl["jumper"] and sl["pins"] == kind):
                sl["part"], sl["part_name"] = c["part_id"], c["name"]
                sl["pin_names"] = pin_order(c)
                break
        else:
            unplaced.append(c["part_id"])
    return unplaced


def place(m, name):
    cx, cy = cell_xy(*CELLS[name])
    return m.translate([cx, cy, H_MOD])


def checks(modules, b, slot_list):
    for name, m in list(modules.items()) + [("base", b)]:
        assert m.status().name == "NoError", name
    for name, m in modules.items():
        assert (place(m, name) ^ b).volume() < 1e-3, f"{name} collides with the base"
        for turn in (90, 180, 270):                   # the fat C/D pins must block a wrong turn
            assert (place(m.rotate([0, 0, turn]), name) ^ b).volume() > 1.0, f"{name} fits turned {turn}"
    assert H_MOD - LEAD_MAX >= 3.0, "leads would touch the base"
    rjm = modules["RJ45"]
    grip = (rj.shell() + rj.cover()).translate([RJ_X, 0, RJ_Z])
    assert (grip ^ rjm).volume() < 1e-3, "RJ45 grip collides with its cradle"
    legs = union([prism(CrossSection([p]), -0.05, 0) for p in rj.handle_outline()]).translate([RJ_X, 0, RJ_Z])
    assert (legs ^ rjm).volume() > 1.0, "RJ45 legs have nothing to sit on"
    for k, d in PIN_DIA.items():                      # each shoulder needs a ring of base top to land on
        clr = PIN_CLR if k in "AB" else CD_CLR
        assert TOE_D / 2 - (d / 2 + clr + SOCKET_CHAMFER) >= 0.5, f"toe {k} shoulder has nothing to sit on"
        x, y = TOES[k]
        ring = cyl(x, y, TOE_D / 2, -H_MOD - 0.05, -H_MOD) - cyl(x, y, d / 2 + clr + SOCKET_CHAMFER + 0.1, -H_MOD - 1, 0)
        assert (place(ring, "S1") ^ b).volume() > 0.5, f"toe {k} shoulder doesn't touch the base"
    assert (place(modules["S1"], "S1") ^ b.translate([0, 0, 0.05])).volume() > 1.0, "shoulders don't reach the base top"
    # RJ45: tips open outside the flares at the pick height must miss the grip and the cradle
    tz = RJ_Z + rj.ZT + 1.0
    xn = (rj.NOTCH_X0 + rj.NOTCH_X1) / 2 + RJ_X
    assert RJ_OPEN <= HANDE_STROKE
    for sgn in (1, -1):
        tip = box(xn - TIP_W / 2, xn + TIP_W / 2, *sorted((sgn * RJ_OPEN / 2, sgn * (RJ_OPEN / 2 + TIP_T))), tz, tz + 20)
        assert (tip ^ (grip + rjm)).volume() < 1e-3, "open RJ45 tips hit the grip or cradle"
    # fingertips must miss every other body on the board and the RJ45 grip, open and while closing:
    # the whole band from the open tips in to the part is swept
    gap = BODY_W / 2 + TIP_OPEN
    assert 2 * gap <= HANDE_STROKE, "Hand-E can't open that far"

    def board(sl):
        cx, cy = cell_xy(*CELLS[sl["module"]])
        return cx + sl["x"], cy + sl["y"], [cx + h[0] for h in sl["holes"]]
    cx, cy = cell_xy(*CELLS["RJ45"])
    rj_box = (cx + RJ_X - 1, cx + rj.SHELL_X1 + RJ_X + 1, cy - rj.H_W / 2 - rj.FLARE_DY, cy + rj.H_W / 2 + rj.FLARE_DY)
    for sl in slot_list:
        x, y, _ = board(sl)
        tips = [(x - TIP_W / 2, x + TIP_W / 2, y - gap - TIP_T, y + gap + TIP_T)]
        bodies = [rj_box]
        for other in slot_list:
            if other is not sl:
                ox, oy, hx = board(other)
                bodies.append((min(hx) - 3, max(hx) + 3, oy - BODY_W / 2, oy + BODY_W / 2))
        for tx0, tx1, ty0, ty1 in tips:
            for bx0, bx1, by0, by1 in bodies:
                assert tx1 <= bx0 or bx1 <= tx0 or ty1 <= by0 or by1 <= ty0, f"tips at {sl['id']} hit a neighbour"


def slots_json(slot_list, unplaced):
    rot = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]
    out_slots = []
    for sl in slot_list:
        cx, cy = cell_xy(*CELLS[sl["module"]])
        x, y = cx + sl["x"], cy + sl["y"]
        z = H_MOD + TIP_Z
        out_slots.append({
            "id": sl["id"], "module": sl["module"], "pin_spacing": sl["spacing"], "pins": sl["pins"],
            "holes": "blind" if sl["jumper"] else "through",
            "part": sl.get("part"), "part_name": sl.get("part_name"), "pin_names": sl.get("pin_names"),
            "hole_xy": [[round(cx + hx, 3), round(cy + hy, 3)] for hx, hy in sl["holes"]],
            "pick": {"x": round(x, 3), "y": round(y, 3), "z": round(z, 3)},
            "approach_z": round(z + APPROACH, 3),
            "open_mm": BODY_W + 2 * TIP_OPEN,
        })
    cx, cy = cell_xy(*CELLS["RJ45"])
    notch = (rj.NOTCH_X0 + rj.NOTCH_X1) / 2 + RJ_X
    rz = H_MOD + RJ_Z + rj.ZT + 1.0
    data = {
        "units": "mm",
        "frame": "origin at the front-left corner of the base top; x along the front edge, y away from it, z up",
        "tool": {
            "R_board_tool": rot, "rotvec_board_tool": [round(math.pi / math.sqrt(2), 4)] * 2 + [0.0],
            "note": "tool z points down and tool x (the Hand-E jaw axis) runs along board y; every pick uses it",
        },
        "pick_point": f"centre between the fingertip faces, at the tip bottom, {TIP_Z} mm above the plate top",
        "tip_bottom_z": H_MOD + TIP_Z,
        "plate_top_z": H_MOD,
        "datums": {
            "top": [[30, 2.5, 0], [BASE_W - 30, 2.5, 0], [2.5, BASE_W / 2, 0]],
            "front_face_y0": [[10, 0, -BASE_T / 2], [BASE_W - 10, 0, -BASE_T / 2]],
            "left_face_x0": [[0, BASE_W / 2, -BASE_T / 2]],
            "right_face_x": [[BASE_W, BASE_W / 2, -BASE_T / 2]],
            "back_face_y": [[10, BASE_W, -BASE_T / 2], [BASE_W - 10, BASE_W, -BASE_T / 2]],
            "note": (f"prints shrink 0.2-0.4 %: measure the base span (CAD {BASE_W:.0f} mm in x and y) by touch or "
                     "calipers and scale every x, y about the origin by measured / CAD"),
        },
        "modules": {n: {"cell": list(c), "centre": [*cell_xy(*c), H_MOD]} for n, c in CELLS.items()},
        "slots": out_slots,
        "rj45": {
            "pick": {"x": round(cx + notch, 3), "y": round(cy, 3), "z": round(rz, 3)},
            "approach_z": round(rz + APPROACH, 3),
            "grip_span_mm": rj.GRIP_SPAN,
            "open_mm": RJ_OPEN,
            "plug_axis": "+x from the tip to the cable", "tip_xy": [round(cx + RJ_X, 3), round(cy, 3)],
            "note": "descend with the tips open to open_mm (outside the flares), then close: the flares and notches centre the grip; tip bottom 1 mm above the shell top",
        },
        "unplaced": unplaced,
    }
    with open(os.path.join(HERE, "slots.json"), "w") as f:
        json.dump(data, f, indent=1)
    return data


if __name__ == "__main__":
    slot_list = slots()
    unplaced = assign(slot_list)
    modules = {n: module_s(n, [s for s in slot_list if s["module"] == n]) for n in LAYOUT}
    modules["RJ45"] = module_rj45()
    b = base()
    checks(modules, b, slot_list)
    data = slots_json(slot_list, unplaced)
    # STL frame = print orientation: modules plate top down on the bed, base bottom down
    n = {"base": rj.write_stl(os.path.join(HERE, "base.stl"), b, (0, 0, BASE_T), "component_table_base")}
    for name, m in modules.items():
        n[name] = rj.write_stl(os.path.join(HERE, f"module_{name.lower()}.stl"), m.rotate([180, 0, 0]), (0, 0, 0),
                               f"component_table_{name.lower()}")
    plate, block = coupons()
    rj.write_stl(os.path.join(HERE, "coupon_holes.stl"), plate.rotate([180, 0, 0]), (0, 0, 0), "coupon_holes")
    rj.write_stl(os.path.join(HERE, "coupon_sockets.stl"), block, (0, 0, BASE_T), "coupon_sockets")
    used = sum(1 for s in slot_list if s.get("part"))
    print(f"base.stl: {n['base']} tris, {BASE_W:.0f} x {BASE_W:.0f} x {BASE_T:.0f} mm, {b.volume() / 1000:.0f} cm3")
    for name, m in modules.items():
        print(f"module_{name.lower()}.stl: {n[name]} tris, {MOD:.0f} x {MOD:.0f} x {H_MOD + PIN_L:.0f} mm, "
              f"{m.volume() / 1000:.1f} cm3")
    print(f"slots.json: {len(slot_list)} slots, {used} loaded from example_components.json, unplaced {unplaced}")
    print(f"picks at z {H_MOD + TIP_Z:.1f}, RJ45 at {data['rj45']['pick']}; M3 x 20 hold-down screws, "
          f"lead ends {H_MOD - LEAD_MAX:.0f} mm above the base")
