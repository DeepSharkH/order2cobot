#!/usr/bin/env python3
"""Glue-together casings that turn every part into the same kind of block for the Hand-E:
a 20 mm tall box split through the lead plane, with the leads leaving the bottom on the 2.54 mm
grid. The jumper casing is the 5 mm pitch design in this folder with its two pin channels moved
to pin_spacing * 2.54 mm; the part casings use the same box with a pocket for the body and a
channel per lead from the body down to its exit hole.

Frame (mm): x along the row of leads, y across the split (halves at y < 0 and y > 0), z up from
the casing bottom. Lead exits sit at z = 0.

Needs manifold3d (installed into ../component_table/.deps):
    uv pip install --python python3 --target ../component_table/.deps manifold3d numpy
Usage: python3 casings.py   -> casing_<part>_s<spacing>_half<1|2>.stl (outer face on the bed), casings.json
"""
import json
import math
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "component_table", ".deps"))
import numpy as np  # noqa: E402
from manifold3d import CrossSection, Manifold, Mesh, OpType  # noqa: E402

JUMPER = os.path.join(HERE, "Jumper casing - 5 mm pitch - glue halves_1.stl")
PITCH = 2.54
H = 20.0             # casing height (the original jumper casing is 40)
WALL = 2.65          # jumper casing: pin channel edge to the side face
CL = 0.2             # pocket clearance round a body
EXIT_D, EXIT_H = 0.8, 3.0    # lead exit hole at the bottom (Ø as in the jumper casing, 3 mm long to hold the lead straight)
LEAD_D = 1.2         # lead channel from the body down to the exit
HALF_T = 2.0         # thickness of each half; bodies thicker than the casing stick out of the faces
WINDOW_D = 3.5       # LED light window through both faces
BUZZ_WALL = 1.2      # buzzer casing: wall over the buzzer on each face


def seg(r):
    return int(min(64, max(16, math.ceil(2 * math.pi * r / 0.5))))


def box(x0, x1, y0, y1, z0, z1):
    return Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def zcyl(x, y, r, z0, z1):
    return Manifold.cylinder(z1 - z0, r, r, seg(r)).translate([x, y, z0])


def xcyl(x0, x1, y, z, r):
    return Manifold.cylinder(x1 - x0, r, r, seg(r)).rotate([0, 90, 0]).translate([x0, y, z])


def ycyl(x, y0, y1, z, r):
    return Manifold.cylinder(y1 - y0, r, r, seg(r)).rotate([-90, 0, 0]).translate([x, y0, z])


def union(parts):
    return Manifold.batch_boolean(parts, OpType.Add)


def load_stl(path):
    with open(path, "rb") as f:
        d = f.read()
    n = struct.unpack("<I", d[80:84])[0]
    rec = np.frombuffer(d[84:84 + 50 * n], dtype=[("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")])
    v, inv = np.unique(rec["v"].reshape(-1, 3).round(4), axis=0, return_inverse=True)
    return Manifold(Mesh(vert_properties=v.astype(np.float32), tri_verts=inv.reshape(-1, 3).astype(np.uint32)))


def jumper_channels():
    """The two pin channels of the 5 mm jumper casing (exit, crimp bore, glue pocket, wire channel),
    each centred on its pin, mirrored to the full y range."""
    half = load_stl(JUMPER)
    lo, hi = half.bounding_box()[:3], half.bounding_box()[3:]
    cav = box(lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]) - half
    pieces = sorted(cav.decompose(), key=lambda m: m.bounding_box()[0])
    out = []
    for p in pieces:
        b = p.bounding_box()
        c = p.translate([-(b[0] + b[3]) / 2, 0, 0])
        out.append(c + c.mirror([0, 1, 0]))
    return out[0], hi[0] - lo[0], 2 * (hi[1] - lo[1])     # one channel (both are the same), width 14, thickness 5


def lead_path(x0, z0, x1, r=LEAD_D / 2):
    """Lead from where it leaves the body (x0, z0) down to its exit at x1: a round channel in the
    split plane, the hull of its two ends, then the exit hole."""
    a, b = zcyl(x0, 0, r, z0 - 0.01, z0 + 0.3), zcyl(x1, 0, r, EXIT_H, EXIT_H + 0.3)
    return [Manifold.batch_hull([a, b]), zcyl(x1, 0, EXIT_D / 2, -1, EXIT_H + 0.1)]


def exits(n, s):
    return [(j - (n - 1) / 2) * s * PITCH for j in range(n)]


# body pockets: returns (cavities, body half-extent in x, half-thickness in y, extras)
def led_body(n_leads, lead_pitch):
    def f(xs):
        x0s = [(j - (n_leads - 1) / 2) * lead_pitch for j in range(n_leads)]
        zb = EXIT_H + max(2.0, max(abs(a - b) for a, b in zip(x0s, xs)) + 1.0)
        cav = [zcyl(0, 0, 2.9 + CL, zb, zb + 1.0), zcyl(0, 0, 2.5 + CL, zb, zb + 6.1),
               Manifold.sphere(2.5 + CL, seg(2.7)).translate([0, 0, zb + 6.1])]
        for a, b in zip(x0s, xs):
            cav += lead_path(a, zb, b)
        if WINDOW_D > 0:
            cav.append(ycyl(0, -10, 10, zb + 5.5, WINDOW_D / 2))      # the light shows through both faces
        return cav, 2.9 + CL, 2.9 + CL
    return f


def axial_body(r, length):
    """Resistor or diode lying along x; its leads bend down from the body ends to the exits."""
    def f(xs):
        e = length / 2 + 0.3
        zc = EXIT_H + max(2.0, max(abs(abs(x) - e) for x in xs) + 1.0) + r
        cav = [xcyl(-e, e, 0, zc, r + CL)]
        for x in xs:
            end = math.copysign(e, x)
            cav += [xcyl(min(end, end + math.copysign(1.0, x)), max(end, end + math.copysign(1.0, x)), 0, zc, LEAD_D / 2)]
            cav += lead_path(end + math.copysign(0.5, x), zc, x)
        return cav, e + 1.0, r + CL
    return f


def disc_body(f_xs):
    """Ceramic disc capacitor standing in the split plane; leads 2.54 apart under the disc."""
    zc = EXIT_H + 3.0 + 2.75
    cav = [ycyl(0, -1.45, 1.45, zc, 2.75 + CL)]
    for a, b in zip((-1.27, 1.27), f_xs):
        cav += lead_path(a, zc - 2.2, b)
    return cav, 2.95, 1.45


def buzzer_body(height):
    """12 mm buzzer standing on its pins (7.6 mm apart). It is far thicker than a plain casing, so
    this casing wraps it with BUZZ_WALL on both faces."""
    def f(xs):
        zb = EXIT_H + max(2.0, max(abs(a - b) for a, b in zip((-3.8, 3.8), xs)) + 1.0)
        cav = [zcyl(0, 0, 6.0 + CL, zb, zb + height + CL)]
        for a, b in zip((-3.8, 3.8), xs):
            cav += lead_path(a, zb, b)
        return cav, 6.0 + CL, 6.0 + CL
    return f


def to92_body(xs):
    """TO-92 with its flat face toward -y; legs 1.27 apart fan out to the exits."""
    zb = EXIT_H + 2.5
    s = CrossSection([[(-2.3 - CL, -1.2 - CL), (2.3 + CL, -1.2 - CL)]
                      + [(2.3 * math.cos(t) + 0, -1.2 + 2.4 * math.sin(t)) for t in np.linspace(0, math.pi, 24)][1:-1]
                      + [(-2.3 - CL, -1.2)]])
    cav = [s.extrude(4.8 + CL).translate([0, 0, zb])]
    for a, b in zip((-1.27, 0.0, 1.27), xs):
        cav += lead_path(a, zb, b)
    return cav, 2.5, 1.4 + CL


def jumper_casing(s, channel, w0):
    p = s * PITCH
    wall_x = (w0 - 5.0) / 2                                        # channel centre to side face in the original
    w = p + 2 * wall_x
    t = 2 * HALF_T
    lift = EXIT_H - 1.0                                            # the original's exit is 1 mm long
    cav = [channel.translate([x, 0, lift]) for x in exits(2, s)]
    cav += [zcyl(x, 0, EXIT_D / 2, -1, EXIT_H + 0.1) for x in exits(2, s)]
    return box(-w / 2, w / 2, -t / 2, t / 2, 0, H) - union(cav), w, t


def part_casing(body, n, s, wrap=False):
    """wrap: the casing is thick enough to cover the body on both faces instead of 2 * HALF_T."""
    xs = exits(n, s)
    cav, hx, hy = body(xs)
    w = max(14.0, 2 * max(hx, max(abs(x) for x in xs) + EXIT_D / 2 + LEAD_D) + 2 * WALL)
    t = 2 * (hy + BUZZ_WALL) if wrap else 2 * HALF_T
    return box(-w / 2, w / 2, -t / 2, t / 2, 0, H) - union(cav), w, t


PARTS = {   # the kit's parts (assets/images): part type -> (body, pin count, spacings, wrap)
    "resistor": (axial_body(1.25, 6.5), 2, (1, 2, 4), False),        # 1/4 W, 10 ohm .. 1 Mohm
    "led": (led_body(2, 2.54), 2, (1, 2, 4), False),                  # 5 mm, all colours
    "rgb_led": (led_body(4, 1.1), 4, (1, 2), False),
    "bjt": (to92_body, 3, (1,), False),                                # S8050 / S8550
    "capacitor_ceramic": (disc_body, 2, (1,), False),                  # 104
    "buzzer_active": (buzzer_body(9.5), 2, (2, 3, 4), True),          # sealed top, 12 x 9.5
    "buzzer_passive": (buzzer_body(8.5), 2, (2, 3, 4), True),         # open PCB bottom, 12 x 8.5
}


def write_stl(path, m, name):
    mesh = m.to_mesh()
    v = np.asarray(mesh.vert_properties)[:, :3]
    t = np.asarray(mesh.tri_verts)
    a, b, c = v[t[:, 0]], v[t[:, 1]], v[t[:, 2]]
    nrm = np.cross(b - a, c - a)
    nrm /= np.linalg.norm(nrm, axis=1, keepdims=True) + 1e-12
    rec = np.zeros(len(t), dtype=[("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")])
    rec["n"], rec["v"] = nrm, np.hstack([a, b, c])
    with open(path, "wb") as f:
        f.write(name.encode().ljust(80, b" "))
        f.write(struct.pack("<I", len(t)))
        f.write(rec.tobytes())


def split(m, t):
    """The two halves in the casing frame: half 1 at y < 0, half 2 at y > 0."""
    return m ^ box(-50, 50, -t, 0, -1, H + 1), m ^ box(-50, 50, 0, t, -1, H + 1)


def halves(m, t):
    """Each half laid on its outer face, split face up, so the pockets print without supports."""
    h1, h2 = split(m, t)
    return h1.rotate([90, 0, 0]).translate([0, 0, t / 2]), h2.rotate([-90, 0, 0]).translate([0, 0, t / 2])


def part_shape(body, n, s):
    """The part itself, for the viewer: the pocket shapes without clearance, leads 0.6 mm,
    legs 7 mm below the casing. Returns (body, leads)."""
    g = globals()
    saved = {k: g[k] for k in ("CL", "LEAD_D", "EXIT_D", "WINDOW_D")}
    xs = exits(n, s)
    try:
        g.update(CL=0.0, WINDOW_D=0.0, LEAD_D=0.02, EXIT_D=0.02)
        b = union(body(xs)[0])
        g.update(LEAD_D=0.6, EXIT_D=0.6)
        full = union(body(xs)[0] + [zcyl(x, 0, 0.3, -7, EXIT_H + 0.2) for x in xs])
    finally:
        g.update(saved)
    return b, full - b


def jumper_shape(s):
    """Jumper wire ends: crimp sleeves and insulated wire in the channels, pins out the bottom."""
    xs = exits(2, s)
    body = union([zcyl(x, 0, 1.3, 1.6, 8.7) for x in xs] + [zcyl(x, 0, 0.65, 8.7, H) for x in xs])
    return body, union([zcyl(x, 0, 0.32, -7, 9.0) for x in xs])


def check(name, m, w, t, n, s):
    assert m.status().name == "NoError", name
    assert len(m.decompose()) == 1, f"{name}: falls apart"
    for x in exits(n, s):                                      # a lead fits every exit, and nothing else is there
        probe = zcyl(x, 0, 0.3, -0.5, 0.5)
        assert (probe ^ m).volume() < 1e-6, f"{name}: exit at x={x:.2f} blocked"
    for h in split(m, t):                                       # bodies may break through a face, but each half stays one piece
        assert len(h.decompose()) == 1, f"{name}: a half falls apart"


if __name__ == "__main__":
    channel, w0, t0 = jumper_channels()
    out = []
    builds = [(f"jumper_s{s}", jumper_casing(s, channel, w0), 2, s) for s in (1, 2, 3, 4)]
    builds += [(f"{k}_s{s}", part_casing(body, n, s, wrap), n, s) for k, (body, n, sp, wrap) in PARTS.items() for s in sp]
    for name, (m, w, t), n, s in builds:
        check(name, m, w, t, n, s)
        h1, h2 = split(m, t)
        assert abs(h1.volume() + h2.volume() - m.volume()) < 1e-3, name
        for i, h in enumerate(halves(m, t), 1):
            write_stl(os.path.join(HERE, f"casing_{name}_half{i}.stl"), h, f"casing_{name}_half{i}")
        out.append({"casing": name, "pins": n, "pin_spacing": s, "exit_x": [round(x, 3) for x in exits(n, s)],
                    "width_x": round(w, 2), "thickness_y": round(t, 2), "height": H})
        print(f"{name:26s} {w:5.2f} x {t:4.2f} x {H:.0f} mm, exits {[round(x, 2) for x in exits(n, s)]}")
    with open(os.path.join(HERE, "casings.json"), "w") as f:
        json.dump({"frame": "x along the leads, y across the split, z up from the bottom; exits at z = 0",
                   "casings": out}, f, indent=1)
