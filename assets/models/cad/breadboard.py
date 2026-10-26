#!/usr/bin/env python3
"""Breadboards rebuilt from the vendor CAD (assets/models/breadboard.sab, the 400-tie-point one):
breadboard = the full-size one on the table (60 rows, nominal length), breadboard_400 = the CAD as is.

The .sab is an ACIS B-rep (SpaceClaim 2012) that nothing open tessellates, so this script
reads the exact numbers out of it (hole centres from the hole-shaft vertices, outline and
channel from the long edges) and rebuilds a solid with manifold3d.

Board frame (mm in the JSON, m in the GLB): origin at the centre of the top face, z up,
x across the board (columns a -> j, + rails on the left of each rail pair), y along it
(row 1 at +y, the last row at -y). Same axes as the CAD; only the origin moved.

Outputs (dashboard/web/models/cad/):
    <name>.glb    three.js-loadable mesh (nodes: body, rail_plus, rail_minus)
    <name>.json   features: every hole by name ('a1'..'j60', 'L+1'..'R-50' on the table board) etc.

Needs manifold3d (gitignored ./.deps one level up):
    uv pip install --python .venv/bin/python --target assets/models/.deps manifold3d
Usage: .venv/bin/python assets/models/cad/breadboard.py
"""
import json
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / '.deps'))
sys.path.insert(0, str(HERE))
import numpy as np  # noqa: E402
from manifold3d import Manifold  # noqa: E402

from gltf import mesh_of, write_glb  # noqa: E402
from sab import edge_segments, points, read  # noqa: E402

ROOT = HERE.parents[2]
SRC = ROOT / 'assets' / 'models' / 'breadboard.sab'
OUT = ROOT / 'dashboard' / 'web' / 'models' / 'cad'


def from_cad():
    ents = read(SRC)
    P = np.array(points(ents)) * 1000
    S = np.array([np.r_[a, b] for a, b in edge_segments(ents)]) * 1000
    lo, hi = P.min(0), P.max(0)
    top = hi[2]
    # hole shafts: 4 vertices per hole on the plane where the entry chamfer ends
    # (the 1600-vertex level 0.015 mm under the top is the entry edge round, not the funnel end)
    zs = np.unique(np.round(P[:, 2], 3))
    per_hole = [z for z in zs if np.sum(np.abs(P[:, 2] - z) < 1e-3) == 1600 and z < top - 0.1]
    shaft_top = max(per_hole)
    Q = P[np.abs(P[:, 2] - shaft_top) < 1e-3]
    xs, ys = np.unique(np.round(Q[:, 0], 3)), np.unique(np.round(Q[:, 1], 3))
    cx, cy = (xs[0::2] + xs[1::2]) / 2, (ys[0::2] + ys[1::2]) / 2
    shaft_w = float(np.median(xs[1::2] - xs[0::2]))
    holes = sorted({(int(np.argmin(abs(cx - x))), int(np.argmin(abs(cy - y)))) for x, y, _ in Q})
    # entry square on the top face around the first hole
    i, j = holes[0]
    T = P[np.abs(P[:, 2] - top) < 1e-3][:, :2] - [cx[i], cy[j]]
    entry_w = float(2 * np.abs(T[(np.abs(T) < 1.2).all(1)]).max())
    shaft_bottom = min(per_hole)
    # long edges along y on the top face: outline, strip gaps, channel, rail-line grooves
    L = np.linalg.norm(S[:, 3:] - S[:, :3], axis=1)
    along = (np.abs(S[:, 0] - S[:, 3]) < 1e-3) & (L > 50)
    xlines = np.unique(np.round(S[along & (np.abs(S[:, 2] - top) < 1e-3), 0], 3))
    floor = S[along & (S[:, 2] < top - 1) & (S[:, 2] > lo[2] + 1)]
    return dict(lo=lo, hi=hi, cx=cx, cy=cy, holes=holes, entry_w=entry_w, shaft_w=shaft_w,
                chamfer_z=shaft_top, shaft_bottom=shaft_bottom, xlines=xlines,
                channel_floor=float(floor[:, 2].min()), lines_y=(S[along & (L < 80), 1].min(),
                                                                  S[along & (L < 80), 4].max()))


# The board on the table is the full-size one (60 numbered rows a-e | f-j, + and - rails on
# both sides, split in the middle), the vendor CAD is the 400 one of the same family
# (same 55 mm cross-section, hole profile, rail lines, channel). The 830 is built from the
# CAD cross-section with a nominal MB-102 length; its rail grouping (2 x 5 groups of 5 per
# rail, split at the middle) is nominal too - the anchors correct the grid on the robot.
VARIANTS = {'breadboard_400': None, 'breadboard': {'rows': 60, 'length': 165.1}}


def _layout(c, o, variant):
    """Hole names -> board-frame xy (mm) and the rail-line y spans."""
    cx, cy = c['cx'] - o[0], c['cy'] - o[1]
    rail_cols = {0: 'L+', 1: 'L-', len(cx) - 2: 'R+', len(cx) - 1: 'R-'}
    term_cols = [k for k in range(len(cx)) if k not in rail_cols]
    by_col = {}
    for i, j in c['holes']:
        by_col.setdefault(i, []).append(j)
    rail_y = sorted({float(cy[j]) for i, js in by_col.items() if i in rail_cols for j in js}, reverse=True)
    y0, y1 = (v - o[1] for v in c['lines_y'])
    holes = {}
    if variant is None:
        rows = len(by_col[term_cols[0]])
        row_y = [((rows + 1) / 2 - r) * 2.54 for r in range(1, rows + 1)]
        rail_ys, spans = rail_y, [(y0, y1)]
    else:
        rows = variant['rows']
        row_y = [((rows + 1) / 2 - r) * 2.54 for r in range(1, rows + 1)]
        half = (rows + 1) / 4 * 2.54                     # centre of each rail half
        rail_ys = [y + half for y in rail_y] + [y - half for y in rail_y]
        ext = (y1 - y0) / 2
        spans = [(half - ext, half + ext), (-half - ext, -half + ext)]
    for i in rail_cols:
        for r, y in enumerate(rail_ys, 1):
            holes[f'{rail_cols[i]}{r}'] = [round(float(cx[i]), 4), round(y, 4), 0.0]
    for k, i in enumerate(term_cols):
        for r, y in enumerate(row_y, 1):
            holes[f'{"abcdefghij"[k]}{r}'] = [round(float(cx[i]), 4), round(y, 4), 0.0]
    return holes, spans, rows


def build(name='breadboard'):
    variant = VARIANTS[name]
    c = from_cad()
    lo, hi = c['lo'], c['hi']
    o = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, hi[2]])     # board frame origin
    W, Lb, H = hi - lo
    if variant:
        Lb = variant['length']
    holes, spans, rows = _layout(c, o, variant)
    xl = sorted(c['xlines'] - o[0])
    # the x lines on the top face, left to right (see the docstring of from_cad)
    # outline | red | blue | strip gap (2) | channel (2) | strip gap (2) | red | blue | outline
    edge_l, red_l, _, blue_l, _, gap_l0, gap_l1, ch0, ch1, gap_r0, gap_r1, red_r, _, blue_r, _, edge_r = xl
    groove_w = xl[2] - xl[1]
    body = Manifold.cube([W, Lb, H]).translate([-W / 2, -Lb / 2, -H])
    gap_d = 0.25
    for a, b in ((gap_l0, gap_l1), (gap_r0, gap_r1)):
        body -= Manifold.cube([b - a, Lb + 2, gap_d + 0.01]).translate([a, -Lb / 2 - 1, -gap_d])
    ch_floor = c['channel_floor'] - o[2]
    body -= Manifold.cube([ch1 - ch0, Lb + 2, -ch_floor + 0.01]).translate([ch0, -Lb / 2 - 1, ch_floor])
    lines = {}
    for line, xs in (('rail_plus', (red_l, red_r)), ('rail_minus', (blue_l, blue_r))):
        cut = [Manifold.cube([groove_w, b - a, 0.3]).translate([x, a, -0.2]) for x in xs for a, b in spans]
        for m in cut:
            body -= m
        lines[line] = Manifold.compose([m.translate([0, 0, -0.08]) for m in cut]).trim_by_plane([0, 0, -1], 0.04)
    e, s = c['entry_w'], c['shaft_w']
    zc, zb = c['chamfer_z'] - o[2], c['shaft_bottom'] - o[2]
    funnel = Manifold.hull(Manifold.cube([e, e, 0.01], center=True).translate([0, 0, 0.005])
                           + Manifold.cube([s, s, 0.01], center=True).translate([0, 0, zc]))
    shaft = Manifold.cube([s, s, zc - zb + 0.02]).translate([-s / 2, -s / 2, zb])
    hole = funnel + shaft
    body -= Manifold.compose([hole.translate([x, y, 0]) for x, y, _ in holes.values()])

    spec = {
        'name': name, 'rows': rows, 'tie_points': len(holes),
        'source': 'assets/models/breadboard.sab (ACIS, SpaceClaim, 400 version)' + (
            '' if variant is None else f"; {rows} rows, length {Lb} mm and rail grouping nominal (MB-102)"),
        'units': 'mm',
        'frame': 'origin = centre of the top face; x columns a->j, y toward row 1, z up',
        'size': [round(float(W), 3), round(float(Lb), 3), round(float(H), 3)],
        'pitch': 2.54,
        'hole': {'entry_square': round(e, 3), 'shaft_square': round(s, 3),
                 'chamfer_depth': round(-zc, 3), 'depth': round(-zb, 3)},
        'channel': {'x': [round(ch0, 3), round(ch1, 3)], 'depth': round(-ch_floor, 3)},
        'rail_lines': {'plus_red_x': [round(red_l + groove_w / 2, 3), round(red_r + groove_w / 2, 3)],
                       'minus_blue_x': [round(blue_l + groove_w / 2, 3), round(blue_r + groove_w / 2, 3)],
                       'y': [[round(a, 3), round(b, 3)] for a, b in spans], 'width': round(groove_w, 3)},
        'strip_gaps_x': [[round(gap_l0, 3), round(gap_l1, 3)], [round(gap_r0, 3), round(gap_r1, 3)]],
        'outline_x': [round(edge_l, 3), round(edge_r, 3)],
        'holes': holes,
        'grid': {'terminal_a1': holes['a1'], 'col_step': [2.54, 0, 0], 'row_step': [0, -2.54, 0],
                 'note': 'terminal hole (col k, row r) = a1 + k*col_step + (r-1)*row_step, '
                         'plus the 7.62 mm channel jump after column e'},
    }
    parts = [
        {'name': 'body', 'm': body, 'color': (0.93, 0.92, 0.88), 'rough': 0.55},
        {'name': 'rail_plus', 'm': lines['rail_plus'], 'color': (0.82, 0.18, 0.16), 'rough': 0.6},
        {'name': 'rail_minus', 'm': lines['rail_minus'], 'color': (0.20, 0.45, 0.85), 'rough': 0.6},
    ]
    return spec, parts


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    for name in VARIANTS:
        spec, parts = build(name)
        glb = []
        for p in parts:
            V, F = mesh_of(p.pop('m'))
            glb.append({**p, 'V': V / 1000, 'F': F})
        write_glb(OUT / f'{name}.glb', glb, extras={'spec': f'{name}.json'})
        (OUT / f'{name}.json').write_text(json.dumps(spec, indent=1))
        n = sum(len(p['F']) for p in glb)
        print(f"{name}: {spec['size']} mm, {len(spec['holes'])} holes, {n} triangles, "
              f"{os.path.getsize(OUT / f'{name}.glb') / 1e6:.1f} MB")


if __name__ == '__main__':
    main()
