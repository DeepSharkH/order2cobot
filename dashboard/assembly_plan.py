"""
The circuit mapper's assembly plan, drawn and explained with its own geometry
(src/circuit_mapper/geometry.py: casing footprints from casings.json, fingertips from gripper.json).

The mapper returns `placements` and a verified `assembly_order`; this module only turns them into
rects for the browser and a reason per step. Manual parts (placement_mode "manual", e.g. the power
connector) have holes but no casing and no step: they are wired by hand afterwards.

Rects are [u0, v0, u1, v1] in mm: u along the row numbers, v across the columns
(geometry.py has x across the columns, y along the rows).
"""

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
MAPPER_DIR = ROOT / 'src' / 'circuit_mapper'
CASINGS = ROOT / 'assets' / 'models' / 'component_casings' / 'casings.json'


def _load_geometry():
    # geometry.py has no ortools dependency, unlike mapper.py, so the dashboard can import it
    spec = importlib.util.spec_from_file_location('circuit_geometry', MAPPER_DIR / 'geometry.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


geo = _load_geometry()


def load_casings():
    return json.loads(CASINGS.read_text())


def load_gripper():
    return json.loads((MAPPER_DIR / 'gripper.json').read_text())


def is_manual(part):
    return part.get('placement_mode') == 'manual'


def has_casing(part, casings):
    try:
        geo.casing_for(part, casings, 2.54)
        return True
    except ValueError:
        return False


def _uv(box):
    x0, y0, x1, y1 = box
    return [round(y0, 3), round(x0, 3), round(y1, 3), round(x1, 3)]


def steps(circuit, mapping, parts, board, casings):
    """The mapper's assembly_order, one entry per step: holes, casing, its shape and why it goes there."""
    by_id = {p['part_id']: p for p in parts}
    g = board['geometry']
    rows, pitch = g['rows'], g['hole_pitch_mm']
    pos = lambda h: (g['column_positions'][h[0]], rows['first_position'] + (int(h[1:]) - rows['first']) * rows['spacing'])  # noqa: E731
    gripper = mapping['gripper']
    holes = {c['id']: [mapping['pin_to_hole'][f"{c['id']}.{pin}"] for pin in by_id[c['part_id']]['pins']]
             for c in circuit['components'] if not is_manual(by_id[c['part_id']])}
    holes.update({j['id']: list(j['pins'].values()) for j in mapping['jumpers']})
    placed = {p['id']: p for p in mapping['placements']}
    shapes = {}
    for iid in mapping['assembly_order']:
        part = by_id[placed[iid]['part_id']]
        casing = geo.casing_for(part, casings, pitch)
        shapes[iid] = {**geo.placement_geometry(holes[iid], {h: pos(h) for h in holes[iid]}, casing, gripper, pitch), 'id': iid}
    margin = gripper['safety_margin_mm']
    order = mapping['assembly_order']
    out = []
    for k, iid in enumerate(order):
        s = shapes[iid]
        rel = lambda other: geo.relation(s, shapes[other], margin)      # noqa: E731
        after = [o for o in order[:k] if rel(o) == 'b_first']
        before = [o for o in order[k + 1:] if rel(o) == 'a_first']
        out.append({
            'item': iid, 'part_id': placed[iid]['part_id'], 'holes': holes[iid],
            'casing': s['casing_id'], 'center_mm': s['center_mm'], 'orientation_deg': s['orientation_deg'],
            'body': _uv(s['body']), 'fingers': [_uv(f) for f in s['fingers']],
            'after': after, 'before': before,
        })
    return out
