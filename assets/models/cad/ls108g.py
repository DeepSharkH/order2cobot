#!/usr/bin/env python3
"""TP-Link LS108G 8-port gigabit switch. There is no public CAD, so it is built from:

- the official spec: case 158 x 99.1 x 25 mm (tp-link.com LS108G spec page),
- the official front/rear product photos (static.tp-link.com LS108G_UN_1.0_overview_01 and
  LS108G(UN)1.0_B), scaled by the 158 mm case width: one black window with 8 jacks,
  **latch slot up**, one green Link/Act LED at the top-left of each jack, Power LED far left,
  DC jack + Kensington slot on the rear,
- our own measurements of this unit: jack pitch 14.51-14.56 mm (porttest/port_survey.py,
  rays agree to 0.1 mm; the photo gives 14.7 +- 0.2 through its perspective) and the jack
  height from the plug-tip touches (configs/products/ls108g.yaml).

The values marked MEASURED below are the ones the first seated insertion corrects (see the
cad-guidance skill): the model gives the first guess, the robot gives the truth.

Switch frame (mm in the JSON, m in the GLB): origin at the centre of the case bottom, z up,
port face toward -y, x along the port row (port 1 at -x, i.e. on the left seen from the
front). Port centre = centre of the jack opening's bounding box, on the face plane;
insertion direction +y.

Outputs (dashboard/web/models/cad/): ls108g.glb (nodes: case, window, jack_1..8, contacts,
led_1..8, power_led, dc_jack, lock_slot) and ls108g.json (features).

Usage: .venv/bin/python assets/models/cad/ls108g.py   (needs manifold3d, see breadboard.py)
"""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / '.deps'))
sys.path.insert(0, str(HERE))
import numpy as np  # noqa: E402
from manifold3d import CrossSection, JoinType, Manifold  # noqa: E402

from gltf import mesh_of, write_glb  # noqa: E402

OUT = HERE.parents[2] / 'dashboard' / 'web' / 'models' / 'cad'

W, D, H = 158.0, 99.1, 25.0          # official
CORNER_R = 1.5                       # photo: tight bend radius of the folded steel case
PITCH = 14.55                        # MEASURED (survey 14.51-14.56; photo 14.7 +- 0.2)
PORT_Z = 14.0                        # MEASURED: photo 14.5, plug-tip touches 13.6
OPEN_W, OPEN_H = 11.8, 6.6           # jack opening, main part (plug body 11.7 wide)
NOTCH_W, NOTCH_H = 6.4, 2.2          # latch slot above it (photo: notch on top)
JACK_DEPTH = 16.0                    # cavity behind the face; the plug seats ~13 mm in
WIN_W, WIN_Z = 118.1, (6.9, 20.4)    # black window around the jacks (photo)
WIN_RECESS = 0.4                     # window face behind the case face
LED_DX, LED_Z = -5.6, 18.5           # Link/Act LED relative to its jack (photo)
POWER_LED = (-65.1, 9.8)             # x, z (photo)
DC_JACK = (-50.0, 12.4)              # rear, x, z (rear photo)
LOCK_SLOT = (57.8, 14.3)

FACE_Y = -D / 2


def port_x(k):
    return (k - 4.5) * PITCH


def opening_bbox_h():
    return OPEN_H + NOTCH_H


def jack_cut(x, depth, grow=0.0):
    """Solid of one jack opening (latch up), front at the face, extending +y by depth."""
    zb = PORT_Z - opening_bbox_h() / 2               # bottom of the opening
    main = Manifold.cube([OPEN_W + 2 * grow, depth, OPEN_H + 2 * grow]).translate(
        [x - OPEN_W / 2 - grow, 0, zb - grow])
    notch = Manifold.cube([NOTCH_W + 2 * grow, depth, NOTCH_H + 1 + grow]).translate(
        [x - NOTCH_W / 2 - grow, 0, zb + OPEN_H - 1])          # overlaps the main part
    return (main + notch).translate([0, FACE_Y - 0.01, 0])


def build():
    case = CrossSection.square([W - 2 * CORNER_R, D - 2 * CORNER_R], center=True) \
        .offset(CORNER_R, JoinType.Round, circular_segments=24).extrude(H)
    win = Manifold.cube([WIN_W, 3.0, WIN_Z[1] - WIN_Z[0]]).translate(
        [-WIN_W / 2, FACE_Y - 0.01, WIN_Z[0]])
    case -= win
    # black jack block behind the window, its face WIN_RECESS behind the case face
    block = Manifold.cube([WIN_W, JACK_DEPTH + 2, WIN_Z[1] - WIN_Z[0]]).translate(
        [-WIN_W / 2, FACE_Y + WIN_RECESS, WIN_Z[0]])
    cuts = Manifold.compose([jack_cut(port_x(k), JACK_DEPTH) for k in range(1, 9)])
    block -= cuts
    case -= cuts
    # 8 gold contacts on the floor at the back of each jack (opposite the latch)
    zb = PORT_Z - opening_bbox_h() / 2
    contacts = Manifold.compose([
        Manifold.cube([0.5, 6.0, 0.6]).translate([port_x(k) - 4.55 + 1.3 * i - 0.25,
                                                  FACE_Y + JACK_DEPTH - 6.5, zb])
        for k in range(1, 9) for i in range(8)])
    leds = [Manifold.cube([1.4, 0.4, 0.9]).translate([port_x(k) + LED_DX - 0.7, FACE_Y + WIN_RECESS - 0.2,
                                                      LED_Z - 0.45]) for k in range(1, 9)]
    pled = Manifold.cylinder(0.6, 1.5, circular_segments=20).rotate([90, 0, 0]) \
        .translate([POWER_LED[0], FACE_Y + 0.3, POWER_LED[1]])
    case -= Manifold.cylinder(1.2, 1.5, circular_segments=20).rotate([90, 0, 0]) \
        .translate([POWER_LED[0], FACE_Y + 1.2, POWER_LED[1]])
    dc = Manifold.cube([9, 1.0, 9]).translate([DC_JACK[0] - 4.5, D / 2 - 0.6, DC_JACK[1] - 4.5]) \
        - Manifold.cylinder(1.2, 2, circular_segments=20).rotate([-90, 0, 0]).translate(
            [DC_JACK[0], D / 2 - 1, DC_JACK[1]])
    lock = Manifold.cube([7.0, 0.6, 3.0]).translate([LOCK_SLOT[0] - 3.5, D / 2 - 0.5, LOCK_SLOT[1] - 1.5])

    ports = {}
    for k in range(1, 9):
        ports[str(k)] = {
            'centre': [round(port_x(k), 3), round(FACE_Y, 3), PORT_Z],
            'axis': [0, 1, 0],
            'led': [round(port_x(k) + LED_DX, 3), round(FACE_Y + WIN_RECESS, 3), LED_Z]}
    spec = {
        'name': 'tp-link_ls108g',
        'source': 'tp-link.com spec (case size) + official photos + our port survey (pitch, height)',
        'units': 'mm',
        'frame': 'origin = centre of the case bottom; port face toward -y; x along the row, '
                 'port 1 at -x; z up. Port centre = centre of the opening bbox on the face plane',
        'size': [W, D, H],
        'face_y': FACE_Y,
        'pitch': PITCH,
        'row_step': [PITCH, 0, 0],
        'port_z': PORT_Z,
        'opening': {'width': OPEN_W, 'height': OPEN_H, 'notch_width': NOTCH_W,
                    'notch_height': NOTCH_H, 'latch': 'up', 'bbox': [OPEN_W, opening_bbox_h()]},
        'window': {'x': [-WIN_W / 2, WIN_W / 2], 'z': list(WIN_Z), 'recess': WIN_RECESS},
        'seat_depth': 13.0,
        'ports': ports,
        'measured': {'pitch': 'survey 14.51-14.56 (data/ports.yaml), photo 14.7+-0.2; '
                              'seated tip coordinates suggested ~15.0 (includes tool-model error)',
                     'port_z': 'photo 14.5, touches 13.6'},
    }
    parts = [
        {'name': 'case', 'm': case, 'color': (0.20, 0.31, 0.37), 'metal': 0.45, 'rough': 0.5},
        {'name': 'window', 'm': block, 'color': (0.03, 0.03, 0.035), 'rough': 0.7},
        {'name': 'contacts', 'm': contacts, 'color': (0.83, 0.66, 0.23), 'metal': 0.9, 'rough': 0.3},
        {'name': 'power_led', 'm': pled, 'color': (0.05, 0.25, 0.08), 'rough': 0.3},
        {'name': 'dc_jack', 'm': dc, 'color': (0.02, 0.02, 0.02), 'rough': 0.6},
        {'name': 'lock_slot', 'm': lock, 'color': (0.02, 0.02, 0.02), 'rough': 0.8},
    ]
    for k in range(1, 9):
        # pickable/tintable jack mouth: a thin frame just inside each opening
        mouth = jack_cut(port_x(k), 0.6, grow=0.0) - jack_cut(port_x(k), 0.8, grow=-0.35) \
            .translate([0, -0.1, 0])
        parts.append({'name': f'jack_{k}', 'm': mouth.translate([0, WIN_RECESS + 0.02, 0]),
                      'color': (0.12, 0.12, 0.13), 'rough': 0.5, 'extras': {'port': k}})
        parts.append({'name': f'led_{k}', 'm': leds[k - 1], 'color': (0.06, 0.12, 0.06),
                      'rough': 0.4, 'extras': {'port': k}})
    return spec, parts


def main():
    spec, parts = build()
    OUT.mkdir(parents=True, exist_ok=True)
    glb = []
    for p in parts:
        V, F = mesh_of(p.pop('m'))
        glb.append({**p, 'V': V / 1000, 'F': F})
    write_glb(OUT / 'ls108g.glb', glb, extras={'spec': 'ls108g.json'})
    (OUT / 'ls108g.json').write_text(json.dumps(spec, indent=1))
    print(f"ls108g: {spec['size']} mm, pitch {PITCH}, {sum(len(p['F']) for p in glb)} triangles")
    print('port 1', spec['ports']['1']['centre'], 'port 8', spec['ports']['8']['centre'])


if __name__ == '__main__':
    main()
