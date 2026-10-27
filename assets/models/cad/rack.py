"""Casing rack (Jan's latest print) for the dashboard's 3D scene.

Input:  assets/models/component_casings/All_Casings_Holder_Print.stl (binary STL, mm; print
        frame: corner at the origin, x along the long side, printed upside down: a 4 mm plate on
        the bed with the 5 mm feet up, so it is turned 180 deg about x to stand on its feet)
Output (dashboard/web/models/cad/): rack.glb, rack.json (size)

Object frame: origin = centre of the footprint on the table, x along the long side, z up;
metres in the .glb, like the other CAD models (gltf.py). The pose in the robot base frame is
the dashboard's own (data/cad/rack_scene.json, set in the scene editor), not cadguide's.

    python assets/models/cad/rack.py
"""
import json
import struct
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
SRC = ROOT / 'assets/models/component_casings/All_Casings_Holder_Print.stl'
OUT = ROOT / 'dashboard/web/models/cad'
sys.path.insert(0, str(HERE))
from gltf import write_glb  # noqa: E402


def read_stl(path):
    """(V (3M,3), F (M,3)) of a binary STL, un-indexed."""
    b = path.read_bytes()
    n = struct.unpack('<I', b[80:84])[0]
    t = np.frombuffer(b[84:84 + 50 * n], dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
    return t['v'].reshape(-1, 3).astype(float), np.arange(3 * n).reshape(-1, 3)


def main():
    V, F = read_stl(SRC)
    V = V * [1, -1, -1]                 # print frame -> standing on the table (180 deg about x)
    lo, hi = V.min(0), V.max(0)
    V = V - [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]]
    size = (hi - lo).round(2).tolist()
    write_glb(OUT / 'rack.glb', [{'name': 'rack', 'V': V / 1000, 'F': F, 'color': (0.18, 0.36, 0.72), 'rough': 0.7}],
              extras={'spec': 'rack.json'})
    (OUT / 'rack.json').write_text(json.dumps({
        'name': 'casing_rack', 'source': str(SRC.relative_to(ROOT)), 'units': 'mm',
        'frame': 'origin = centre of the footprint on the table; x along the long side, z up',
        'size': size}, indent=1) + '\n')
    print(f'rack.glb: {len(F)} triangles, {size[0]} x {size[1]} x {size[2]} mm')


if __name__ == '__main__':
    main()
