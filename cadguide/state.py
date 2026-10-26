"""Per-object alignment state: data/cad/<name>.yaml (+ a copy for the dashboard).

    registration: T_base_obj + scale from the cameras (prior; never overwritten by anchors)
    anchors:      seated insertions, measured in the robot's tool coordinates
    fit:          current T_base_obj + scale = registration corrected by the anchors
"""
import json
import time
from pathlib import Path

import numpy as np
import yaml

from cadguide.model import CAD_DIR, ROOT

STATE_DIR = ROOT / 'data' / 'cad'
OBJECTS = ('ls108g', 'breadboard')


def now():
    return time.strftime('%Y-%m-%dT%H:%M:%S')


def _clean(o):
    if isinstance(o, np.ndarray):
        return _clean(o.tolist())
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    if isinstance(o, dict):
        return {k: _clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_clean(v) for v in o]
    return o


def load(name):
    f = STATE_DIR / f'{name}.yaml'
    s = yaml.safe_load(f.read_text()) if f.exists() else None
    return s or {'object': name, 'registration': None, 'anchors': [], 'fit': None}


def save(name, s):
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    (STATE_DIR / f'{name}.yaml').write_text(yaml.safe_dump(_clean(s), sort_keys=False))
    publish()


def current(s):
    """(T, scale, source) of the best available pose, or None."""
    for key in ('fit', 'registration'):
        r = s.get(key)
        if r:
            return np.array(r['T']), np.array(r.get('scale', [1.0, 1.0])), key
    return None


def publish():
    """dashboard/web/models/cad/poses.json: what the world scene draws (gitignored)."""
    out = {}
    for name in OBJECTS:
        s = load(name)
        c = current(s)
        if c:
            T, scale, src = c
            out[name] = {'T': T.tolist(), 'scale': scale.tolist(), 'source': src,
                         'anchors': len([a for a in s['anchors'] if a.get('used', True)]),
                         'stamp': (s.get(src) or {}).get('stamp')}
    (CAD_DIR / 'poses.json').write_text(json.dumps(out, indent=1))
