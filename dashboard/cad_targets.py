"""CAD targets of python -m cadguide (the LS108G switch, the breadboard) on the dashboard.

Their poses, T_base_obj + pitch scales, are cadguide's (data/cad/<name>.yaml: the fit of the
registration and the anchors). This gives the browser those poses (state), the camera feeds
their bounding boxes (boxes: drawn like the Hestia dashboard, the box in the target's colour
under a solid name tag) and the scene editor its manual registration (save): a pose lined up
by hand becomes a 'manual' registration, and as with any new registration the anchors are
dropped (python -m cadguide register <obj> --manual does the same from numbers).
"""
import json
import threading
import time

import numpy as np
import yaml

from cadguide import state as cad_state
from cadguide.model import CadModel

# the browser's colours; the feeds draw them in BGR (bgr)
COLOURS = {'ls108g': '#fef5da', 'breadboard': '#6fb3a8', 'rack': '#5b8fe0'}
LABELS = {'ls108g': 'LS108G switch', 'breadboard': 'Breadboard', 'rack': 'Rack'}
PLANK_FILE = cad_state.STATE_DIR / 'plank.yaml'       # python -m cadguide plank

# The casing rack (assets/models/cad/rack.py): not a cadguide target (no features, no anchors);
# the scene editor sets its pose, kept here. Until then: the touched pose of the rack on the
# table (configs/breadboard/rack.yaml, 10-27 16:26), with this model centred on that footprint.
RACK_FILE = cad_state.STATE_DIR / 'rack_scene.json'
RACK_SPEC = cad_state.CAD_DIR / 'rack.json'
RACK_DEFAULT = {'centre_mm': [-220.57, 17.80, 2.92], 'yaw_deg': 90.98,
                'source': 'configs/breadboard/rack.yaml touch pose 10-27 16:26 (original print), footprint centre'}


def rack_default_T():
    a = np.radians(RACK_DEFAULT['yaw_deg'])
    T = np.eye(4)
    T[:2, :2] = [[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]
    T[:3, 3] = np.array(RACK_DEFAULT['centre_mm']) / 1000
    return T


def bgr(colour):
    h = colour.lstrip('#')
    return tuple(int(h[i:i + 2], 16) for i in (4, 2, 0))


def checked_pose(T, scale):
    """A pose from the browser -> (T 4x4 rigid, scale (2,)); ValueError if it is not one."""
    T, scale = np.array(T, float), np.array(scale, float)
    if T.shape != (4, 4) or scale.shape != (2,) or not (np.isfinite(T).all() and np.isfinite(scale).all()):
        raise ValueError('the pose must be a 4x4 matrix and two pitch scales')
    U, _, Vt = np.linalg.svd(T[:3, :3])
    R = U @ Vt                                   # the JSON round trip leaves it a hair off
    if np.linalg.det(R) < 0 or np.abs(R - T[:3, :3]).max() > 1e-3:
        raise ValueError('not a rotation')
    if np.linalg.norm(T[:3, 3]) > 1.5:
        raise ValueError('more than 1.5 m from the robot base')
    if np.abs(scale - 1).max() > 0.05:
        raise ValueError('pitch scale beyond ±5 %')
    T[:3, :3], T[3] = R, [0, 0, 0, 1]
    return T, scale


class CadTargets:
    def __init__(self, board_thickness=0.004):
        self.models = {n: CadModel(n) for n in cad_state.OBJECTS}
        self.thickness = board_thickness     # the plank before it is measured: the board plane lowered by it
        self._files = {}                     # path -> (mtime, parsed): state() runs at 10 Hz per browser
        self._save_lock = threading.Lock()

    def _read(self, path, load):
        try:
            m = path.stat().st_mtime_ns
        except FileNotFoundError:
            return None
        hit = self._files.get(path)
        if hit is None or hit[0] != m:
            try:
                hit = self._files[path] = (m, load())
            except Exception:        # caught mid-write (cadguide's CLI saves too): the last good one, retried next time
                return hit[1] if hit else None
        return hit[1]

    @property
    def names(self):
        """Everything the scene draws and edits: the cadguide targets and the rack."""
        return [*self.models, 'rack']

    def rack(self):
        """(T_base_rack, its record) from rack_scene.json, else the default from the touched pose."""
        r = self._read(RACK_FILE, lambda: json.loads(RACK_FILE.read_text()))
        if r and r.get('T'):
            return np.array(r['T'], float), r
        return rack_default_T(), {'source': RACK_DEFAULT['source'], 'stamp': None}

    def rack_size(self):
        spec = self._read(RACK_SPEC, lambda: json.loads(RACK_SPEC.read_text()))
        return np.array(spec['size'] if spec else [142.16, 82.5, 9.0]) / 1000

    def current(self, name):
        """(T_base_obj, scale, cadguide state) of a target, or None while it is not registered."""
        if name == 'rack':
            T, r = self.rack()
            return T, np.ones(2), r
        s = self._read(cad_state.STATE_DIR / f'{name}.yaml', lambda: cad_state.load(name))
        c = s and cad_state.current(s)
        return (c[0], c[1], s) if c else None

    def box(self, name, T, scale):
        """The target's bounding box in base: 8 corners, the bottom then the top
        (agents.vision.locate.EDGES order), stretched by the pitch scales like its features."""
        if name == 'rack':           # origin at the footprint centre on the table
            w, d, h = self.rack_size()
            C = np.array([[-w / 2, -d / 2, 0], [w / 2, -d / 2, 0], [w / 2, d / 2, 0], [-w / 2, d / 2, 0]])
            C = np.r_[C, C + [0, 0, h]]
            return C @ T[:3, :3].T + T[:3, 3]
        m = self.models[name]
        w, d, h = m.size
        z0 = m.bottom_z()
        C = np.array([[-w / 2, -d / 2, z0], [w / 2, -d / 2, z0], [w / 2, d / 2, z0], [-w / 2, d / 2, z0]])
        return m.place(T, scale, np.r_[C, C + [0, 0, h]])

    def boxes(self, T_world_base, preview):
        """(tag, corners 8x3 in the world, BGR colour) of each target for the feed overlays;
        preview {name: (T, scale)}: the editor's unsaved poses, drawn instead of the saved ones."""
        out = []
        for name in self.names:
            p = preview.get(name) or self.current(name)
            if p is None:
                continue
            C = self.box(name, p[0], p[1])
            out.append((name.upper() + (' (EDITING)' if name in preview else ''),
                        C @ T_world_base[:3, :3].T + T_world_base[:3, 3], bgr(COLOURS[name])))
        return out

    def plank(self):
        """The plank z = a x + b y + c [m] in the world (what the parts stand on)."""
        p = self._read(PLANK_FILE, lambda: yaml.safe_load(PLANK_FILE.read_text()))
        if p and p.get('plane'):
            return {**{k: float(p['plane'][k]) for k in 'abc'}, 'measured': p.get('stamp')}
        return {'a': 0.0, 'b': 0.0, 'c': -float(self.thickness), 'measured': None}

    def state(self, preview):
        objs = {}
        for name, m in self.models.items():
            c = self.current(name)
            s = c[2] if c else {}
            reg = s.get('registration') or {}
            objs[name] = {'label': LABELS[name], 'colour': COLOURS[name],
                          'T': None if c is None else c[0].tolist(), 'scale': None if c is None else c[1].tolist(),
                          'kind': reg.get('kind'), 'stamp': reg.get('stamp'),
                          'anchors': len([a for a in s.get('anchors', []) if a.get('used', True)]),
                          'size': m.size.tolist(), 'bottom_z': m.bottom_z(), 'scale_origin': m.scale_origin.tolist(),
                          'pitch_mm': [m.spec['pitch'], None if m.kind == 'switch' else m.spec['pitch']],
                          'editing': name in preview}
        T, r = self.rack()
        objs['rack'] = {'label': LABELS['rack'], 'colour': COLOURS['rack'], 'T': T.tolist(), 'scale': [1.0, 1.0],
                        'kind': 'manual' if r.get('stamp') else 'touch', 'stamp': r.get('stamp'), 'anchors': 0,
                        'size': self.rack_size().tolist(), 'bottom_z': 0.0, 'scale_origin': [0.0, 0.0, 0.0],
                        'pitch_mm': [None, None], 'fixed_scale': True, 'editing': 'rack' in preview}
        return {'objects': objs, 'plank': self.plank()}

    def save(self, name, T, scale):
        """The editor's pose as the target's manual registration (the rack: rack_scene.json)."""
        if name == 'rack':
            T, _ = checked_pose(T, [1.0, 1.0])
            old, _ = self.rack()
            rec = {'T': T.tolist(), 'stamp': time.strftime('%Y-%m-%dT%H:%M:%S'), 'source': 'dashboard scene editor',
                   'model': 'assets/models/component_casings/All_Casings_Holder_Print.stl (rack.py)'}
            with self._save_lock:
                RACK_FILE.parent.mkdir(parents=True, exist_ok=True)
                tmp = RACK_FILE.with_suffix('.tmp')
                tmp.write_text(json.dumps(rec, indent=1) + '\n')
                tmp.replace(RACK_FILE)
            x, y, z = T[:3, 3] * 1000
            yaw = np.degrees(np.arctan2(T[1, 0], T[0, 0]))
            moved = np.linalg.norm(T[:3, 3] - old[:3, 3]) * 1000
            return f'Rack: at {x:.1f}, {y:.1f}, {z:.1f} mm, yaw {yaw:.2f}°, moved {moved:.1f} mm — saved'
        if name not in self.models:
            raise ValueError(f'unknown CAD target {name!r}')
        T, scale = checked_pose(T, scale)
        from cadguide import __main__ as cg         # the anchor fit (scipy): only when saving
        with self._save_lock:
            r = cg.register_manual(name, T, scale, method='manual (dashboard editor)')
        x, y, z = r['origin_mm']
        moved = r['moved_vs_previous_mm']
        return (f"{LABELS[name]}: manual registration at {x:.1f}, {y:.1f}, {z:.1f} mm, yaw {r['yaw_deg']:.2f}°"
                + (f', moved {np.linalg.norm(moved):.1f} mm' if moved else '')
                + (f", {r['anchors_dropped']} anchor{'s' if r['anchors_dropped'] > 1 else ''} dropped"
                   if r['anchors_dropped'] else '') + ' — saved')
