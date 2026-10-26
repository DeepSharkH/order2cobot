"""Contact world of the simulated workcell: plate, LS108G switch with its 8 ports, breadboard.

Contact is computed for one point, the active TCP (plug tip / pin midpoint): every object is a
box with pockets (ports, holes) cut into one face. A point inside the solid is pushed out
along the shortest escape (a box face, or sideways into the nearest pocket, or back out of a
pocket bottom) by a spring: F = stiffness * penetration. A pocket has a clearance around its
axis and a funnel at the mouth, so a compliant tip that reaches the mouth is guided in.
"""
import json
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
WORLD_FILE = Path(__file__).with_name('world.yaml')
CAD_DIR = ROOT / 'dashboard' / 'web' / 'models' / 'cad'


class Box:
    """Axis-aligned box in its object frame with pockets in one face.

    pockets: (N, 3) mouth centres on the face, `axis` into the solid, `eu`/`ev` the lateral
    axes; `c` clearance half width, `f` funnel extra half width at the mouth, `df` funnel
    depth, `depth` pocket depth. All object frame, metres."""

    def __init__(self, name, kind, T, lo, hi, pockets, labels, axis, eu, ev, c, f, df, depth, hold, hold_from, hold_full):
        self.name, self.kind = name, kind
        self.T = np.asarray(T, float)
        self.R, self.t = self.T[:3, :3], self.T[:3, 3]
        self.lo, self.hi = np.asarray(lo, float), np.asarray(hi, float)
        self.pockets = np.asarray(pockets, float).reshape(-1, 3)
        self.labels = labels
        self.axis, self.eu, self.ev = (np.asarray(v, float) for v in (axis, eu, ev))
        self.c, self.f, self.df, self.depth, self.hold, self.hold_from = c, f, df, depth, hold, hold_from
        self.hold_full = max(hold_full, hold_from)

    def contact(self, p):
        """(penetration m, outward normal base, info) for base point p."""
        x = self.R.T @ (p - self.t)
        if np.any(x <= self.lo) or np.any(x >= self.hi):
            return 0.0, None, None
        # escapes through the 6 faces
        d_lo, d_hi = x - self.lo, self.hi - x
        i_lo, i_hi = int(np.argmin(d_lo)), int(np.argmin(d_hi))
        if d_lo[i_lo] < d_hi[i_hi]:
            pen, n = d_lo[i_lo], -np.eye(3)[i_lo]
        else:
            pen, n = d_hi[i_hi], np.eye(3)[i_hi]
        info = None
        if len(self.pockets):
            d = x - self.pockets
            u, v = d @ self.eu, d @ self.ev
            k = int(np.argmin(u * u + v * v))
            s, u, v = float(d[k] @ self.axis), float(u[k]), float(v[k])
            a = self.c + self.f * max(0.0, 1.0 - s / self.df) if s >= 0 else self.c + self.f
            # distance to the pocket's free region (clearance/funnel around the axis, down to its bottom)
            du, dv, ds = max(abs(u) - a, 0.0), max(abs(v) - a, 0.0), max(s - self.depth, 0.0)
            info = {'object': self.name, 'pocket': self.labels[k], 'depth': s, 'u': u, 'v': v}
            if s >= 0.0:
                dist = float(np.sqrt(du * du + dv * dv + ds * ds))
                if dist == 0.0:
                    return 0.0, None, {**info, 'inside': True}
                if dist < pen:
                    nl = -(np.sign(u) * du * self.eu + np.sign(v) * dv * self.ev + ds * self.axis) / dist
                    if s < self.df and ds == 0.0:        # funnel wall: tilted back out
                        nl = nl - self.axis * (self.f / self.df)
                        nl /= np.linalg.norm(nl)
                    pen, n = dist, nl
                    info['inside'] = True        # in the pocket, pressing on its wall or bottom
        return float(pen), self.R @ n, info

    def describe(self):
        return {'name': self.name, 'kind': self.kind, 'T': self.T.tolist(), 'lo': self.lo.tolist(),
                'hi': self.hi.tolist(), 'axis': self.axis.tolist(), 'depth': self.depth,
                'pockets': self.pockets.tolist() if self.kind == 'switch' else len(self.pockets),
                'labels': self.labels if self.kind == 'switch' else None}


def _pose(name, spec):
    """(T, scale, source): cadguide state of this checkout, else the snapshot."""
    try:
        from cadguide import state
        cur = state.current(state.load(name))
        if cur is not None:
            return cur[0], cur[1], f'cadguide {cur[2]}'
    except Exception:  # noqa: BLE001 - cadguide is optional for the simulator
        pass
    snap = spec['snapshot']
    return np.array(snap['T'], float), np.array(snap.get('scale', [1, 1]), float), 'snapshot'


def _offset(T, mm):
    T = np.array(T, float)
    T[:3, 3] += np.asarray(mm, float) / 1000
    return T


def _switch(name, spec, T, scale):
    mm = 1e-3
    L, W, H = (np.array(spec['size_mm']) * mm)
    face = spec['face_y_mm'] * mm
    xs = np.array(spec['port_x_mm']) * mm * scale[0]
    pockets = [[x, face, spec['port_z_mm'] * mm] for x in xs]
    return Box(name, 'switch', T, [-L / 2, -W / 2, 0.0], [L / 2, W / 2, H], pockets,
               [f'port:{i + 1}' for i in range(len(xs))], [0, 1, 0], [1, 0, 0], [0, 0, 1],
               spec['clearance_mm'] * mm, spec['funnel_mm'] * mm, spec['funnel_depth_mm'] * mm,
               spec['seat_depth_mm'] * mm, spec.get('hold_N', 0.0), spec.get('hold_from_mm', 0.0) * mm,
               spec.get('hold_full_mm', 0.0) * mm)


def _breadboard(name, spec, T, scale):
    mm = 1e-3
    L, W, H = (np.array(spec['size_mm']) * mm)
    labels, pockets = [], []
    f = CAD_DIR / 'breadboard.json'
    if f.exists():
        for k, p in json.loads(f.read_text())['holes'].items():
            labels.append(f'hole:{k}')
            pockets.append([p[0] * mm * scale[0], p[1] * mm * scale[1], 0.0])
    else:                                    # terminal strips only: a1 + col/row steps, channel after e
        for col in range(10):
            for row in range(1, 61):
                x = -13.97 + 2.54 * col + (5.08 if col >= 5 else 0.0)
                labels.append(f'hole:{"abcdefghij"[col]}{row}')
                pockets.append([x * mm * scale[0], (74.93 - 2.54 * (row - 1)) * mm * scale[1], 0.0])
    return Box(name, 'breadboard', T, [-L / 2, -W / 2, -H], [L / 2, W / 2, 0.0], pockets, labels,
               [0, 0, -1], [1, 0, 0], [0, 1, 0], spec['clearance_mm'] * mm, spec['funnel_mm'] * mm,
               spec['funnel_depth_mm'] * mm, spec['depth_mm'] * mm, spec.get('hold_N', 0.0),
               spec.get('hold_from_mm', 0.0) * mm, spec.get('hold_full_mm', 0.0) * mm)


class World:
    def __init__(self, path=WORLD_FILE, truth_offset_mm=None):
        cfg = yaml.safe_load(Path(path).read_text())
        self.cfg = cfg
        self.floor_z = float(cfg['floor_z'])
        self.k = float(cfg['stiffness'])
        self.mu = float(cfg['friction'])
        self.stop_N = float(cfg['protective_stop_N'])
        self.noise = float(cfg['ft_noise_N'])
        self.home_q = list(cfg['home_q'])
        self.objects, self.sources = [], {}
        for name, spec in cfg['objects'].items():
            T, scale, src = _pose(name, spec)
            off = truth_offset_mm if truth_offset_mm is not None else spec.get('truth_offset_mm', [0, 0, 0])
            T = _offset(T, off)
            self.sources[name] = {'source': src, 'scale': scale.tolist(), 'truth_offset_mm': list(off)}
            build = _switch if spec['kind'] == 'switch' else _breadboard
            self.objects.append(build(name, spec, T, scale))

    def contact(self, p, vel):
        """Contact wrench on the tool at base point p moving with vel: (F[3], info)."""
        best_pen, best_n, info, pocket = 0.0, None, None, None
        if p[2] < self.floor_z:
            best_pen, best_n, info = self.floor_z - p[2], np.array([0.0, 0.0, 1.0]), {'object': 'plate'}
        for o in self.objects:
            pen, n, inf = o.contact(p)
            if inf and inf.get('inside'):
                pocket = (o, inf)
            if pen > best_pen:
                best_pen, best_n, info = pen, n, inf
        F = np.zeros(3)
        if best_pen > 0:
            fn = self.k * best_pen
            F = fn * best_n
            vt = vel - (vel @ best_n) * best_n
            if np.linalg.norm(vt) > 1e-5:
                F -= self.mu * fn * vt / np.linalg.norm(vt)
            info = {**(info or {}), 'pen': best_pen, 'normal': best_n.tolist()}
        if pocket:
            # holding friction along the pocket (contact springs, latch): static up to `hold`.
            # Force mode applies it as a deadband (robot.py); here it shows up when moved by position
            o, inf = pocket
            ax = o.R @ o.axis
            va = vel @ ax
            d = inf['depth']
            ramp = 1.0 if o.hold_full <= o.hold_from else (d - o.hold_from) / (o.hold_full - o.hold_from)
            hold = o.hold * float(np.clip(ramp, 0.0, 1.0)) if d > o.hold_from else 0.0
            Fh = -hold * np.sign(va) * ax if hold and abs(va) > 1e-5 else np.zeros(3)
            F += Fh
            if info is None or info.get('object') == o.name:
                info = {**(info or {}), **inf}
            info.update(hold=hold, hold_axis=ax.tolist(), hold_F=Fh.tolist())
        return F, info

    def describe(self):
        return {'floor_z': self.floor_z, 'objects': [o.describe() for o in self.objects],
                'sources': self.sources}
