"""CAD feature models (built by assets/models/cad/*.py) and their fit parameters.

A target object's pose is T_base_obj (4x4, metres) plus per-axis pitch scales: the CAD
grid is nominal, the real part (or the robot's tool chain) can stretch it slightly — the
LS108G seats came out at ~15.0 mm/port in tip coordinates against 14.55 mm nominal. The
scale multiplies the object-frame x (switch: along the port row; breadboard: across the
columns) and y (breadboard: along the rows) about the object origin.
"""
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
CAD_DIR = ROOT / 'dashboard' / 'web' / 'models' / 'cad'


@dataclass
class Feature:
    name: str
    p: np.ndarray        # object frame, metres
    axis: np.ndarray     # insertion direction, object frame (unit)


class CadModel:
    """Features by name: 'port:4' / 'port:4/seat' (switch), 'hole:a1' (breadboard)."""

    def __init__(self, name):
        self.name = name
        self.spec = json.loads((CAD_DIR / f'{name}.json').read_text())
        self.kind = 'switch' if 'ports' in self.spec else 'breadboard'

    @property
    def size(self):
        return np.array(self.spec['size']) / 1000

    def feature(self, fid):
        kind, _, key = fid.partition(':')
        key, _, sub = key.partition('/')
        if kind == 'port' and self.kind == 'switch':
            pt = self.spec['ports'][key]
            p, axis = np.array(pt['centre']) / 1000, np.array(pt['axis'], float)
            if sub == 'seat':
                p = p + axis * self.spec['seat_depth'] / 1000
            return Feature(fid, p, axis)
        if kind == 'hole' and self.kind == 'breadboard':
            p = np.array(self.spec['holes'][key]) / 1000
            if sub == 'seat':
                p = p + np.array([0, 0, -self.spec['hole']['chamfer_depth'] / 1000])
            return Feature(fid, p, np.array([0.0, 0.0, -1.0]))
        raise KeyError(f'{self.name}: unknown feature {fid!r} '
                       f'(use {"port:1..8[/seat]" if self.kind == "switch" else "hole:a1..j30|L+1..R-25[/seat]"})')

    def features(self):
        keys = self.spec['ports'] if self.kind == 'switch' else self.spec['holes']
        pre = 'port' if self.kind == 'switch' else 'hole'
        return [f'{pre}:{k}' for k in keys]

    def outline_top(self):
        """Top-face rectangle corners in the object frame (for image refinement)."""
        w, d, h = self.size
        z = h if self.kind == 'switch' else 0.0
        cy = 0.0
        return np.array([[-w / 2, cy - d / 2, z], [w / 2, cy - d / 2, z],
                         [w / 2, cy + d / 2, z], [-w / 2, cy + d / 2, z]])

    def top_z(self):
        """Height of the top face above the object origin."""
        return self.size[2] if self.kind == 'switch' else 0.0

    def bottom_z(self):
        return 0.0 if self.kind == 'switch' else -self.size[2]


def apply(T, scale, p):
    """Object-frame point(s) (3,) or (N,3) -> base, with the pitch scales on object x / y."""
    q = np.asarray(p, float) * [scale[0], scale[1], 1.0]
    return q @ T[:3, :3].T + T[:3, 3]


def pose(x, y, z, yaw, roll=0.0, pitch=0.0):
    """4x4 from translation + Rz(yaw) Ry(pitch) Rx(roll)."""
    cz, sz, cy, sy, cx, sx = np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch), np.cos(roll), np.sin(roll)
    R = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]) @ np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]]) \
        @ np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, [x, y, z]
    return T


def yaw_of(T):
    return float(np.arctan2(T[1, 0], T[0, 0]))
