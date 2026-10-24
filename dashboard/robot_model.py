"""UR3e surface model for registration: the official collision meshes
(Universal_Robots_ROS2_Description, BSD-3) on the URDF joint chain, sampled to
points with normals and posed by the joint angles. Same chain as
web/ur3e_model.js; the root is the controller base frame."""

from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

MESHES = Path(__file__).resolve().parent / 'models' / 'ur3e_collision'
PI = np.pi
# mesh, joint origin (xyz, rpy) from the parent link, mesh offset (xyz, rpy)
CHAIN = [
    ('base', None, ([0, 0, 0], [0, 0, PI])),
    ('shoulder', ([0, 0, 0.15185], [0, 0, 0]), ([0, 0, 0], [0, 0, PI])),
    ('upperarm', ([0, 0, 0], [PI / 2, 0, 0]), ([0, 0, 0.120], [PI / 2, 0, -PI / 2])),
    ('forearm', ([-0.24355, 0, 0], [0, 0, 0]), ([0, 0, 0.027], [PI / 2, 0, -PI / 2])),
    ('wrist1', ([-0.2132, 0, 0.13105], [0, 0, 0]), ([0, 0, -0.104], [PI / 2, 0, 0])),
    ('wrist2', ([0, -0.08535, 0], [PI / 2, 0, 0]), ([0, 0, -0.08535], [0, 0, 0])),
    ('wrist3', ([0, 0.0921, 0], [PI / 2, PI, PI]), ([0, 0, -0.0921], [PI / 2, 0, 0])),
]


def _origin(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = R.from_euler('xyz', rpy).as_matrix()     # URDF: Rz(yaw) Ry(pitch) Rx(roll)
    T[:3, 3] = xyz
    return T


def _rotz(a):
    T = np.eye(4)
    T[:2, :2] = [[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]
    return T


def read_stl(path):
    """Binary STL -> triangles (n, 3, 3)."""
    data = Path(path).read_bytes()
    n = int(np.frombuffer(data, '<u4', 1, 80)[0])
    rec = np.frombuffer(data, np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]), n, 84)
    return rec['v'].astype(np.float64)


def sample(tris, spacing, rng):
    """Area-weighted surface samples with face normals."""
    e1, e2 = tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]
    cr = np.cross(e1, e2)
    area = 0.5 * np.linalg.norm(cr, axis=1)
    n = max(1, int(area.sum() / spacing ** 2))
    idx = rng.choice(len(tris), n, p=area / area.sum())
    u, v = rng.random((2, n))
    flip = u + v > 1
    u[flip], v[flip] = 1 - u[flip], 1 - v[flip]
    pts = tris[idx, 0] + u[:, None] * e1[idx] + v[:, None] * e2[idx]
    nrm = cr[idx] / np.maximum(np.linalg.norm(cr[idx], axis=1, keepdims=True), 1e-12)
    return pts, nrm


class UR3eSurface:
    def __init__(self, spacing=0.006, seed=0):
        rng = np.random.default_rng(seed)
        self.links = []
        for mesh, joint, vis in CHAIN:
            p, n = sample(read_stl(MESHES / f'{mesh}.stl'), spacing, rng)
            V = _origin(*vis)
            self.links.append((None if joint is None else _origin(*joint),
                               p @ V[:3, :3].T + V[:3, 3], n @ V[:3, :3].T))

    def link_frames(self, q):
        T = np.eye(4)
        out = []
        for i, (joint, _, _) in enumerate(self.links):
            if joint is not None:
                T = T @ joint @ _rotz(q[i - 1])
            out.append(T.copy())
        return out

    def points(self, q):
        """Surface points and outward normals in the base frame at joints q."""
        P, N = [], []
        for T, (_, p, n) in zip(self.link_frames(q), self.links):
            P.append(p @ T[:3, :3].T + T[:3, 3])
            N.append(n @ T[:3, :3].T)
        return np.vstack(P), np.vstack(N)
