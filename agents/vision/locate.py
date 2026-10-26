"""Locate a box-shaped object (e.g. a protoboard) in the scene and return its bounding box.

World frame: the calibration checkerboard (origin at its centre, z up, metres). The object
rests on the plate, so its box is world-z aligned and its pose has three degrees of freedom:
x, y and yaw (long axis from world x).

1. Coarse, cloud: the cloud becomes a height map over the fitted support plane; a model-
   sized rectangle is matched against the cells at the object's height over x, y and yaw.
   Further matches are kept only while they continue the object (parallel and flush), so a
   half-breadboard model finds both halves of a full board; one rectangle is then refit over
   all of them (`ObjectSpec.instances` fixes their number when clutter breaks that test).
2. Fine, cloud: the rectangle's sides are fitted to the object-top and plate points around
   them, with a bootstrap 1σ and the share of each side the cloud actually sees.
3. Fine, images, when step 2 is not enough (1σ or coverage over tolerance): the box's
   projected outline is fitted to the edges of the frames of the cameras whose pose is
   trusted. The base height comes from `ObjectSpec.plate_z`, since one view cannot tell it
   from y. Other cameras are fitted alone as a cross-check.

Only numpy and scipy are needed, so the calibration dashboard (its own environment) and the
agents share this module. Developed in tmp/locate_breadboard.py.
"""

import json
import re
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Literal

import numpy as np
import numpy.typing as npt
import yaml
from scipy import fft, ndimage
from scipy.optimize import minimize

from agents.vision.icp import icp_fit_either_way, object_footprint

# box corner order: bottom (−u−v, +u−v, +u+v, −u+v), then the same on top
EDGES = [
    (0, 1),
    (1, 2),
    (2, 3),
    (3, 0),
    (4, 5),
    (5, 6),
    (6, 7),
    (7, 4),
    (0, 4),
    (1, 5),
    (2, 6),
    (3, 7),
]
Array = npt.NDArray[Any]
FRAME = "world (checkerboard centre, z up), metres; box edges vertical along world z"


class ObjectNotFound(RuntimeError):
    """No footprint of the object's size scored above `min_score`."""


@dataclass(frozen=True)
class ObjectSpec:
    """An object to look for: a box of `size` (one model instance) resting on the plate."""

    name: str
    label: str
    size: tuple[float, float, float]  # L >= W along the plate, H the height; metres
    instances: int | None = None  # model copies end to end that make the object; None = detect
    min_score: float = 0.45  # template score to accept a match (1 = perfect)
    plate_z: float | None = None  # world z of the plate top (m); None = from the cloud
    yaw_deg: float | None = None  # fix the yaw (only x and y are fitted)
    images: Literal["auto", "always", "never"] = "auto"
    tol_mm: float = 0.5  # auto: a cloud 1σ above this (x or y) calls for images
    tol_deg: float = 0.15  # auto: a cloud 1σ above this (yaw) calls for images
    min_coverage: float = 0.5  # auto: a side seen less than this calls for images
    cell_mm: float = 2.0  # height-map cell of the coarse search
    check_tol_mm: float = 3.0  # cross-checks (other cameras, plate z) disagreeing more: a warning
    check_tol_deg: float = 1.5
    fit: Literal["icp", "edges"] = "icp"  # cloud fit: 3-DoF ICP against the CAD, or the edge fit
    model: str | None = None  # CAD file (STEP): the ICP's footprint; else the size's rectangle


def load_objects(path: str | Path) -> dict[str, ObjectSpec]:
    """`objects:` mapping of name -> spec fields (sizes in mm) from a YAML file."""
    raw = yaml.safe_load(Path(path).read_text()) or {}
    specs = {}
    for name, o in (raw.get("objects") or {}).items():
        o = dict(o)
        L, W, H = (v / 1000 for v in o.pop("size_mm"))
        plate = o.pop("plate_z_mm", None)
        if o.get("model") and not Path(o["model"]).is_absolute():  # relative to the repository root
            o["model"] = str(Path(path).resolve().parent.parent / o["model"])
        specs[name] = ObjectSpec(
            name=name,
            label=o.pop("label", name),
            size=(L, W, H),
            plate_z=None if plate is None else plate / 1000,
            **o,
        )
    return specs


@dataclass
class CameraView:
    """One camera's frame with the pose and intrinsics the dashboard holds for it."""

    name: str
    K: Array
    dist: Array
    size: tuple[int, int]  # (w, h) the intrinsics are for
    img: Array  # RGB uint8; may be downscaled from `size`
    T_world_cam: Array
    trusted: bool = True
    why: str = ""
    used: bool = field(default=False, init=False)

    def __post_init__(self) -> None:
        self.T_cam_world = np.linalg.inv(self.T_world_cam)


# ── models and clouds ────────────────────────────────────────────────────────

_STEP_PREFIX = {
    "$": 1.0,
    ".MILLI.": 1e-3,
    ".CENTI.": 1e-2,
    ".DECI.": 1e-1,
    ".MICRO.": 1e-6,
    ".KILO.": 1e3,
}


def read_step(path: str | Path) -> Array:
    """Vertex positions of a single-part STEP (AP203/AP214) file, in metres.

    Only VERTEX_POINTs are used: the other CARTESIAN_POINTs include line origins and axis
    placements, which lie outside the solid.
    """
    t = Path(path).read_text(errors="replace")
    if "NEXT_ASSEMBLY_USAGE_OCCURRENCE" in t:
        raise ValueError("STEP assemblies are not supported: export a single part")
    unit = re.search(
        r"LENGTH_UNIT\s*\(\s*\)\s*NAMED_UNIT\s*\(\s*\*\s*\)\s*SI_UNIT\s*\(\s*([^,]+),\s*\.METRE\.",
        t,
    )
    scale = _STEP_PREFIX[unit.group(1).strip()] if unit else 1e-3
    cp = dict(re.findall(r"#(\d+)\s*=\s*CARTESIAN_POINT\s*\(\s*'[^']*'\s*,\s*\(([^)]*)\)", t))
    vp = re.findall(r"VERTEX_POINT\s*\(\s*'[^']*'\s*,\s*#(\d+)", t)
    return np.array([list(map(float, cp[v].split(","))) for v in vp]) * scale


def model_size(points: Array) -> tuple[float, float, float]:
    """(L, W, H) of a model's vertices: the two longest extents lie on the plate."""
    ext = np.sort(points.max(0) - points.min(0))[::-1]
    return float(ext[0]), float(ext[1]), float(ext[2])


def read_ply(path: str | Path) -> tuple[Array, Array]:
    """Binary little-endian PLY with float x, y, z and (optionally) uchar red, green, blue."""
    data = Path(path).read_bytes()
    end = data.index(b"end_header\n") + len(b"end_header\n")
    header = data[:end].decode().splitlines()
    n = next(int(line.split()[2]) for line in header if line.startswith("element vertex"))
    types = {"float": "<f4", "double": "<f8", "uchar": "u1", "int": "<i4"}
    fields = [(ln.split()[2], types[ln.split()[1]]) for ln in header if ln.startswith("property")]
    v = np.frombuffer(data, np.dtype(fields), n, end)
    xyz = np.stack([v["x"], v["y"], v["z"]], 1).astype(float)
    if "red" in (v.dtype.names or ()):
        rgb = np.stack([v["red"], v["green"], v["blue"]], 1)
    else:
        rgb = np.full((n, 3), 128, np.uint8)
    return xyz, rgb


_RGB = [("red", "u1"), ("green", "u1"), ("blue", "u1")]


def write_ply(path: str | Path, xyz: Array, rgb: Array, comment: str = FRAME) -> None:
    v = np.empty(len(xyz), [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), *_RGB])
    v["x"], v["y"], v["z"] = np.asarray(xyz).T
    v["red"], v["green"], v["blue"] = np.asarray(rgb).T
    head = (
        f"ply\nformat binary_little_endian 1.0\ncomment {comment}\nelement vertex {len(v)}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n"
    )
    Path(path).write_bytes(head.encode() + v.tobytes())


# ── coarse search ────────────────────────────────────────────────────────────


def fit_plane(
    P: Array, iters: int = 400, tol: float = 0.002, seed: int = 0, sample: int = 4000
) -> tuple[Array, Array]:
    """RANSAC plane, refined by SVD on its inliers: (normal with +z, centroid).

    Hypotheses are scored on a random subsample of `sample` points (the best one wins by
    a wide margin on a table scene); the inliers of the winner are taken from all points.
    """
    rng = np.random.default_rng(seed)
    S = P[rng.choice(len(P), sample, replace=False)] if len(P) > sample else P
    best, best_n, best_a = -1, None, None
    for _ in range(iters):
        a, b, c = S[rng.integers(0, len(S), 3)]  # a repeated point gives n = 0: skipped
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        k = int((np.abs((S - a) @ n) < tol).sum())
        if k > best:
            best, best_n, best_a = k, n, a
    assert best_n is not None
    Q = P[np.abs((P - best_a) @ best_n) < tol]
    c = Q.mean(0)
    n = np.linalg.eigh((Q - c).T @ (Q - c))[1][:, 0]  # least-spread direction of the inliers
    return n * np.sign(n[2]), c


def plane_frame(n: Array) -> Array:
    """Rotation whose z is the plane normal and whose x is world x projected on the plane."""
    x = np.array([1.0, 0, 0]) - n[0] * n
    x /= np.linalg.norm(x)
    return np.stack([x, np.cross(n, x), n], 1)  # columns: plane axes in world


def rect_kernel(L: float, W: float, yaw: float, ring: float, cell: float) -> Array:
    """+1/area inside an L x W rectangle at yaw, −1/area in a ring around it."""
    h = int(np.ceil((np.hypot(L, W) / 2 + ring) / cell)) + 1
    g = np.arange(-h, h + 1) * cell
    X, Y = np.meshgrid(g, g, indexing="ij")
    u = np.cos(yaw) * X + np.sin(yaw) * Y
    v = -np.sin(yaw) * X + np.cos(yaw) * Y
    inside = (np.abs(u) <= L / 2) & (np.abs(v) <= W / 2)
    outer = (np.abs(u) <= L / 2 + ring) & (np.abs(v) <= W / 2 + ring) & ~inside
    return np.asarray(inside / inside.sum() - outer / outer.sum())


def correlate(F: Array, shape: tuple[int, int], k: Array) -> Array:
    """Correlation of a precomputed padded rFFT with a kernel, cropped to `shape`."""
    pad = (F.shape[0], (F.shape[1] - 1) * 2)
    K = fft.rfft2(k[::-1, ::-1], pad, workers=-1)
    out = fft.irfft2(F * K, pad, workers=-1)
    h0, h1 = k.shape[0] // 2, k.shape[1] // 2
    return np.asarray(out[h0 : h0 + shape[0], h1 : h1 + shape[1]])


def adjacent(
    a: dict[str, Any],
    b: dict[str, Any],
    L: float,
    W: float,
    tol: float = 0.006,
    max_dyaw: float = np.radians(5),
) -> bool:
    """Two footprints of the same size, parallel and touching end to end or side to side."""
    dyaw = abs((a["yaw"] - b["yaw"] + np.pi / 2) % np.pi - np.pi / 2)
    if dyaw > max_dyaw:
        return False
    u = np.array([np.cos(a["yaw"]), np.sin(a["yaw"])])
    v = np.array([-u[1], u[0]])
    d = b["centre_xy"] - a["centre_xy"]
    du, dv = abs(d @ u), abs(d @ v)
    return bool((abs(du - L) < tol and dv < tol) or (abs(dv - W) < tol and du < tol))


def coarse_search(
    xyz: Array,
    L: float,
    W: float,
    H: float,
    cell: float = 0.002,
    ring: float = 0.006,
    min_score: float = 0.45,
    max_boards: int = 6,
    fixed_yaw: float | None = None,
    instances: int | None = None,
) -> dict[str, Any]:
    """Best model-sized footprint, plus further ones only where they continue the object.

    The object rests on the support plane (z, roll and pitch fixed): the search is over x, y
    and yaw, or x and y only when `fixed_yaw` (radians) is given.
    """
    near = np.abs(xyz[:, 2]) < 0.03  # the support plane: the dominant plane near z = 0
    n, c = fit_plane(xyz[near])
    R = plane_frame(n)
    p = (xyz - c) @ R  # plane coordinates (z = height over the plate)
    roi = np.abs(p[:, 2]) < 0.05
    # the search runs in world x, y (the box stays world-z aligned); heights over the plane
    lo = xyz[roi, :2].min(0)
    ij = np.floor((xyz[:, :2] - lo) / cell).astype(int)
    shape = tuple(ij[roi].max(0) + 1)

    # per-cell share of points at the object's height: raised (1), ground or unseen (0)
    band = roi & (p[:, 2] > H - 0.0035) & (p[:, 2] < H + 0.005)
    tot, hit = np.zeros(shape), np.zeros(shape)
    np.add.at(tot, tuple(ij[roi].T), 1)
    np.add.at(hit, tuple(ij[band].T), 1)
    filled = np.where(tot > 0, hit / np.maximum(tot, 1), 0.0)
    filled = ndimage.grey_closing(filled, size=3)  # bridge the gaps of a sparse cloud

    kmax = int(np.ceil((np.hypot(3 * L, 3 * W) / 2 + ring) / cell)) + 1  # room for the object
    pad = (shape[0] + 2 * kmax + 1, shape[1] + 2 * kmax + 1)
    F = F_all = fft.rfft2(filled, pad, workers=-1)
    work = filled.copy()

    def best_match(F: Array, yaws: Any, best: tuple[Any, ...]) -> tuple[Any, ...]:
        for yaw in yaws:
            s = correlate(F, shape, rect_kernel(L, W, yaw, ring, cell))
            k = int(np.argmax(s))
            if s.flat[k] > best[0]:
                best = (s.flat[k], yaw, np.unravel_index(k, shape))
        return best

    boards: list[dict[str, Any]] = []
    for _ in range(max_boards):
        if fixed_yaw is None:
            best = best_match(F, np.radians(np.arange(0, 180, 2.0)), (-np.inf, None, None))
            best = best_match(F, best[1] + np.radians(np.arange(-2, 2.01, 0.25)), best)
        else:
            best = best_match(F, [fixed_yaw], (-np.inf, None, None))
        score, yaw, (i, j) = best
        if score < min_score:
            break
        centre = lo + (np.array([i, j]) + 0.5) * cell
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        uv = (xyz[:, :2] - centre) @ rot
        on = band & (np.abs(uv[:, 0]) < L / 2) & (np.abs(uv[:, 1]) < W / 2)
        found = {
            "score": float(score),
            "yaw": float(yaw),
            "centre_xy": centre,
            "top_mm": float(np.median(p[on, 2]) * 1000) if on.any() else None,
            "n_points": int(on.sum()),
        }
        if boards and not any(adjacent(b, found, L, W) for b in boards):
            break  # the next best match is somewhere else: the object is complete
        boards.append(found)
        # suppress this footprint in the data, then search again
        k = rect_kernel(L, W, yaw, 0.0005, cell) > 0
        mask = np.zeros(shape, bool)
        h = k.shape[0] // 2
        si = slice(max(i - h, 0), i + h + 1)
        sj = slice(max(j - h, 0), j + h + 1)
        mask[si, sj] = k[
            h - (i - si.start) : h + (si.stop - i), h - (j - sj.start) : h + (sj.stop - j)
        ]
        work[mask] = 0
        F = fft.rfft2(work, pad, workers=-1)

    # the whole object: one rectangle spanning all the instances, refit with a single pose
    obj = None
    if boards:
        u0 = np.array([np.cos(boards[0]["yaw"]), np.sin(boards[0]["yaw"])])
        v0 = np.array([-u0[1], u0[0]])
        d = np.array([b["centre_xy"] for b in boards])
        if instances:  # known composition: slide it along the long axis
            nu, nv = instances, 1
            win = int(((instances - 1) * L / 2 + 0.01) / cell)
        else:
            nu = int(round(np.ptp(d @ u0) / L)) + 1
            nv = int(round(np.ptp(d @ v0) / W)) + 1
            win = int(0.01 / cell)  # stay within 1 cm of the instances' centre
        oL, oW = nu * L, nv * W
        gi, gj = np.floor((d.mean(0) - lo) / cell).astype(int)
        yaw0 = np.mean([b["yaw"] for b in boards])
        yaws = (
            [fixed_yaw] if fixed_yaw is not None else yaw0 + np.radians(np.arange(-4, 4.01, 0.25))
        )
        whole: tuple[Any, Any, Any] = (-np.inf, None, None)
        for yaw in yaws:
            s = correlate(F_all, shape, rect_kernel(oL, oW, yaw, ring, cell))
            i0, j0 = max(gi - win, 0), max(gj - win, 0)
            s = s[i0 : gi + win + 1, j0 : gj + win + 1]
            k2 = np.unravel_index(np.argmax(s), s.shape)
            if s[k2] > whole[0]:
                whole = (s[k2], yaw, (k2[0] + i0, k2[1] + j0))
        score, yaw, (i, j) = whole
        obj = {
            "score": float(score),
            "yaw": float(yaw),
            "L": oL,
            "W": oW,
            "grid": (nu, nv),
            "centre_xy": lo + (np.array([i, j]) + 0.5) * cell,
        }
    return {"n": n, "c": c, "boards": boards, "obj": obj}


# ── 3-DOF pose of a world-z aligned box ──────────────────────────────────────


def box_corners(xy: Array, yaw: float, L: float, W: float, z0: float, H: float) -> Array:
    """8 world corners of an L x W x H box, vertical edges along world z, base at z0."""
    u = np.array([np.cos(yaw), np.sin(yaw)])
    v = np.array([-u[1], u[0]])
    q = [xy + su * L / 2 * u + sv * W / 2 * v for su, sv in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    return np.array([[x, y, z] for z in (z0, z0 + H) for x, y in q])


def signed_dist(xy: Array, pose: Any, L: float, W: float) -> Array:
    """Signed distance (m) of points to the pose's L x W rectangle; negative inside."""
    x, y, yaw = pose
    d = xy - [x, y]
    u = d @ [np.cos(yaw), np.sin(yaw)]
    v = d @ [-np.sin(yaw), np.cos(yaw)]
    qx, qy = np.abs(u) - L / 2, np.abs(v) - W / 2
    return np.asarray(
        np.hypot(np.maximum(qx, 0), np.maximum(qy, 0)) + np.minimum(np.maximum(qx, qy), 0)
    )


def plate_height(xyz: Array, h: Array, pose: Any, L: float, W: float) -> tuple[float, int]:
    """World z of the plate around the footprint: median of ground points 5–40 mm outside."""
    sd = signed_dist(xyz[:, :2], pose, L, W)
    ring = (sd > 0.005) & (sd < 0.04) & (np.abs(h) < 0.0025)
    if not ring.any():
        return float(np.median(xyz[np.abs(h) < 0.0025, 2])), 0
    return float(np.median(xyz[ring, 2])), int(ring.sum())


_SCALE = np.array([1000.0, 1000.0, np.degrees(1)])  # (m, m, rad) -> (mm, mm, deg)


def _refine(
    cost: Any, pose0: Any, step_mm: float = 1.0, step_deg: float = 0.3
) -> tuple[Any, float]:
    """Nelder–Mead over (x mm, y mm, yaw deg) around pose0."""
    x0 = np.asarray(pose0) * _SCALE
    simplex = np.array([x0, x0 + [step_mm, 0, 0], x0 + [0, step_mm, 0], x0 + [0, 0, step_deg]])
    r = minimize(
        lambda q: cost(q / _SCALE),
        x0,
        method="Nelder-Mead",
        options={"initial_simplex": simplex, "xatol": 0.01, "fatol": 1e-9, "maxiter": 2000},
    )
    return r.x / _SCALE, float(r.fun)


def sigmoid(x: Array) -> Array:
    return 0.5 * (1 + np.tanh(0.5 * x))


def cloud_fit(
    xyz: Array,
    h: Array,
    pose0: Any,
    L: float,
    W: float,
    H: float,
    n_boot: int = 40,
    seed: int = 0,
) -> dict[str, Any]:
    """Edge fit on the cloud: object-top points inside, plate points outside the footprint.

    The cost counts soft misclassifications (logistic in the signed distance), so it is
    driven by the points near the four sides; intermediate heights (side walls, mixed depth
    pixels) are left out. Returns the pose, its bootstrap 1σ and per-side coverage.
    """
    sd0 = signed_dist(xyz[:, :2], pose0, L, W)
    near = np.abs(sd0) < 0.02
    top = xyz[near & (h > H - 0.0035) & (h < H + 0.004), :2]
    gnd = xyz[near & (np.abs(h) < 0.0025), :2]

    def make_cost(t: Array, g: Array, sigma: float) -> Any:
        return lambda q: (
            sigmoid(signed_dist(t, q, L, W) / sigma).sum()
            + sigmoid(-signed_dist(g, q, L, W) / sigma).sum()
        )

    pose = np.asarray(pose0, float)
    for sigma in (0.002, 0.001, 0.0005):
        pose, _ = _refine(make_cost(top, gnd, sigma), pose)
    rng = np.random.default_rng(seed)
    boot = []
    for _ in range(n_boot):
        t = top[rng.integers(0, len(top), len(top))]
        g = gnd[rng.integers(0, len(gnd), len(gnd))]
        boot.append(_refine(make_cost(t, g, 0.0005), pose, 0.3, 0.1)[0])
    std = np.array(boot).std(0) if boot else np.zeros(3)
    cover = side_coverage(top, gnd, pose, L, W)
    return {"pose": pose, "std": std, "coverage": cover, "n_top": len(top), "n_ground": len(gnd)}


def side_coverage(top: Array, gnd: Array, pose: Any, L: float, W: float) -> dict[str, float]:
    """Per side of the L x W footprint: the share of 2 mm stations along it with object-top
    points (`top`, x y) just inside and plate points (`gnd`) just outside, i.e. seen."""
    x, y, yaw = pose
    u = np.array([np.cos(yaw), np.sin(yaw)])
    v = np.array([-u[1], u[0]])
    cover = {}
    sides = (
        ("+u end", u, v, W / 2, L / 2),
        ("-u end", -u, v, W / 2, L / 2),
        ("+v side", v, u, L / 2, W / 2),
        ("-v side", -v, u, L / 2, W / 2),
    )
    for name, nrm, along, half_len, off in sides:
        st = np.arange(-half_len + 0.004, half_len - 0.004, 0.002)
        base = np.array([x, y]) + off * nrm + st[:, None] * along
        ok = []
        for b in base:
            din = np.hypot(*(top - (b - 0.002 * nrm)).T).min() if len(top) else 1
            dout = np.hypot(*(gnd - (b + 0.002 * nrm)).T).min() if len(gnd) else 1
            ok.append(din < 0.0025 and dout < 0.0025)
        cover[name] = float(np.mean(ok)) if ok else 0.0
    return cover


def fit_cloud(
    spec: ObjectSpec,
    xyz: Array,
    h: Array,
    pose0: Any,
    L: float,
    W: float,
    H: float,
    grid: tuple[int, int],
    n_boot: int = 10,
) -> dict[str, Any]:
    """The cloud fit `spec.fit` asks for, with a common result: pose, 1σ, side coverage."""
    if spec.fit == "edges":
        r = cloud_fit(xyz, h, pose0, L, W, H, n_boot=2 * n_boot)
        r["method"] = "edges"
        return r
    poly = object_footprint(spec.size, grid, spec.model)
    r = icp_fit_either_way(xyz, h, pose0, poly, H, n_boot=n_boot)
    obj, free = r.pop("obj"), r.pop("free")
    top = obj[np.abs(obj[:, 2] - H) < 0.0035, :2]
    r.update(
        method="icp",
        coverage=side_coverage(top, free, r["pose"], L, W),
        n_top=len(top),
        n_ground=len(free),
        n_object=len(obj),
        footprint_vertices=len(poly),
    )
    return r


# ── image fit ────────────────────────────────────────────────────────────────


def project(cam: CameraView, P: Array) -> tuple[Array, Array]:
    """World points -> image pixels (pinhole + Brown–Conrady distortion), and camera depth."""
    T = cam.T_cam_world
    x = P @ T[:3, :3].T + T[:3, 3]
    a, b = x[:, 0] / x[:, 2], x[:, 1] / x[:, 2]
    k1, k2, p1, p2, k3 = (list(cam.dist) + [0.0] * 5)[:5]
    r2 = a * a + b * b
    rad = 1 + k1 * r2 + k2 * r2**2 + k3 * r2**3
    ad = a * rad + 2 * p1 * a * b + p2 * (r2 + 2 * a * a)
    bd = b * rad + p1 * (r2 + 2 * b * b) + 2 * p2 * a * b
    s = cam.img.shape[1] / cam.size[0]  # the image may be downscaled
    K = cam.K
    uv = np.stack([(K[0, 0] * ad + K[0, 2]) * s, (K[1, 1] * bd + K[1, 2]) * s], 1)
    return uv, x[:, 2]


def _hull(uv: Array) -> list[int]:
    """Indices of the 2D convex hull, counter-clockwise (monotone chain)."""
    order = np.lexsort((uv[:, 1], uv[:, 0]))

    def cross(a: int, b: int, c: int) -> float:
        return float(
            (uv[b, 0] - uv[a, 0]) * (uv[c, 1] - uv[a, 1])
            - (uv[b, 1] - uv[a, 1]) * (uv[c, 0] - uv[a, 0])
        )

    lo: list[int] = []
    hi: list[int] = []
    for i in order:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], i) <= 0:
            lo.pop()
        lo.append(int(i))
    for i in order[::-1]:
        while len(hi) >= 2 and cross(hi[-2], hi[-1], i) <= 0:
            hi.pop()
        hi.append(int(i))
    return lo[:-1] + hi[:-1]


def silhouette(
    cam: CameraView, corners: Array, trim: float = 0.1, step_px: float = 2.0
) -> list[tuple[Any, Any]]:
    """The box's projected outline, per edge: sample points and their outward normals.

    Edges are sampled in 3D every ~`step_px` and projected point by point (all edges in one
    call), so lens distortion bends them as in the image; the ends of each edge (near
    corners) are left out.
    """
    uv, z = project(cam, corners)
    if (z <= 0).any():
        return []
    hull = _hull(uv)
    centre = uv[hull].mean(0)
    pairs = list(zip(hull, hull[1:] + hull[:1], strict=True))
    ns = [max(int(np.linalg.norm(uv[j] - uv[i]) / step_px), 4) for i, j in pairs]
    P = np.concatenate(
        [
            corners[i] + np.linspace(trim, 1 - trim, n)[:, None] * (corners[j] - corners[i])
            for (i, j), n in zip(pairs, ns, strict=True)
        ]
    )
    Q, _ = project(cam, P)
    out = []
    for q in np.split(Q, np.cumsum(ns)[:-1]):
        d = np.gradient(q, axis=0)
        nn = np.stack([d[:, 1], -d[:, 0]], 1)
        nn /= np.linalg.norm(nn, axis=1, keepdims=True) + 1e-12
        nn *= np.sign(((q - centre) * nn).sum(1, keepdims=True))
        out.append((q, nn))
    return out


def edge_score(
    cam: CameraView, grad: Any, q: Any, L: float, W: float, z0: float, H: float
) -> float:
    """Mean image gradient across the outline, each outline edge weighted the same.

    Median along each edge, absolute gradient: the object is brighter than the plate in some
    places and darker in others. Equal edge weights keep the long sides from outvoting the
    short ends, which are the only ones that fix the position along the object.
    """
    gy, gx = grad
    h, w = gy.shape
    per_edge = []
    for pts, nn in silhouette(cam, box_corners(q[:2], q[2], L, W, z0, H)):
        ok = (pts[:, 0] > 1) & (pts[:, 0] < w - 2) & (pts[:, 1] > 1) & (pts[:, 1] < h - 2)
        if ok.sum() < 10:
            continue
        vy = ndimage.map_coordinates(gy, [pts[ok, 1], pts[ok, 0]], order=1)
        vx = ndimage.map_coordinates(gx, [pts[ok, 1], pts[ok, 0]], order=1)
        # median: a cable across an edge can't dominate it
        per_edge.append(np.median(np.abs(vx * nn[ok, 0] + vy * nn[ok, 1])))
    return float(np.mean(per_edge)) if per_edge else 0.0


def image_fit(
    cams: list[CameraView],
    init: Any,
    L: float,
    W: float,
    z0: float,
    H: float,
    fixed_yaw: float | None = None,
    span_mm: float = 20.0,
    span_deg: float = 2.0,
    restarts: int = 6,
) -> dict[str, Any]:
    """Pose whose projected box outline sits on the strongest image edges, in all `cams`.

    Coarse grid around `init` on a blurred image (wide basin), then Nelder–Mead from the
    best few cells, first blurred then sharp. Returns the best pose and how far apart the
    restarts that reach (nearly) the same score ended up.
    """
    grads = {
        sg: [np.gradient(ndimage.gaussian_filter(c.img.astype(float).mean(2), sg)) for c in cams]
        for sg in (2.5, 1.2)
    }

    def cost(q: Any, sg: float) -> float:
        if fixed_yaw is not None:
            q = np.array([q[0], q[1], fixed_yaw])
        return -float(
            np.mean(
                [edge_score(c, g, q, L, W, z0, H) for c, g in zip(cams, grads[sg], strict=True)]
            )
        )

    yaws = (
        [0.0] if fixed_yaw is not None else np.radians(np.arange(-span_deg, span_deg + 1e-9, 0.5))
    )
    grid = []
    steps = np.arange(-span_mm, span_mm + 1e-9, 2.0) / 1000
    for dx in steps:
        for dy in steps:
            for dyaw in yaws:
                q = np.asarray(init, float) + [dx, dy, dyaw]
                grid.append((cost(q, 2.5), tuple(q)))
    grid.sort()
    ends = []
    for _, q0 in grid[:restarts]:
        q = np.array(q0)
        for sg, step in ((2.5, 1.0), (1.2, 0.3)):
            x0 = q * _SCALE
            simplex = [x0, x0 + [step, 0, 0], x0 + [0, step, 0], x0 + [0, 0, step / 3]]
            r = minimize(
                lambda x, sg=sg: cost(x / _SCALE, sg),
                x0,
                method="Nelder-Mead",
                options={"initial_simplex": simplex, "xatol": 0.01, "fatol": 1e-7},
            )
            q = r.x / _SCALE
        if fixed_yaw is not None:
            q[2] = fixed_yaw
        ends.append((cost(q, 1.2), q))
    ends.sort(key=lambda t: t[0])
    best_c, best = ends[0]
    same = np.array([q for c, q in ends if c <= best_c * 0.99])  # within 1 % of the best score
    return {"pose": best, "score": -best_c, "restarts_agree": len(same), "spread": np.ptp(same, 0)}


# ── the whole pipeline ───────────────────────────────────────────────────────


def _pose(q: Any) -> dict[str, float]:
    return {
        "x": round(float(q[0]), 5),
        "y": round(float(q[1]), 5),
        "yaw_deg": round(float(np.degrees(q[2])), 3),
    }


def _mm_deg(v: Any, nd: int = 3) -> list[float]:
    return [round(float(a), nd) for a in np.asarray(v) * _SCALE]


def _delta(a: Any, b: Any) -> list[float]:
    """a - b (mm, mm, deg), yaw modulo 180°: a box turned half a turn covers the same spot."""
    d = np.asarray(a, float) - np.asarray(b, float)
    d[2] = (d[2] + np.pi / 2) % np.pi - np.pi / 2
    return _mm_deg(d, 2)


def locate_object(
    spec: ObjectSpec,
    xyz: Array,
    cams: list[CameraView] | None = None,
    log: Any = None,
    check_clouds: dict[str, Array] | None = None,
) -> dict[str, Any]:
    """Bounding box of `spec` in the world cloud `xyz` (N x 3), refined on `cams` if needed.

    `xyz` should come from cameras whose world pose is trusted; the clouds of the others go
    in `check_clouds` (name -> world cloud) and are fitted alone as a cross-check. Only
    trusted `cams` are used for the image fit; the others are fitted alone as a check.

    Returns a JSON-ready dict: the box (pose, centre, size, corners, model instances), the
    coarse and cloud-fit poses with 1σ and coverage, why images were (not) used, the image
    fit, the cross-checks and a `quality` verdict whose warnings say what to distrust.
    Raises ObjectNotFound when no footprint of the object's size scores high enough.
    """
    say = log or (lambda msg: None)
    t0 = time.time()
    cams = cams or []
    L, W, H = spec.size
    fixed_yaw = None if spec.yaw_deg is None else np.radians(spec.yaw_deg)

    res = coarse_search(
        xyz,
        L,
        W,
        H,
        cell=spec.cell_mm / 1000,
        min_score=spec.min_score,
        fixed_yaw=fixed_yaw,
        instances=spec.instances,
    )
    o = res["obj"]
    if o is None:
        raise ObjectNotFound(
            f"no {spec.label} in the cloud ({len(xyz)} points): no footprint of "
            f"{L * 1000:.0f} x {W * 1000:.0f} mm scored {spec.min_score} or more"
        )
    oL, oW, (nu, nv) = o["L"], o["W"], o["grid"]
    h = (xyz - res["c"]) @ res["n"]  # height above the fitted plate
    coarse = np.array([*o["centre_xy"], o["yaw"]])
    say(f"coarse ({nu} x {nv} instances, score {o['score']:.2f}): {_pose(coarse)}")

    cf = fit_cloud(spec, xyz, h, coarse, oL, oW, H, (nu, nv))
    if fixed_yaw is not None:
        cf["pose"][2], cf["std"][2] = fixed_yaw, 0.0
    z0_cloud, n_ring = plate_height(xyz, h, cf["pose"], oL, oW)
    z0 = z0_cloud if spec.plate_z is None else spec.plate_z
    sd_mm, sd_deg = cf["std"][:2].max() * 1000, np.degrees(cf["std"][2])
    say(f"cloud fit: {_pose(cf['pose'])}, 1σ {sd_mm:.2f} mm / {sd_deg:.3f}°")
    reasons = [
        r
        for r, bad in (
            (f"x/y 1σ {sd_mm:.2f} mm > {spec.tol_mm} mm", sd_mm > spec.tol_mm),
            (f"yaw 1σ {sd_deg:.3f}° > {spec.tol_deg}°", sd_deg > spec.tol_deg),
            (
                f"a side seen < {spec.min_coverage:.0%}",
                min(cf["coverage"].values()) < spec.min_coverage,
            ),
        )
        if bad
    ]

    # cross-checks: the other cameras' clouds alone (their pose is not trusted)
    checks: dict[str, Any] = {}
    own: dict[str, Array] = {}  # camera -> its own cloud fit
    for name, other in (check_clouds or {}).items():
        try:
            r = coarse_search(
                other,
                L,
                W,
                H,
                cell=spec.cell_mm / 1000,
                min_score=spec.min_score,
                fixed_yaw=fixed_yaw,
                instances=spec.instances,
            )
            if r["obj"] is None:
                raise ObjectNotFound("no match")
            ro = r["obj"]
            c2 = fit_cloud(
                spec,
                other,
                (other - r["c"]) @ r["n"],
                np.array([*ro["centre_xy"], ro["yaw"]]),
                oL,
                oW,
                H,
                (nu, nv),
                n_boot=8,
            )
            d = _delta(c2["pose"], cf["pose"])
            own[name] = c2["pose"]
            checks[f"{name} cloud"] = {
                **_pose(c2["pose"]),
                "delta_mm_mm_deg": d,
                "coverage": {k: round(v, 2) for k, v in c2["coverage"].items()},
            }
        except (ObjectNotFound, ValueError, AssertionError) as exc:
            checks[f"{name} cloud"] = {"error": str(exc)[:200]}

    final, method, img = cf["pose"], "cloud", None
    want = spec.images == "always" or (spec.images == "auto" and reasons)
    use = [c for c in cams if c.trusted]
    warnings = []
    if want and not use:
        warnings.append(
            "the cloud fit is not precise enough ("
            + "; ".join(reasons or ["image fit requested"])
            + ") and no camera pose is trusted for an image fit"
        )
    if want and use:
        for c in use:
            c.used = True
        say(
            "image fit on "
            + ", ".join(c.name for c in use)
            + ": "
            + "; ".join(reasons or ["requested"])
        )
        img = image_fit(use, cf["pose"], oL, oW, z0, H, fixed_yaw=fixed_yaw)
        final, method = img["pose"], "images (" + ", ".join(c.name for c in use) + ")"
        img["check_cameras"] = {}
        for c in [c for c in cams if not c.used]:  # cameras not used: a cross-check only
            # from that camera's own cloud fit when there is one: its pose may be off by more
            # than the search span, and the box's own parallel edges make local optima
            r = image_fit(
                [c],
                own.get(c.name, final),
                oL,
                oW,
                z0,
                H,
                fixed_yaw=fixed_yaw,
                span_mm=10.0,
                span_deg=1.0,
                restarts=3,
            )
            img["check_cameras"][c.name] = {
                **_pose(r["pose"]),
                "delta_mm_mm_deg": _delta(r["pose"], final),
            }
        say(f"image fit: {_pose(final)}")
        # the base height is not observable from one oblique view: what 1 mm of it does
        dz = [
            image_fit(
                use,
                final,
                oL,
                oW,
                z0 + d,
                H,
                fixed_yaw=fixed_yaw,
                span_mm=3.0,
                span_deg=0.5,
                restarts=2,
            )["pose"]
            for d in (-0.001, 0.001)
        ]
        img["per_mm_plate_z"] = (dz[1] - dz[0]) / 2
        shift = np.abs(img["per_mm_plate_z"] * (z0_cloud - z0) * 1000)
        if spec.plate_z is not None and (shift[:2].max() * 1000 > spec.check_tol_mm):
            warnings.append(
                f"plate z: configured {z0 * 1000:.1f} mm, cloud {z0_cloud * 1000:.1f} mm; "
                "the difference moves "
                f"the box {shift[:2].max() * 1000:.1f} mm (per mm of plate z: "
                + ", ".join(f"{v:+.2f}" for v in _mm_deg(img["per_mm_plate_z"])[:2])
                + " mm)"
            )

    for name, c in (img or {}).get("check_cameras", {}).items():
        checks[f"{name} image"] = c
    for name, c in checks.items():
        d = c.get("delta_mm_mm_deg")
        if d and (max(abs(d[0]), abs(d[1])) > spec.check_tol_mm or abs(d[2]) > spec.check_tol_deg):
            warnings.append(
                f"{name} disagrees by {d[0]:+.1f}, {d[1]:+.1f} mm, {d[2]:+.1f}°: the camera poses "
                "are inconsistent (recalibrate) or that view is poor"
            )
    if checks:
        say(
            "cross-checks: "
            + "; ".join(
                f"{k} {v.get('delta_mm_mm_deg', v.get('error'))}" for k, v in checks.items()
            )
        )

    box = box_corners(final[:2], final[2], oL, oW, z0, H)
    u = np.array([np.cos(final[2]), np.sin(final[2])])
    v = np.array([-u[1], u[0]])
    inst = [
        box_corners(
            final[:2] + (iu - (nu - 1) / 2) * L * u + (iv - (nv - 1) / 2) * W * v,
            final[2],
            L,
            W,
            z0,
            H,
        )
        for iu in range(nu)
        for iv in range(nv)
    ]
    return {
        "object": spec.name,
        "label": spec.label,
        "frame": FRAME,
        "t": t0,
        "duration_s": round(time.time() - t0, 1),
        "points": len(xyz),
        "dof": "x, y" if fixed_yaw is not None else "x, y, yaw",
        "method": method,
        "box": {
            "pose": _pose(final),
            "centre": [round(float(final[0]), 5), round(float(final[1]), 5), round(z0 + H / 2, 5)],
            "size_mm": [round(oL * 1000, 2), round(oW * 1000, 2), round(H * 1000, 2)],
            "z_range": [round(z0, 5), round(z0 + H, 5)],
            "corners": box.round(5).tolist(),
            "instances": [nu, nv],
            "instance_corners": [b.round(5).tolist() for b in inst],
        },
        "coarse": {**_pose(coarse), "score": round(o["score"], 3)},
        "cloud_fit": {
            "pose": _pose(cf["pose"]),
            "std_mm_mm_deg": [
                round(cf["std"][0] * 1000, 3),
                round(cf["std"][1] * 1000, 3),
                round(sd_deg, 4),
            ],
            "coverage": {k: round(c, 2) for k, c in cf["coverage"].items()},
            "points": {"top": cf["n_top"], "plate": cf["n_ground"], "object": cf.get("n_object")},
            "method": cf["method"],
            **{k: cf[k] for k in ("residual_mm", "inliers", "rounds", "weakest") if k in cf},
            "corners": None
            if method == "cloud"
            else box_corners(cf["pose"][:2], cf["pose"][2], oL, oW, z0, H).round(5).tolist(),
        },
        "images_needed": reasons,
        "image_fit": None
        if img is None
        else {
            "pose": _pose(img["pose"]),
            "restarts_agree": img["restarts_agree"],
            "restart_spread_mm_mm_deg": _mm_deg(img["spread"]),
            "check_cameras": img["check_cameras"],
            "per_mm_plate_z_mm_mm_deg": _mm_deg(img["per_mm_plate_z"]),
        },
        "plate_z": {"used": round(z0, 5), "cloud": round(z0_cloud, 5), "ring_points": n_ring},
        "checks": checks,
        "quality": {"ok": not warnings, "warnings": warnings},
    }


def save_result(path: str | Path, result: dict[str, Any]) -> None:
    Path(path).write_text(json.dumps(result, indent=1) + "\n")
