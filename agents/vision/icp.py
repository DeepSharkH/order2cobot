"""3-DoF ICP of a CAD model against a point cloud: x, y and yaw of an object resting on a plate.

The model is a prism: its footprint is the convex hull of the CAD's vertices (model copies
end to end when the object is made of several), its height the CAD's. Each iteration pairs
every cloud point with its *exact* closest point on that surface (top face or a side wall,
not a sampled copy of it), and the pose minimises

    sum over object points   rho(|p - closest(p)|)                    (point-to-surface)
  + sum over plate points    rho(w * depth inside the footprint)       (free space)

with a robust soft-L1 rho. Two things make this work for a thin slab where plain point-to-
plane ICP would not:
  * exact closest points: top points that fall outside the footprint pull back laterally, so
    the top silhouette constrains x, y and yaw even in a top-down view with no wall points;
  * the free-space term: plate seen where the model would be pushes it away, which fixes the
    sides whose walls the camera cannot see.
The plate is the plane fitted to the cloud; heights `h` are measured from it, so the depth
camera's height bias cancels. Correspondences are re-taken after each solve until the pose
settles; the 1σ comes from a bootstrap over the points.
"""

import contextlib
from functools import lru_cache
from typing import Any

import numpy as np
import numpy.typing as npt
from scipy.optimize import least_squares

Array = npt.NDArray[Any]
_UNITS = np.array([1000.0, 1000.0, np.degrees(1)])  # (m, m, rad) -> (mm, mm, deg)


# ── footprint polygon ────────────────────────────────────────────────────────


def convex_hull(P: Array) -> Array:
    """Counter-clockwise convex hull of 2D points (monotone chain), without repeating the start."""
    pts = np.unique(np.round(np.asarray(P, float), 9), axis=0)
    if len(pts) < 3:
        return pts

    def cross(o: Array, a: Array, b: Array) -> float:
        return float((a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]))

    lo: list[Array] = []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    hi: list[Array] = []
    for p in pts[::-1]:
        while len(hi) >= 2 and cross(hi[-2], hi[-1], p) <= 0:
            hi.pop()
        hi.append(p)
    return np.array(lo[:-1] + hi[:-1])


def rectangle(L: float, W: float) -> Array:
    return np.array([[-L / 2, -W / 2], [L / 2, -W / 2], [L / 2, W / 2], [-L / 2, W / 2]])


def model_axes(vertices: Array) -> Array:
    """Model vertices in box axes: longest extent to u, middle to v, shortest (height) to z,
    right-handed, centred on the bounding box and with the base at z = 0."""
    order = np.argsort(vertices.max(0) - vertices.min(0))[::-1]
    R = np.eye(3)[order]
    if np.linalg.det(R) < 0:
        R[1] *= -1
    V = vertices @ R.T
    lo, hi = V.min(0), V.max(0)
    return np.asarray(V - [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]])


@lru_cache(maxsize=8)
def _model_hull(path: str) -> Array:
    from agents.vision.locate import read_step

    return convex_hull(model_axes(read_step(path))[:, :2])


def object_footprint(
    size: tuple[float, float, float], grid: tuple[int, int], model: str | None = None
) -> Array:
    """Footprint of the whole object, centred, long axis along u: the hull of `grid` = (nu, nv)
    model copies end to end. From the CAD's vertices when `model` is a readable STEP file,
    else the size's rectangle."""
    L, W, _ = size
    one = rectangle(L, W)
    if model:
        with contextlib.suppress(OSError, ValueError):  # unreadable: the size's rectangle
            one = _model_hull(model)
    nu, nv = grid
    offsets = [
        ((iu - (nu - 1) / 2) * L, (iv - (nv - 1) / 2) * W) for iu in range(nu) for iv in range(nv)
    ]
    return convex_hull(np.concatenate([one + o for o in offsets]))


def boundary_closest(uv: Array, poly: Array) -> tuple[Array, Array, Array]:
    """For 2D points: closest point on the polygon's boundary, distance to it, and inside or not.

    `poly` must be convex and counter-clockwise.
    """
    a = poly
    b = np.roll(poly, -1, axis=0)
    e = b - a  # K edges
    ee = (e * e).sum(1)
    d = uv[:, None, :] - a[None]  # N x K x 2
    t = np.clip((d * e[None]).sum(2) / ee[None], 0.0, 1.0)
    c = a[None] + t[..., None] * e[None]  # closest point on each edge
    dist = np.linalg.norm(uv[:, None, :] - c, axis=2)
    k = np.argmin(dist, 1)
    idx = np.arange(len(uv))
    inside = ((e[None, :, 0] * d[..., 1] - e[None, :, 1] * d[..., 0]) >= 0).all(1)
    return c[idx, k], dist[idx, k], inside


def prism_closest(q: Array, poly: Array, H: float) -> Array:
    """Closest point on the prism's top face or side walls (no bottom: it rests on the plate)
    for points q = (u, v, height) in the object frame."""
    return _closest(q, poly, H)[0]


def _closest(q: Array, poly: Array, H: float) -> tuple[Array, Array, Array]:
    """prism_closest, plus whether the top face was the closest and the signed 2D distance
    to the footprint's boundary (negative inside)."""
    b, d2, inside = boundary_closest(q[:, :2], poly)
    top = np.where(inside[:, None], q[:, :2], b)  # top face: the point itself if over it
    d_top = np.hypot(np.where(inside, 0.0, d2), q[:, 2] - H)
    wall_z = np.clip(q[:, 2], 0.0, H)
    d_wall = np.hypot(d2, q[:, 2] - wall_z)
    use_top = d_top <= d_wall
    out = np.empty_like(q)
    out[:, :2] = np.where(use_top[:, None], top, b)
    out[:, 2] = np.where(use_top, H, wall_z)
    return out, use_top, np.where(inside, -d2, d2)


# ── fit ──────────────────────────────────────────────────────────────────────


def to_object(xy: Array, pose: Array) -> Array:
    x, y, yaw = pose
    c, s = np.cos(yaw), np.sin(yaw)
    d = xy - [x, y]
    return np.stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]], 1)


def select(
    xyz: Array, h: Array, pose: Array, poly: Array, H: float, margin: float = 0.015
) -> tuple[Array, Array]:
    """Points near the footprint: object points (above the plate, up to just over the top;
    wires and parts standing on it are left out) and plate points (free space)."""
    _, dist, inside = boundary_closest(to_object(xyz[:, :2], pose), poly)
    near = inside | (dist < margin)
    obj = near & (h > 0.003) & (h < H + 0.006)
    free = near & (np.abs(h) < 0.0025)
    return np.c_[xyz[obj, :2], h[obj]], xyz[free, :2]


def _residuals(
    pose: Array, obj: Array, free: Array, poly: Array, H: float, free_weight: float, margin: float
) -> Array:
    """Wall points: their 3D distance to the wall. Top points: their height off the top, and
    laterally how far they lie outside the footprint shrunk by `margin`; plate points: how far
    they lie inside it grown by `margin`. The margins make the silhouette terms meet in the
    middle of the gap between the last top point and the first plate point: with hard edges
    every pose inside that gap costs the same and the fit would stop anywhere in it."""
    q = np.c_[to_object(obj[:, :2], pose), obj[:, 2]]
    c, top, sd = _closest(q, poly, H)
    r = q - c
    r[top, 0] = np.maximum(0.0, sd[top] + margin)
    r[top, 1] = 0.0
    _, d, inside = boundary_closest(to_object(free, pose), poly)
    sd_free = np.where(inside, -d, d)
    r_free = free_weight * np.maximum(0.0, margin - sd_free)
    return np.concatenate([r.ravel(), r_free])


def _solve(
    pose0: Array,
    obj: Array,
    free: Array,
    poly: Array,
    H: float,
    free_weight: float,
    scale: float,
    margin: float,
) -> Any:
    return least_squares(
        _residuals,
        np.asarray(pose0, float),
        args=(obj, free, poly, H, free_weight, margin),
        loss="soft_l1",
        f_scale=scale,
        x_scale=np.array([1e-3, 1e-3, 1e-2]),
        xtol=1e-9,
        ftol=1e-10,
        max_nfev=200,
    )


def icp_fit(
    xyz: Array,
    h: Array,
    pose0: Any,
    poly: Array,
    H: float,
    n_boot: int = 10,
    seed: int = 0,
    free_weight: float = 1.0,
    scale: float = 0.0015,
    margin: float = 0.0015,
    max_rounds: int = 6,
) -> dict[str, Any]:
    """Pose (x, y, yaw) of the prism `poly` x `H` in the cloud `xyz`, from `pose0`.

    `h` is each point's height over the plate. Returns the pose, its bootstrap 1σ, the
    direction least constrained by the data, the point-to-surface residuals and the point
    sets used (for the side coverage).
    """
    pose = np.asarray(pose0, float)
    rounds = 0
    while rounds < max_rounds:  # re-take correspondences (and the points) until it settles
        rounds += 1
        obj, free = select(xyz, h, pose, poly, H)
        if len(obj) < 20:
            raise ValueError(f"only {len(obj)} object points near the model")
        new = _solve(pose, obj, free, poly, H, free_weight, scale, margin).x
        step = np.abs((new - pose) * [1000, 1000, np.degrees(1)])
        pose = new
        if step.max() < 0.02:
            break
    obj, free = select(xyz, h, pose, poly, H)
    sol = _solve(pose, obj, free, poly, H, free_weight, scale, margin)
    pose = sol.x

    std, weakest = bootstrap(pose, obj, free, poly, H, n_boot, seed, free_weight, scale, margin)

    q = np.c_[to_object(obj[:, :2], pose), obj[:, 2]]
    dist = np.linalg.norm(q - prism_closest(q, poly, H), axis=1)
    return {
        "pose": pose,
        "cost": float(sol.cost),
        "std": std,
        "rounds": rounds,
        "residual_mm": {
            "median": float(np.median(dist) * 1000),
            "p90": float(np.percentile(dist, 90) * 1000),
        },
        "inliers": float(np.mean(dist < 0.002)),
        "weakest": weakest,
        "obj": obj,
        "free": free,
    }


def symmetric(poly: Array, tol: float = 0.0005) -> bool:
    """The footprint is the same turned 180° (within `tol`): its two ends can't be told apart."""
    _, d, _ = boundary_closest(-poly, poly)
    return bool(d.max() < tol)


def bootstrap(
    pose: Array,
    obj: Array,
    free: Array,
    poly: Array,
    H: float,
    n: int,
    seed: int = 0,
    free_weight: float = 1.0,
    scale: float = 0.0015,
    margin: float = 0.0015,
) -> tuple[Array, dict[str, Any]]:
    """Spread of the pose over `n` refits on resampled points: (1σ per x, y, yaw; the weakest
    direction, i.e. the principal axis of that spread in mm, mm, deg, and its 1σ)."""
    rng = np.random.default_rng(seed)
    boot = []
    for _ in range(n):
        o = obj[rng.integers(0, len(obj), len(obj))]
        f = free[rng.integers(0, len(free), len(free))] if len(free) else free
        boot.append(_solve(pose, o, f, poly, H, free_weight, scale, margin).x)
    if len(boot) < 2:
        return np.zeros(3), {"direction_mm_mm_deg": [0.0, 0.0, 0.0], "sigma": 0.0}
    B = (np.array(boot) - pose) * _UNITS
    w, V = np.linalg.eigh(np.cov(B.T))
    d = V[:, -1] * np.sign(V[np.argmax(np.abs(V[:, -1])), -1])
    weakest = {
        "direction_mm_mm_deg": [round(float(v), 3) for v in d],
        "sigma": round(float(np.sqrt(max(w[-1], 0))), 3),
    }
    return np.array(boot).std(0), weakest


def icp_fit_either_way(
    xyz: Array, h: Array, pose0: Any, poly: Array, H: float, **kw: Any
) -> dict[str, Any]:
    """icp_fit from pose0 and from pose0 turned 180°: the coarse search cannot tell the ends
    apart, while a CAD footprint need not be symmetric. Keeps the lower cost; only that one
    is bootstrapped."""
    n_boot = kw.pop("n_boot", 10)
    best = None
    for turn in (0.0,) if symmetric(poly) else (0.0, np.pi):
        try:
            r = icp_fit(xyz, h, np.asarray(pose0, float) + [0, 0, turn], poly, H, n_boot=0, **kw)
        except ValueError:
            continue
        if best is None or r["cost"] < best["cost"]:
            best = r
    if best is None:
        raise ValueError("too few object points near the model")
    opts = {k: kw[k] for k in ("seed", "free_weight", "scale", "margin") if k in kw}
    best["std"], best["weakest"] = bootstrap(
        best["pose"], best["obj"], best["free"], poly, H, n_boot, **opts
    )
    return best
