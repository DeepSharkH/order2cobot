"""Plane-based board vision on a camera point cloud (moved from scripts/wire_from_cloud.py,
2026-10-27): back-project a depth image to base-frame points, fit the board / plate plane,
resample the colour image onto it, find the printed rail lines and the objects standing on it.
Used by agents/vision/pins.py (protoboard-wire track)."""

from typing import Any

import cv2
import numpy as np


class PlaneNotFound(RuntimeError):
    pass


def cloud(depth: np.ndarray, K: np.ndarray, T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Base-frame points (N, 3) and their pixel coordinates (N, 2)."""
    # drop "flying pixels": depth edges interpolate between foreground and background and
    # leave a ghost streak behind every object, right where a target hole often is
    k = np.ones((5, 5), np.uint8)
    far = cv2.dilate(depth, k)
    near = cv2.erode(np.where(depth > 0, depth, 10.0).astype(np.float32), k)
    v, u = np.nonzero((depth > 0) & (far - near < 0.004))
    z = depth[v, u]
    P = np.c_[(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z]
    return P @ T[:3, :3].T + T[:3, 3], np.c_[u, v]


def fit_plane(P: np.ndarray, tol: float, seed: int = 0, up: np.ndarray | None = None,
              max_tilt_deg: float = 14.0) -> tuple[np.ndarray, np.ndarray]:
    """RANSAC + SVD refit of a plane within max_tilt_deg of `up` (default base z):
    (point, upward unit normal)."""
    up = np.array([0.0, 0.0, 1.0]) if up is None else up
    min_cos = np.cos(np.radians(max_tilt_deg))
    rng = np.random.default_rng(seed)
    S = P[rng.choice(len(P), min(len(P), 30000), replace=False)]
    best, best_n = None, 0
    for _ in range(600):
        a, b, c = S[rng.choice(len(S), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-12:
            continue
        n = n / np.linalg.norm(n)
        n *= np.sign(n @ up or 1)
        if n @ up < min_cos:
            continue
        k = int((np.abs((S - a) @ n) < tol).sum())
        if k > best_n:
            best, best_n = (a, n), k
    if best is None:
        raise PlaneNotFound("no horizontal plane found in the work area")
    a, n = best
    inl = P[np.abs((P - a) @ n) < tol]
    c = inl.mean(0)
    n = np.linalg.svd(inl - c, full_matrices=False)[2][2]
    return c, n * np.sign(n[2])


def plane_view(color: np.ndarray, T: np.ndarray, K: np.ndarray, c: np.ndarray, n: np.ndarray,
               area: dict[str, list[float]], res: float) -> np.ndarray:
    """The camera image resampled onto the board plane: rows = base x, cols = base y."""
    X, Y = np.meshgrid(np.arange(area["x"][0], area["x"][1], res),
                       np.arange(area["y"][0], area["y"][1], res), indexing="ij")
    Z = c[2] - (n[0] * (X - c[0]) + n[1] * (Y - c[1])) / n[2]
    Ti = np.linalg.inv(T)
    q = np.stack([X, Y, Z], -1).reshape(-1, 3) @ Ti[:3, :3].T + Ti[:3, 3]
    uv = (q[:, :2] / q[:, 2:]) * [K[0, 0], K[1, 1]] + [K[0, 2], K[1, 2]]
    uv = uv.reshape(X.shape + (2,)).astype(np.float32)
    return cv2.remap(color, uv[..., 0], uv[..., 1], cv2.INTER_LINEAR, borderValue=0)


def board_rails(view: np.ndarray, area: dict[str, list[float]], res: float,
                min_len: float) -> list[tuple[np.ndarray, np.ndarray]]:
    """The breadboard's printed red/blue rail lines in the plane view: (unit direction xy,
    points xy) per line. Tape and occlusion only shorten them, so they give the column
    axis far more reliably than the board outline, which the oblique camera half shadows."""
    B, G, R = (view[..., i].astype(np.int16) for i in range(3))
    masks = ((B - R > 25) & (B - G > 5), (R - G > 40) & (R - B > 20) & (G < 170))
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 3))
    lines = []
    for m in masks:
        m8 = cv2.morphologyEx(m.astype(np.uint8) * 255, cv2.MORPH_CLOSE, kernel)
        n, lab = cv2.connectedComponents(m8)
        for k in range(1, n):
            r, c = np.nonzero(lab == k)
            if len(r) < 50:
                continue
            pts = np.c_[area["x"][0] + r * res, area["y"][0] + c * res]
            e, V = np.linalg.eigh(np.cov((pts - pts.mean(0)).T))
            d = V[:, 1] * np.sign(V[0, 1] or 1)  # columns count towards base +x
            if np.ptp((pts - pts.mean(0)) @ d) >= min_len and 4 * np.sqrt(max(e[0], 0)) < 0.003:
                lines.append((d, pts))
    return lines


def objects_on_board(P: np.ndarray, h: np.ndarray, det: dict[str, Any]) -> list[np.ndarray]:
    """Clusters of points standing on the board (indices into P), largest first."""
    lo, hi = det["object_height"]
    idx = np.nonzero((h > lo) & (h < hi))[0]
    if len(idx) == 0:
        return []
    res = 0.001
    q = P[idx, :2]
    mn = q.min(0)
    ij = ((q - mn) / res).astype(int)
    grid = np.zeros(ij.max(0)[::-1] + 1, np.uint8)
    grid[ij[:, 1], ij[:, 0]] = 255
    grid = cv2.dilate(grid, np.ones((3, 3), np.uint8))
    _, lab = cv2.connectedComponents(grid)
    labels = lab[ij[:, 1], ij[:, 0]]
    out = []
    for k in np.unique(labels):
        sel = idx[labels == k]
        if (len(sel) >= det["object_min_points"] and h[sel].min() <= det["object_touch_max"]
                and h[sel].max() >= det["object_min_height"]):
            out.append(sel)
    return sorted(out, key=len, reverse=True)
