"""Pin map of the protoboard from gripper-cam images (skill protoboard-wire, S3): which grid holes
the casings' pins sit in, and where every hole is in the robot base frame.

Everything is in the base frame through FK + hand-eye: a scene's gripper-cam pose is
`T_world_base · FK · mount` (dashboard), so `inv(T_world_base) · T_world_cam` gives it back
without the world (checkerboard) frame's error. The fixed cam only gives the board's region of
interest (a `locate_object` box, a few mm off).

Two gripper views, because the Gemini gives no depth closer than ~0.3 m:
  far   (~0.45 m, depth)   board top plane (it is 9.5 mm above the plate it lies on) and the
                           casings standing on it (height -> casing type, top face -> footprint)
  near  (~0.2 m, colour)   the holes: the colour image resampled onto the board top plane,
                           dark square blobs, and the CAD grid (`BoardGrid.fit`) onto them

Board frame (`BoardFrame`): origin on the board top plane under the box centre, u along the
box's long side, v = n x u, n the plane normal (up): the frame `BoardGrid` uv is given in.
"""

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import numpy.typing as npt
from scipy.spatial import cKDTree

from agents.vision.board_grid import PITCH, BoardGrid
from agents.vision.scene import depth_to_cloud
from agents.vision.plane import fit_plane

Array = npt.NDArray[Any]
CASINGS = Path("assets/models/component_casings/casings.json")
# not in casings.json: the original jumper casing (2 leads 5.0 mm apart, wire loop on top)
ORIGINAL_JUMPER = {
    "casing": "jumper_orig_5mm",
    "pins": 2,
    "exit_x": [-2.5, 2.5],
    "width_x": 14.0,
    "thickness_y": 5.0,
    "height": 40.0,
}
BOARD_H = 0.0095  # board top above the plate it lies on


def casing_catalogue() -> list[dict[str, Any]]:
    return [ORIGINAL_JUMPER, *json.loads(CASINGS.read_text())["casings"]]


@dataclass
class CamFrames:
    """One camera of a saved scene: intrinsics, base-frame pose, all colour and depth frames."""

    name: str
    K: Array
    dist: Array
    T_base_cam: Array
    colors: list[Array]  # BGR
    depths: list[Array]  # metres, 0 = none

    def median_color(self) -> Array:
        return np.median(np.stack(self.colors), 0).astype(np.uint8)

    def project(self, P: Array) -> tuple[Array, Array]:
        """Pixels (with distortion) and depth along the optical axis of base points."""
        T = np.linalg.inv(self.T_base_cam)
        z = P @ T[:3, :3].T + T[:3, 3]
        rvec, _ = cv2.Rodrigues(T[:3, :3])
        uv, _ = cv2.projectPoints(np.asarray(P, float).reshape(-1, 3), rvec, T[:3, 3], self.K, self.dist)
        return uv.reshape(-1, 2), z[:, 2]

    def cloud(self, stride: int = 2) -> Array:
        """Base-frame points of every depth frame."""
        out = []
        for d in self.depths:
            xyz, _ = depth_to_cloud(None, d, self.K, stride=stride)
            out.append(xyz @ self.T_base_cam[:3, :3].T + self.T_base_cam[:3, 3])
        return np.concatenate(out) if out else np.zeros((0, 3))


def T_world_base(meta: dict[str, Any]) -> Array:
    b = meta["base"]
    c, s = np.cos(np.radians(b["yaw_deg"])), np.sin(np.radians(b["yaw_deg"]))
    T = np.eye(4)
    T[:2, :2] = [[c, -s], [s, c]]
    T[:3, 3] = b["x"], b["y"], b.get("z", 0.0)
    return T


def load_camera(scene: str | Path, index: int = 1) -> CamFrames:
    """Camera `index` of a scene (1 = gripper cam), posed in the base frame."""
    scene = Path(scene)
    meta = json.loads((scene / "scene.json").read_text())
    cam = meta["cams"][index]
    T = np.linalg.inv(T_world_base(meta)) @ np.array(cam["T_world_cam"], float)
    return CamFrames(
        name=cam["name"],
        K=np.array(cam["K"], float),
        dist=np.array(cam.get("dist") or [0.0] * 5, float),
        T_base_cam=T,
        colors=[cv2.imread(str(scene / f)) for f in cam["color"]],
        depths=[np.load(scene / f) for f in cam["depth"]],
    )


@dataclass
class BoardFrame:
    o: Array
    u: Array
    v: Array
    n: Array

    def to_base(self, uv: Array, h: float | Array = 0.0) -> Array:
        uv = np.atleast_2d(uv)
        return self.o + np.outer(uv[:, 0], self.u) + np.outer(uv[:, 1], self.v) + np.outer(np.broadcast_to(h, len(uv)), self.n)

    def to_uvh(self, P: Array) -> Array:
        d = np.atleast_2d(P) - self.o
        return np.c_[d @ self.u, d @ self.v, d @ self.n]

    def as_dict(self) -> dict[str, list[float]]:
        return {k: getattr(self, k).tolist() for k in ("o", "u", "v", "n")}


def board_planes(P: Array, centre: Array, half: tuple[float, float] = (0.08, 0.13), tol: float = 0.001) -> dict[str, Any]:
    """Board top plane from base points near the board: the plate it lies on is the lowest big
    plane, the top the plane ~9.5 mm above it (RANSAC + SVD refit, `tol`). `half` is the region
    (base x, y half sizes) around `centre`; generous, the board may be yawed."""
    roi = (np.abs(P[:, 0] - centre[0]) < half[0]) & (np.abs(P[:, 1] - centre[1]) < half[1])
    Q = P[roi]
    if len(Q) < 500:
        raise ValueError(f"only {len(Q)} depth points near the board")
    z = Q[:, 2]
    hist, edges = np.histogram(z, bins=np.arange(z.min(), z.max() + 0.001, 0.001))
    big = np.nonzero(hist > 0.15 * hist.max())[0]
    lid_z = edges[big[0]] + 0.0005  # lowest big peak
    lid = fit_plane(Q[np.abs(z - lid_z) < 0.003], tol)
    h = (Q - lid[0]) @ lid[1]
    top = fit_plane(Q[np.abs(h - BOARD_H) < 0.003], tol, seed=1, up=lid[1], max_tilt_deg=3.0)
    step = float((top[0] - lid[0]) @ lid[1])
    inl = int((np.abs((Q - top[0]) @ top[1]) < tol).sum())
    return {"top": top, "plate": lid, "step": step, "top_inliers": inl, "points": len(Q)}


def board_from_top(P: Array, top: tuple[Array, Array], tol: float = 0.0015, size: tuple[float, float] = (0.166, 0.056)) -> tuple[BoardFrame, dict[str, Any]]:
    """Board frame from the depth points on the board top plane: the outline of the largest
    patch (minimum-area rectangle in the plane), u along its long side. Good to ~1 mm / 1 deg,
    the start of the grid fit (no world frame involved)."""
    c, n = top
    u0 = np.cross([0.0, 1.0, 0.0], n)
    u0 /= np.linalg.norm(u0)
    v0 = np.cross(n, u0)
    on = np.abs((P - c) @ n) < tol
    q = np.c_[(P[on] - c) @ u0, (P[on] - c) @ v0]
    res = 0.001
    ij = np.floor((q - q.min(0)) / res).astype(int)
    img = np.zeros(ij.max(0)[::-1] + 1, np.uint8)
    img[ij[:, 1], ij[:, 0]] = 255
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    nl, lab, st, _ = cv2.connectedComponentsWithStats(img)
    k = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    ys, xs = np.nonzero(lab == k)
    (cx, cy), (w, h), ang = cv2.minAreaRect(np.c_[xs, ys].astype(np.float32))
    a = np.radians(ang if w >= h else ang + 90)
    centre = q.min(0) + (np.array([cx, cy]) + 0.5) * res
    o = c + centre[0] * u0 + centre[1] * v0
    u = np.cos(a) * u0 + np.sin(a) * v0
    L, W = max(w, h) * res, min(w, h) * res
    info = {"outline_mm": [round(L * 1000, 1), round(W * 1000, 1)], "expected_mm": [s * 1000 for s in size], "points": int(on.sum())}
    return BoardFrame(o, u, np.cross(n, u), n), info


def board_frame(located: dict[str, Any], T_bw: Array, top: tuple[Array, Array]) -> BoardFrame:
    """Frame of a `locate_object` box (world) on the board top plane (base)."""
    C = np.array(located["box"]["corners"]) @ T_bw[:3, :3].T + T_bw[:3, 3]
    c, n = top
    o = C[:4].mean(0)
    o = o - ((o - c) @ n) * n
    u = C[1] - C[0]
    u = u - (u @ n) * n
    u /= np.linalg.norm(u)
    return BoardFrame(o, u, np.cross(n, u), n)


# ── holes ────────────────────────────────────────────────────────────────────


def rectify(img: Array, cam: CamFrames, frame: BoardFrame, u_rng: tuple[float, float], v_rng: tuple[float, float], res: float) -> Array:
    """The image resampled onto the board top plane: column = u, row = v, `res` m per pixel,
    pixel (0, 0) at (u_rng[0], v_rng[0])."""
    U, V = np.meshgrid(np.arange(*u_rng, res), np.arange(*v_rng, res))
    px, _ = cam.project(frame.to_base(np.c_[U.ravel(), V.ravel()]))
    px = px.reshape(U.shape + (2,)).astype(np.float32)
    return cv2.remap(img, px[..., 0], px[..., 1], cv2.INTER_CUBIC, borderValue=(0, 0, 0))


def board_mask(rect: Array) -> Array:
    """Pixels that can be board surface: light and unsaturated (not tape, casing shadow, black)."""
    hsv = cv2.cvtColor(rect, cv2.COLOR_BGR2HSV)
    tape = (hsv[..., 1] > 90) & (hsv[..., 0] > 15) & (hsv[..., 0] < 45)
    tape = cv2.dilate(tape.astype(np.uint8), np.ones((9, 9), np.uint8)) > 0
    return ~tape & (rect.sum(2) > 0)


def detect_holes(rect: Array, res: float, mask: Array | None = None, min_contrast: float = 18.0) -> tuple[Array, Array]:
    """Hole centres (pixel x, y) in a rectified image and their contrast: dark compact blobs
    of the hole's size (1.7 mm entry) after a 3 mm black-hat."""
    g = cv2.cvtColor(rect, cv2.COLOR_BGR2GRAY).astype(np.float32)
    k = int(round(0.003 / res)) | 1
    bh = cv2.morphologyEx(g, cv2.MORPH_BLACKHAT, cv2.getStructuringElement(cv2.MORPH_RECT, (k, k)))
    if mask is not None:
        bh = bh * mask
    on = bh[bh > 2]
    thr = max(min_contrast, float(cv2.threshold(on.astype(np.uint8), 0, 255, cv2.THRESH_OTSU)[0])) if len(on) else min_contrast
    n, lab, st, _ = cv2.connectedComponentsWithStats((bh > thr).astype(np.uint8))
    side = 0.0017 / res
    pts, val = [], []
    for i in range(1, n):
        x, y, w, h, a = st[i]
        if not (0.15 * side**2 <= a <= 2.5 * side**2 and max(w, h) <= 2.0 * side and 0.5 < w / h < 2.0):
            continue
        sel = lab[y : y + h, x : x + w] == i
        wts = bh[y : y + h, x : x + w] * sel
        yy, xx = np.mgrid[y : y + h, x : x + w]
        pts.append(((xx * wts).sum() / wts.sum(), (yy * wts).sum() / wts.sum()))
        val.append(float(wts.max()))
    return np.array(pts).reshape(-1, 2), np.array(val)


def holes_near(
    cam: CamFrames,
    frame: BoardFrame,
    u_rng: tuple[float, float] = (-0.095, 0.095),
    v_rng: tuple[float, float] = (-0.032, 0.032),
    res: float = 0.0002,
    image: Array | None = None,
) -> dict[str, Any]:
    """Detected hole centres (board-frame uv) in one gripper image (default: the median frame)."""
    img = cam.median_color() if image is None else image
    rect = rectify(img, cam, frame, u_rng, v_rng, res)
    mask = board_mask(rect)
    px, val = detect_holes(rect, res, mask)
    uv = np.c_[u_rng[0] + px[:, 0] * res, v_rng[0] + px[:, 1] * res]
    return {"uv": uv, "contrast": val, "rect": rect, "mask": mask, "res": res, "u_rng": u_rng, "v_rng": v_rng}


def row_pitch(uv: Array, min_holes: int = 20) -> tuple[float, int]:
    """Median spacing of neighbouring detected holes along u within rows (board-frame uv)."""
    ps: list[float] = []
    for vrow in np.unique(np.round(uv[:, 1] * 2000) / 2000):
        sel = np.abs(uv[:, 1] - vrow) < 0.0006
        if sel.sum() < min_holes:
            continue
        d = np.diff(np.sort(uv[sel, 0]))
        ps += list(d[np.abs(d - PITCH) < 0.0004])
    return (float(np.median(ps)) if ps else float("nan")), len(ps)


def plane_offset_from_pitch(cam: CamFrames, frame: BoardFrame, image: Array | None = None, probe: float = 0.002) -> dict[str, Any]:
    """Shift of the board top plane along n that makes the detected pitch 2.54 mm in this view.
    The depth plane (another view, stereo bias) and the hand-eye leave the close-up's scale ~1 %
    off, i.e. its plane ~2 mm off at 0.2 m; the CAD pitch is the ruler. Pitch is ~linear in the
    shift: two probes and a step to 2.54 mm, then one check."""
    img = cam.median_color() if image is None else image

    def at(dz: float) -> float:
        f = BoardFrame(frame.o + dz * frame.n, frame.u, frame.v, frame.n)
        return row_pitch(holes_near(cam, f, image=img)["uv"])[0]

    p0, p1 = at(0.0), at(probe)
    dz = probe * (p0 - PITCH) / (p0 - p1)
    return {"dz": float(dz), "pitch_at_0": p0, "pitch_at_dz": at(dz)}


def fit_grid(grid: BoardGrid, uv: Array, shifts_u: int = 10, shifts_v: int = 2, turn: bool | None = False) -> dict[str, Any]:
    """`BoardGrid.fit` with a wide search along the board (the fixed-cam box can be 20 mm off
    along it when the ends are taped). `turn` False keeps the grid's labels (the orientation is
    decided by a printed number), None tries both."""
    turns = (0.0, np.pi) if turn is None else ((np.pi,) if turn else (0.0,))
    # whole-pitch hypotheses catch only starts within the 0.9 mm match radius: also start half a
    # pitch over (the frame comes from the board outline, the grid from its holes' centre)
    best = None
    for off in ((0.0, 0.0), (PITCH / 2, 0.0), (0.0, PITCH / 2), (PITCH / 2, PITCH / 2)):
        f = grid.fit(np.asarray(uv) + off, shifts_u=shifts_u, shifts_v=shifts_v, turns=turns)
        if best is None or f["score"] > best[0]["score"]:
            best = (f, np.array(off))
    fit, off = best
    t = fit["seen_to_grid"]
    c, s = np.cos(t["dyaw"]), np.sin(t["dyaw"])
    du, dv = np.array([[c, -s], [s, c]]) @ off + [t["du"], t["dv"]]
    placed = grid.place_fitted(fit) - off
    fit["seen_to_grid"] = {"du": float(du), "dv": float(dv), "dyaw": t["dyaw"]}
    d, k = cKDTree(placed).query(uv)
    m = d < 0.0009
    fit["residual_rms"] = float(np.sqrt(np.mean(d[m] ** 2))) if m.any() else None
    fit["detected"] = len(uv)
    fit["matched_n"] = int(m.sum())
    fit["placed_uv"] = placed
    fit["seen_holes"] = sorted({grid.holes[j].name for j in k[m]})
    return fit


# ── casings ──────────────────────────────────────────────────────────────────


def find_casings(P: Array, frame: BoardFrame, extent: tuple[float, float] = (0.095, 0.035), min_h: float = 0.004, max_h: float = 0.09) -> list[dict[str, Any]]:
    """Objects standing on the board top (far-view cloud): per cluster its top-face height
    (the most common height above 8 mm, 1 mm bins), the top face's centre, long axis and size
    in the board frame. The top face of a vertical casing is its footprint."""
    uvh = frame.to_uvh(P)
    on = (np.abs(uvh[:, 0]) < extent[0]) & (np.abs(uvh[:, 1]) < extent[1]) & (uvh[:, 2] > min_h) & (uvh[:, 2] < max_h)
    Q = uvh[on]
    if len(Q) < 20:
        return []
    res = 0.001
    ij = np.floor((Q[:, :2] - Q[:, :2].min(0)) / res).astype(int)
    img = np.zeros(ij.max(0)[::-1] + 1, np.uint8)
    img[ij[:, 1], ij[:, 0]] = 255
    img = cv2.dilate(img, np.ones((3, 3), np.uint8))
    _, lab = cv2.connectedComponents(img)
    labels = lab[ij[:, 1], ij[:, 0]]
    out = []
    for k in np.unique(labels):
        C = Q[labels == k]
        if len(C) < 20 or C[:, 2].max() < 0.008:
            continue
        hs = C[C[:, 2] > 0.008, 2]
        hist, edges = np.histogram(hs, bins=np.arange(0.008, hs.max() + 0.002, 0.001))
        H = edges[np.argmax(hist)] + 0.0005
        face = C[np.abs(C[:, 2] - H) < 0.0015]
        face_h = float(np.median(face[:, 2]))
        uv = face[:, :2]
        c = (np.percentile(uv, 2, axis=0) + np.percentile(uv, 98, axis=0)) / 2
        e, V = np.linalg.eigh(np.cov((uv - uv.mean(0)).T))
        a = V[:, 1]
        a = a * np.sign(a[0] if abs(a[0]) > abs(a[1]) else a[1])
        along, across = (uv - c) @ a, (uv - c) @ np.array([-a[1], a[0]])
        size = [float(np.percentile(along, 98) - np.percentile(along, 2)), float(np.percentile(across, 98) - np.percentile(across, 2))]
        c = c + a * (np.percentile(along, 98) + np.percentile(along, 2)) / 2
        out.append(
            {
                "points": len(C),
                "face_points": len(face),
                "height": face_h,
                "max_height": float(C[:, 2].max()),
                "centre_uv": c.tolist(),
                "axis_uv": a.tolist(),
                "face_size": size,
            }
        )
    return sorted(out, key=lambda o: -o["points"])


def identify(casing: dict[str, Any], catalogue: list[dict[str, Any]], tol: float = 0.0015) -> list[dict[str, Any]]:
    """Catalogue casings whose height matches the measured top face within `tol`."""
    return [c for c in catalogue if abs(c["height"] / 1000 - casing["height"]) <= tol]


def casing_pins(casing: dict[str, Any], spec: dict[str, Any], grid: BoardGrid, fit: dict[str, Any], axis_uv: Array | None = None) -> dict[str, Any]:
    """The casing's exits (footprint centre +- exit_x along its long axis, snapped to the board
    axes: casings sit along u or v) snapped to the fitted grid's holes."""
    a = np.asarray(casing["axis_uv"] if axis_uv is None else axis_uv, float)
    a = np.array([np.sign(a[0]), 0.0]) if abs(a[0]) >= abs(a[1]) else np.array([0.0, np.sign(a[1])])
    ex = np.array(spec["exit_x"]) / 1000
    exits = np.asarray(casing["centre_uv"]) + np.outer(ex - ex.mean(), a)
    placed = fit["placed_uv"]
    tree = cKDTree(placed)
    d, k = tree.query(exits)
    names = [grid.holes[j].name if dj < PITCH / 2 else None for dj, j in zip(d, k)]
    allowed = {tuple(sorted(p["holes"])) for p in grid.placements(ex, same_net=True)}
    ok = None not in names and tuple(sorted(names)) in allowed
    return {
        "axis_uv": a.tolist(),
        "exits_uv": exits.tolist(),
        "holes": names,
        "residual": [float(x) for x in d],
        "holes_uv": placed[k].tolist(),
        "is_placement": bool(ok),
    }


def surface_points(P: Array, frame: BoardFrame, holes_uv: Array, r: float = 0.001, tol: float = 0.001) -> Array:
    """Board-surface depth points within `r` of each hole (seen-check for targets)."""
    uvh = frame.to_uvh(P)
    surf = uvh[np.abs(uvh[:, 2]) < tol, :2]
    if not len(surf):
        return np.zeros(len(holes_uv), int)
    return np.array([len(x) for x in cKDTree(surf).query_ball_point(holes_uv, r)])


__all__ = [
    "BOARD_H",
    "ORIGINAL_JUMPER",
    "BoardFrame",
    "CamFrames",
    "T_world_base",
    "board_frame",
    "board_from_top",
    "board_mask",
    "board_planes",
    "casing_catalogue",
    "casing_pins",
    "detect_holes",
    "find_casings",
    "fit_grid",
    "holes_near",
    "identify",
    "load_camera",
    "rectify",
    "surface_points",
]
