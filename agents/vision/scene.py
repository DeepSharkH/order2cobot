"""A saved scene: what both cameras saw at one moment, for detection work without the live rig.

Captured from the calibration dashboard (it owns the cameras) by scripts/capture_scene.py:

    <dir>/
        scene.json            time, robot joints, and per camera: name, K, dist, size, T_world_cam,
                              whether that pose is trusted, and the frame/depth files
        cam<i>_color_<k>.png  colour frames (full resolution, no overlays)
        cam<i>_depth_<k>.npy  depth aligned to colour (float32 metres, 0 = no reading)
        cloud_world.ply       all depth frames as one cloud in the world frame (for viewers)

`load_scene` rebuilds the world cloud from the depth frames (same stride and outlier
filter as the dashboard) and the camera views for the image fit.
"""

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from agents.vision.locate import Array, CameraView

STRIDE = 4  # depth pixels per cloud point, as the dashboard's cloud_stride
MAX_RANGE = 2.0


@dataclass
class Scene:
    path: Path
    meta: dict[str, Any]
    clouds: list[tuple[int, Array, Array]]  # per depth frame: camera, world xyz, rgb
    views: list[CameraView]  # one per camera (its first frame), for the image fit

    def cloud(self, frames: int | None = 1, start: int = 0) -> Array:
        """World cloud of depth frames start..start+frames of every camera (frames None = all)."""
        per_cam: dict[int, list[Array]] = {}
        for cam, xyz, _ in self.clouds:
            per_cam.setdefault(cam, []).append(xyz)
        end = None if frames is None else start + frames
        return np.concatenate([x for v in per_cam.values() for x in v[start:end]])

    def frames_per_camera(self) -> int:
        return min(
            sum(1 for c, _, _ in self.clouds if c == cam) for cam in {c for c, _, _ in self.clouds}
        )


def depth_to_cloud(
    color: Array | None,
    depth: Array,
    K: Array,
    stride: int = STRIDE,
    max_range: float = MAX_RANGE,
) -> tuple[Array, Array]:
    """Camera-frame points (and their colours) of every `stride`-th depth pixel."""
    d = depth[::stride, ::stride]
    v, u = np.mgrid[0 : depth.shape[0] : stride, 0 : depth.shape[1] : stride]
    ok = (d > 0.1) & (d < max_range)
    z = d[ok]
    x = (u[ok] - K[0, 2]) * z / K[0, 0]
    y = (v[ok] - K[1, 2]) * z / K[1, 1]
    rgb = (
        color[::stride, ::stride][ok][:, ::-1]
        if color is not None
        else np.full((len(z), 3), 128, np.uint8)
    )
    return np.stack([x, y, z], 1).astype(np.float32), rgb.astype(np.uint8)


def remove_outliers(xyz: Array, rgb: Array, k: int = 8, std: float = 2.0) -> tuple[Array, Array]:
    """Statistical outlier removal as in the dashboard: drop points whose mean distance to
    their k nearest neighbours (over depth: spacing grows with it) is `std` sigmas above
    average. Removes flying pixels at edges."""
    if std <= 0 or len(xyz) <= k:
        return xyz, rgb
    from scipy.spatial import cKDTree

    d, _ = cKDTree(xyz).query(xyz, k + 1, workers=-1)
    m = d[:, 1:].mean(1) / xyz[:, 2]
    keep = m < m.mean() + std * m.std()
    return xyz[keep], rgb[keep]


def load_scene(path: str | Path, outlier_std: float = 2.0) -> Scene:
    import cv2

    path = Path(path)
    meta = json.loads((path / "scene.json").read_text())
    clouds, views = [], []
    for i, cam in enumerate(meta["cams"]):
        if cam.get("T_world_cam") is None or cam.get("K") is None:
            continue
        K, T = np.array(cam["K"], float), np.array(cam["T_world_cam"], float)
        colors: list[Array] = []
        for f in cam["color"]:
            img = cv2.imread(str(path / f))
            if img is None:
                raise OSError(f"cannot read {path / f}")
            colors.append(img)
        for k, f in enumerate(cam["depth"]):
            depth = np.load(path / f)
            color = colors[min(k, len(colors) - 1)] if colors else None
            xyz, rgb = depth_to_cloud(color, depth, K)
            xyz, rgb = remove_outliers(xyz, rgb, std=outlier_std)
            clouds.append((i, (xyz @ T[:3, :3].T + T[:3, 3]).astype(float), rgb))
        if colors:
            h, w = colors[0].shape[:2]
            views.append(
                CameraView(
                    name=cam["name"],
                    K=K,
                    dist=np.array(cam.get("dist") or [0] * 5, float),
                    size=(w, h),
                    img=np.ascontiguousarray(colors[0][:, :, ::-1]),
                    T_world_cam=T,
                    trusted=bool(cam.get("trusted")),
                    why=cam.get("why", ""),
                )
            )
    return Scene(path=path, meta=meta, clouds=clouds, views=views)
