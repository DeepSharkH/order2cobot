#!/usr/bin/env python3
"""Find an object's bounding box in a saved scene (scripts/capture_scene.py) and check it by eye.

    uv run scripts/locate_object.py data/scenes/<scene>                 # protoboard, both cameras
    uv run scripts/locate_object.py data/scenes/<scene> --cams 0 --images never --frames 3

Writes <scene>/located_<object>.json (agents/vision/locate.py's result) and
<scene>/located_<object>.jpg: every camera frame with the box drawn into it (brass; the
cloud-only fit dashed cream when the image fit moved it; model instances grey) and a top
view of the cloud around the object, coloured by height above the plate.
"""

import argparse
import dataclasses
import json
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from agents.vision.locate import (
    EDGES,
    Array,
    CameraView,
    ObjectNotFound,
    load_objects,
    locate_object,
    project,
)
from agents.vision.scene import Scene, load_scene

BRASS, CREAM, GREY = (93, 161, 201), (218, 245, 254), (119, 133, 140)


def draw_box(
    img: Array,
    cam: CameraView,
    corners: Array,
    colour: tuple[int, int, int],
    width: int,
    dashed: bool = False,
) -> None:
    """Box edges sampled in 3D, so they bend with the lens distortion like the real ones."""
    t = np.linspace(0, 1, 40)[:, None]
    for a, b in EDGES:
        uv, z = project(cam, corners[a] + t * (corners[b] - corners[a]))
        if (z <= 0).any():
            continue
        pts = uv.astype(np.int32)
        if dashed:
            for k in range(0, len(pts) - 1, 4):
                cv2.line(
                    img,
                    tuple(pts[k]),
                    tuple(pts[min(k + 2, len(pts) - 1)]),
                    colour,
                    width,
                    cv2.LINE_AA,
                )
        else:
            cv2.polylines(img, [pts.reshape(-1, 1, 2)], False, colour, width, cv2.LINE_AA)


def camera_panels(scene: Scene, r: dict[str, Any]) -> list[Array]:
    out = []
    box = np.array(r["box"]["corners"])
    inst = (
        [np.array(b) for b in r["box"]["instance_corners"]]
        if len(r["box"]["instance_corners"]) > 1
        else []
    )
    cloud_box = r["cloud_fit"].get("corners")
    for cam in scene.views:
        img = np.ascontiguousarray(cam.img[:, :, ::-1])  # RGB -> BGR for OpenCV
        for b in inst:
            draw_box(img, cam, b, GREY, 1)
        if cloud_box is not None:
            draw_box(img, cam, np.array(cloud_box), CREAM, 2, dashed=True)
        draw_box(img, cam, box, BRASS, 2)
        # zoom on the box
        uv, z = project(cam, box)
        if (z > 0).all():
            m = 0.4 * np.ptp(uv, 0).max()
            x0, y0 = np.maximum(uv.min(0) - m, 0).astype(int)
            x1, y1 = np.minimum(uv.max(0) + m, [img.shape[1], img.shape[0]]).astype(int)
            if x1 - x0 > 40 and y1 - y0 > 40:
                img = img[y0:y1, x0:x1]
        tag = f"{cam.name}: {'used' if cam.used else 'check only'} ({cam.why})"
        img = cv2.resize(img, (640, round(img.shape[0] * 640 / img.shape[1])))
        cv2.putText(img, tag, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(
            img, tag, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA
        )
        out.append(img)
    return out


def top_view(xyz: Array, r: dict[str, Any], px_mm: float = 0.5) -> Array:
    """Cloud around the box seen from above (x right, y up), coloured by height over the plate."""
    box = np.array(r["box"]["corners"])
    c = box[:4, :2].mean(0)
    half = np.ptp(box[:, :2], 0).max() / 2 + 0.04
    win = (np.abs(xyz[:, 0] - c[0]) < half) & (np.abs(xyz[:, 1] - c[1]) < half)
    P = xyz[win]
    n = int(2 * half * 1000 / px_mm)
    img: Array = np.full((n, n, 3), 30, np.uint8)
    h = (P[:, 2] - r["plate_z"]["used"]) / (r["box"]["z_range"][1] - r["plate_z"]["used"])
    col = cv2.applyColorMap(
        np.clip(h * 200, 0, 255).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_VIRIDIS
    )[:, 0]

    def to_px(p: Array) -> Array:
        return np.stack(
            [(p[:, 0] - c[0] + half) * 1000 / px_mm, (half - (p[:, 1] - c[1])) * 1000 / px_mm], 1
        )

    uv = to_px(P).astype(int)
    ok = (uv >= 0).all(1) & (uv < n).all(1)
    img[uv[ok, 1], uv[ok, 0]] = col[ok]
    for b, colour in [*((np.array(b), GREY) for b in r["box"]["instance_corners"]), (box, BRASS)]:
        cv2.polylines(
            img,
            [to_px(b[[0, 1, 2, 3]]).astype(np.int32).reshape(-1, 1, 2)],
            True,
            colour,
            2,
            cv2.LINE_AA,
        )
    if r["cloud_fit"].get("corners"):
        cb = np.array(r["cloud_fit"]["corners"])
        cv2.polylines(
            img,
            [to_px(cb[[0, 1, 2, 3]]).astype(np.int32).reshape(-1, 1, 2)],
            True,
            CREAM,
            1,
            cv2.LINE_AA,
        )
    img = np.asarray(cv2.resize(img, (640, 640), interpolation=cv2.INTER_NEAREST))
    cv2.putText(
        img,
        "top view, height over plate",
        (8, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return img


def sheet(panels: list[Array], title: str) -> Array:
    w = 640
    cols = 2
    rows = [panels[i : i + cols] for i in range(0, len(panels), cols)]
    out = []
    for row in rows:
        hgt = max(p.shape[0] for p in row)
        row = [np.pad(p, ((0, hgt - p.shape[0]), (0, 0), (0, 0))) for p in row]
        row += [np.zeros((hgt, w, 3), np.uint8)] * (cols - len(row))
        out.append(np.hstack(row))
    img = np.vstack(out)
    bar = np.full((34, img.shape[1], 3), 17, np.uint8)
    cv2.putText(
        bar, title, (8, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (218, 245, 254), 1, cv2.LINE_AA
    )
    return np.vstack([bar, img])


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("scene", type=Path, help="a folder from scripts/capture_scene.py")
    ap.add_argument("--object", default="protoboard", help="name in --objects")
    ap.add_argument("--objects", type=Path, default=Path("configs/wire/objects.yaml"))
    ap.add_argument(
        "--images", choices=("auto", "always", "never"), help="override the object's image-fit mode"
    )
    ap.add_argument(
        "--cams", type=int, nargs="+", help="cameras whose clouds are fitted (default: the trusted)"
    )
    ap.add_argument(
        "--frames", type=int, default=1, help="depth frames per camera merged into the cloud"
    )
    ap.add_argument("--start", type=int, default=0, help="first depth frame")
    ap.add_argument(
        "--trust", type=int, nargs="+", default=[], help="treat these cameras' poses as trusted"
    )
    ap.add_argument(
        "--plate-z", help="override the object's plate z: mm, or 'cloud' to take it from the cloud"
    )
    ap.add_argument("--out", help="output name (default: located_<object>)")
    a = ap.parse_args()

    spec = load_objects(a.objects)[a.object]
    if a.images:
        spec = dataclasses.replace(spec, images=a.images)
    if a.plate_z is not None:
        spec = dataclasses.replace(
            spec, plate_z=None if a.plate_z == "cloud" else float(a.plate_z) / 1000
        )
    scene = load_scene(a.scene)
    for i, v in enumerate(scene.views):
        if i in a.trust:
            v.trusted, v.why = True, f"{v.why}; trusted by --trust"
    names = [c["name"] for c in scene.meta["cams"]]
    trusted = {i for i, c in enumerate(scene.meta["cams"]) if c.get("trusted") or i in a.trust}
    per_cam: dict[int, list[Array]] = {}
    for cam, pts, _ in scene.clouds:
        per_cam.setdefault(cam, []).append(pts)

    def frames(i: int) -> Array:
        return np.concatenate(per_cam[i][a.start : a.start + a.frames])

    # fitted: the trusted cameras (or --cams); the others' clouds are fitted alone as checks
    main = a.cams or [i for i in per_cam if i in trusted] or list(per_cam)
    xyz = np.concatenate([frames(i) for i in main if i in per_cam])
    checks = {names[i]: frames(i) for i in per_cam if i not in main}
    print(
        f"cloud: {len(xyz)} points from {', '.join(names[i] for i in main)} "
        f"({a.frames} frame(s) each from #{a.start}); checks: {', '.join(checks) or 'none'}"
    )
    try:
        r = locate_object(spec, xyz, scene.views, log=print, check_clouds=checks)
    except ObjectNotFound as exc:
        sys.exit(f"not found: {exc}")
    b = r["box"]
    print(
        f"box ({r['method']}, {r['duration_s']} s): "
        f"centre {np.round(np.array(b['centre']) * 1000, 2).tolist()} mm, "
        f"yaw {b['pose']['yaw_deg']}°, {b['size_mm']} mm"
    )
    name = a.out or f"located_{a.object}"
    (a.scene / f"{name}.json").write_text(json.dumps(r, indent=1) + "\n")
    title = (
        f"{r['label']}: x {b['pose']['x'] * 1000:.1f} y {b['pose']['y'] * 1000:.1f} mm "
        f"yaw {b['pose']['yaw_deg']:.2f} deg"
        f" - {r['method']}"
    )
    cv2.imwrite(
        str(a.scene / f"{name}.jpg"),
        sheet([*camera_panels(scene, r), top_view(xyz, r)], title),
        [cv2.IMWRITE_JPEG_QUALITY, 85],
    )
    print(f"wrote {a.scene / name}.json/.jpg")


if __name__ == "__main__":
    main()
