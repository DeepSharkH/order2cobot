#!/usr/bin/env python3
"""Save what both cameras see now, from the calibration dashboard, for offline detection work.

    uv run scripts/capture_scene.py --name protoboard
    uv run scripts/capture_scene.py --frames 8 --out data/scenes

The dashboard owns the cameras; this takes its raw frames (/api/frame, no overlays), the
aligned depth (/api/depth) and, from its websocket state, each camera's intrinsics and world
pose plus the robot joints. Writes <out>/<timestamp>_<name>/ (see agents/vision/scene.py).
"""

import argparse
import io
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import httpx
import numpy as np

from agents.cameras.recorder import WebSocket
from agents.core.config import read_yaml
from agents.vision.locate import write_ply
from agents.vision.scene import load_scene

DEFAULT_VISION = Path("configs/vision.yaml")


def dashboard_state(url: str, timeout_s: float = 10.0) -> dict[str, Any]:
    ws = WebSocket(url.replace("http://", "ws://", 1).rstrip("/") + "/ws", timeout_s)
    try:
        t_end = time.monotonic() + timeout_s
        while time.monotonic() < t_end:
            op, data = ws.recv()
            if op == 1:
                m = json.loads(data)
                if m.get("type") == "state":
                    return dict(m)
    finally:
        ws.close()
    raise TimeoutError(f"no state from {url}")


def capture(url: str, out: Path, frames: int, gap_s: float, max_check_mm: float) -> Path:
    state = dashboard_state(url)
    meta: dict[str, Any] = {
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "dashboard": url,
        "frame": "world (checkerboard centre, z up), metres",
        "robot": {k: state["robot"].get(k) for k in ("q", "mode", "dh")},
        "base": state.get("base"),
        "cams": [],
    }
    with httpx.Client(timeout=15) as http:
        for i, cam in enumerate(state["cams"]):
            if i == 0:
                trusted, why = (
                    bool(cam.get("located")),
                    "located from the board" if cam.get("located") else "not located",
                )
            else:
                d = (cam.get("check") or {}).get("dpos_mm")
                trusted = d is not None and d < max_check_mm
                why = f"FK vs board check {d:.1f} mm" if d is not None else "unverified pose"
            entry: dict[str, Any] = {
                k: cam.get(k) for k in ("name", "K", "dist", "size", "T_world_cam", "intrinsics")
            }
            entry.update(trusted=trusted, why=why, color=[], depth=[])
            for k in range(frames):
                r = http.get(f"{url}/api/frame/{i}")
                if r.status_code == 200:
                    f = f"cam{i}_color_{k}.png"
                    (out / f).write_bytes(r.content)
                    entry["color"].append(f)
                r = http.get(f"{url}/api/depth/{i}")
                if r.status_code == 200:
                    f = f"cam{i}_depth_{k}.npy"
                    np.save(out / f, np.load(io.BytesIO(r.content)))
                    entry["depth"].append(f)
                time.sleep(gap_s)  # the dashboard refreshes depth at ~5 Hz
            meta["cams"].append(entry)
            print(
                f"{cam.get('name')}: {len(entry['color'])} frames, {len(entry['depth'])} depth, "
                "pose "
                f"{'trusted' if trusted else 'not trusted'} ({why})"
            )
    (out / "scene.json").write_text(json.dumps(meta, indent=1) + "\n")
    scene = load_scene(out)
    xyz = np.concatenate([c[1] for c in scene.clouds])
    rgb = np.concatenate([c[2] for c in scene.clouds])
    write_ply(out / "cloud_world.ply", xyz, rgb)
    print(f"cloud_world.ply: {len(xyz)} points from {len(scene.clouds)} depth frames")
    return out


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--name", default="scene", help="folder name suffix")
    ap.add_argument("--out", type=Path, default=Path("data/scenes"))
    ap.add_argument("--frames", type=int, default=5, help="colour + depth frames per camera")
    ap.add_argument("--gap", type=float, default=0.3, help="seconds between frames")
    ap.add_argument("--dashboard-url", help=f"default: dashboard_url in {DEFAULT_VISION}")
    ap.add_argument(
        "--max-check-mm",
        type=float,
        default=2.0,
        help="gripper cam pose trusted below this FK check",
    )
    a = ap.parse_args()
    url = (a.dashboard_url or read_yaml(DEFAULT_VISION)["dashboard_url"]).rstrip("/")
    out = a.out / f"{datetime.now():%Y%m%d_%H%M%S}_{a.name}"
    out.mkdir(parents=True)
    try:
        capture(url, out, a.frames, a.gap, a.max_check_mm)
    except (OSError, TimeoutError, httpx.HTTPError) as exc:
        sys.exit(f"error: {exc}")
    print(f"saved {out}")


if __name__ == "__main__":
    main()
