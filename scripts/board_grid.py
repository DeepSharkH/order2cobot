#!/usr/bin/env python3
"""The protoboard's holes (agents/vision/board_grid.py) placed with a located box, drawn into
the camera frames, and the discrete placements a casing can take.

    uv run scripts/board_grid.py <scene>/located_protoboard.json --scene <scene>
    uv run scripts/board_grid.py <located.json> --scene <scene> --casing jumper_s2 \\
        --occupied H1:A20,H1:A22 --flip-u

Writes to --out (default: next to the located JSON, board_grid/):
  holes.json         every hole: label, kind, net, board-frame uv, world xyz of the entry
                     (board top) and of the floor
  cam<i>_holes.png   the camera frame with a small x at every hole (cyan terminal, magenta rail)
                     and the four corner terminal holes labelled
  cam<i>_holes_zoom.png  the board region upscaled, fine x's, so the holes stay visible
  placements.json    with --casing / --exit-x: every allowed (hole set, axis) for that casing

World coordinates come from the box (fixed-cam accuracy, a few mm): good for the overview and
for labels, not for the robot. The labels depend on --flip-u / --flip-v / --rail-plus, which
only the printed numbers and the red/blue rail lines in an image can decide.
"""

import argparse
import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from agents.vision.board_grid import FLOOR_Z, TOP_Z, BoardGrid, box_frame
from agents.vision.locate import load_objects, project
from agents.vision.scene import load_scene

TERMINAL, RAIL = (255, 230, 0), (255, 0, 200)  # BGR
CASINGS = Path("assets/models/component_casings/casings.json")


def cross(img: Any, x: float, y: float, s: int, colour: tuple[int, int, int]) -> None:
    x, y = int(round(x)), int(round(y))
    cv2.line(img, (x - s, y - s), (x + s, y + s), colour, 1, cv2.LINE_AA)
    cv2.line(img, (x - s, y + s), (x + s, y - s), colour, 1, cv2.LINE_AA)


def overlays(scene_dir: Path, grid: BoardGrid, P: Any, out: Path) -> None:
    scene = load_scene(scene_dir)
    colours = [TERMINAL if h.kind == "terminal" else RAIL for h in grid.holes]
    term = [h for h in grid.holes if h.kind == "terminal"]
    corners = [h.name for h in term if h.num in (1, 30) and h.row in ("A", "J")]
    for i, cam in enumerate(scene.views):
        uv, depth = project(cam, P)
        ok = depth > 0
        if not ok.any():
            continue
        img = np.ascontiguousarray(cam.img[:, :, ::-1])
        for (x, y), c, good in zip(uv, colours, ok, strict=True):
            if good:
                cross(img, x, y, 3, c)
        for name in corners:
            x, y = uv[grid.index[name]].astype(int)
            for colour, width in (((0, 0, 0), 3), ((255, 255, 255), 1)):
                cv2.putText(
                    img,
                    name,
                    (x + 5, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    colour,
                    width,
                    cv2.LINE_AA,
                )
        cv2.imwrite(str(out / f"cam{i}_holes.png"), img)

        lo = np.maximum(uv[ok].min(0) - 40, 0).astype(int)
        hi = np.minimum(uv[ok].max(0) + 40, [img.shape[1], img.shape[0]]).astype(int)
        raw = np.ascontiguousarray(cam.img[lo[1] : hi[1], lo[0] : hi[0], ::-1])
        f = max(2, int(2400 / max(raw.shape[:2])))
        z = cv2.resize(raw, None, fx=f, fy=f, interpolation=cv2.INTER_CUBIC)
        for (x, y), c, good in zip(uv, colours, ok, strict=True):
            if good:
                cross(z, (x - lo[0] + 0.5) * f - 0.5, (y - lo[1] + 0.5) * f - 0.5, 4, c)
        cv2.imwrite(str(out / f"cam{i}_holes_zoom.png"), z)
        print(f"{cam.name}: cam{i}_holes.png, cam{i}_holes_zoom.png (x{f})")


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("located", type=Path, help="a locate_object result (scripts/locate_object.py)")
    ap.add_argument("--scene", type=Path, help="draw the holes into this scene's camera frames")
    ap.add_argument("--out", type=Path, help="output folder (default: <located dir>/board_grid)")
    ap.add_argument("--objects", type=Path, default=Path("configs/objects.yaml"))
    ap.add_argument("--flip-u", action="store_true", help="number 1 / half H1 at the box's +u end")
    ap.add_argument("--flip-v", action="store_true", help="row A on the box's +v side")
    ap.add_argument("--rail-plus", choices=("outer", "inner"), default="outer")
    ap.add_argument("--casing", help="casing name in casings.json: list its placements")
    ap.add_argument("--exit-x", help="exit positions in mm along the casing: --exit-x=-2.5,2.5")
    ap.add_argument("--occupied", default="", help="hole labels that are taken, comma separated")
    ap.add_argument("--same-net", action="store_true", help="keep placements that short the part")
    a = ap.parse_args()

    r = json.loads(a.located.read_text())
    spec = load_objects(a.objects)[r.get("object", "protoboard")]
    nu = r["box"]["instances"][0]
    grid = BoardGrid.from_spec(spec, nu, flip_u=a.flip_u, flip_v=a.flip_v, rail_plus=a.rail_plus)
    origin, u, n = box_frame(r)
    top, floor = grid.place(origin, u, n, TOP_Z), grid.place(origin, u, n, FLOOR_Z)
    out = a.out or a.located.parent / "board_grid"
    out.mkdir(parents=True, exist_ok=True)
    (out / "holes.json").write_text(
        json.dumps(
            {
                "frame": "world (checkerboard centre, z up), metres",
                "uv": "board frame: box centre, u along the long side, metres",
                "located": str(a.located),
                "cad": spec.model,
                "board": {"origin": origin.tolist(), "u_axis": u.tolist(), "normal": n.tolist()},
                "orientation": grid.orientation,
                "holes": [
                    {
                        "name": h.name,
                        "kind": h.kind,
                        "net": h.net,
                        "uv": list(h.uv),
                        "entry": top[k].tolist(),
                        "floor": floor[k].tolist(),
                    }
                    for k, h in enumerate(grid.holes)
                ],
            },
            indent=1,
        )
        + "\n"
    )
    print(f"{len(grid.holes)} holes -> {out / 'holes.json'} (orientation {grid.orientation})")

    exit_x = None
    if a.casing:
        c = next(c for c in json.loads(CASINGS.read_text())["casings"] if c["casing"] == a.casing)
        exit_x = c["exit_x"]
    elif a.exit_x:
        exit_x = [float(x) for x in a.exit_x.split(",")]
    if exit_x is not None:
        occ = [s for s in a.occupied.split(",") if s]
        pl = grid.placements(np.array(exit_x) / 1000, occupied=occ, same_net=a.same_net)
        (out / "placements.json").write_text(
            json.dumps({"exit_x_mm": exit_x, "occupied": occ, "placements": pl}, indent=1) + "\n"
        )
        print(f"{len(pl)} placements for exits {exit_x} mm -> {out / 'placements.json'}")

    if a.scene:
        overlays(a.scene, grid, top, out)


if __name__ == "__main__":
    main()
