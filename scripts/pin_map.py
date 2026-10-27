#!/usr/bin/env python3
"""Pin map of the protoboard from two gripper-cam scenes (skill protoboard-wire, S3/S8).

    uv run scripts/pin_map.py --far <scene> --near <scene> [--near <scene2>] \\
        [--flip-u] [--flip-v] [--turn no|yes|auto] [--targets H1:C28,H1:C30] --out <dir>

--far   a view from >= 0.35 m (the Gemini gives no depth closer): board top plane (9.5 mm above
        the plate), board outline (the grid fit's start), casings standing on the board
--near  a close-up (~0.2 m): holes detected in the colour image resampled onto that plane, the
        CAD grid fitted onto them (agents/vision/board_grid.py), labels from --flip-u/--flip-v
        (decided by a printed number: check it in near<i>_grid.png and pass the flags)

Everything is in the base frame through FK + hand-eye (no world frame). Writes to --out:
  pinmap.json      plane, frame, grid fit (rms, matched, runner-up), every hole -> base xyz +
                   status, casings (type, footprint, height, pins -> labels + residuals),
                   targets (allowed?, seen, free, pose)
  near<i>_grid.png the close-up with the fitted grid (x per hole), every 5th number labelled,
                   casing pins (red) and targets (green)
  near<i>_rect.png the resampled close-up with detections (red o) and the fitted grid (cyan x)
"""

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from agents.vision.board_grid import BoardGrid  # noqa: E402
from agents.vision.locate import load_objects  # noqa: E402
from agents.vision.pins import (  # noqa: E402
    board_from_top,
    board_planes,
    casing_catalogue,
    casing_pins,
    find_casings,
    fit_grid,
    holes_near,
    identify,
    load_camera,
    plane_offset_from_pitch,
    surface_points,
)
from agents.vision.pins import BoardFrame  # noqa: E402

CYAN, RED, GREEN, WHITE, BLACK = (255, 230, 0), (0, 0, 255), (0, 200, 0), (255, 255, 255), (0, 0, 0)


def text(img: Any, s: str, x: float, y: float, scale: float = 0.4, colour: tuple[int, int, int] = WHITE) -> None:
    for c, w in ((BLACK, 3), (colour, 1)):
        cv2.putText(img, s, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, scale, c, w, cv2.LINE_AA)


def cross(img: Any, x: float, y: float, s: int, colour: tuple[int, int, int], w: int = 1) -> None:
    x, y = int(round(x)), int(round(y))
    cv2.line(img, (x - s, y - s), (x + s, y + s), colour, w, cv2.LINE_AA)
    cv2.line(img, (x - s, y + s), (x + s, y - s), colour, w, cv2.LINE_AA)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--far", type=Path, required=True)
    ap.add_argument("--near", type=Path, action="append", required=True)
    ap.add_argument("--cam", type=int, default=1, help="gripper camera index in the scenes")
    ap.add_argument("--centre", default=None, help="board centre guess, base x,y in m (default: far view axis)")
    ap.add_argument("--flip-u", action="store_true")
    ap.add_argument("--flip-v", action="store_true")
    ap.add_argument("--turn", choices=("no", "yes", "auto"), default="no")
    ap.add_argument("--shifts-u", type=int, default=10)
    ap.add_argument("--targets", default="", help="target hole labels, pin order: H1:C28,H1:C30")
    ap.add_argument("--casing", default=None, help="force the casing type (casings.json name)")
    ap.add_argument("--objects", type=Path, default=Path("configs/wire/objects.yaml"))
    ap.add_argument("--board", default="halves",
                    help="halves = two half boards (objects.yaml STEP); or a hole table JSON, e.g. "
                         "dashboard/web/models/cad/breadboard.json (one-piece full-size board)")
    ap.add_argument("--numbers", type=int, default=None, help="with a hole table: numbers on the board (830-point: 63); terminals only")
    ap.add_argument("--terminal-band", type=float, default=None, help="use only detections with |v| < this (mm): the terminal strips")
    ap.add_argument("--dz", default="auto", help="board plane shift along n in mm for the near views, or auto (pitch = 2.54 mm)")
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    rep: dict[str, Any] = {"far": str(a.far), "near": [str(p) for p in a.near], "gates": {}}

    # ── far view: plane, outline, casings ──
    far = load_camera(a.far, a.cam)
    P = far.cloud(stride=1)
    if a.centre:
        centre = np.array([float(v) for v in a.centre.split(",")])
    else:
        o, z = far.T_base_cam[:3, 3], far.T_base_cam[:3, 2]
        centre = (o + z * (0.017 - o[2]) / z[2])[:2]
    pl = board_planes(P, centre)
    top = pl["top"]
    frame, outline = board_from_top(P, top)
    rep["plane"] = {"c": top[0].tolist(), "n": top[1].tolist(), "step_mm": round(pl["step"] * 1000, 2),
                    "tilt_deg": round(float(np.degrees(np.arccos(top[1][2]))), 2), "inliers": pl["top_inliers"]}
    rep["frame"] = frame.as_dict()
    rep["outline"] = outline
    print(f"plane: top z {top[0][2] * 1000:.1f} mm, {pl['step'] * 1000:.2f} mm above the plate, "
          f"outline {outline['outline_mm']} mm, centre {np.round(frame.o * 1000, 1).tolist()}")

    cat = casing_catalogue()
    casings = find_casings(P, frame)
    for c in casings:
        c["types"] = [t["casing"] for t in identify(c, cat)]
    rep["casings"] = casings

    if a.board == "halves":
        grid = BoardGrid.from_spec(load_objects(a.objects)["protoboard"], 2, flip_u=a.flip_u, flip_v=a.flip_v)
    else:
        grid = BoardGrid.from_holes_json(a.board, flip_u=a.flip_u, flip_v=a.flip_v, numbers=a.numbers)
    rep["board"] = a.board
    turn = {"no": False, "yes": True, "auto": None}[a.turn]

    # ── near views: holes, grid ──
    views = []
    for i, scene in enumerate(a.near):
        cam = load_camera(scene, a.cam)
        if i == 0:
            if a.dz == "auto":
                po = plane_offset_from_pitch(cam, frame)
                dz = po["dz"]
                print(f"plane shift {dz * 1000:.2f} mm along n: pitch {po['pitch_at_0'] * 1000:.4f} -> {po['pitch_at_dz'] * 1000:.4f} mm")
            else:
                dz, po = float(a.dz) / 1000, None
            frame = BoardFrame(frame.o + dz * frame.n, frame.u, frame.v, frame.n)
            rep["plane_shift_mm"] = round(dz * 1000, 3)
            rep["plane_shift_check"] = po
            rep["frame"] = frame.as_dict()
        det = holes_near(cam, frame)
        if a.terminal_band:
            keep = np.abs(det["uv"][:, 1]) < a.terminal_band / 1000
            det["uv"], det["contrast"] = det["uv"][keep], det["contrast"][keep]
        fit = fit_grid(grid, det["uv"], shifts_u=a.shifts_u, turn=turn)
        # per-frame repeatability: the same fit on each single frame
        per = []
        for img in cam.colors:
            d1 = holes_near(cam, frame, image=img)
            if a.terminal_band:
                d1["uv"] = d1["uv"][np.abs(d1["uv"][:, 1]) < a.terminal_band / 1000]
            f1 = fit_grid(grid, d1["uv"], shifts_u=a.shifts_u, turn=turn)
            per.append(f1["placed_uv"])
        spread = float(np.max([np.abs(p - fit["placed_uv"]).max() for p in per])) if per else None
        views.append({"cam": cam, "det": det, "fit": fit})
        v = {k: fit[k] for k in ("seen_to_grid", "turned", "rms", "matched", "score", "runner_up_score",
                                 "residual_rms", "detected", "matched_n")}
        v["frame_spread_mm"] = None if spread is None else round(spread * 1000, 3)
        v["seen_holes"] = len(fit["seen_holes"])
        rep.setdefault("views", []).append(v)
        gap = None if fit["runner_up_score"] is None else fit["score"] - fit["runner_up_score"]
        print(f"near {i}: {len(det['uv'])} blobs, fit rms {fit['rms'] * 1000:.3f} mm, matched {fit['matched']:.2f}, "
              f"score {fit['score']:.3f}, runner-up gap {gap if gap is None else round(gap, 3)}, turned {fit['turned']}, "
              f"frame spread {v['frame_spread_mm']} mm, shift {np.round([fit['seen_to_grid']['du'] * 1000, fit['seen_to_grid']['dv'] * 1000], 2).tolist()} mm")

    fit = views[0]["fit"]
    placed = fit["placed_uv"]
    holes_xyz = frame.to_base(placed)
    if len(views) > 1:
        d = [float(np.abs(v["fit"]["placed_uv"] - placed).max()) for v in views[1:]]
        rep["gates"]["views_agree_mm"] = [round(x * 1000, 3) for x in d]

    # ── casings -> pins ──
    occupied: list[str] = []
    for c in casings:
        types = [a.casing] if a.casing else c["types"]
        if not types:
            continue
        spec_c = next(t for t in cat if t["casing"] == types[0])
        pins = casing_pins(c, spec_c, grid, fit)
        c["pins"] = pins
        c["casing"] = spec_c["casing"]
        occupied += [h for h in pins["holes"] if h]
        print(f"casing {spec_c['casing']}: height {c['height'] * 1000:.1f} mm, face {np.round(np.array(c['face_size']) * 1000, 1).tolist()} mm, "
              f"pins {pins['holes']} residual {np.round(np.array(pins['residual']) * 1000, 2).tolist()} mm, placement {pins['is_placement']}")

    # ── holes table ──
    seen_det = set(fit["seen_holes"])
    surf = surface_points(P, frame, placed, r=0.0015)
    rep["orientation"] = {"flip_u": a.flip_u ^ bool(fit["turned"]), "flip_v": a.flip_v ^ bool(fit["turned"]),
                          "as_fitted": {"flip_u": a.flip_u, "flip_v": a.flip_v, "turned": fit["turned"]}}
    rep["holes"] = [
        {"name": h.name, "net": h.net, "xyz": holes_xyz[k].tolist(),
         "status": "occupied" if h.name in occupied else ("free" if h.name in seen_det else "unseen"),
         "detected": h.name in seen_det, "far_surface_points": int(surf[k])}
        for k, h in enumerate(grid.holes)
    ]

    # ── targets ──
    targets = [t for t in a.targets.split(",") if t]
    if targets and casings and "pins" in casings[0]:
        c = casings[0]
        spec_c = next(t for t in cat if t["casing"] == c["casing"])
        ex = np.array(spec_c["exit_x"]) / 1000
        allowed = grid.placements(ex, occupied=occupied)
        ok = tuple(sorted(targets)) in {tuple(sorted(p["holes"])) for p in allowed}
        idx = [grid.index[t] for t in targets]
        T = holes_xyz[idx]
        uvh = frame.to_uvh(P)
        others = np.ones(len(P), bool)
        cu = np.array(c["centre_uv"])
        others &= np.linalg.norm(uvh[:, :2] - cu, axis=1) > 0.012   # the moving casing itself
        foreign = []
        for t in idx:
            near_t = np.linalg.norm(uvh[:, :2] - placed[t], axis=1) < 0.0025
            foreign.append(int((near_t & others & (uvh[:, 2] > 0.0015) & (uvh[:, 2] < 0.012)).sum()))
        x = T[-1] - T[0]
        x = x - (x @ frame.n) * frame.n
        x /= np.linalg.norm(x)
        rep["targets"] = {
            "holes": targets, "allowed": ok, "placements": len(allowed),
            "xyz": T.tolist(), "midpoint": T.mean(0).tolist(), "x_axis": x.tolist(), "normal": frame.n.tolist(),
            "detected": [targets[i] in seen_det for i in range(len(idx))],
            "far_surface_points": [int(surf[t]) for t in idx], "foreign_points": foreign,
            "current": c["pins"]["holes"],
        }
        print(f"targets {targets}: allowed {ok} (of {len(allowed)} placements), detected {rep['targets']['detected']}, "
              f"far surface pts {rep['targets']['far_surface_points']}, foreign pts {foreign}, midpoint {np.round(T.mean(0) * 1000, 2).tolist()} mm")

    # ── overlays ──
    mark = {h: RED for h in occupied}
    mark.update({h: GREEN for h in targets})
    for i, v in enumerate(views):
        cam, det, f = v["cam"], v["det"], v["fit"]
        img = cam.median_color()
        xyz = frame.to_base(f["placed_uv"])
        px, z = cam.project(xyz)
        for k, h in enumerate(grid.holes):
            if z[k] <= 0:
                continue
            cross(img, *px[k], 2, mark.get(h.name, CYAN), 2 if h.name in mark else 1)
            if h.kind == "terminal" and h.row in ("A", "J") and (h.num % 5 == 0 or h.num == 1):
                text(img, h.name, px[k][0] + 3, px[k][1] - 3, 0.3)
            if h.name in mark:
                text(img, h.name, px[k][0] + 3, px[k][1] + 10, 0.35, mark[h.name])
        cv2.imwrite(str(a.out / f"near{i}_grid.png"), img)
        r = det["rect"].copy()
        res, u0, v0 = det["res"], det["u_rng"][0], det["v_rng"][0]
        for x, y in (det["uv"] - [u0, v0]) / res:
            cv2.circle(r, (int(round(x)), int(round(y))), 4, RED, 1)
        for k, (x, y) in enumerate((f["placed_uv"] - [u0, v0]) / res):
            cross(r, x, y, 3, mark.get(grid.holes[k].name, CYAN))
        cv2.imwrite(str(a.out / f"near{i}_rect.png"), r)

    def clean(o: Any) -> Any:
        if isinstance(o, dict):
            return {k: clean(v) for k, v in o.items() if k != "placed_uv"}
        if isinstance(o, (list, tuple)):
            return [clean(v) for v in o]
        if isinstance(o, np.ndarray):
            return o.tolist()
        if isinstance(o, (np.floating, np.integer, np.bool_)):
            return o.item()
        return o

    (a.out / "pinmap.json").write_text(json.dumps(clean(rep), indent=1) + "\n")
    print(f"wrote {a.out / 'pinmap.json'} and overlays")


if __name__ == "__main__":
    main()
