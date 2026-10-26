#!/usr/bin/env python3
"""Locate the wire on the breadboard from the fixed D435 point cloud and hand the result
to the robot program, which then only runs trajectories (no camera access on the robot).

    uv run scripts/wire_from_cloud.py measure                     # capture + checks, no robot writes
    uv run scripts/wire_from_cloud.py apply --program challenge2_hard_v2
    uv run scripts/wire_from_cloud.py verify --program challenge2_hard_v2   # after the run

The target is read from the program header (TARGET_DCOL / TARGET_DROW, in holes); --dcol and
--drow override it for `measure`. Every run writes results/<timestamp>_wire_from_cloud/ with
the measurement (measurement.json) and an overlay of the detections on the camera image.

Frames: base = UR base. Board axes: col_axis along the board's long side (columns, oriented
towards base +x), row_axis = normal x col_axis (across the rows), normal = board top, up.
"""

import argparse
import asyncio
import io
import json
import re
import sys
import time
import warnings
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import httpx
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from agents.core.config import load_yaml  # noqa: E402
from agents.robot import Bridge, RobotConfig, acquire_client_name  # noqa: E402

VISION_CONFIG = ROOT / "configs" / "vision.yaml"
ROBOT_CONFIG = ROOT / "configs" / "robot.yaml"
CLIENT_NAME = "claude-cli-task-builder"  # a connection the pendant already grants Adjust
MEASURE_MARK = "# STEP 2a"


class CheckFailed(RuntimeError):
    """A pre-motion check failed; the robot must not move."""


# ── capture ──────────────────────────────────────────────────────────────────


def capture(cfg: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    """Colour frame and the median of several aligned depth frames (metres, 0 = none)."""
    url, cam = cfg["dashboard_url"].rstrip("/"), cfg["camera"]
    with httpx.Client(timeout=10) as http:
        r = http.get(f"{url}/api/frame/{cam}")
        r.raise_for_status()
        color = cv2.imdecode(np.frombuffer(r.content, np.uint8), cv2.IMREAD_COLOR)
        depths = []
        for _ in range(cfg["depth_frames"]):
            r = http.get(f"{url}/api/depth/{cam}")
            r.raise_for_status()
            depths.append(np.load(io.BytesIO(r.content)))
            time.sleep(0.25)  # the dashboard refreshes depth at ~5 Hz
    stack = np.stack(depths).astype(np.float32)
    stack[stack <= 0] = np.nan
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)  # pixels never seen -> 0
        depth = np.nan_to_num(np.nanmedian(stack, axis=0), nan=0.0)
    return color, depth


def camera_in_base(cfg: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    """(T_base_cam, K) from the dashboard's scene calibration."""
    sc = yaml.safe_load((ROOT / cfg["scene_file"]).read_text())
    b = sc["base"]
    c, s = np.cos(np.radians(b["yaw_deg"])), np.sin(np.radians(b["yaw_deg"]))
    T_world_base = np.eye(4)
    T_world_base[:2, :2] = [[c, -s], [s, c]]
    T_world_base[:3, 3] = b["x"], b["y"], b.get("z", 0.0)
    T_base_cam = np.linalg.inv(T_world_base) @ np.array(sc["T_world_fcam"])
    return T_base_cam, np.array(sc["fcam"]["K"])


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


# ── geometry ─────────────────────────────────────────────────────────────────


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
        raise CheckFailed("no horizontal plane found in the work area")
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


# ── measurement ──────────────────────────────────────────────────────────────


def measure(cfg: dict[str, Any], dcol: int, drow: int, out: Path, accept_unseen: bool,
            target_checks: bool = True) -> dict[str, Any]:
    color, depth = capture(cfg)
    T, K = camera_in_base(cfg)
    P, uv = cloud(depth, K, T)
    area, det, brd = cfg["work_area"], cfg["detection"], cfg["board"]
    inside = (P[:, 0] > area["x"][0]) & (P[:, 0] < area["x"][1]) & (P[:, 1] > area["y"][0]) & (P[:, 1] < area["y"][1])
    P, uv = P[inside], uv[inside]

    table_c, table_n = fit_plane(P, 0.002)
    ht_table = (P - table_c) @ table_n
    lo, hi = brd["height_above_table"]
    board_c, nrm = fit_plane(P[(ht_table > lo) & (ht_table < hi)], 0.001, seed=1, up=table_n,
                             max_tilt_deg=3.0)
    h = (P - board_c) @ nrm
    surface = np.abs(h) < det["plane_tol"]

    res = 0.00025
    rails = board_rails(plane_view(color, T, K, board_c, nrm, area, res), area, res,
                        brd["min_rail_length"])
    if len(rails) < 2:
        raise CheckFailed(f"breadboard rail lines not found ({len(rails)} of at least 2)")
    weights = np.array([len(p) for _, p in rails], float)
    long_xy = (np.array([d for d, _ in rails]) * weights[:, None]).sum(0)
    long_xy /= np.linalg.norm(long_xy)
    spread = max(abs(np.degrees(np.arcsin(long_xy[0] * d[1] - long_xy[1] * d[0])))
                 for d, _ in rails)
    if spread > brd["max_rail_spread_deg"]:
        raise CheckFailed(f"rail lines disagree on the board direction by {spread:.1f} deg")
    across = np.array([-long_xy[1], long_xy[0]])
    rail_pts = np.vstack([p for _, p in rails])
    u_span = (rail_pts @ long_xy).min(), (rail_pts @ long_xy).max()
    v_span = (rail_pts @ across).min(), (rail_pts @ across).max()
    col = np.r_[long_xy, 0.0]
    col -= (col @ nrm) * nrm
    col /= np.linalg.norm(col)
    row = np.cross(nrm, col)

    def on_board(xy: np.ndarray, margin: float = 0.0) -> bool:
        """Within the length the rails cover and between the outermost rails, `margin` inside
        (negative: slack past the rail ends only)."""
        u, v = xy @ long_xy, xy @ across
        return bool(u_span[0] + margin <= u <= u_span[1] - margin
                    and v_span[0] + max(margin, 0) <= v <= v_span[1] - max(margin, 0))

    # objects may stand where tape hides the rails, so allow slack along the board
    objs = [o for o in objects_on_board(P, h, det)
            if on_board(P[o, :2].mean(0), -det["object_rail_slack"])]
    if not objs:
        raise CheckFailed("no wire standing on the breadboard")
    if len(objs) > 1:
        sizes = ", ".join(str(len(o)) for o in objs)
        raise CheckFailed(f"{len(objs)} objects on the board (points: {sizes}); expected one wire")
    wire = objs[0]
    # centre from the body only: skip the depth fringe at the board and the wire on top
    hw = h[wire]
    Q = P[wire][(hw > det["object_min_height"]) & (hw < 0.9 * hw.max())]
    u = (Q - board_c) @ col
    v = (Q - board_c) @ row
    uc = (np.percentile(u, 2) + np.percentile(u, 98)) / 2
    vc = (np.percentile(v, 2) + np.percentile(v, 98)) / 2
    hc = board_c + uc * col + vc * row  # wire centre on the board top
    if target_checks and dcol == 0 and drow == 0:
        raise CheckFailed("target is the current hole (TARGET_DCOL = TARGET_DROW = 0)")
    pitch = brd["pitch"]
    ht = hc + dcol * pitch * col + drow * pitch * row

    # target checks: on the board, free, actually seen by the camera
    notes: list[str] = []
    near_t = np.linalg.norm(P[:, :2] - ht[:2], axis=1)
    seen = int((surface & (near_t < det["target_seen_radius"])).sum())
    if target_checks and not on_board(ht[:2], det["edge_margin"]):
        raise CheckFailed("target hole lies outside the breadboard area the camera can confirm")
    others = np.ones(len(P), bool)
    others[wire] = False
    # only low points can be something seated in the hole; the moving wire's own cable
    # arches high over the board and must not count
    up = (h > det["object_height"][0]) & (h < det["target_free_height"]) & others
    if target_checks and (up & (near_t < det["target_free_radius"])).any():
        raise CheckFailed("target hole is occupied")
    if target_checks and seen < det["target_seen_min_points"]:
        msg = f"target hole not visible to the camera ({seen} surface points; shadowed by the wire?)"
        if not accept_unseen:
            raise CheckFailed(msg + " - rerun with --accept-unseen-target to proceed anyway")
        notes.append(msg)

    m = {
        "stamp": datetime.now().isoformat(timespec="seconds"),
        "dcol": dcol,
        "drow": drow,
        "normal": nrm.tolist(),
        "col_axis": col.tolist(),
        "row_axis": row.tolist(),
        "current": hc.tolist(),
        "target": ht.tolist(),
        "board": {"rails": len(rails), "rail_spread_deg": spread,
                  "col_axis_deg": float(np.degrees(np.arctan2(long_xy[1], long_xy[0]))),
                  "top_above_table": float((board_c - table_c) @ table_n)},
        "wire": {"points": len(wire), "height": float(h[wire].max()),
                 "extent_col": float(np.percentile(u, 98) - np.percentile(u, 2)),
                 "extent_row": float(np.percentile(v, 98) - np.percentile(v, 2))},
        "target_surface_points": seen,
        "warnings": notes,
    }
    out.mkdir(parents=True, exist_ok=True)
    (out / "measurement.json").write_text(json.dumps(m, indent=2))
    overlay(color, T, K, hc, ht, uv[wire], out / "overlay.png")
    return m


def overlay(color: np.ndarray, T: np.ndarray, K: np.ndarray, hc: np.ndarray, ht: np.ndarray,
            wire_px: np.ndarray, path: Path) -> None:
    img = color.copy()
    img[wire_px[:, 1], wire_px[:, 0]] = (0, 0, 255)
    Ti = np.linalg.inv(T)
    for p, col in ((hc, (0, 0, 255)), (ht, (0, 200, 0))):
        q = Ti[:3, :3] @ p + Ti[:3, 3]
        x, y = K[0, 0] * q[0] / q[2] + K[0, 2], K[1, 1] * q[1] / q[2] + K[1, 2]
        cv2.drawMarker(img, (int(x), int(y)), col, cv2.MARKER_CROSS, 18, 2)
    cv2.imwrite(str(path), img)


# ── robot program ────────────────────────────────────────────────────────────


def fmt(v: list[float]) -> str:
    return "[" + ", ".join(f"{x:.6f}" for x in v) + "]"


def measurement_script(m: dict[str, Any]) -> str:
    cur = np.array(m["current"]) * 1000
    lines = [
        f"{MEASURE_MARK} - measurement from the workstation D435 point cloud ({m['stamp']}),",
        "# written by scripts/wire_from_cloud.py apply. Do not edit by hand: re-run the script.",
        f'MEAS_STAMP = "{m["stamp"]}"',
        f"MEAS_DCOL = {m['dcol']}",
        f"MEAS_DROW = {m['drow']}",
        f'cur_label = "wire at x={cur[0]:.1f} y={cur[1]:.1f} mm"',
        f'tgt_label = "{m["dcol"]:+d} col / {m["drow"]:+d} row"',
        f"nrm = {fmt(m['normal'])}",
        f"xh = {fmt(m['col_axis'])}",
        f"yh = {fmt(m['row_axis'])}",
        f"hc = {fmt(m['current'])}",
        f"ht = {fmt(m['target'])}",
        "g_o = hc",
        "wd = yh",
        "if MEAS_DCOL != TARGET_DCOL or MEAS_DROW != TARGET_DROW:",
        '  abort_reason = "Measurement was taken for another target - re-run scripts/wire_from_cloud.py apply."',
        '  textmsg("challenge2: ABORT: ", abort_reason)',
        "  popup(abort_reason, ABORT_TITLE, False, True, blocking=True)",
        "  halt",
        "end",
        'textmsg("challenge2: measurement ", MEAS_STAMP)',
        'textmsg("challenge2: CURRENT ", cur_label)',
        'textmsg("challenge2: TARGET ", tgt_label)',
        'textmsg("challenge2: current hole (base) = ", hc)',
        'textmsg("challenge2: target hole (base) = ", ht)',
        'textmsg("challenge2: board normal / column axis = ", [nrm, xh])',
    ]
    return "\n".join(lines)


def read_target(expression: str) -> tuple[int, int]:
    def num(name: str) -> int:
        hit = re.search(rf"^{name}\s*=\s*(-?\d+)", expression, re.M)
        if not hit:
            raise CheckFailed(f"{name} not found in the program header")
        return int(hit.group(1))
    return num("TARGET_DCOL"), num("TARGET_DROW")


def walk(node: dict[str, Any]) -> list[dict[str, Any]]:
    out = [node]
    for c in node.get("children", []):
        out += walk(c)
    return out


def expr(node: dict[str, Any]) -> str:
    return str(node.get("contributedNode", {}).get("parameters", {}).get("expression") or "")


async def program_nodes(bridge: Bridge, name: str) -> tuple[str, str, list[dict[str, Any]]]:
    """(appId, programId, all nodes) of a program on the robot."""
    progs = (await bridge.call("program_list"))["programs"]
    app = next((p["id"] for p in progs if p["name"] == name), None)
    if app is None:
        raise CheckFailed(f"program {name} not on the robot")
    doc = json.loads(await bridge.download_urpx(name))
    return app, doc["program"]["id"], walk(doc["program"]["programContent"])


async def with_bridge(fn: Any) -> Any:
    robot = load_yaml(ROBOT_CONFIG, RobotConfig)
    async with acquire_client_name(CLIENT_NAME, robot.client_pool_size, robot.lock_dir) as name, \
            Bridge(robot, name) as bridge:
        return await fn(bridge)


async def target_of(program: str) -> tuple[int, int]:
    async def go(bridge: Bridge) -> tuple[int, int]:
        _, _, nodes = await program_nodes(bridge, program)
        header = next((expr(n) for n in nodes if "TARGET_DCOL" in expr(n)), None)
        if header is None:
            raise CheckFailed("program header with TARGET_DCOL not found")
        return read_target(header)
    result: tuple[int, int] = await with_bridge(go)
    return result


async def write_measurement(program: str, m: dict[str, Any]) -> dict[str, Any]:
    async def go(bridge: Bridge) -> dict[str, Any]:
        app, pid, nodes = await program_nodes(bridge, program)
        node = next((n for n in nodes if expr(n).startswith(MEASURE_MARK)), None)
        if node is None:
            raise CheckFailed(f"no script node starting with '{MEASURE_MARK}' in {program}")
        await bridge.call("program_update_node", appId=app, programId=pid, guid=node["guid"],
                          parameters={"expression": measurement_script(m)})
        result: dict[str, Any] = await bridge.call("program_validate", appId=app)
        return result
    result: dict[str, Any] = await with_bridge(go)
    return result


# ── verification ─────────────────────────────────────────────────────────────


def verify(cfg: dict[str, Any], before: dict[str, Any], out: Path) -> dict[str, Any]:
    """Re-measure (no target checks) and compare the wire position with the planned target."""
    after = measure(cfg, before["dcol"], before["drow"], out, accept_unseen=True, target_checks=False)
    moved_to = np.array(after["current"])
    err_t = float(np.linalg.norm((moved_to - np.array(before["target"]))[:2]))
    err_c = float(np.linalg.norm((moved_to - np.array(before["current"]))[:2]))
    pitch = cfg["board"]["pitch"]
    return {"wire_now": after["current"], "dist_to_target_mm": err_t * 1000,
            "dist_to_original_mm": err_c * 1000, "ok": err_t < pitch / 2 and err_c > pitch / 2}


def consistent_measure(cfg: dict[str, Any], dcol: int, drow: int, out: Path,
                       accept_unseen: bool) -> dict[str, Any]:
    """Several independent captures; the one nearest their median, if they all agree.
    A single capture now and then fits the board plane badly, and the arm must not move on it."""
    runs = [measure(cfg, dcol, drow, out / f"capture_{i}", accept_unseen)
            for i in range(cfg["repeat"]["captures"])]
    cur = np.array([r["current"] for r in runs])
    med = np.median(cur, axis=0)
    dev = np.abs(cur - med).max(axis=1)
    if np.sort(dev)[-1] > cfg["repeat"]["max_spread"]:
        spread = ", ".join(f"{d * 1000:.2f}" for d in dev)
        raise CheckFailed(f"captures disagree (mm from median: {spread}); is the wire moving?")
    best = runs[int(np.argmin(dev))]
    (out / "measurement.json").write_text(json.dumps(best, indent=2))
    return best


# ── cli ──────────────────────────────────────────────────────────────────────


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["measure", "apply", "verify"])
    ap.add_argument("--program", default="challenge2_hard_v2")
    ap.add_argument("--dcol", type=int, help="target offset in columns (default: program header)")
    ap.add_argument("--drow", type=int, help="target offset in rows (default: program header)")
    ap.add_argument("--accept-unseen-target", action="store_true",
                    help="proceed when the camera cannot see the target hole")
    ap.add_argument("--config", type=Path, default=VISION_CONFIG)
    args = ap.parse_args()
    cfg = yaml.safe_load(args.config.read_text())
    out = ROOT / "results" / f"{datetime.now():%Y%m%d_%H%M%S}_wire_from_cloud"
    try:
        if args.command == "verify":
            last = sorted((ROOT / "results").glob("*_wire_from_cloud/measurement.json"))
            applied = [p for p in last if (p.parent / "applied").exists()]
            if not applied:
                raise CheckFailed("no applied measurement to verify against")
            before = json.loads(applied[-1].read_text())
            res = verify(cfg, before, out)
            print(json.dumps(res, indent=2))
            return 0 if res["ok"] else 1
        dcol, drow = args.dcol, args.drow
        if dcol is None or drow is None:
            hd, hr = asyncio.run(target_of(args.program))
            dcol = hd if dcol is None else dcol
            drow = hr if drow is None else drow
        m = consistent_measure(cfg, dcol, drow, out, args.accept_unseen_target)
        print(json.dumps(m, indent=2))
        print(f"saved {out.relative_to(ROOT)}")
        if args.command == "apply":
            res = asyncio.run(write_measurement(args.program, m))
            (out / "applied").write_text(args.program + "\n")
            print("program updated; validate:", json.dumps(res))
        return 0
    except CheckFailed as exc:
        print(f"STOP: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
