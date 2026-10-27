# Breadboard insertion

The robot side of Circuit Assembly: place casings on the breadboard in the order the CSP
solver verified, with Jan's custom fingers, at positions from the CAD board placed by a manual
calibration. This is a separate workflow from the switch / RJ45 task (`porttest/`) and from
the camera pin-map protoboard track (`.claude/skills/protoboard-wire`, `configs/wire/`).

## Pipeline

| Stage | Where | State |
|---|---|---|
| Natural language → circuit JSON | Dashboard Circuit Assembly tab (`dashboard/circuit_design.py`, `web/circuit.js`); offline: skill `circuit-netlist` | works |
| CSP solver → placements + verified `assembly_order` | `src/circuit_mapper/` (Jan: `mapper.py`, `geometry.py`, `board_connections.json`, `gripper.json`), casings `assets/models/component_casings/casings.json` | works (2 parallel LEDs time out on the Jetson) |
| Plan hand-off | "Start assembly" → `data/circuit_designs/<id>/plan.json`, `current.json`, `run.json`; progress `POST /api/circuit/<id>/run` | works |
| Board model: hole id -> board mm | `configs/breadboard/board.yaml` (calipers + BB830 datasheet: 63 rows, pitch 2.54, channel 7.62) | **works** (10-27); the CAD (60 rows) and the solver channel (10.16) are wrong |
| Board pose in base | `breadboard/touch_locate.py` -> `configs/breadboard/board_pose.yaml` (guarded touches, no teaching) | **works** (10-27) |
| ~~Manual calibration (corner inserts)~~ | `breadboard/teach.py`, `calibration.py` | **superseded** by the touch pose (user 10-27); kept |
| Tool model of the custom fingers | `configs/breadboard/gripper.yaml` | fingertip **measured** (0.2269 m, by touch); grasp height still provisional |
| Grasp calibration after every grasp | `breadboard/grasp_calib.py` (legs on the strip, casing face vs the row-1 end) | **works** for x along the legs (10-27); y / tilt open |
| Place a held part | `breadboard/place_probe.py`, skill `breadboard-place` | **works**: 8 runs, 6 hits (see HANDOVER.md) |
| Executor: pick -> grasp calib -> move -> insert -> verify | `breadboard/run_plan.py` (`--dry-run` prints the targets) | **TODO**: picking + release (picking team, HANDOVER.md) |

**Start here: `breadboard/HANDOVER.md`.**

Robot motion: `urcore` (`URExec` with the breadboard's own app/client and tool model, the shared
primitives `seat_z`, `pull_out`, `spiral_from`, `release`, `guarded_axial`). Never import
`porttest` (switch) from here. Run logging: `python -m urcore.runlog`, `urcore.telemetry`,
`urcore.record`.

## Calibration plan (user, 2026-10-26) — superseded 10-27 by touch_locate + grasp_calib

1. CAD hole table (`board.py`): 63 rows (user-checked 10-27); measure e1 → f1, fix the CAD,
   the solver board and the corner annotations (TODO list in `board.py`) before teaching.
2. Manual corner inserts: seat a centre-gripped part by hand at 3+ corner hole pairs (same
   tool orientation), `python -m breadboard.teach corner A1 A3` each, plus the workspace poses
   (`teach pose <name>`), then `teach fit`. Residuals > 0.3 mm point at the board model or a
   corner.
3. After the first successful grasps: a grasp calibration step per grasp (`grasp_calib.py`).

## Open questions (for Jan)

- At what height do the fingertips grip a 20 mm casing? (sets `tcp_pins` z and the taper the
  solver must model: 2.0 x 5.0 mm at the tip, 4.8 x 7.5 mm 20 mm up; `gripper.json` models a
  constant 2 x 5 mm prism.)
- Release: the Hand-E only opens fully through the bridge smart skills (50 mm stroke), the
  solver assumes 1 mm per finger (`opening_per_finger_mm`).
- What does "R7" in the gripper file name mean?
- `verifyer/verifying_server.py` (MiniCPM-V yes/no check): meant for the verify phase?

Lessons from the real robot that carry over: `breadboard/NOTES.md`.
