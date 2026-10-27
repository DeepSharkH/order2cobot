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
| Board model: hole id → board mm | `breadboard/board.py` (CAD hole table `dashboard/web/models/cad/breadboard.json`) | **TODO**: real board = 63 rows (user 10-27); fix CAD (60) + solver (64), measure e1→f1 |
| Manual calibration: corner inserts + workspace poses → board pose | `breadboard/teach.py`, `breadboard/calibration.py` → `configs/breadboard/calibration.yaml` | code done; **TODO** teach on the robot (2026-10-27) |
| Tool model of the custom fingers | `configs/breadboard/gripper.yaml` (CAD `assets/models/gripper/`) | **TODO** measure: provisional numbers |
| Grasp calibration after every grasp | `breadboard/grasp_calib.py` | **TODO** after the first successful grasps |
| Executor: pick → grasp calib → move → insert → verify | `breadboard/run_plan.py` (`--dry-run` prints the targets) | **TODO** |

Robot motion: `urcore` (`URExec` with the breadboard's own app/client and tool model, the shared
primitives `seat_z`, `pull_out`, `spiral_from`, `release`, `guarded_axial`). Never import
`porttest` (switch) from here. Run logging: `python -m urcore.runlog`, `urcore.telemetry`,
`urcore.record`.

## Calibration plan (user, 2026-10-26)

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
