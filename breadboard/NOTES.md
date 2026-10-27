# Notes for breadboard insertion

Copied from the real-robot runs of other tracks (sources in brackets); the originals stay
where they are.

## Inserting pins (protoboard-wire run 20261026-174057, porttest/LEARNINGS.md "Protoboard wire")

- Pins go in by **straight z moves only** (user rule). x/y-compliant `seat_compliant` (8 N,
  stale F/T zero) faulted with `force_mode_singularity`, powered off and pulled the pins out.
  `seat_z` straight down to a 15 N cap seated a jumper 5.5 mm past the surface.
- Force-mode selection vectors are 0/1: use `n = [0, 0, -1]` exactly.
- Re-zero the F/T sensor in every force step: it read ~6 N off after a 12.2 N pull.
- Holding force of a seated jumper: extraction peak 12.2 N; pin length ~7.8 mm.
- A 0.8 mm spiral missed when grasp → target was 60 mm with ~0.7 deg hand-eye yaw (~1 mm);
  1.2 mm found the holes 1.0 mm off. The hole entry funnel is 1.7 mm square: know where the
  pins are before descending after any grip change (→ grasp calibration).

## Gripper (same run)

- The Hand-E works only through the bridge smart skills of close_gripper's application
  (100 % speed / force, full stroke). The URCap lost the Hand-E once ("Gripper with ID 9 not
  found"); the operator reconnected it. Partial-stroke attempts (porttest/hande_app.py,
  grip_app_import.py) failed: the .urpx import was rejected.
- Stock fingertips sit 222 mm below the flange; open fingers span ±33 mm and touched a
  neighbouring 20 mm casing.

## Bridge and cell (porttest/LEARNINGS.md "Robot control through the AI Agent Bridge")

- No jog tool: motion = URScript in one ur-script node, played per step (`urcore.urexec`).
  Needs the move tier for the client name (granted once on the pendant) and Remote Control.
- `play` often times out although the program started; `program_not_ready` right after an
  edit: retry after 2 s. ~9-20 s per step: pack sequences into one step.
- `set_tcp` persists after a program: every step ends with the flange TCP (URExec does it).
- Wrist 3 winds the camera cable: stay within ±0.8 rad of the reference (memory
  ur3-cable-limits); `move_checked(w3_ref=...)`.
- Tool force limit in the safety config is 100 N: limit force in the script.
