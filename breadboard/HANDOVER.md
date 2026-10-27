# Breadboard placing — handover to the picking team (2026-10-27)

**State:** the robot places a gripped 2-leg part into two named holes, finds and checks the hole
pair itself, with no operator teaching. **The part is still put into the gripper by hand.**
Picking it up (and letting go of it) is the next job. Code: branch `aspire` (on top of
`restructure`), skill `.claude/skills/breadboard-place/SKILL.md`, lessons `skills_lib/breadboard/`.

## What works (real robot, 2026-10-27)

| Capability | How | File |
|---|---|---|
| Board model | caliper measurements + BB830 datasheet: 63 rows, pitch 2.54 (confirmed over 32 rows), channel 7.62 | `configs/breadboard/board.yaml` |
| Board pose | guarded touches (3 top, 2 far edge, 1 end), ~90 s; rough box from the fixed cam + ArUco cube | `breadboard/touch_locate.py`, `configs/breadboard/board_pose.yaml` |
| Tool length | measured by the same touches, kept in `gripper.yaml` (0.2269 m, CAD said 0.2545) | `touch_locate.update_gripper` |
| Grasp calibration | legs on the hole-free strip (length), casing face against the row-1 end (offset along the legs), 1.5 N; repeatable to 0.01 mm | `breadboard/grasp_calib.py` |
| Placement | low approach, peck search at 2 N, seat <= 6 N, lift 5 mm, hold; learned board offset; hole-pair self-check | `breadboard/place_probe.py` |
| Run records | per run: contacts, offsets, leg length, grip settings + operator note, fixed / gripper cam shots, tilt residual, operator's hole reading | `data/runs/place_probe_*/` (gitignored), `data/breadboard_state.json` |

### Evidence: 8 placements, 6 hits (operator read every pair)

| Holes | Part | Grasp offset | Result |
|---|---|---|---|
| j10/j11 | jumper_s1 | — (hand-centred) | hit |
| j3/j4 | jumper_s1 | — | hit |
| j35/j36 | jumper_s1 | — | hit, first peck |
| j20/j22 | led_s2 | not measured | **miss: j19/j21.** Legs ~+1.6 mm off the fingertip (> half pitch): the search started nearer the neighbour pair |
| j20/j22 | led_s2 | +0.85 (wrong reference) | **miss: j19/j21**, flagged by the agent (`ONE HOLE OFF suspected`) |
| j20/j22 | led_s2 | +1.58 | hit, first peck |
| a25/a27 | led_s2 | +1.46 | hit, first peck (far half) |
| j30/j34 | led_s4 | +0.19 | hit, first peck, tilt residual -0.02 mm |

## What placing expects from picking (the interface)

1. **The part in the fingers, legs straight down**, the leg pair **along the finger faces**
   (across the jaw gap), casing bottom at the fingertips (legs ~5-7 mm below them).
2. **Its casing name** (`jumper_s1`, `led_s2`, `led_s4`, ... from `casings.json`): the leg
   spacing and casing width come from it. Today the operator has to know it - the picker should
   identify it (the operator could not tell which LED casing a part was).
3. Any offset of the part in the grip up to about +-3 mm along the legs is measured by
   `--calib`; across the legs (y) it is **not** measured yet (the board offset covers it while
   grasps are hand-centred).
4. Then: `python -m breadboard.place_probe <h1> <h2> --part <casing> --calib --run --note "..."`.

## Open problems the picking team inherits

- **Gripping force / release.** The bridge's Gripper Close / Open smart skills run at force 100,
  speed 100, full stroke (~185 N; it broke PLA fingers before). There is **no position or force
  readout** of the Hand-E (no bridge tool, no `rq_*` in our programs). The smart-skill list shows
  `force` / `speed` / `targetPosition` parameters - **untested whether `smartskill_run` accepts
  lower values**: that may be the gentle grip / partial release we need. Opening fully next to
  other parts can knock them (~+-25 mm per finger with Jan's fingers).
- **Leaving a part in the board** needs that release. Not done yet: every part was taken out
  again (step 3 of the stepping stones, "connect two parts", waits for it).
- **Per-grasp offset across the legs (y) and tilt.** The custom fingers are not snug on the
  Hand-E caps and parts tilt (the LED tilts easily, user). Only x is calibrated; tilt shows as
  `calib_residual_mm`. A second calibration height or a y face would close it.
- **Polarity.** LED / BJT orientation (which leg goes where) is not tracked: `place_probe`
  treats parts as symmetric. The picker has to know which way the part sits in the fingers.
- **Verification.** The operator reads the holes. A camera check (fixed / gripper cam shots
  are already saved per run) or a continuity / LED test should replace that.
- **Reach.** On the taped board only rows 1-~41 are reachable (x > -0.46 m, elbow singularity).
  The planned L fixture + ArUco (user decision) should sit ~5 cm closer to the robot.
- **Board pose source.** Today: taped board, fixed-cam ArUco cube for the rough box, touches
  for the rest. The dashboard's object locate (`protoboard`, two 83 mm halves) hung and does
  not match this board.
- **Assumption baked in:** the learned `offset_mm` is a board-pose error *because* the jumper
  grasps were hand-centred. Once the robot picks, re-learn it with a calibrated grasp.

## Not used / superseded

- Manual corner teaching (`breadboard/teach.py`, `calibration.py`): superseded by the touch
  pose (user: the operator must not teach per board). Kept for reference.
- The team CAD board (`dashboard/web/models/cad/breadboard.json`, 60 rows) and the solver's
  10.16 mm channel: wrong for this board; use `configs/breadboard/board.yaml` (63 rows, 7.62).
  **The CSP solver's `board_connections.json` still has the 10.16 channel - fix before
  planning real circuits.**
- `breadboard/run_plan.py` (plan executor) is still a skeleton: wire it to `place_probe` once
  picking exists.
