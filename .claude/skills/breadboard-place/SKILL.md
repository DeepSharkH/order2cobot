---
name: breadboard-place
description: Place a gripped 2-leg casing (jumper, LED, ... from casings.json) into two named breadboard holes on the UR3e (AI Agent Bridge) with no operator teaching - board pose by touch, grasp calibration by touch, peck search, straight-z seat at low force, hole-pair self-check. Use when asked to insert / place a part on the breadboard, to re-locate the board, or to measure a grasp.
---

# Breadboard place — skill

Goal of a run: **the held part's legs in the two target holes**, found and checked by the agent,
with forces low enough not to bend the legs. Count operator actions (target: 0 per placement
once the part is in the gripper).

Read first: `breadboard/HANDOVER.md` (state, evidence, open problems), the lessons in
`skills_lib/breadboard/` (`python -m porttest.skilllib list` with domain breadboard, or read the
YAML files), and the memories `ur3-cable-limits`, `ur-bridge-motion`.

## Conventions (do not mix them up)

- **Holes**: column `a`-`j` + row `1`-`63`, e.g. `j20`. **Row 1 is the end closest to the robot**,
  row **a is the far long edge** (towards the checkerboard). Rows run along base -x, columns
  a -> j along base -y (`configs/breadboard/board_pose.yaml`).
- **Casings** use the `s<N>` spacing of `assets/models/component_casings/casings.json`:
  `led_s4` = legs **4 rows apart** (10.16 mm, 3 empty holes between). Never "holes in between".
- **TCP** = the fingertip centre, `fingertip_z` in `configs/breadboard/gripper.yaml` (0.2269 m,
  measured by touch; the CAD said 0.2545). The part's legs sit `grasp_dx` from it (measured).
- **Bridge client** `breadboard-explore`, app `breadboard_explore` (the operator grants Move &
  run once; Remote Control for every motion).

## URScript gate (before any program goes to the robot)

Check every piece of URScript you write or change against the **`polyscope-script`** skill
before it is played. That covers a new or edited primitive in `urlib.LIB`, a one-off step
script, a program generator (`breadboard/*.py`, `porttest/rj45_b.py`) and the skill_b dispatcher.

1. **Load `polyscope-script`** (Skill tool, once per session) and check the new code against it:
   missing built-ins, `local` for scratch variables in LIB helpers, every loop using up robot
   time (`speedl` with `t`, `sync()`), `force_mode` selection 0/1 and held-axis limits about
   0.001, `zero_ftsensor()` at rest, a force check in every force-mode loop, guarded IK, no
   `halt`, and the program never ending while pressing.
2. **Lint before spending a bridge round trip**: `python -m porttest.urscript_lint` for the LIB,
   or `from urcore.urscript_lint import lint` on the generated text. `URExec.run` refuses a
   script that fails the lint anyway.
3. **On a failure** (`compile_error_*`, `invalid_argument_value`, a `force_mode_*` abort, "Lost
   communication with Controller", a protective stop), look up the cause in `polyscope-script`
   before changing parameters. If the cause is new, add a **[seen]** entry there.

Validated primitives that only get new parameters skip step 1. Only check that the force caps
and `force_mode` limits are within the ranges in `polyscope-script`.

## Steps

### S0 Preflight (read-only)
`bridge_my_status` tier `move`, `robot_get_state` controlmode REMOTE, safety NORMAL. If not:
ask the operator, do not retry. Check `data/breadboard_state.json` (`inserted` = a part left in
the board, `offset_mm` = learned board offset).

### S1 Board pose (once per session, or after the board moved) — `breadboard/touch_locate.py`
`python -m breadboard.touch_locate` (plan) then `--run`: 3 top + 2 far-edge + 1 end guarded
touches at 4 N, 5 mm/s, ~90 s. Rough box: fixed cam + ArUco cube (see HANDOVER). Writes the run
to `data/runs/touch_locate_*`; update `board_pose.yaml` from it. It also re-measures the tool
length and fixes `gripper.yaml` when it changed by > 1 mm (sign: contact below the scene top =
tool SHORTER).

### S2 Place — `breadboard/place_probe.py`
```
python -m breadboard.place_probe <hole> <hole> --part <casing> --calib            # plan + IK, no motion
python -m breadboard.place_probe <hole> <hole> --part <casing> --calib --run --note "<how it was gripped>"
python -m breadboard.place_probe --confirm <hole> <hole>                         # what the operator / a check saw
python -m breadboard.place_probe --pull --run                                    # take the part out
```
One `--run` does: pull out a part left in the board -> **grasp calibration** (legs measured on
the hole-free strip, then the casing face against the row-1 end face at 1.5 N: `grasp_dx`) ->
low approach (2 cm) -> pecks at 2 N (centre, rings 0.7 / 1.4 mm) until the legs drop > 1.5 mm
-> `seat_z` capped at 6 N -> lift 5 mm (legs stay in, readable) -> hold. It prints
`pair_check` (legs within half a pitch of the target pair, else "ONE HOLE OFF suspected") and
`calib_residual_mm` (tilt indicator).

Holes must be in one column, `pin_spacing` rows apart. Rows past ~41 are refused (reach).

### S3 Verify
Until a camera check exists, the operator reads the holes; record it with `--confirm`. A
`pair_check` that is not ok is a miss even if the legs dropped.

### S4 Learn
Every miss gets its reason in the run's `result.json` (`miss_reason`) and in the state history,
and a lesson in `skills_lib/breadboard/` (status proposed -> validated only by a passing re-run).

## Hard rules (from real runs, see skills_lib/breadboard)

- Straight z only for pins; never x/y force mode (singularity fault 10-26).
- Touch 2 N, seat <= 6 N, side push 1.5 N: legs bend, parts tilt in the fingers.
- Never `halt` in a step (it skips the TCP reset); abort through if/else.
- A bridge IK that times out (status gap) means unreachable; x < -0.46 m is the elbow
  singularity at board height.
- A grasp offset over half a pitch (1.27 mm) lands one hole off and the search still "finds"
  holes: always `--calib` for a new grasp.
- Low approach: 2 cm above the board, then sideways.
- Wrist 3 within +-0.8 rad (camera cable): the planner picks the tool yaw for it.
