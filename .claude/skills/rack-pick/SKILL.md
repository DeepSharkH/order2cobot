---
name: rack-pick
description: Pick a 2-leg casing (jumper, LED, ... from casings.json) out of the casing rack on the UR3e + Hand-E (AI Agent Bridge) and hand it to breadboard-place - rack pose by touch, height calibration by touch at the slot, grasp height and position from the rack and casing CAD, straight-z pull out, checks for the placing hand-off. Use when asked to pick/fetch/take a component, part, LED, resistor, jumper or casing from the rack/holder, to locate the rack, or to retry a pick.
---

# Rack pick — skill

**Goal:** the requested casing out of the rack and in the fingers the way `breadboard-place`
needs it (see "Hand-off"), found and checked by the agent. Count operator actions; the target
is 0 per pick once the rack is located (one-time setup does not count).

This is the picking half of the breadboard task. Placing works (`breadboard-place`, 8 runs,
6 hits on 2026-10-27). Until now a human put every part into the gripper.

**Read first:** `breadboard/HANDOVER.md` (written for this job: how we work, the interface,
constraints, assumptions), `.claude/skills/breadboard-place/SKILL.md` (conventions, hard
rules), the lessons in `skills_lib/breadboard/` (read before writing any robot step), the
memories `ur3-cable-limits`, `ur-bridge-motion`, `pin-insertion-z-only`, and
`assets/models/component_casings/casings.json`.

## Setup today (2026-10-27)

- The **blue rack** `assets/models/All_Casings_Holder_Print.stl`: a plate with 20 slits.
- A **pink post-it** on the rack top turns the slits into pin holes: the casing's legs are
  pushed through the paper into a slit, and the casing stands **on the post-it, on the rack
  top**. It does not sit in the pocket.
- A new rack with real pin holes is being printed. The steps stay the same; only the slot
  positions come from the new CAD (and the casing x' is fixed by the holes).

## The rack (from the STL, mm)

The STL is saved upside down (print orientation). On the table it is flipped: feet down,
labels on top and readable.

**Rack frame:** origin at the front-left corner, on the table. x' along the front edge, y'
away from the front, z' up. Front = the side the labels read from: row S1 nearest, slot 1 on
the left. From STL coordinates: `x' = x`, `y' = 71.2 − y`, `z' = 18 − z`.

- Footprint 181.2 × 71.2, corners rounded (~2). Plate z' 10–18: **rack top = z' 18**. Four
  feet Ø14 × 10.
- **Slot k** (1–20): row r = (k − 1) // 5 (0 = S1), column c = (k − 1) % 5.
  Centre **x' = 20.55 + 35.05·c, y' = 11.15 + 16.3·r**.
- Each slot: a pocket 24.2 (x') × 5.4 (y') × 4.5 deep, and under it a slit 16.5 × 1.9 through
  the plate for the legs.
- Labels engraved 0.4 deep: the slot number just in front of its pocket (y' −6.2 to −2.8 from
  the slot centre), the row name `S1`…`S4` at the left edge (x' 2.2–5.5).
- Between two pockets of a row there is solid rack top from 12.1 to 23.0 mm off the slot
  centre along x' (good spot for a touch at **±17.5**).
- **It looks the same turned 180°.** Touches cannot tell which way round it is (slot k ↔
  slot 21 − k). The orientation comes from `configs/breadboard/rack.yaml`, set once by the
  operator or by a label read in a camera image.
- **Casing on the post-it:** bottom on the rack top (+ paper, ~0.1), top 20 above it (the
  casing CAD). The slit holds the legs ±0.5 in y'. Along x' the casing can be anywhere the
  slit allows (±6 for jumper_s1): today assumed where the operator pinned it, at the slot
  centre.

## Constraints and assumptions (HANDOVER style; give each a verdict before coding)

Constraints (fixed today): every motion is a played URScript program (no jogging); the
Hand-E opens and closes only through the **MCP smart skills** Gripper Open / Gripper Close
(`smartskill_run(type="robotiq-gripper-open-smart-skill" | "robotiq-gripper-close-smart-skill")`,
no parameters, 100 % force ~185 N, full 50 mm stroke; never change the skills' parameters),
with no position or force readout; the PLA fingers are not snug on the Hand-E caps and broke
under ~185 N before; reach x > −0.46 m; wrist 3 within ±0.8 rad.

| Assumption | Verdict |
|---|---|
| The close skill accepts a lower force / target position | **Drop** (10-27): `smartskill_run` takes no parameters and the skills stay at 100/100; the full-force close on a 4 mm casing works (R4 pick). |
| The rack does not move when touched | **Investigate**: it is light PLA. Tape it down; re-touch the first top point at the end of the locate run (> 0.3 mm = it moved). |
| A camera box is good enough to find the slots | **Drop** for the rack pose: the fixed cam was ~20 mm off in base (10-27); the open fingers clear the casings two rows away by 2.3 mm. Rack pose by touch. **Keep** for *which* slot holds a casing: `find` measures the casing against the rack's own slits in the same image (relative, not absolute). |
| The casing sits at the slot centre along x' | **Investigate**: true if the operator pinned it there. Check with the camera to ±3 mm (that is what `--calib` in placing can absorb). Exact with the pin-hole rack. |
| The casing bottom is on the rack top | **Check every pick** by the height calibration (S2). A casing that sank through the paper into the pocket cannot be gripped low enough. |
| The post-it lies flat | **Keep**, but look at the camera image: a curled edge under the open fingers (1 mm above the top) can be caught. |
| The slot inventory is right | **Check every pick**: casing height and length against its CAD. The operator cannot tell LED casings apart; the slot is the identity. |

## Conventions

- **Slots:** `R<k>`, k = the number on the rack. **Casings:** `s<N>` from `casings.json`
  (`led_s2` = legs 2 rows apart).
- **TCP** = the fingertip centre, `fingertip_z` from `configs/breadboard/gripper.yaml`
  (0.2269 m, measured by touch with these fingers). Programs use `tcp_tip = [0, 0,
  fingertip_z, 0, 0, 0]`, like placing. `tcp_pins` there has z **into** the board.
- **Jaw** closes along flange x. Close it **across the rows** (along y'): it squeezes the
  casing's flat 4 mm faces, so the leg pair lies along the finger faces (flange y), as placing
  needs. Of the two yaws 180° apart take the one that keeps wrist 3 near the cable reference.
- **Files:** `configs/breadboard/rack.yaml` (rack geometry from the CAD, rack pose by touch,
  orientation; every number with `source:` and `stamp:`), `data/rack_state.json` (what is in
  each slot, `pin1` direction, history), runs in `data/runs/rack_locate_*` and
  `data/runs/rack_pick_*` (`result.json`, with `miss_reason` on a miss).
- **Code:** `breadboard/rack_pick.py`, built on `urcore` (`URExec`, `urlib`
  `guarded_axial` / `back_off` / `pull_out`, `urscript_lint`). Never import `porttest`. Reuse
  from `breadboard/`: `touch_locate.py` (touch program, plane/edge fit, `tool_down`),
  `place_probe.py` (`ik_ok`, `grip_record`, `snapshot`). Own bridge client
  **`breadboard-pick`**, app `breadboard_pick` (the operator grants Move & run once; Remote
  Control for every motion).
- Every command has a plan mode (lint + IK of every point, no motion); `--run` plays it.

```
python -m breadboard.rack_pick locate [--run]                   # S1, once: writes the rack.yaml pose
python -m breadboard.rack_pick find                             # which slots hold a casing (fixed cam)
python -m breadboard.rack_pick pick R7 --part led_s2 [--run]    # S2-S3: holds at the grasp, fingers open
python -m breadboard.rack_pick pick R7 --part led_s2 --close --run     # S3 close + S4 + S5
python -m breadboard.rack_pick --confirm R7 "<what the operator / a check saw>"
```
`--close --closed` pulls only (fingers already closed, e.g. after a stopped pull); `--pull-stop N`
changes the 8 N stop.

## Typical use

The rack is located **once** (S1, `rack.yaml` pose by touch; again only if the rack moved).
A pick is then: S0 → `find` (the slot) → `pick R<k> --run` → `pick R<k> --close --run` → S5.
Only the slot changes between picks; the agent finds it, the operator does not tell it.

## Steps

### S0 Preflight (read-only)
- `bridge_my_status` tier `move`, `robot_get_state` controlmode REMOTE, safety NORMAL. If
  not: ask the operator, do not retry.
- Gripper empty. Fingers on: the rack fingers (`Gripper_R7_W15to5_L30_T2_C0p2`).
- Read `data/rack_state.json` (what is in the slot asked for) and `data/breadboard_state.json`
  (a part left in the board?).
- **Hand-E activated** (pendant). Not activated, every MCP open/close fails in the URCap's
  `driver_griper_client.setSpeed` with `xmlrpc_unknown_problem ... "Invalid request" ... "3"`
  in `psx_logs` while `smartskill_run` still says `started` (10-27, lesson
  `gripper-skill-lost-hande`). After a reboot or an activation, re-check the arm pose.
- Logging: `python -m urcore.runlog start pick-R<k> --goal "..."`, then a `step` per stage
  and `note assumption/observed/interpretation/operator`; append-only.

### S1 Rack pose by touch (once per session, or after the rack moved)
- **Rough box:** fixed camera (+ ArUco cube, as in HANDOVER) → rack corner and yaw to ~10 mm.
  Check the orientation (which end has the `S1`…`S4` labels) against `rack.yaml`.
- **Touches**, one program, closed fingertips, tool straight down:
  - 3 on the top, on solid rack between pockets (e.g. x' ±17.5 from a slot centre, on a row
    line, towards the rack centre), at **2 N, 5 mm/s**: top plane.
  - 2 on the long edge on the **+y base side** (free table), moving −y, 4 mm below the top, at
    **1.5 N** (the rack is light): edge line. Start 15 mm outside the rough edge, push up to
    30 mm (10-27: the camera box was 20 mm off; the first try missed the edge).
  - 1 on the **breadboard end**, moving +x base, same height and force, on the −y half (the
    white cube sits on the +y rail). Not the robot end: it folds the arm back over its base.
  - The first edge and the first top point again at the end: > 0.3 mm = the rack moved.
  - Both faces work for either orientation (`rack.yaml` `slot1_end`); the CAD gives the rest.
- Fit: top plane, front edge line, left end → `T_base_rack`. Slot positions from the CAD on
  that frame. Write it to `rack.yaml` (`source`: run dir, `stamp`).
- Like `touch_locate`, the top touches also check `fingertip_z` (sign: contact below the
  expected top = tool **shorter**, lesson `tool-length-sign`).

### S2 Height calibration at the slot (every pick)
- **Slot:** `find` (fixed cam): white blobs on the rack top, foot → nearest slot (`in_slot`
  within ±10 mm x', ±6 mm y'). Cross-check with `rack_state.json`.
- **Fingers closed** first (MCP Gripper Close on the empty gripper; the touches use the closed tips).
- **Surface:** closed fingertips touch the rack top (post-it) right next to the target casing:
  x' = slot centre ± 17.5 (towards the rack centre), y' = row centre, 2 N, 2 mm/s.
  → `z_surface`.
- **Casing top:** closed fingertips touch the casing top at the slot centre, **1 N**, 2 mm/s.
  → `z_top`. If the contact keeps sinking (the casing is pushed through the paper), stop.
- **CAD check:** `z_top − z_surface` = casing `height` (20 mm) ± 1 mm. Else: wrong casing,
  sunk into the pocket, or tilted → miss, log why, do not grasp.
- Casing x' along the slit: the slot centre (CAD), checked in a camera image to ±3 mm.

### S3 Grasp
- **Grasp point** (fingertip TCP): x' = casing centre, y' = row centre, tips at **z_surface +
  1 mm**: the casing bottom is at the fingertips, as placing needs. Jaw along y'.
- **Finger room:** open fingertips at ~±25 mm along y'. Casings one row away: 6.7 mm clear;
  two rows away: 2.3 mm. That only holds with the touched rack pose, a yaw error < 1°, and no
  leaning casing (camera check).
- **Moves:** open the gripper (MCP Gripper Open) → transit 2 cm above the highest casing top →
  down over the slot, guarded at 2 N, 5 mm/s, to the grasp height (a stop before it = a finger
  hit something: abort, do not close) → close (MCP Gripper Close) → read `psx_logs` (any
  WARNING = the fingers did not move: stop).

### S4 Pull out
- `pull_out([0, 0, -1], 20 N cap, 30 mm, 2 mm/s)`: straight up only, the legs leave the paper
  and the slit. The post-it holds the legs: peak **4.06 N** at R4 (10-27); stop at **8 N**
  (lesson `post-it-holds-legs`; 3 N stopped a clean pick) = a leg is caught: hold, image.
- Then up to 2 cm above the highest casing top. Never `halt`: abort through if/else.

### S5 Check and hand-off
- `snapshot` (fixed + gripper cam) into the run dir. Pass when:
  - the slot is empty (the post-it shows the holes), the neighbours unchanged;
  - the casing is in the fingers, **legs straight down**, the casing not tilted;
  - `--confirm` records what the operator saw (until a camera check exists).
- Update `data/rack_state.json`: slot k empty, the pick in the history.
- **Hand-off** to `breadboard-place` (HANDOVER "The interface"):
  1. legs straight down, leg pair along the finger faces, casing bottom at the fingertips;
  2. the casing name (from the slot, checked by the CAD height in S2);
  3. along the legs: within ±3 mm (measured by `--calib`); across the legs: centred by the
     jaw; tilt: not measured by placing, so the image check above matters;
  4. polarity: `pin1` from `rack_state.json` (placing treats parts as symmetric today, say it
     in the note);
  5. `python -m breadboard.place_probe <h1> <h2> --part <casing> --calib --run --note
     "rack-pick R<k>, run <id>, pin1 <dir>"`.

### S6 Learn
Every miss gets its exact cause in the run's `result.json` (`miss_reason`), in
`data/rack_state.json` history, and a lesson in `skills_lib/breadboard/` (`applies_to:
pick_step`, status proposed → validated only by a passing re-run). Never delete lessons;
supersede them. One new variable per run (same slot first, then other slots, other casings).

## Hard rules (from `skills_lib/breadboard` and HANDOVER)

- Straight z only while legs are in the paper / slit; never x/y force mode.
- Touch 2 N (1 N on a casing top), side push 1.5 N, pull cap 20 N; guarded moves first.
- Low approach: 2 cm above the highest casing top, then sideways.
- Never `halt` in a step (it skips the TCP reset).
- A bridge IK that times out means unreachable; x < −0.46 m is the elbow singularity; wrist 3
  within ±0.8 rad. Plan mode checks IK for every point.
- Gripper only through the MCP smart skills Gripper Open / Close, Hand-E activated first,
  parameters never changed; check `psx_logs` after every gripper command.
- Before the first real run of any new program: show the plan (moves, speeds, force caps,
  abort branches) to the operator and get a "go". Protective stops are cleared by the
  operator. If something is refused: `bridge_my_status`, then ask, do not retry.
- No alignment hints during a run: the agent finds the rack and the casing itself.

## Simulator

`python -m simulation` (UR3e + AI Agent Bridge on :8095), then in the shell that runs the
script `export UR_BRIDGE_URL=http://127.0.0.1:8095/universal-robots/ai-agent-bridge/mcp/v1/`.
Cameras, F/T contact and the gripper are not simulated, so touches never stop: use it for plan,
IK and motion checks only. A simulated pass is not a robot pass.

## Report

Slot → casing (name, pin 1), result, `z_top − z_surface` vs the CAD, pull peak, the rack pose
source (this run or `rack.yaml`), operator actions (count), run dir, the hand-off command for
placing, and any miss with its reason.
