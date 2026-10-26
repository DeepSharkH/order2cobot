---
name: protoboard-wire
description: Move a wire (a jumper or part in its printed casing) on the protoboard/breadboard from the holes it sits in to target holes given as pins, with the UR3e + Hand-E (AI Agent Bridge). Pin positions come straight from the camera images and point clouds (no touch references); then grasp in place, pull out, spiral search, insert straight down in z, re-image to verify. Logs every step for a session trace. Use when asked to change/move/re-plug a wire or casing on the protoboard, or to retry that move.
---

# Protoboard wire — skill

Goal of a run: **the casing's pins moved from their current holes to the target holes, seated
flush, confirmed by re-imaging**, with **no alignment hints** and **as few operator actions as
possible**. Count every operator action; the target is 0 per run (one-time setup does not count).

Read first: `porttest/LEARNINGS.md` (section "Protoboard wire", create it after the first run),
your memories `protoboard-detection-status`, `ur-bridge-motion`, `ur3-cable-limits`, and the
casing models in `assets/models/component_casings/` (`casings.json`: pins, exit spacing,
size per casing; frame: x along the leads, exits at z = 0).

## What is on the board (seen 2026-10-26, scene `data/scenes/20261026_122658_wire_example`)

- The full board is two half breadboards end to end (`configs/objects.yaml` `protoboard`,
  166 x 56 x 9.5 mm), printed number labels along it, letters A–E | F–J across, red/blue
  rail lines on both long edges.
- The "wire" is the **original jumper casing** (`Jumper casing - 5 mm pitch`): white block
  14 x 5 x 40 mm, wire loop out of the top, 2 pins at x = ±2.5 mm (5.0 mm ≈ 2 pitches, 0.08 mm
  off the grid), Ø0.8 exits. Newer casings are 20 mm tall (`casings.json`). The height above
  the board tells the casing type apart.
- It sits on the left half near the high-number end, next to the A-side rail. Which holes it
  occupies has **not** been determined yet (that is S3's job).
- The fixed D435 sees the board obliquely and the far end and right edge leave its frame. On
  that scene the box image fit moved the board 27 mm off (wrong) and the gripper cam disagreed
  by 7.7 mm: **fixed-cam absolute coordinates are not good enough for a 2.54 mm grid.**
  (After the 2026-10-26 14:24 recalibration: 2.7 mm / 0.7° between the cameras; still more
  than one pitch.)

**The board and what stands on it change between runs: re-detect, never reuse.** On
2026-10-26 18:20 (run `20261026-174057`) the board had been swapped for a **one-piece 65-number
board** (MB-102 layout: rows a–j, channel 7.62 mm, no 9.35 mm joint, labels H1:<row><1–65>),
untaped, long axis along base x at base (-443, -42) mm. A second casing (20 mm LED, yellow
dome) and red/black wires appeared during the run, and the checkerboard (world origin) was
moved, which made every fixed-cam → base conversion wrong by ~110 mm (dashboard "FK vs board
check 109.7 mm"). Tell the board type from the detections (S3.3), not from this file.

## The hole grid (from the CAD; `agents/vision/board_grid.py`)

The board's holes are **not** a generic lattice: they come from the half-breadboard STEP
(`configs/objects.yaml` `protoboard.model`), two copies end to end. Pins can only ever be at
these 800 points, so every position in this skill is one of them.

- 600 terminal holes (per half: 30 numbers x rows A–E | F–J) and 200 rail holes (per half:
  4 rows of 25, in groups of 5). Pitch 2.54 mm. Nets: the 5 holes A–E (or F–J) of one number;
  one rail row of one half.
- Gaps that are **not** whole pitches: centre channel E–F 7.80 mm; rail row to A/J 7.43 mm;
  rails sit half a pitch (1.27 mm) off the number columns; the two halves' number-30 and
  number-1 columns are 9.35 mm apart (no casing straddles the joint).
- A hole is a funnel: 1.7 mm square at the board top, 0.7 mm square from 0.5 mm down, floor
  7.0 mm below the top. Board top = box base + 9.5 mm.
- The pattern is symmetric under a half turn: hole **positions** can be fitted from images,
  hole **labels** need a printed number (and the red line for `+`/`-`).
- `BoardGrid` gives: labels and nets (`grid["H1:A12"]`), `place(origin, u, normal)` (3D points
  in any frame), `snap(uv)` (nearest hole + distance), `fit(uv_seen)` (rigid correction of
  the grid onto detected hole centres, whole-pitch shifts and the half turn tried; reports
  rms, matched share and the runner-up), `placements(exit_x, occupied=...)` (every allowed
  pin → hole assignment of a casing; see S6).
- `uv run scripts/board_grid.py <located.json> --scene <scene> [--casing <name>]
  [--occupied ...] [--flip-u] [--flip-v]` writes `holes.json`, `placements.json` and a small
  x per hole into every camera frame (`cam<i>_holes_zoom.png`: check that the x's sit on the
  holes before trusting a grid).
- **One-piece boards:** `BoardGrid.from_holes_json("dashboard/web/models/cad/breadboard.json",
  numbers=65)` re-lays the full-size CAD's rows a–j to N numbers (terminals only; rails left
  out, their grouping differs per board). `fit(..., turns=(0.0,))` keeps the grid's labels
  once the orientation is decided (otherwise the 0.67 mm turned twin is the runner-up).
  How to tell: two half boards show a 9.35 mm gap between numbers 30 and 1 in a detected row;
  a one-piece board has none. Count the columns between both board ends (`pin_map` reports
  detections beyond the grid ends: if any, raise `--numbers`).

## Simulator (same commands, no robot)

To rehearse or debug without the arm: `python -m simulation` (simulated UR3e + AI Agent Bridge on
:8095), then `export UR_BRIDGE_URL=http://127.0.0.1:8095/universal-robots/ai-agent-bridge/mcp/v1/`
in the shell that runs this skill's Python. URExec, SkillB and move_checked then drive the
simulator unchanged; watch it in the dashboard's **Simulation** tab. Not simulated: the cameras
(frames still come from the real dashboard), RTDE telemetry, the link check. Say in the run log
that a run was simulated; a simulated pass is not a robot pass. See `simulation/README.md`.

## Principles (why this skill looks like this)

1. **Vision gives the pins, touch only enters.** There are no touch references. Hole and pin
   positions come from images + point cloud. Contact is used only in the last ≤ 1 mm (spiral
   search at the target), where the hole itself tells the pin it has arrived.
2. **Work in hole labels, not absolute numbers.** The holes are the CAD grid above. The
   current pins and the targets are hole labels (`H1:A12`, `H2:+L7` for rails). They become
   base coordinates through the CAD grid fitted **in the same image**. A detection that does
   not snap onto a grid hole is not a pin, and a pose that is not one of the casing's
   `placements` is not a target: the robot only ever moves pins to grid holes.
3. **The gripper cam measures, the fixed cam oversees.** The gripper cam close-up goes to base
   through FK + hand-eye, with no world frame in between. Use it for everything the robot moves
   to. Use the fixed cam only for the overview, labels and the before/after change check (it
   is off by up to ~12 mm in world, memory `protoboard-detection-status`).
4. **Measure the tool from the board.** The grip changes every grasp. Grasp the casing while it
   is still seated: its pins are then at holes you just measured, so flange → pins is
   measured, not assumed.
5. **A seated pin must be proven, not assumed.** Advance alone is not a hole (a pin can bend or
   slip off the edge). It counts once the wiggle test is constrained, the casing bottom is
   flush, and the re-imaged pin map shows it at the target labels.
6. **LLM decides, code executes.** One URScript program per stage (one bridge step). You set
   the parameters and read the `textmsg` results.
7. **Stop on surprises.** Protective stop, bent pin, casing left behind, two views that
   disagree on a label: stop, log it, re-measure. Do not retry blindly.
8. **Pins go in straight along z.** Once the pins are over their holes, every insertion and
   seating move is a position-controlled straight move along base −z with a force cap
   (`seat_z`). Never let x/y float in force mode with the pins in the board: on 2026-10-26 an
   x/y/z-compliant `seat_compliant` (8 N, no fresh F/T zero) dragged the arm into
   `force_mode_singularity` → FAULT, POWER_OFF, elbow straight, pins pulled out. In-plane
   motion is allowed only in the spiral search, before the pins drop.
9. **Know where the pins are before going down.** After any change of the grip (operator
   re-grip, slip, fault recovery) the measured tool is void: locate the pins again (gripper-cam
   image against the grasp-time image, or a new grasp in place) before a descent towards the
   board. A force guard alone is not a safeguard for where the pins land.

## Logging protocol (mandatory)

Same as `rj45-insert`. Tool calls and user messages are logged by the hook into
`data/runs/current/tools.jsonl`; you add the meaning:

```
python -m porttest.runlog start wire-<from>-<to> --goal "move jumper H1:A20-A22 -> H1:C30-C32, seated"
python -m porttest.runlog step S<k> "<title>" --intent "<what you will do and why>"
python -m porttest.runlog note assumption "<what you rely on>" --status unverified|verified|false
python -m porttest.runlog note observed "<number/result that came back>"
python -m porttest.runlog note interpretation "<how you read it / decision taken>"
python -m porttest.runlog note operator "<anything the human did or said>"
python -m porttest.runlog done --status success|partial|failure|aborted --summary "<one line>"
...
python -m porttest.runlog end --status ... --summary "..."
python -m porttest.runlog trace
```

The log is **append-only**: never edit or delete lines in `data/runs/`. To mark something as
not yours, add `note observed "EXTERNAL: ..."`. Open a step before acting and close it with
`done`. Log **every assumption** (lattice index, casing type, "board did not move", "grip
unchanged", "pin straight") and flip it to `verified`/`false` once you know. Numbers go in
`observed`; log every operator action.

## Conventions

- **Hole labels** (as `BoardGrid` names them): `H<half>:<letter><number>` for the terminal
  strips (`A`–`E`, `F`–`J`; numbers 1–30 printed on each half; one number = two 5-hole
  strips, A–E and F–J, each connected), and `H<half>:+L<k>`, `-L<k>`, `+R<k>`, `-R<k>` for
  rail holes (L = the A side, k = 1–25 from the number-1 end). H1 is the half whose numbers
  continue into H2. A request without a half (`A20`) is ambiguous: ask once, or take the half
  the casing is on. Numbers are the printed ones read from the image, never assumed: the
  orientation (`flip_u`, `flip_v`, `rail_plus`) is an **assumption to log** until a printed
  number (and the red rail line) in a gripper close-up confirmed it.
- **Board frame:** `u` = along the board towards higher numbers, `v` = across from A to J,
  `n` = board normal up (from the cloud plane fit). Insertion direction = `−n` (≈ base −z).
  In-plane search axes `a = u`, `b = v`. (`BoardGrid` uv is the box's frame; with the
  orientation fixed, its u/v are these up to sign.)
- **Pins TCP:** midpoint of the casing's two exits at the casing bottom (z = 0 of
  `casings.json`), x along pin 1 → pin 2, z along the casing up. It is stored in
  `configs/grip_wire.yaml` (`tcp_pins`, 6D, flange frame). `configs/grip.yaml` stays the RJ45
  plug model: do not overwrite it.
- Bridge-level poses (`move_checked`, IK) are **flange** poses in base. Inside a URScript step
  the preamble calls `set_tcp(tcp_pins)`, so `get_actual_tcp_pose()` and `movel` targets are
  **pins** poses. Never mix the two in one expression.
- **`set_tcp` persists after the program ends**, so every step script ends with
  `set_tcp(p[0, 0, 0, 0, 0, 0])`. A step that faults never reaches that line: after a fault
  the controller still has the pins TCP active (`robot_get_pose` then reports pins poses;
  `URExec._in_active_tcp` detects it). Check that the loaded program is ours right before each
  play; others use the same robot. Motion app: `protoboard_wire2` (app 28; app 27
  `protoboard_wire` holds a stray Robotiq node and is refused with HTTP 412).
- `force_mode` selection vectors take only 0/1: pass `n = [0, 0, -1]` exactly (the board's
  0.5° tilt in n gave 0.009 and `invalid_argument_value`), and flatten a/b into the xy plane.
- Python on this checkout: run bridge scripts with `/home/lab/order2cobot/.venv/bin/python`
  (`uv run` here lacks `requests`); vision scripts with `uv run`.
- Write YAML numbers in fixed point (`0.00001`, not `1e-05`: YAML 1.1 reads that as a string
  and `URExec` fails on `round`).
- Units: metres / N / m/s in URScript; log in mm.

## Steps

### S0 Start
**If the user asks to record** ("rec", "record", "video"): first run
`python -m porttest.record start <run name>`, show the sync cue ("clap once in view of the
fixed camera and the phone") and wait ~5 s before any motion. After S10 run
`python -m porttest.record stop` and report the folder path.
`runlog start`. Start telemetry in the background: `python -m porttest.telemetry &`, and keep
the PID. Phases: 1 approach, 2 contact, 3 search, 4 engage, 5 wiggle, 6 seat, 7 back-off,
8 extract, 9 touch/grasp.

### S1 Preflight (read-only)
- `bridge_my_status`: tier `move`, Remote Control. If not, ask the operator once (log it).
- `robot_get_state`: NORMAL safety mode, no protective stop, joints inside the limits of
  memory `ur3-cable-limits`.
- **Gripper empty.** If it still holds the RJ45 plug, stop and ask the operator to unload it
  (operator action, log it). Hand-E reachable, open.
- Dashboard up with both cameras (`GET /api/frame/0` and `/api/frame/1` = 200). Gripper-cam
  pose check (the dashboard's FK-vs-board check) < 2 mm, else the hand-eye must be re-verified
  first (principle 3 depends on it).
- **Target known:** the target holes from the request (see S6). If the user gave none, ask
  once; that is the only question of the run.
- Pass: all true. Else stop and report.

### S2 Code (no robot motion): once only, skip if already done
1. Own bridge client name and program in `URExec` (e.g. `protoboard-wire` / `protoboard_wire`).
2. **`configs/grip_wire.yaml`**: `tcp_pins`, `casing` (name from `casings.json` or
   `jumper_orig_5mm`), `grasp_z` (finger centre above the casing bottom), `finger` box
   (width, thickness, length below the flange), `collision_points` (fingertips, casing
   bottom corners, wire loop top), `stamp`, `source`. `move_checked` reads its clearance
   points from it while a casing is held.
3. **Pin map tool** `agents/vision/pins.py` + CLI `scripts/pin_map.py <scene> [--targets H1:A20,H1:A22]`
   (S3). Build it on `agents/vision/board_grid.py` (the grid, `fit`, `snap`, `placements`;
   do not fit a free lattice) and reuse `scripts/wire_from_cloud.py` (`fit_plane`,
   `plane_view`, `board_rails`, `objects_on_board`) and `agents/vision/scene.py`. What it adds
   is the hole detector (dark square blobs in the rectified close-up) and the casing finder.
   Output `pinmap.json` with: board plane, grid fit (correction, rms, matched share,
   runner-up, turned, orientation and the anchors that fixed it), every grid hole label →
   base xyz + free/occupied/unseen, each casing found (type, footprint, height, pins →
   labels + snap residuals), plus an overlay image per camera with the grid and labels drawn
   in (as `scripts/board_grid.py` draws them).
4. **Gripper: through the bridge (MCP), not from URScript** (state 2026-10-26):
   - What works: load the operator's app `close_gripper` (app 4: its application has the
     Hand-E set up) and run its smart skills: `smartskill_run(type=
     "robotiq-gripper-open-smart-skill")` / `"robotiq-gripper-close-smart-skill"`, then poll
     `smartskill_status` until `active` is null and read `psx_logs` for warnings. Fixed at
     100 % speed and force and full stroke (open = 50 mm); no parameters. The next URExec
     step reloads the motion app by itself.
   - What does not: the :63352 socket from URScript times out; the URCap's `rq_*` functions
     exist only in the preamble of an app whose application has the gripper (none of ours);
     a `robotiq-gripper-move-program` node in our own app fails "Gripper with ID 9 not found";
     the bridge cannot edit URCap node parameters after creation ("not found among owned
     nodes"); a `.urpx` copy of close_gripper is rejected by `program_import` (HTTP 400,
     reported as program_exists); `artifact_put` fails on the host (no staging dir).
     `porttest/hande_app.py` / `grip_app_import.py` are those attempts, kept for reference;
     `urscript_lint` already whitelists the `rq_*` names (slave id 9).
   - If the skills return `xmlrpc ... Invalid request` or "Gripper with ID 9 not found" and
     the fingers do not move, the URCap has lost the Hand-E: ask the operator to reconnect /
     scan / activate it on the pendant (one operator action), then retry.
   - There is no width readout: check open/closed in the fixed-cam image.
5. **`pull_out`** primitive (see "Reference URScript") and a `lim` argument for the force-mode
   limits of `spiral_search`, `engage`, `seat_compliant` and `release`. Today these limits are
   ordered x, y, z and tuned for n = +y (y = 0.004 is the compliant speed). For n = −z the z
   entry must carry that value.
6. Validate: run each new or changed primitive once **in free air** 20 mm above the plate
   (expected: no contact, "not found", program ends cleanly). Print the generated script for
   the operator before its first real run (no-sim safety gate; log it as setup).

### S3 Scene and pin map (images + point cloud; replaces touch references)
1. **Overview:** `uv run scripts/capture_scene.py --name wire_<run>` (both cameras, 5 frames).
   From the fixed cam: board plane, coarse board box (`scripts/locate_object.py`, used only as
   the region of interest, ±30 mm), the casing(s) standing on the board, and roughly which half
   and which numbers. `scripts/board_grid.py <located.json> --scene <scene>` gives the grid's
   first guess and its overlay (a few mm off: only the starting point of step 3).
2. **Close-up with the gripper cam:** `porttest/view_pose.py` / `go_view.py` to a view
   150–200 mm from the board, looking down within ~25° of the normal, with the casing, the
   target holes, one printed number label and one rail or the centre channel in the frame.
   Use two views if the target is far from the casing. Capture 5 frames per view.
3. **Grid** per view: board plane from the depth (RANSAC + SVD refit, tol 1 mm), colour
   image rectified onto it (`plane_view`, 0.1 mm/px), dark square holes detected, their
   centres put in the board frame of the step-1 guess, then `BoardGrid.fit` (the CAD grid, not
   a free lattice: its pitch is fixed, and the channel, rail groups, board ends and the
   half-board joint fix the index). The fit fixes **positions**; the **labels** come from the
   orientation: read at least one printed number in the close-up (and the red line for the
   rail sign), log it as the anchor, and flip `turned`/`flip_u`/`flip_v` to match.
4. **Casing → pins:** the casing's cloud points above the board top give its footprint
   rectangle in the board plane and its height (identifies the casing type: 40 mm original
   jumper, 20 mm `casings.json`). The casing's bottom edge in the image refines the footprint.
   Exits = footprint centre ± `exit_x` along its long side, each snapped with `grid.snap`.
   The pin pair must be one of `grid.placements(exit_x, same_net=True)` (along u or v, every
   exit on a hole): if not, the footprint or the casing type is wrong. The holes under the
   casing are hidden, so they are inferred from the grid, never seen.
5. Gates (all must hold, log every number):
   - grid fit rms < 0.15 mm, matched share > 0.8 of the detected holes, runner-up score
     ≥ 0.04 below the best (else the index is ambiguous: a view with more of the board's
     gaps in it), the x's of the fitted grid sit on the holes in the overlay;
   - the printed-number anchor agrees with the label the fit gives it;
   - casing height matches one known casing ± 1.5 mm;
   - snap residual < 0.5 mm per pin, and the pins form one of the casing's placements;
   - two gripper frames/views agree on the labels and within 0.5 mm in base;
   - the fixed cam agrees on the **labels** (not on the absolute coordinates).
   If a gate fails: another close-up from a different direction, then stop.
6. Save `data/runs/current/pinmap.json` and the overlays; log the current pins as labels
   (`observed "pins H1:A20, H1:A22, residual 0.21 / 0.18 mm"`).

**What worked on 2026-10-26 (use this before the generic text above):**
- **Two gripper views, because the Gemini gives no depth closer than ~0.31 m** (and none on a
  thin 40 mm casing below ~0.38 m): a **far view** (camera ≥ 0.34 m from the board: board top
  plane, 9.5 mm above the plate it lies on; board outline; casing heights when they return
  depth) and a **near view** (200 mm, flange down, camera 24° off vertical, fingertips ≥ 75 mm
  above the board: holes at ~3 px/mm). Far view used: the hand-moved pose raised 80–120 mm.
  `scripts/pin_map.py --far <scene> --near <scene> --centre=<x>,<y> --board
  dashboard/web/models/cad/breadboard.json --numbers 65 --terminal-band 16 --shifts-u 12
  --targets H1:C28,H1:C30 --out <run>/pinmap`. Poses for views: the planner pattern in the
  run's scratchpad (`plan_view.py`: look point, distance, flange tilt/yaw, finger clearance and
  keep-outs, bridge IK with the w3 / elbow checks).
- **Board frame from gripper depth, no world frame:** `pins.board_from_top` (outline of the top
  plane, ~1 mm / 1°) replaces the fixed-cam box when the checkerboard moved or the ends are
  taped (the box was 19 mm off along the board with taped ends).
- **Plane height from the pitch:** the far-view plane is ~1.6 mm off for the near view (stereo
  bias + hand-eye): `--dz auto` shifts it along n until the detected pitch is 2.54 mm. Without
  it the fit rms was 0.37–0.51 mm; with it **0.094 mm, 98 % matched, 0.008 mm over 5 frames**.
- `fit_grid` starts each whole-pitch hypothesis also half a pitch over (otherwise nothing is
  within the 0.9 mm match radius when the outline and the grid centre differ by ~1.3 mm).
- **Runner-up gap on a one-piece board stays < 0.04 by construction** (a one-column shift
  loses only an end column, ~1.6 %). Accept the index when both board ends are seen and all
  columns are matched; say so in the log.
- **Printed numbers are unreadable at 3–4 px/mm** (labels are blobs; their spacing is mirror
  symmetric, so it does not give the orientation). Reading them needs ~5 px/mm (≤ 120 mm),
  which the finger clearance did not allow. Ask the operator for the orientation, or log the
  labels "as fitted, unverified" if the user skips it (they did on 2026-10-26).
- **Casing footprint from hidden holes** (depth missed both casings): put the camera centre
  (not the look point) ~45 mm along u on each side of the casing at the same height and list
  the grid holes not detected in its rows (`scratchpad/footprint_view.py`). From the low-u side
  the first hidden hole is the casing's low edge, from the high-u side the last hidden hole is
  its high edge (the far side is parallax shadow, the wire loop hides more). Jumper 14 mm long
  → 5 columns m−2…m+2, pins m±1. Row: the row whose neighbour on the camera side stays
  visible. Result: jumper J49/J51 (centre 50), LED casing columns 49–52 rows B–D.
- The casing type can come from an earlier depth measurement of the same object (39.6 mm top)
  plus its look (wire loop) when depth fails now; log it as an assumption.

### S4 Grasp in place (measures the tool)
- **Finger corridor:** the Hand-E fingers close across the casing's thickness (casing y). No
  cloud point of another object inside either finger volume (finger box from `grip_wire.yaml`
  + 2 mm) down to the grasp height. Choose `grasp_z` so that the fingertips stay ≥ 3 mm above
  the board top and above every neighbour's top. If no height is free, stop and report.
- `move_checked` above the casing (fingers open to thickness + 6 mm, aligned with the casing's
  long axis), descend to `grasp_z` at 5 mm/s, `hande_close` at low force/speed.
- Read the TCP force after closing. A lateral force > 3 N means the vision centre was off and
  the fingers pushed the seated casing: log the vector and the distance it implies.
- **Tool:** `tcp_pins = inv(T_base_flange) · T_base_pins`, where T_base_pins is from S3 (pin
  midpoint on the board top, x along pin 1 → 2, z = n). Store it in `grip_wire.yaml` with stamp
  and source. Sanity check against the finger geometry: within 3 mm, else stop.

**Measured 2026-10-26:**
- **Fingertips are ~222 mm below the flange** (not 198: an open finger touched a 20 mm casing
  at flange z 260 mm). The jaw closes along **flange x**; the fingertips are 24 mm wide along
  flange y.
- **Fully open the fingers reach ±33 mm across the jaw axis**, and the only gripper commands
  are full open / full close (S2.4). So check neighbours within 33 mm of the casing along the
  jaw axis down to the grasp height. If one is there, either grasp from above its top, or
  turn the jaw 90° to close along the casing's long axis (fingers over empty holes), or ask
  the operator to clear it.
- Grasp that worked: flange down, psi 270° (w3 −3.41, 0.37 from the cable reference), guarded
  descent at 5 mm/s with a 3 N stop to fingertips ~35 mm (17 mm above the board top), "Gripper
  Close" smart skill. Force after closing 0.15 N. `tcp_pins` = [0.00001, 0.00004, 0.23937,
  2.21085, -2.22438, 0.01254] (pins midpoint 239.4 mm below the flange).
- `move_checked` counts the held casing's collision points even when the gripper is empty:
  pass `min_clear` explicitly for approach and view poses (0.02–0.07 m), and keep the
  straight descents guarded.

### S5 Extract
- `pull_out` along +n: speed 2 mm/s, force cap 15 N, until the force has dropped to the
  casing's weight and the pins are ≥ 3 mm clear of the board top (pin length unknown: allow
  12 mm). Log the peak pull force (it tells you the holding force for S7).
- Then move up to 30 mm above the highest object on the board.
- Checks: `hande_width` = casing thickness (still held, not slipped); a gripper or fixed cam
  image of the pins shows both present and straight. Measure `pin_len` (below the casing
  bottom) and log it. A pin left in the board, missing or bent → stop (principle 7).
- Measured 2026-10-26 (original jumper): **peak pull 12.2 N**, `pin_len` ≈ 7.8 mm (guarded
  touch of the pin tips on the board, S7). After 12 mm the F/T still read ~6 N down / 3 N
  sideways with the pins visibly free: drift and cable pull, not a tether. Every force step
  must zero first. The fixed cam shows the pins (gripper cam: the casing hides them).

### S6 Target (defined by the pins to insert)
- **Input:** one target hole per casing pin, as labels: `H1:A30, H1:A32` (pin order free for
  a jumper) or `pin1→H1:A30, pin2→H1:A32` (ordered for polarised parts: LED anode, BJT
  E/B/C), or relative to the current pins ("+10 numbers").
- **Allowed set:** the targets must be one of
  `grid.placements(exit_x, occupied=<holes taken in the pin map, incl. the casing's own>)`
  (`ordered=True` for polarised parts). That set is every assignment of the casing's exits
  to grid holes along u or v within 0.3 mm, with no pin on a taken hole and no two pins in one
  net. It already encodes the geometry: exits spaced `pin_spacing` pitches, the 7.80 mm
  channel (e.g. a 4-pitch casing fits D→F across it, a 2-pitch one does not), the rail
  groups, no straddling of the two halves. The original jumper (`exit_x` ±2.5 mm, 0.08 mm off
  2 pitches) has 560 placements on the empty board. Log the size of the set.
  If the request is not in the set: say why (which pin has no hole, which net is shared,
  which hole is taken) and offer the nearest placements of the set; never round a request
  onto the grid silently. **Warn and ask** before a `same_net=True` placement (it shorts
  the part).
- **Checks**, from the pin map (take a new close-up if the targets were not in view):
  - every target hole was **seen** in the fitted grid's view (≥ 5 board-surface points
    within 1 mm), and the fitted grid's x is on it in the overlay;
  - free: no foreign object point within 2.5 mm up to 12 mm above the board;
  - the finger corridor at the target is free (as in S4);
  - not the current holes.
- **Target pose** (pins TCP, base): the target holes' positions from the **fitted grid** of
  the gripper-cam view (`place` with the fitted board frame, z = board top), not from the
  box: position = their midpoint; x along target pin 1 → 2; z = n. For a symmetric jumper
  there are two yaws (180° apart): take the one whose wrist-3 stays closest to the cable
  reference (memory `ur3-cable-limits`).
- Search radius 0.8 mm: inside the hole's 1.7 mm entry funnel (± 0.85 mm) and below half a
  pitch, so the search cannot end in a neighbouring hole (the gates in S3 give < 0.5 mm).
  **In practice the error was ~1.0 mm** (0.8 mm search: no hit; 1.2 mm: hit 1.0 mm off). The
  grasp and target positions come from the same image, so a common offset cancels, but the
  grid's yaw/scale error from the hand-eye (~0.7°) times the grasp-to-target distance (60 mm)
  does not. Use rmax 1.2 mm (still < half a pitch) and keep that distance short, or re-fit the
  grid in a near view centred on the target.

### S7 Insert (one URScript program per stage, results via textmsg)
Values below are from the 2026-10-26 run (one success). Log every force and keep tuning.
1. `move_checked` to the target pose with the **tool vertical** (flange z = −base z) and the
   casing bottom ~25 mm above the board (pin tips ~17 mm). If the arm was moved by hand, first
   check the elbow: from a straight elbow, IK may come back in the elbow-down branch (it did:
   elbow −0.57 rad, elbow joint 239 mm up, still clear); it works, but do not flip back and
   forth.
2. **Touch:** `guarded_axial([0, 0, -1], ≤ 25 mm, 1.5 N, 2–3 mm/s)` until the pin tips meet the
   board. The stop height gives `pin_len` (7.8 mm). Back off 0.3 mm.
3. **Search** (the only in-plane motion): `contact_l([0, 0, -1], 2 N, lim=[0.005, 0.005,
   0.004, 0.1, 0.1, 0.1])` then `spiral_from(ref, n, a, b, 0.2 mm, 1.2 mm, adv 0.6 mm,
   1 mm/s)`. Candidate = a drop > 0.6 mm (both pins over holes). `end_force_mode()`
   right after the hit.
4. **Insert straight down:** `seat_z(0.0075 − drop, 15 N, 2 mm/s)`: zero the F/T, speedl along
   −z only, stop at depth or at the force cap, then `release` until the push is < 1 N.
   2026-10-26: drop 0.76 mm, then 4.7 mm more, stopped at 15.3 N (5.5 mm past the surface),
   released to 0.45 N; the operator saw the casing in. Do **not** use `engage` /
   `seat_compliant` (x/y compliant force mode, principle 8). The wiggle test (`wiggle_test_l`,
   ±0.4 mm, 1.5 N) did not prove thin leads 2 mm in (forces 0.5–1.65 N) and drifted the pose
   2.3 mm: treat it as information, not a gate, and verify by image (S8).
5. "Gripper Open" (S2.4), then `guarded_axial([0, 0, 1], 5 mm, 2 N, 2 mm/s)`. A pull means
   the casing comes along: stop. 2026-10-26: no pull. Then 30 mm up.

### S8 Verify
Re-run S3 (gripper close-up + fixed overview). **PASS** = the pin map finds the casing at the
target labels (both pins snap to the target holes of the fitted grid, residual < 0.5 mm,
and the pair is the requested placement), the old holes are free, and the casing's height
above the board equals its model height ± 0.7 mm (seated flush) with tilt < 3°. The fixed cam
must show the same change (old spot empty, casing at the new numbers). There is no electrical
check wired up yet: say so in the report.

### S9 Retry / escalate
- Max **3** insert attempts per move. Between attempts: lift 3 mm, image the pins, re-run the
  S3 close-up around the target (the board may have been pushed). After 2 false hits at the
  same spot, re-check the grid index (one pitch off is the classic error: compare the fit's
  runner-up, read another printed number). A retry goes to the re-fitted position of the
  **same** target holes; never shift the target by a free offset.
- Bent pin, dropped casing, casing left in the gripper after open, protective stop: stop
  immediately, log `observed` + `operator` (recovery). Do not resume until the cause is
  understood.
- After 3 failures: `done --status failure`, write what you would need (view, sensor, tool) as
  a `note interpretation`, and hand over to the human. "Cannot do" is a valid result.
- **After a fault or a hand move:** the operator resets on the pendant (and often switches to
  Local: ask for Remote again). Read the joints (a straight elbow is a singularity: leave it
  with a joint move, never `movel`), note that the pins TCP may still be active, image the
  scene, and treat the grip as changed (principle 9) if the casing was touched.
- **Operator watching:** a user statement like "it went into the pins" / "it is inside" is
  operator evidence; log it as `operator` and continue, but still run S8 when the cameras are
  up.

### S10 Close
Stop the telemetry recorder (`kill <PID>`) and log its CSV path and sample count as `observed`.
`runlog end`, `runlog trace`, fill `interpretation`/`summary` where empty in
`data/runs/current/session_trace.json`, and regenerate with
`python porttest/session_trace/session_trace_to_drawio.py <json> <xml>`. Update
`porttest/LEARNINGS.md` section "Protoboard wire" (lattice gates, forces, pin length, what
changed). Report the run dir path.

## Reference URScript

Reuse `porttest/urlib.py` (`contact_l`, `spiral_from`, `release`, `guarded_axial`, `pull_out`,
`seat_z`, `tel`). Downward insertion: `n = [0, 0, -1]` exactly (selection vectors are 0/1),
`a`/`b` = board u/v flattened to xy. `engage`, `seat_compliant` and `wiggle_test` exist but
float x/y in force mode: not for pins in the board (principle 8).

`seat_z` (in `urlib.LIB`, linted):

```
def seat_z(depth, fmax, v):
  # straight down along base -z only (x/y and rotation position-controlled); stop after depth
  # or when the push exceeds fmax, then back off until the push is < 1 N
  tel(fmax, 6)
  zero_ftsensor()
  sleep(0.3)
  q0 = get_actual_tcp_pose()
  s = [0, 0, -1]
  ...speedl([0, 0, -v, 0, 0, 0], 0.2, 0.008), f = get_tcp_force()[2], break if f > fmax...
  textmsg("seated_z <d> peak <f> last <f>")
  release(s, 1.0, 0.001)
  return d
end
```

`pull_out` (in `urlib.LIB`, used on 2026-10-26: 12 mm at 2 mm/s, peak 12.2 N):

```
def pull_out(n, fmax, maxd, v):
  # move along -n (out of the board) at v until maxd; stop if the pull exceeds fmax.
  # Reports the peak pull force (the pins' holding force) and the travel.
  tel(fmax, 8)
  zero_ftsensor()
  sleep(0.3)
  q0 = get_actual_tcp_pose()
  s = [-n[0], -n[1], -n[2]]
  peak = 0
  t = 0
  while along(q0, s) < maxd and t < maxd / v * 3 + 1.0:
    speedl([s[0]*v, s[1]*v, s[2]*v, 0, 0, 0], 0.2, 0.008)
    t = t + 0.008
    F = get_tcp_force()
    f = -(F[0]*s[0] + F[1]*s[1] + F[2]*s[2])
    if f > peak:
      peak = f
    end
    if f > fmax:
      stopl(0.5)
      textmsg(str_cat("pull limit ", f))
      textmsg("pose ", get_actual_tcp_pose())
      return False
    end
  end
  stopl(0.5)
  textmsg(str_cat(str_cat("pulled ", along(q0, s)), str_cat(" peak ", peak)))
  textmsg("pose ", get_actual_tcp_pose())
  return True
end
```

Notes (check on the robot, tune, log what you change):
- `zero_ftsensor()` in `pull_out` is called with the casing held and still seated, so the
  casing's weight is inside the zero. "Force back to weight" after the pins clear means the
  reading goes back to ~0.
- `seat_compliant` lets a/b float at zero force: with a stale F/T zero (~6 N after the pull)
  it drove the arm into a singularity. Pins use `seat_z` instead (principle 8).
- URScript has no `abs`, `min`, `max`, `len`, `round`, `range`, `for`, `**`; see the
  `rj45-insert` skill notes. **Run `python -m porttest.urscript_lint` after every change to
  `urlib.LIB`.**

## Done criteria for the run
PASS = S8 confirms the casing at the target labels, seated flush, **and** the trace exists.
Report: from → to labels, result, attempts, false hits, extraction peak force, seat depth,
operator actions (count), wall time, run dir, and the 3 biggest surprises.
