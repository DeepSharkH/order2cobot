# RJ45 port QA — learnings from the first robot session (2026-10-24)

Goal: UR3e + Robotiq Hand-E holding an RJ45 plug (3D-printed H shell, `~/rj45_grip`)
tests the 8 ports of a TP-Link LS108G one by one; the Jetson checks the link on
`enP7p1s0f0` (`dashboard/check_link.py`, 1000 Mb/s full duplex = pass).

## Robot control through the AI Agent Bridge

- The bridge (`https://192.168.2.12/universal-robots/ai-agent-bridge/mcp/v1/`, client
  `dashboard/ur_bridge.py`) has **no direct move/jog tool**. Motion = program playback:
  author → `program_load` → `program_run play`. Needs the `move` tier (granted by the
  operator on the pendant, per client name) **and** the pendant in Remote Control.
- A `ur-script` node accepts multi-line URScript including `def` functions, `force_mode`,
  `speedl`, `zero_ftsensor`, `get_tcp_force`, `set_tcp`. That is the practical motion API:
  `porttest/urexec.py` keeps one app (`rj45_qa`, appId 10) with one script node, rewrites
  it per step and plays it.
- `textmsg()` output shows up in `psx_logs` (`data.lines`, with `ts` in ms) — that is the
  return channel for step results (contact poses, forces, search hits). Entries of one
  step often share the same millisecond, so read new entries in batches (`ts > last`).
- Quirks:
  - `program_run play` often hangs until a ~8 s timeout although the program **did**
    start; treat the timeout as "maybe started" and confirm via the log
    (`PROGRAM_XXX_STARTED` / `STOPPED`).
  - Right after editing the node, play can return `program_not_ready` — retry after 2 s.
  - `stop` on an already stopped program returns HTTP 409.
  - Sessions get `unauthorized: Unknown or revoked credential` now and then (the dashboard
    shares the client name `order2cobot-calib`); re-`initialize()` and retry.
  - `URBridge.call(name, **args)` clashes with tools whose argument is `name`
    (`program_create`) — post the arguments dict directly (`urexec.call`).
- Cost: ~20 s per step, mostly bridge overhead. Pack whole sequences (probe scans, search
  grids, insert + extract) into one step instead of one move per step.
- The F/T sensor reads exactly 0 until `zero_ftsensor()`; after zeroing the noise is
  ~±0.2 N. A 6 N stop threshold at 5 mm/s worked well for contact probing.
- No payload is configured in the apps; the script sets `set_target_payload(1.0, [0,0,0.05])`.
- Safety config: tool speed 0.25 m/s, tool force 100 N (far above what an RJ45 needs —
  limit force in the script). Never add a Robotiq activate node: activation may open the
  fingers and drop the plug.

## Frames and directions (old setup — re-measure after any change)

- Gripper camera: tool pointing down (rotvec [π,0,0]): image right ≈ base +x,
  image up ≈ base +y, ~2.1 px/mm at flange z 0.34 m.
- Tip from the operator: twisting joint 4 (wrist1) pans the wrist camera. Here **−0.5 rad**
  tilted it toward +y and showed the switch front face with all 8 ports; +0.35 rad looked
  the wrong way.
- Tool axes when squared: tool x = base x, tool y = base −y; the plug points along
  −tool y = base +y; the Hand-E jaw closes along base x.
- Ports faced the robot (−y). The switch face was yawed ~4° (dy/dx = −0.069).

## Calibration

- `configs/calibration/result.yaml` `T_base_fcam` was wrong (ICP base alignment, 34 %
  inliers): it put the switch 0.44 m above the base and the camera looking steeply down.
  Do not use the fixed camera for absolute positions until it is recalibrated.
- The gripper camera has no hand-eye calibration yet.
- Camera accuracy is not enough for insertion anyway (RJ45 needs ~±0.3 mm in x and z).
  Use cameras for coarse location only, then touch.

## Mistakes to avoid

1. **Check the gripped part's geometry before reading contacts.** The bare plug tip
   sticks out 15.5 mm ahead of the white shell, and the shell/H legs are 0.8 mm *below*
   the plug and 24 mm wide (`rj45_grip.py`). The first probes were the shell and legs
   riding the switch's top front edge while the plug tip overhung the top — so "face y"
   was off by 15.5 mm, "left edge" by 6 mm, and the height by ~19 mm. A search grid built
   on those numbers would have driven the tip into the face. Always ask which part of the
   tool touched first.
2. **Fixed-camera parallax.** The D435 sat above the switch top looking slightly down,
   with the Hand-E between it and the ports. A plug at the top edge looked like it was at
   port height; a plug at port height looked below the switch. Single-view image
   estimates of height and x were unreliable — the operator could see the error directly.
3. Use the operator's eyes: one "N mm left/right, M mm up/down" from them is cheaper than
   a blind 2-D search.
4. **Say when an image is not good enough.** The fixed-camera frames were too dark and
   noisy to judge the plug against the port, and I kept zooming instead of telling the
   operator — they added a phone flash on their own. Whenever a frame is too dark,
   blurry, occluded or low-res (D435 runs at 640x480 over USB2) to support the decision,
   say so right away and ask for light / a camera move / a different view.

## Second setup (camera moved to the switch's front-left, switch moved)

- The switch moved ~13 cm to -x and ~7 cm to -y; yaw now ~1.8°. All of the earlier
  geometry was void — re-surveyed with the wrist camera from z 0.34 m (image right = +x,
  up = +y, ~2.04 px/mm), using the old switch position as the pixel->base reference.
- New D435 view (low, diagonal from the front-left) shows the plug side-on next to the
  ports; image-left ≈ base +y.
- Tip-on-face scans (absolute x/z targets — relative moves let z sag 0.05 mm per peck):
  face at tip y ≈ 0.169, switch left end x ≈ -0.4267, port 1 ≈ -0.400. 90 pecks along the
  face at flange z 0.2088 found no opening: the face is flush, so a wrong height gives no
  signal at all — get z right (operator / better image) before scanning x.

## Touch measurements that worked (method, not the numbers)

- Guarded move (`urlib.guarded`): `speedl` toward the target, stop at |F| > 6 N or max
  distance; report pose and force.
- Peck scan (`urlib.peck_scan`): back off 1 mm, shift by a step, push again; a peck that
  goes > 2 mm deeper than the face marks an edge (switch end, top edge) or an opening.
  Measured the face yaw and the left end to ~±0.5 mm in ~40 s for 45 pecks.
- After probing the face with the plug **tip** (at port height) the corrected model
  agreed within 1.2 mm in y.

## Where the session ended

- No port was inserted. Last state: the tip rests on the face at port 1's estimated x
  (flange x -0.400, y 0.1353, z 0.2088, tool down), with light guarded contact. The best
  image (phone flash, dashboard overlays off) suggested the plug was ~4-5 mm too low and
  1-2 mm toward port 2. The next step would have been a peck grid z -0.8..+6.8 mm,
  x ±3 mm.
- Dashboard overlays (robot skeleton, point cloud) are drawn into the camera stream;
  turn them off with the websocket message `{"type": "set_overlays", "on": false}`
  before judging images.
- Too much light is also a problem: the phone flash overexposed the white shell.

## Gripper-camera hand-eye calibration (session 3)

- Done without the AI Accelerator URCap, with the dashboard's own checkerboard solver run
  offline: `porttest/handeye_capture.py` (orbit poses), `handeye_fix_q.py`,
  `handeye_solve.py`. Result in `configs/calibration/handeye_gripper.yaml`: 13 samples,
  0.42 px with the controller DH. Camera origin in the flange frame is (-40, 115, 100) mm,
  with the optical axis within 2.3° of the tool axis.
- **Stale joints:** the dashboard polls joints over its own bridge session with the same
  client name (`order2cobot-calib`). When porttest reconnects, the dashboard's session
  drops and it keeps serving old joints; its "robot still moving" check then passes on the
  stale values. 7 of 12 samples had joints up to 1.4 rad off, which gave 160 px RMS.
  Check the invariant (flange rotation angle == camera rotation angle per pair). Fix:
  use the commanded poses, or give the dashboard its own client name.
- Square size: a sweep of the bundle adjustment over the square size gives a sharp
  minimum (robot motion is metric): **15.2 mm**, so the config's 15 mm was right. Twice the
  RMS at 14 / 16.5 mm.
- **Gemini depth is not trustworthy yet:** fresh (it moves 1:1 with the robot) but
  ~27 cm beyond the board plane predicted by the calibration, and readings at the same
  pose drifted 0.33 → 0.43 → 0.50 m over time. Check the camera depth settings (depth
  work mode, disparity offset, D2C alignment) and verify with a ruler before using depth
  for the switch.
- The UR AI Accelerator manual gives no mount offsets (only 480 g, M6 bolts, 5 Nm, and
  calibration via the URCap); the Gemini 335L series depth starts at ~0.17 m.

## First successful port (session 4, 2026-10-24 21:43)

Port 1 seated by the robot without hints: link up 1000 Mb/s full, 0 flaps.

**How it was found:**
- Ports: wrist-camera triangulation from 3 views shifted ±5 cm in x
  (`porttest/port_survey.py`), with rays agreeing to ~0.1 mm. Pitch 14.56 mm, yaw -1.3°. x was
  right; z from the dark-box centre was ~3 mm too low.
- Plug height below the flange: guarded descent onto the checkerboard, whose surface z is known
  from the calibrated camera. The plug centre line is 185.4 mm below the flange (this grip).
- Face y: a vertical touch profile at a solid spot (the power-LED area) gives the face y, the
  top edge (z ≈ 40 mm = plate + 25) and the band where the face is solid.
- Grid of pecks (x ±2 mm, z 26-30.5 mm, 20 N) inside that band: entry at plug centre z 29.0 mm,
  x as triangulated. Then push in at ≤40 N and 3 mm/s, and the link came up.

**Pitfalls in this session:**
- **F/T drift and cable spring:** the force reading drifts several N as the arm moves, and
  the taut cable adds ~1.2 N per mm of forward travel. A single zero plus a 5-6 N threshold gave
  false contacts mid-air ("jiggling without touching"). Re-zero at rest before every peck,
  use 12-20 N, count only the reaction against the motion, and give the cable slack.
- **Free travel ≠ port:** below the switch face the plug moves freely too. Only count travel
  past the face as a hit inside a z band known to be on the face.
- **Never end a program while pressing:** ending at 40 N in the port raised a protective
  stop (C153). `urlib.insert` backs off 0.5 mm after seating. Clearing a protective stop
  is the operator's job (the agent's unlock was blocked).
- **Never move to an unchecked target near the reach limit:** a movel to (-0.44, 0.07, 0.38)
  stretched the arm until the elbow was straight (-0.003 rad) and the shoulder went into a
  safety FAULT (C281, joint 1), which needs a restart on the pendant. Check every target with
  IK first, move to those joints, and reject targets with |elbow| < 0.5 rad.
- **Wrist twist winds the camera cable:** the gripper camera's cable is routed along the
  arm; turning wrist 3 by ~1.3 rad (-2.2 -> -3.5) wrapped it around the last links and the
  tug raised a protective stop (C153, joint 5). Keep wrist 3 near the insertion orientation
  (about -2.2 rad, within ~±0.8 rad); plan camera views with a free roll about the view
  axis instead of fixing image x, and reject plans that need large wrist-3 turns.
- **Check the cable, not just the plug tip, for clearance:** a low tilted camera-view pose
  (flange z 0.15, plug tip 3.6 cm above the plate) drove the cable, which leaves the shell
  rear and droops, into the table and raised a protective stop. Keep the shell rear ≥ 10 cm
  above the plate in any pose; for face views prefer a longer, shallower view (≈30 cm,
  30° down) with the flange high. Recover by moving straight up first.
- **The switch must be fixed solidly:** insertion pushes ~25 N and pulling out a seated
  RJ45 takes a similar force. A taped switch slid along with the plug (port 2 extraction
  dragged the whole switch); it looked like "the plug stayed in". After any movement of
  the switch, re-run the port survey.
- **Extraction lost the plug:** pulling back 20 mm at 2 mm/s left the plug in port 1
  (link still up); the shell's snap lips slip under pull. Use the zip-tie groove at the rear
  of the shell (`rj45_grip.py` ZIP_*) so the cable/plug is locked to the shell and the
  extraction force goes through the H legs.
- After a regrip, re-measure everything about the plug: tip reach and height changed by
  10-20 mm.
- The operator won't give alignment hints ("that's cheating"); setup help such as lighting,
  Remote mode or cable slack is fine.

## A better way to know where the switch is (ideas for the next attempt)

The core problem: port height and x were never known better than ±2-5 mm. The face is
flush, so touch only gives "face/no face" and finds an opening only when x **and** z are
both within ~0.3 mm. Cameras without calibration and at 640x480 could not close that gap.
Options, cheapest/most robust first:

1. **Measure the switch once with calipers** — port 1 centre from the left end, port
   bottom above the table, pitch, opening size — and put the numbers in
   `geometry.py`. This removes the image estimates that caused the 4-5 mm height error.
   Then touch probing (left end, top edge or table, face + yaw — all tip-based) gives the
   port centres to ~0.5 mm.
2. **Fixture the switch**: a 3D-printed bracket/corner stop screwed or taped to the plate
   so the switch always sits at the same pose. Find ports 1 and 8 once, store them, and
   only re-check the face y by touch on later runs.
3. **Fiducial on the switch**: an AprilTag/ArUco on the top or front next to the ports,
   at a measured offset from port 1. Hand-eye-calibrate the Gemini wrist camera
   (checkerboard flow already in the dashboard), then the tag pose in base gives all 8
   port poses to ~0.5-1 mm even after the switch moves.
4. **Port detection in the wrist camera**: tilt joint 4 (≈ -0.5 rad) so the camera looks
   at the face, detect the 8 dark openings, fit a row with the known pitch, and servo the
   plug to the fitted centre (needs hand-eye calibration or a reference image taken with
   the plug seated once).
5. **Compliance for the last millimetre**: `force_mode` with x/z compliant and a slow
   spiral (0.3-0.4 mm pitch, ≤3 mm radius) while pushing 5-8 N, instead of rigid pecks.
   Also yaw the plug to the face normal (TCP at the tip) before searching.
6. **Faster loop**: each bridge step costs ~20 s. Pack sequences into one script, or ask
   whether a long-running program talking to the Jetson over a URScript socket is
   acceptable, for interactive control.
7. **Camera hygiene**: D435 on USB3 (1280x720), side view at port height, steady diffuse
   light (no phone flash on the white shell), overlays off; say early when a frame is
   not good enough.

Recommendation for the PoC: 1 + 2 (calipers + fixture, then touch-calibrate once) are
enough to test all 8 ports quickly. Add 3 if the switch has to be placed freely.

## Insertion plan (not run yet)

- TCP at the plug tip centre: tool (0, −0.0342, 0.156) (STL notch centre 34.2 mm behind
  the tip, Hand-E length ~0.15 m — both to verify). Rotate about the tip, otherwise the 4°
  yaw correction swings the tip ~2.4 mm sideways.
- Align the plug yaw with the face normal; plug pins face down, so the plug body sits on
  the cavity floor (latch slot unused at the top).
- Find port 1 with a centre-out peck grid (`urlib.grid_search`, x ±2.5 mm / 0.5 mm,
  z ±2 mm / 0.4 mm); "found" = > 3 mm advance. Then push to ~15 mm depth with a force
  limit, check the link, pull straight back, check that it drops.
- Ports 2–8 from the face direction and pitch (~14.85 mm image estimate); refine the pitch
  after two ports have been found; small search per port.
- Report mechanical failures (did not seat) separately from electrical ones (seated, but
  no link or 100 Mb/s).

## Setup recommendations for the next attempt

- Fixed camera 25–35 cm from the ports at port height, 30–45° to the side, so the Hand-E
  does not hide the opening and the plug tip is visible at the port.
- ≥ 3 cm free on both sides of the switch (the H legs are 24 mm wide; the switch end is a
  touch reference). Tape the switch down.
- After moving the switch, re-probe the top edge, left end and face yaw — every number in
  `porttest/geometry.py` belongs to the old setup.

## Board-frame calibration (2026-10-25)

The dashboard dropped hand-eye, sample capture, DH refinement and the depth-fit base
alignment. World = checkerboard centre, z up; base = (x, y, yaw) on z = 0; gripper cam =
base · FK · mount; fixed cam = board PnP. Stored in `configs/calibration/scene.yaml`.
- Fixed D435 located from the board: 0.80 px over 8 frames. Its depth over the board fits
  z = +1.7 mm with 0.3–0.8° tilt, so D435 depth and colour agree.
- Gripper cam: a one-view mount ("Mount from board view") soaks up any base error, so it
  only holds at the pose where it was taken. After the arm moved, it was 75 mm off.
- **The Gemini depth is fine.** Probed alone, through the dashboard's own camera class,
  and with the D435 emitter on, depth / board-plane = 0.99 with every board pixel valid.
  The 1.44 "depth error" (and probably session 3's "27 cm beyond the board") came from
  a wrong camera *pose*, not from bad depth.
- **Recalibrated:** `python -m dashboard.scene_solve data/calib/20261025-111444 --save`
  solves the mount + base (x, y, yaw on z = 0) from 17 board views by reprojection,
  starting from the photo guess (side bracket on +y, ~100 mm down, ~23° tilt). Result:
  1.78 px; mount t = (-40, 95, 109) mm with the optical axis **23.8°** off the tool
  axis; base (26, 374) mm, yaw 90.5°. Holdout at a new pose: 6.7 mm / 1.0° vs the live
  board view, and gripper depth on the board at z +6 mm, tilt 1.0°. The fixed cam was
  re-located at 0.73 px; its depth on the board is z +1.6 mm, tilt 0.3°.
- The sample views all sit at a similar height. Views from higher, lower and more
  tilted poses should bring the gripper holdout under the current 6 mm.
- `porttest/locate.py` still reads `handeye_gripper.yaml` (the old mount), and
  `handeye_capture.py` sent the dashboard's removed `capture` message.

## Port survey with the remounted camera (2026-10-25 afternoon)

- **Use full-resolution frames.** The dashboard's websocket stream is downscaled to 960x540
  (`stream_width`), for display only. For judging an image, measuring pixels or detecting
  ports, fetch the full 1280x720 colour frame: `GET http://localhost:8090/api/frame/1`
  (gripper) or `/0` (fixed), which is what `port_survey.capture_png()` uses now.
- The dashboard's `capture` action is gone; `capture_png()` uses the frame endpoint.
- The plug cable hung across the port row from the survey pose (camera on the -y side,
  32 cm, 30° down). The operator routed it back over the gripper; check for it first.
- At 32 cm and 30° down, straight in front, the openings are ~30 px apart and the bezel
  between them is nearly as dark as the openings: `ports.detect()` merged them or took the
  black tape above the switch. The views shifted ±5 cm along x detected cleanly. A fitted
  "comb" (evenly spaced columns at the pitch) was tried and dropped: it latched onto the
  face edge and stretched the pitch. Now `detect_rotated()` returns nothing unless it finds 8
  evenly spaced boxes, and `survey2` uses views at ±3 / ±5 cm and needs ≥ 2 good ones.
- A wrong detection in one view wrecks the triangulation (10 mm ray rms, 16 mm pitch, 10°
  yaw). Check ray rms (~0.1 mm when right), pitch (~14.5 mm) and yaw before using a survey.
- Wrist-3 cable reference: the cable has to be relaxed at the insertion orientation's
  wrist 3. After the remount it was relaxed at -4.97. It was re-routed to -2.0 (for the old
  grip, tool at (0, π, 0)), then to **-3.78** after the regrip (tool turned +90°, below).
  `go_view.W3_REF` holds it (-3.78), and `move_checked` keeps every move within ±0.8 rad of
  it. Ask the operator again whenever the cable or the grip changes.
- **Regrip (operator): the plug now points along flange -x**, so the gripper cam sees the
  insertion from the side. Measured on the gripper-cam frame: tip 41.7 mm out along -x and
  ~5 mm (±4) to flange -y; the axis is straight -x. Insertion therefore needs the tool turned
  +90° about the vertical (rotvec (-2.2214, 2.2214, 0)), and IK puts wrist 3 at ≈ -3.78 there.
  `qa_run.P1`, `plan_views.TIP` and `geometry.py` describe the old grip.
- What happened with the old grip assumption: the "found" peck and the 12 mm force-mode
  push were free travel. The plug pointed along base +x, slid sideways past the switch's
  left end, and the link stayed down. Free travel is not a port; confirm with the link.
- The checkerboard was removed and a breadboard (~10 mm high) put in its place. The board
  touch landed on the breadboard (top ≈ 15 mm in base by D435 depth): lowest gripped point
  ≈ 197–199 mm under the flange, plug centre line ≈ 193 mm.
- Port-1 attempt with the regripped orientation: the first good-looking peck (x +1 mm, z -2 mm)
  travelled > 13 mm, stopping only ~3 mm past the surveyed face. Force mode then raised C157
  (protective stop) with x/z held within 1 mm. The operator recovered it. No link. Either a
  partial entry at the wrong height, or the face/tip-reach estimates were off by mm.
- Approach picture set (no contact), `porttest/approach_shots.py`, saved in
  `/home/lab/order2cobot/.claude/worktrees/rj45-port-qa/data/approach/approach_20261025/`
  (overview: `contact_sheet.jpg` there; `data/` is not in git). Per shot: `<shot>_cam0.png`
  (fixed D435) and `<shot>_cam1.png` (gripper Gemini) at 1280x720, `<shot>_cam{0,1}.npy`
  aligned depth (float32 m, `GET /api/depth/{cam}`), and `<shot>.json`: flange pose, joints,
  both camera poses (world + base), K/dist, `T_base_world`, grip model. Also `ports.yaml`
  (the survey used), `01_survey_view_cam1.png` and the `survey2_*.png` detection crops.
  Shots: 00 parked, 01 survey view, 02 high, 03–06 tip 30/15/8/5 mm in front of the face at
  port 1, 07/08 ±3 mm in x at 8 mm.
- What the set shows: the gripper cam (now looking along the face from the switch's left end)
  shows the tip-to-face gap and the plug height but not the openings. The fixed cam loses
  the port row behind the Hand-E from 30 mm in. Neither resolves the x alignment, and the
  ±3 mm shifts are barely visible in either camera.

## Where the port work stopped (2026-10-25, end of afternoon)

- The arm is parked above and in front of port 1 in the insertion orientation (flange
  (-0.3299, 0.0472, 0.3143), rotvec (-2.2214, 2.2214, 0), wrist 3 ≈ -3.78). The plug is still
  gripped; the switch is unmoved. No port has been seated with the new grip.
- Trusted: the scene calibration (`configs/calibration/scene.yaml`: base (25.2, 374.5) mm,
  yaw 90.57°, 9.1 mm under the board plane; gripper mount from 17 views at 0.33 px; fixed
  cam 0.73 px) and the port survey (`data/ports.yaml`: rays agree to 0.1–0.4 mm, pitch
  14.51 mm, yaw 0.0°, port 1 centre (-324.6, 118.9, 21.3) mm in base).
- Not trusted to < 3 mm: the face y relative to the plug tip (tip reach), the plug's
  sideways offset (±4 mm, camera depth ambiguity) and the plug height (breadboard touch +
  depth, ±1.5 mm). These are what break the peck search.
- Open questions for the offline session:
  1. Measure the tip-to-face gap in the gripper-cam side view (shots 03–06) to calibrate the
     face y and the tip reach before any contact.
  2. Touch references instead of absolute camera numbers: face at a solid spot (y), switch
     top edge (z), left end (x). Then place the ports with the survey's relative positions.
  3. A compliant search (force mode with x/z free, small spiral, 5–8 N) instead of rigid
     pecks plus a stiff seat (C157 came from x/z held within 1 mm).
  4. Move the fixed cam to port height, 30–45° to the side, so the opening stays visible
     while the plug approaches.
