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
