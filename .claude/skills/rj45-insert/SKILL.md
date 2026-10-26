---
name: rj45-insert
description: Seat the gripped RJ45 plug into a port of the TP-Link LS108G switch on the UR3e (AI Agent Bridge), the way a human plugs in USB-C - camera to get close, then slide in light contact until it drops in, then seat compliantly and check the link. Logs every step for a session trace. Use when asked to insert/test a switch port or to retry the RJ45 insertion.
---

# RJ45 insert — skill

Goal of a run: **port N (default 1) seated by the robot, link up at 1000 Mb/s full duplex**,
with **no alignment hints** and **as few operator actions as possible**. Count every operator
action; the target is 0 per run (one-time setup does not count).

Read first: `porttest/LEARNINGS.md` ("Current state" + "Where the port work stopped") and your
memories `rj45-port-qa-status`, `ur3-cable-limits`, `ur-bridge-motion`. Targets come from the
**`cad-guidance`** skill (`python -m cadguide ensure ls108g --compact`): first port = strong
guess + full search, then anchor it; the other ports are offsets of the anchored CAD model.

## Simulator (same commands, no robot)

To rehearse or debug without the arm: `python -m simulation` (simulated UR3e + AI Agent Bridge on
:8095), then `export UR_BRIDGE_URL=http://127.0.0.1:8095/universal-robots/ai-agent-bridge/mcp/v1/`
in the shell that runs this skill's Python. URExec, SkillB and move_checked then drive the
simulator unchanged; watch it in the dashboard's **Simulation** tab. Not simulated: the cameras
(frames still come from the real dashboard), RTDE telemetry, the link check. Say in the run log
that a run was simulated; a simulated pass is not a robot pass. See `simulation/README.md`.

## Principles (why this skill looks like this)

1. **Vision gets close, touch gets in.** Cameras bring the plug tip within ~2 mm. The last
   millimetres are found by staying in light contact with the face and sliding until the
   tip advances — never by pecking (a peck throws away the contact information).
2. **A hit must be proven, not assumed.** Travel alone is not a port: the plug also slides
   freely past the switch end or below the face (that is how attempt 1 failed on 10-25).
   A candidate is a port only if the **lateral wiggle test** shows the tip is constrained,
   and it is a pass only when **the link comes up**.
3. **Measure the tool before trusting it.** The grip changes (regrip on 10-25 flipped the
   plug direction). Grip comes from the shell tag every run, never from old constants.
4. **Touch references cancel tool errors.** Face / top edge / left end are touched with the
   plug tip itself, so the tip-offset error is inside the measurement.
5. **LLM decides, code executes.** Each search is one URScript program (one bridge step);
   you set parameters and read `textmsg` results. No per-move bridge round trips.
6. **Stop on surprises.** Any protective stop, false hit or disagreement between two
   measurements > tolerance: stop, log it, re-measure. Do not retry blindly.

## Fast path: Option B (use this for routine ports)

Every program edit through the bridge costs ~9 s (`program_update_node`, measured 2026-10-26)
plus ~5 s per new bridge session. **Do not upload a new script per step.** Instead:

- `porttest/skill_b.py`: the skill program (urlib + socket dispatcher) is uploaded **once per
  session** (`SkillB().upload()`); every step is **one play** (`with sk.session() as s:`) that
  runs any sequence of primitives (`s.creep`, `s.contact`, `s.slide`, `s.search_port`,
  `s.engage`, `s.wiggle`, `s.seat_x`, `s.guarded_axial`, `s.movel`, `s.movej`, `s.ik`, …)
  sent from Python over a socket (round trip 2 ms), results returned as (value, tip pose).
  The per-step play still goes through the bridge (move tier + Remote gate).
- `porttest/rj45_b.py port N`: one full port (touch refs → insert → link → extract → lift) as
  3 plays with run log + telemetry. **Measured: port 1 in 123 s** (vs 283 s per-step upload).
- Use one long-lived Python process for a whole run (a new process = new bridge session ~5 s).
- If the robot does not connect within 20 s: the program was replaced (someone else edited or
  loaded another app) → `SkillB().upload()` again. Bump `VERSION` when urlib or the dispatcher
  changes. Use the old per-step `urexec.run` only for new, not-yet-supported primitives.

## Logging protocol (mandatory)

Tool calls and user messages are logged automatically (hook → `data/runs/current/tools.jsonl`).
You add the meaning:

```
python -m porttest.runlog start rj45-port1 --goal "seat port 1, link 1000/full"
python -m porttest.runlog step S<k> "<title>" --intent "<what you will do and why>"
python -m porttest.runlog note assumption "<what you rely on>" --status unverified|verified|false
python -m porttest.runlog note observed "<number/result that came back>"
python -m porttest.runlog note interpretation "<how you read it / decision taken>"
python -m porttest.runlog note operator "<anything the human did or said>"
python -m porttest.runlog done --status success|partial|failure|aborted --summary "<one line>"
...
python -m porttest.runlog end --status ... --summary "..."
python -m porttest.runlog trace     # session_trace.json + .drawio.xml in the run dir
```

Rules: the log is **append-only** — never edit or delete lines in `data/runs/` (deleting the
"other team" notes on 10-25 also deleted our own S2 close and the shared-robot evidence). To
mark something as not ours, add `note observed "EXTERNAL: ..."`. Open a step before acting,
close it with `done`. Log **every assumption you rely on**
(geometry, frames, sensor meaning, "switch did not move", "grip unchanged") and flip it to
`verified`/`false` when you learn more. Log every operator action. Numbers go in `observed`.

## Conventions

- Bridge-level poses (`move_checked`, IK) are **flange** poses in base. Inside a URScript step
  the preamble calls `set_tcp(<tip from grip.yaml>)`, so `get_actual_tcp_pose()` and `movel`
  targets are **plug-tip** poses. Never mix them in one expression.
- **`set_tcp` persists after the program ends** (seen 2026-10-25): the bridge's
  `robot_get_pose`/IK then use the tip TCP, and so would **anyone else's program** on the
  shared controller. Every step script must end with `set_tcp(p[0, 0, 0, 0, 0, 0])` (restore
  the flange TCP) — leave the controller as you found it. Also check the loaded program is
  `rj45_qa` right before each play (others use the same robot).
- Face normal into the switch = base **+y** (ports face −y, yaw ≈ 0°). In-plane axes:
  a = base +x (along the port row), b = base +z.
- Units: metres / N / m/s in URScript; log in mm.

## Steps

### S0 Start
**If the user asks to record** ("rec", "record", "video"): first of all run
`python -m porttest.record start <run name>` (both cameras → `data/recordings/<ts>_<name>/`;
the laptop's `tools/rec_watch.py` starts the screen capture from it), show its sync cue to
the user ("clap once in view of the fixed camera and the phone"), and wait ~5 s before any
motion. At the very end (after S10) run `python -m porttest.record stop` — it copies this
run's logs and telemetry into the recording folder. Report the folder path.
`runlog start`. Note the operator actions so far this session (none should be needed).
Start the telemetry recorder in the background (read-only RTDE, 125 Hz, writes
`data/runs/current/telemetry_HHMMSS.csv`): `python -m porttest.telemetry &` — keep its PID.
It records TCP actual/target, measured TCP force, the active TCP offset, safety state, and
registers 24 (force limit) / 25 (phase) that every `urlib` primitive sets via `tel()`.
Stop it with `kill <PID>` (SIGTERM, flushes) at S10. New primitives must call `tel(flim, phase)`
first (phases: 1 approach, 2 contact, 3 search, 4 engage, 5 wiggle, 6 seat, 7 back-off,
8 extract, 9 touch).

### S1 Preflight (read-only)
- `bridge_my_status`: tier `move`, Remote Control. If not → ask the operator once (log it).
- `robot_get_state`: NORMAL safety mode, no protective stop. Joints: elbow |q2| ≥ 0.5, wrist 3
  within ±0.8 of `go_view.W3_REF` (−3.78).
- Dashboard up (`GET /api/frame/1` = 200), link baseline (`GET /api/qa`): port N link **down**.
- Pass: all true. Else stop and report.

### S2 Code fixes (no robot motion) — only once, skip if already done
1. **Own bridge client name** in `URExec` (e.g. `rj45-skill`), not `order2cobot-calib`.
2. **`configs/grip.yaml`** = the tool model: `tcp_tip` (flange→tip, 6D), `tag_to_tip` (from S3),
   `collision_points` (tip, shell rear, cable exit, in flange frame), `stamp`, `source`.
   `move_checked` must read its clearance points from it (today they are hard-coded for the
   old grip) and the URScript preamble must `set_tcp` from `tcp_tip`.
3. **Add the primitives in "Reference URScript"** below to `urlib.LIB`
   (`slide_until_advance`, `spiral_search`, `wiggle_test`, `seat_compliant`), and keep
   `insert_fm`/`grid_search` but do not use them.
4. Validate: run each new primitive once **in free air** 20 mm in front of the face (expected:
   no contact, "not found", program ends cleanly). Before its first real run, print the
   generated script for the operator to glance at (the no-sim safety gate; log it as setup).

### S3 Grip check (tag on the shell, ArUco DICT_4X4_50 id 0, **12 mm** — measure if unsure)
- `porttest/tag_test.py 20 12 1 0` (copy of the tested script): average of 20 frames.
  Pass: 20/20 detected, mean-of-frames spread < 0.3 mm, flip resolved (chosen normal
  < 10° from a flange axis, the other > 10° worse).
- **First run (no `tag_to_tip` yet):** derive the tip in the flange frame from the
  gripper-cam image (as `plug_axis.py` did) and refine it with S5's touches; store
  `tag_to_tip` = inverse(tag pose) · tip pose in `grip.yaml`.
- **Later runs:** tip = tag pose · `tag_to_tip`. If it moved > 1 mm or > 2° vs `grip.yaml`
  → grip changed: update `tcp_tip` + collision points, log `observed` + `interpretation`.
- Note: the tag normal sits ~6–7° off flange +y (glue or grip tilt) — use the transform,
  not the axis assumption.

### S4 Scene check — CAD guidance (skill `cad-guidance`)
- `python -m cadguide ensure ls108g --compact` (no motion). It returns every port's entry
  point in base mm, σ, a search `budget` and `next_anchor`; log the line as `observed`.
- `status` unregistered / stale with **no anchor in the current epoch** → re-run the port
  survey (`python -m porttest.port_survey`, full-res frames), then
  `python -m cadguide register ls108g --survey data/ports.yaml` (gate: fit rms < 1 mm).
  The fixed camera cannot register the switch (black tape on top gives no depth).
- **Fast path** (≥ 1 anchor in the current epoch, same grip, σ of port N ≤ 0.5 mm): skip the
  top-edge and left-end touches of S5; keep the local face touch above port N.

### S5 Touch references with the plug tip (one URScript program per reference)
Use `slide_until_advance` (press the face lightly, slide, detect the tip falling over an edge).
1. **Face y**: approach a solid spot of the face beside port N (between port rows/LED area,
   from the survey), contact at 3–4 N → face y. Repeat once 2 mm away: agree < 0.3 mm.
2. **Top edge z**: from the face contact, slide +z (b) until the tip advances > 1.5 mm
   → top edge z. Back off, repeat: < 0.3 mm.
3. **Left end x** (port 1 side): slide −x (−a) at port height until advance → left end x.
Cross-check each against the survey: |touch − survey| < 3 mm, else stop and log (one of the
two is wrong — find out which before continuing). Repeat the **left-end** touch too (it sets x).
- **Re-touch before every port** while the switch is only taped: the 25 N seat of port 1
  pushed it +2.25 mm in y and −1.6 mm in x (10-25; the fixed cam confirmed 6–7 px). Compare
  with the last run's touches; a change > 0.3 mm = the switch moved → use the new touches
  (targets are relative to them, so this is handled) and log `assumption false`.
- **Tell the operator when the fixture is not good enough — and propose the fix.** If the
  switch moved > 1 mm since the last run, is yawed > 1°, or lifted during a seat (tip z rose
  > 1 mm, or the fixed cam shows it): say so in plain words with the numbers (e.g. "the switch
  moved +2.3 mm in y and −1.6 mm in x since port 1; the 25 N seat pushes it"), log it as
  `note interpretation`, and propose a concrete change: a printed corner stop / bracket
  against the switch's back and left side, screws or clamps instead of tape, or a lower seat
  force. Keep going with re-touching meanwhile — it is a recommendation, not a stop. (5 runs
  on 10-25: face +4 mm, left end −2 mm, ~2° yaw, one 7 mm lift; re-touching cost 35–40 % of
  the run time.)
- Also touch the face **locally above port N** (z between the port top and the top edge):
  the face can be yawed after a push (0.77 mm over 27 mm between port 1 and 2 on 10-25).

### S6 Target
**CAD first:** port N x / z = `ports.N.entry` from the S4 brief (tip coordinates once anchored),
face y = the local face touch, search radius = `ports.N.budget` (clamp 0.5–3 mm).
Before the first anchor, cross-check with the touch formula below: |CAD − touch| > 3 mm → stop
and find out which is wrong. Touch formula (`configs/products/ls108g.yaml`, after port 1):
**port N x = left_end_pass_x + port1_from_left_pass_x + (N−1)·pitch** (ports go **+x**),
**port z = top_pass_z − port1_below_top_pass_z**. Search radius 1.5 mm.

### S7 Insert (one URScript program per stage, results via textmsg)
1. `move_checked` to 15 mm in front of the target (flange pose from `grip.yaml`).
2. **Search** — `spiral_search` at **4 N**, pitch 0.4 mm, radius 1.5 mm (routine) / 3 mm,
   2 mm/s. Reference = the **measured local face y** (not "where contact happened"): going
   past the face on approach is a candidate, not "no face".
   Candidate = tip > **0.8 mm past the local face** within the radius of the predicted port.
3. **Engage** — at the candidate, push **10–12 N with x/z compliant** until ≥ 4 mm past the
   face (4 N stalls at the mouth: port 1 stopped 1.1 mm in).
4. **Prove** — `wiggle_test` **amp 0.8 mm** at ≥ 4 mm depth: in port = the larger side
   **> 4 N on both axes** (the plug sits off-centre, so only one wall per axis answers; port 2:
   −a 5.05 N, +b 4.08 N; at 0.3 mm everything read < 2.6 N = false negative). Log all four
   forces every time — the threshold is tuned from these. Geometry backs it up: past the
   *measured* face, at the predicted port, ≥ 40 mm from the switch end, a slide-off is impossible.
   Free → false hit: log position, back off 2 mm, continue the search.
5. **Seat** — `seat_compliant`, x/z free, push 25 N (port 1: 3.3 s, port 2: 1.7 s; try 15–18 N —
   25 N moves the taped switch) to 13 mm past the face or 5 s. Then **back off until |F| < 3 N**
   (0.5 mm left 12.3 N pressing on port 1), never end a program while pressing.
Pass to S8 only if seat depth ≥ 10 mm.

### S8 Verify
`POST /api/qa/<N>/insert` → link up 1000/full = **PASS**. Then extract straight back (the
cable is glued into the shell) and confirm the link drops (`/api/qa/<N>/extract`). If the user
says to leave the plug in (e.g. demo), skip the extraction and say so in the report.

### S8b Anchor (after every PASS)
The tip position at the **end of the engage** (plug constrained, before the seat push):
`python -m cadguide anchor ls108g port:N X Y Z --run <run dir> --note "engage end, link up"`
(metres, base, tip TCP). Log the GATE lines: `PASS` = model holds; `part moved` = new epoch
(fine, the next search uses the new pose); `fit residual` = stop and check the seat / grip.
Next port: take `next_anchor` (the far end) second — after two anchors the other ports need
only the local face touch and a ~0.5 mm search. After any regrip: `cadguide reset ls108g --anchors`.

### S9 Retry / escalate
- Max **3** insert attempts per port. Between attempts, re-run S7 from the last good
  candidate list; after 2 false hits at the same spot, re-check S5 face/edges.
- Protective stop: stop immediately, log `observed` (code) + `operator` (recovery). Do not
  resume until the cause is understood (most likely: stiff axes or ending while pressing).
- After 3 failures: `done --status failure`, write what you would need (sensor, view, info)
  as a `note interpretation`, and escalate to the human. "Cannot do" is a valid result.

### S10 Close
Stop the telemetry recorder (`kill <PID>`), note its CSV path and sample count as `observed`.
`runlog end`, `runlog trace`, then fill `interpretation`/`summary` where empty in
`data/runs/current/session_trace.json` and the `summary` block (time sinks, failure
patterns, missing capabilities, keep), regenerate with
`python porttest/session_trace/session_trace_to_drawio.py <json> <xml>`. Update
`porttest/LEARNINGS.md` "Current state" with what changed. Report the run dir path.

## Reference URScript (add to `urlib.LIB`; validate before use)

Face normal n, in-plane a/b are base unit vectors. `sel`/`wr` select the compliant axis
(n = +y → sel `[0,1,0,0,0,0]`, wr `[0,fn,0,0,0,0]`). In force mode, non-compliant axes
follow `speedl`; compliant axes are force-controlled (limits = max speed there).

```
def adv_along(n, ref):
  p = get_actual_tcp_pose()
  return p[0]*n[0] + p[1]*n[1] + p[2]*n[2] - ref
end

def seg(s, L, v, n, ref, adv):
  # slide along s for L metres (in-plane distance) unless the tip advances along n by > adv
  q0 = get_actual_tcp_pose()
  while (get_actual_tcp_pose()[0]-q0[0])*s[0] + (get_actual_tcp_pose()[1]-q0[1])*s[1] + (get_actual_tcp_pose()[2]-q0[2])*s[2] < L:
    speedl([s[0]*v, s[1]*v, s[2]*v, 0, 0, 0], 0.2, 0.008)
    if adv_along(n, ref) > adv:
      stopl(0.5)
      return True
    end
  end
  stopl(0.5)
  return False
end

def contact(sel, wr, n):
  # enter force mode pressing along n; returns the face coordinate along n
  zero_ftsensor()
  sleep(0.3)
  force_mode(p[0, 0, 0, 0, 0, 0], sel, wr, 2, [0.01, 0.005, 0.01, 0.1, 0.1, 0.1])
  sleep(1.0)
  return adv_along(n, 0)
end

def slide_until_advance(sel, wr, n, s, maxd, adv, v):
  ref = contact(sel, wr, n)
  hit = seg(s, maxd, v, n, ref, adv)
  p = get_actual_tcp_pose()
  textmsg(str_cat("face ", ref))
  textmsg(str_cat("advance ", hit))
  textmsg("pose ", p)
  end_force_mode()
  movel(pose_add(p, p[-0.002*n[0], -0.002*n[1], -0.002*n[2], 0, 0, 0]), a=0.1, v=0.005)
  return hit
end

def spiral_search(sel, wr, n, a, b, pitch, rmax, adv, v):
  # contact, then a square spiral centre-out in the a/b plane; stays in force mode on a hit
  ref = contact(sel, wr, n)
  textmsg(str_cat("face ", ref))
  k = 1
  d = 0
  while k * pitch <= 2 * rmax:
    s = a
    if d % 4 == 1:
      s = b
    elif d % 4 == 2:
      s = [-a[0], -a[1], -a[2]]
    elif d % 4 == 3:
      s = [-b[0], -b[1], -b[2]]
    end
    if seg(s, k * pitch, v, n, ref, adv):
      textmsg("candidate ", get_actual_tcp_pose())
      return True
    end
    if d % 2 == 1:
      k = k + 1
    end
    d = d + 1
  end
  textmsg("not found")
  end_force_mode()
  p = get_actual_tcp_pose()
  movel(pose_add(p, p[-0.002*n[0], -0.002*n[1], -0.002*n[2], 0, 0, 0]), a=0.1, v=0.005)
  return False
end

def lateral(s, amp, v):
  # move amp along s, return the reaction force along s, come back
  q0 = get_actual_tcp_pose()
  while (get_actual_tcp_pose()[0]-q0[0])*s[0] + (get_actual_tcp_pose()[1]-q0[1])*s[1] + (get_actual_tcp_pose()[2]-q0[2])*s[2] < amp:
    speedl([s[0]*v, s[1]*v, s[2]*v, 0, 0, 0], 0.2, 0.008)
    F = get_tcp_force()
    if -(F[0]*s[0] + F[1]*s[1] + F[2]*s[2]) > 8.0:
      break
    end
  end
  F = get_tcp_force()
  stopl(0.5)
  f = -(F[0]*s[0] + F[1]*s[1] + F[2]*s[2])
  movel(q0, a=0.1, v=0.003)
  return f
end

def wiggle_test(a, b, amp, fthr):
  # call while still in force mode at a candidate; constrained in a AND b = inside the port
  fa1 = lateral(a, amp, 0.002)
  fa2 = lateral([-a[0], -a[1], -a[2]], amp, 0.002)
  fb1 = lateral(b, amp, 0.002)
  fb2 = lateral([-b[0], -b[1], -b[2]], amp, 0.002)
  textmsg(str_cat("wiggle ", str_cat(str_cat(fa1, str_cat(" ", fa2)), str_cat(" ", str_cat(fb1, str_cat(" ", fb2))))))
  return fa1 > fthr and fa2 > fthr and fb1 > fthr and fb2 > fthr
end

def seat_compliant(n, fy, depth, tmax):
  # x/z compliant with zero force (self-align), push fy along n; back off 0.5 mm before return
  end_force_mode()
  p0 = get_actual_tcp_pose()
  force_mode(p[0, 0, 0, 0, 0, 0], [1, 1, 1, 0, 0, 0], [0.0, fy, 0.0, 0.0, 0.0, 0.0], 2,
             [0.003, 0.004, 0.003, 0.1, 0.1, 0.1])
  t = 0
  while t < tmax:
    if adv_along(n, p0[0]*n[0] + p0[1]*n[1] + p0[2]*n[2]) > depth:
      break
    end
    sync()
    t = t + get_steptime()
  end
  dy = adv_along(n, p0[0]*n[0] + p0[1]*n[1] + p0[2]*n[2])
  end_force_mode()
  p = get_actual_tcp_pose()
  movel(pose_add(p, p[-0.0005*n[0], -0.0005*n[1], -0.0005*n[2], 0, 0, 0]), a=0.1, v=0.002)
  textmsg(str_cat("seated ", str_cat(dy, str_cat(" t ", t))))
  return dy
end
```

Notes on the reference code (check on the robot, tune, log what you change):
- `wiggle_test` assumes the force mode from `spiral_search` is still active (n pushing).
  The `fthr` default 4 N with fn 4 N: face friction alone gives ~1–2 N; inside the
  opening the wall answers within 0.3 mm. Log the four forces every time — they are the
  data to tune the threshold.
- `seat_compliant` lets x/z float at zero force: a wrong payload makes the tip sag. The
  preamble's `set_target_payload` must match the tool (Hand-E + camera + shell).
- If URScript rejects a construct (e.g. list negation, `elif`), fix it and note the change.
- **URScript has no `abs()`** (compile_error_name_not_found on 2026-10-25). Use `norm(x)`;
  `fabs` is not in the PolyScope X 10.14 directory either. Also absent: `min`, `max`, `len`
  (→ `size(x)` / `x.length()`), `round` (→ `floor(x+0.5)`), `range`, `print` (→ `textmsg`),
  `str`/`int`/`float` (→ `to_str`/`to_num`), `for` loops (→ `while`), `**` (→ `pow`).
- **Run `python -m porttest.urscript_lint` after every change to `urlib.LIB`** (and on any
  generated script: `from porttest.urscript_lint import lint`). It checks every call against
  the 267 documented functions of the PolyScope X 10.14 directory plus the script's own
  `def`s. Best: call `lint()` inside `URExec.run` and refuse to send a script with problems.

## Done criteria for the run
PASS = link up 1000/full on port N **and** the trace exists. Report: result, attempts,
false hits, operator actions (count), wall time, run dir, and the 3 biggest surprises.
