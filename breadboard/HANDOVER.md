# Handoff: breadboard picking (for the picking team's agent)

You take over **picking** of breadboard parts on the UR3e: get a casing (jumper, LED, ...) from
storage into the fingers so that the existing **placing** skill can put it into the board, and let
go of it there. Placing works (2026-10-27): 8 runs, 6 hits, both misses explained. Picking does
not exist yet; until now a human put every part into the gripper.

Repo: Jetson `/home/lab/order2cobot`, branch **`aspire`** (on `restructure`; merges cleanly into
local `main`). Bridge client `breadboard-explore`, app `breadboard_explore`.

## Read first (do not re-derive what is written there)

1. `.claude/skills/breadboard-place/SKILL.md` — the placing skill: conventions, steps, hard rules.
2. `skills_lib/breadboard/*.yaml` — 12 lessons from the real robot (symptom -> cause -> fix ->
   validating run). Read before you write any robot step; add to it after every miss.
3. `breadboard/README.md` (pipeline status), `breadboard/NOTES.md` (10-26 protoboard run:
   forces, the Hand-E smart-skill limits), `skills_lib/rj45/` (switch lessons that carry over:
   bridge edit cost, `set_tcp` persists, no Python builtins in URScript).
4. `configs/breadboard/{board,board_pose,gripper}.yaml` — measured board, board in base, tool.
5. `assets/models/component_casings/casings.json` — every casing: legs, spacing, width.
6. `git log restructure..aspire` — the story of the day, one commit per step.

## How we think (keep working this way)

**Principles first, before any code.** For each new problem write down: (1) the immutable
constraints, (2) the assumptions, each challenged and given a verdict (Keep / Drop /
Investigate), (3) the simplest architecture that meets the success criterion. No boilerplate
first. Example: we dropped "teach the board corners by hand" (verdict Drop: the operator must not
teach per board) and "use the team CAD" (Drop: it had 60 rows, the board has 63) before writing
anything.

**Ground truth comes from touch, not from models.** Cameras give a rough box (~10 mm). CAD and
config numbers are guesses until the robot has touched them. We measured the board edges, the
tool length (the CAD was 27.5 mm off) and the leg length by guarded touches. Every number in a
config says where it came from (`source:`, `stamp:`) and is re-measured when the hardware changes.

**Coarse, then fine.** A measurement only has to beat the next step's capture range. The grasp
calibration only needs +-1.27 mm (half a pitch) to pick the right hole pair; the hole's funnel
and the peck drop give the fine position. Don't over-engineer precision a later step provides.

**Know what your search can confuse.** The peck search accepts *any* hole pair, so a part held
2 mm off-centre landed one hole off and "succeeded". Ask of every search: what wrong answer looks
like a right one? Then add a self-check (`pair_check`). A self-reported success is not ground
truth until it is verified.

**Every miss is recorded with its exact cause.** In the run's `result.json` (`miss_reason`), the
state history (`data/breadboard_state.json`) and a lesson in `skills_lib/`. A lesson becomes
`validated` only through a passing re-run. Never delete lessons; supersede them.

**Stepping stones.** One new variable per run: same part, other rows -> other half of the board
-> a wider part -> a new grasp -> two parts connected. Each step is confirmed before the next.
The operator reads the result; `--confirm` records it.

**The human is supervisor and hands, never the programmer.** The operator grants access,
switches Remote Control, loads parts (for now), reads holes, and gives feedback in words. No
alignment hints during a run ("that's cheating"): telling the agent where the holes are, then
correcting with it, does not count - the agent must find it out itself (the LED miss was only
fixed when the agent could measure its own grasp offset).

**Low forces, straight moves, low approach.** Legs bend and parts tilt in the fingers: touch 2 N,
side push 1.5 N, seat <= 6 N; pins go in along z only; travel 2 cm above the board, then sideways.
When unsure, a guarded move that stops early is always the right first version.

**LLM decides, code executes.** One URScript program per step (~20 s per bridge round trip), all
contacts reported via `textmsg` and parsed in Python into compact JSON. Plan mode first (lint +
IK of every point, no motion), then `--run`.

**Be honest about your own mistakes.** Today's bugs: the tool-length sign (robot stayed ~55 mm
high), `halt` skipping the TCP reset, a stale joint vector, a calibration measured in the wrong
frame. Each was said out loud, fixed, and written into `skills_lib/`.

## Where placing stands (details in SKILL.md and the lessons)

| Holes | Part | Result |
|---|---|---|
| j10/j11, j3/j4, j35/j36 | jumper_s1 (hand-centred) | hit, hit, hit (first peck) |
| j20/j22 | led_s2 | miss (j19/j21): grasp offset > half a pitch, no calibration |
| j20/j22 | led_s2 | miss (j19/j21): calibration in the wrong frame; flagged by the agent |
| j20/j22, a25/a27 | led_s2 | hit, hit (first peck) |
| j30/j34 | led_s4 | hit (first peck, tilt residual -0.02 mm) |

Run data is on the Jetson only (gitignored): `data/runs/{touch_locate,jumper_probe,place_probe}_*`.

## The interface: what placing needs from picking

1. The part in the fingers, **legs straight down**, the leg pair **along the finger faces**
   (across the jaw gap), casing bottom at the fingertips.
2. **The casing name** (`s<N>` = legs N rows apart). The operator could not tell an LED's
   casing apart - the picker must identify it (storage slot, label, or measurement).
3. Grip offset along the legs up to ~+-3 mm is measured by `--calib`; **across the legs (y) and
   tilt are not** - a picker that centres the part across the jaw keeps that assumption true.
4. Polarity: which leg is which (LED anode) - placing treats parts as symmetric today.
5. Then: `python -m breadboard.place_probe <h1> <h2> --part <casing> --calib --run --note "<grip>"`.

## Your problem: constraints and assumptions to start from

Constraints (immutable today): no jogging - every motion is a played URScript program; Hand-E only
through the bridge smart skills **Gripper Close / Gripper Open** (listed with `force`, `speed`,
`targetPosition` = 100/100/100); **no Hand-E position or force readout** (no bridge tool, no
`rq_*` functions in our step preamble); custom PLA fingers that are **not snug** on the Hand-E
caps and broke under ~185 N before; reach x > -0.46 m at board height; wrist 3 within +-0.8 rad.

Assumptions to give a verdict to first:
- *"`smartskill_run` accepts lower force / a target position"* — **Investigate first**, read-only
  `smartskill_list` / `smartskill_status`, then one supervised run with an empty gripper. This
  decides gentle gripping, partial opening and release in the board.
- *"Full opening next to other parts is safe"* — Investigate: ~+-25 mm per finger; fine at an
  empty spot, not between placed parts. The CSP solver's `opening_per_finger_mm` assumes 1 mm.
- *"A part picked from the storage table sits centred"* — Drop until measured: grasp
  calibration exists for x; you need y (and tilt, `calib_residual_mm`) for robot grasps.
- *"The learned board offset (+0.8, -0.95 mm) is board error"* — Keep only while grasps are
  hand-centred; re-learn it with calibrated robot grasps.
- *"Storage poses can be taught"* — Drop by our rule; locate the storage table the way we located
  the board (rough box from a camera / fiducial, then touches).

## Open problems (inherited)

- Release a part in the board without disturbing it (blocks "connect two parts").
- Grasp offset across the legs (y) and tilt; a second calibration height or a y face.
- Polarity of LEDs / BJTs through pick and place.
- Automatic verification of the hole pair (cam shots are saved per run; continuity / LED test).
- Reach: rows past ~41 unreachable on the taped board; the planned L fixture + ArUco should sit
  ~5 cm closer to the robot.
- The CSP solver's `src/circuit_mapper/board_connections.json` still has the 10.16 mm channel and
  must use `configs/breadboard/board.yaml` (7.62) before real circuits are planned.
- `breadboard/run_plan.py` (plan executor) is a skeleton; wire pick -> calib -> place into it.
- Merging: local `main` and `origin/main` already conflict on `mapping.json` (unrelated to
  `aspire`); resolve before pushing `main`.

## Safety protocol (real, shared robot)

Never play a program the operator did not expect. Before the first real run of any new program:
show the plan (what moves, speeds, force caps, abort branches), get a "go". If something is
refused: `bridge_my_status`, then ask the operator - do not retry. Protective stops are cleared by
the operator. Give every process its own bridge client name.

## Suggested skills

- `breadboard-place` — the placing skill you hand parts to; read its hard rules.
- `grill-me` — run the principles-first interview on the picking design before coding.
- `diagnose` — for every miss: reproduce, hypothesise, instrument, fix, record the lesson.
- `rj45-insert` — reference for the tag-based grip check and session logging patterns.
- `handoff` — leave the same kind of document for the next agent when you stop.
