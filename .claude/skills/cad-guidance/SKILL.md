---
name: cad-guidance
description: CAD-guided targeting for insertion tasks on the UR3e workcell - register the CAD models of the TP-Link LS108G switch and the breadboard in the robot base frame (fixed D435 cloud + image, or the gripper-cam port survey), get every port/hole target in base mm with an uncertainty and a search budget, and turn each seated insertion into an anchor that tightens the rest to sub-mm. Use before any RJ45/breadboard insertion, when asked to align/calibrate/register the switch or breadboard, or to predict where a port/hole is.
---

# CAD guidance — skill

**Idea:** the cameras give the full 6-DOF pose to ~1 mm; that is a strong *first guess*, not a
target. The table plane already fixes z / roll / pitch, so after the **first insertion** only
the in-plane 3 DOF (x, y, yaw) plus the pitch (ports/holes per mm in the robot's tool
coordinates) remain, and seated insertions measure exactly those. Insert one, anchor it, and
the rest are a linear offset away: port N = port 1 + (N−1)·pitch vector; hole (c, r) =
a1 + k·col_step + (r−1)·row_step. A second anchor far from the first pins yaw and pitch.

This matches the literature (coarse CAD registration → compliant capture → contact-based
re-anchoring: GenCHiP 2024 compliant action spaces, NIST task-board work, IndustReal /
FORGE capture regions, touch-based re-localisation). Pose estimators like FoundationPose /
MegaPose reach cm-level in third-party tests, so they would only seed, never target.

Code: `cadguide/` (CLI `python -m cadguide`, API `cadguide.api`). CAD:
`assets/models/cad/{ls108g,breadboard}.py` → `dashboard/web/models/cad/*.{glb,json}`.
State: `data/cad/<obj>.yaml`. The dashboard's **3D Visualization** tab draws both models
semi-transparent at their current poses (toggle **CAD**): look there to sanity-check.

## The one call before inserting

```
python -m cadguide ensure [ls108g|breadboard ...] --compact
```

Never moves the robot. Registers from the fixed camera when an object is not registered yet
(or with `--refresh`), then prints one JSON line per object:

- `status` ok / stale (no evidence newer than `--max-age-min`, default 60) / unregistered (+ `do`)
- `anchors`, `epochs`, `fit_residual_mm`, `yaw_deg`, `pitch_mm`, `axis` (insertion direction, base)
- switch: `ports.N.entry` (base mm, opening centre on the face plane), `sigma`, `budget`
  (= 3σ lateral, ≥ 0.3 mm: use it as the search radius)
- breadboard: `grid` (a1, col_step, row_step, channel_extra in base mm + the rule), `sigma`
- `next_anchor`: which feature to insert next to pin yaw / pitch

One target in full: `python -m cadguide predict ls108g port:5` (entry, seat, axis, σ, budget).
Units: the CLI prints base **mm** in `ensure`, **metres** in `predict`/`anchor` (URScript units).

## Workflow

1. **Ensure.** `ensure <obj> --compact`. Read the GATE lines.
   - Switch `unregistered`/`stale` with no anchors in the current epoch: the fixed camera
     cannot see it (black tape on the top, no D435 depth), so run the gripper-cam survey
     (`python -m porttest.port_survey`, moves the camera along x) then
     `python -m cadguide register ls108g --survey data/ports.yaml`. Gate: fit rms < 1 mm.
   - Breadboard: `register breadboard` (or `ensure --refresh`). Gate: image edge inliers
     > 0.55 and edge cost < 2.5 px. **Open the overlay PNG** it names and check that the
     yellow outline and magenta holes sit on the board. A cable or the arm across the board
     is fine (the image fit handles it); the arm covering most of it is not: move it out of
     view and retry.
   - A failed gate never replaces a good registration (`--force` to override).
2. **First insertion** (no anchors in this epoch): target = `entry`, search radius =
   `budget` (expect 3–7 mm). Use the insertion skill's full method (touch references, spiral,
   engage, prove, seat) — the CAD only replaces the survey arithmetic.
3. **Anchor it** right after the plug is *constrained* (end of the x/z-compliant engage,
   before the seat push, which can lift a taped part) and the link/seat is confirmed:
   ```
   python -m cadguide anchor ls108g port:N X Y Z --run <run dir> --note "<what>"   # tip TCP, metres, base
   ```
   Only the error across the insertion axis is used (depth is the force controller's).
   Read the GATE lines:
   - `PASS: anchor 0.2 mm lateral from prediction` → the model holds.
   - `WARN: part moved (> 1.0 mm switch / 0.6 mm board)` → a new epoch starts: the pitch
     learned so far is kept, the pose is re-anchored by this insertion.
   - `WARN: fit residual …` → an anchor disagrees with the rigid model (bad seat, regrip,
     a turn): check before the next insertion; `reset <obj> --anchors` if in doubt.
4. **Second insertion: far away** (`next_anchor`: port 8 after port 1, j60 after a1).
   Its search radius is already small (~1 mm). After it, yaw + pitch are measured.
5. **The rest:** `ensure` again (σ ≈ 0.2–0.5 mm) → targets with radius `budget`; skip the
   long touch sequence, keep one local face touch (depth reference). Anchor every seat:
   creep from each push is absorbed epoch by epoch.

## Rules

- **Grip change = new tool coordinates.** After a regrip/regrasp, anchors no longer match
  the TCP: `reset <obj> --anchors` and do a first insertion again (the registration stays).
- **Anchors are only as good as the seat.** Anchor only proven insertions (link up / wiggle
  constrained), with the tip TCP of the current `grip.yaml`.
- **Do not trust yaw from one anchor**: predictions far from a single anchor carry the
  registration's yaw (0.5° = 0.6 mm at 70 mm). σ in the brief already says so.
- The switch creeps ~1–2 mm per 18–25 N seat (taped). Prefer lower seat forces; propose a
  hold-down; the epoch logic copes but every move costs a wider search.
- Log the brief you used (`note observed`) and each anchor result in the run log.

## Model facts (verify if in doubt)

- LS108G (`ls108g.json`): 158 × 99.1 × 25 mm (official), latch slot **up**, Link/Act LED
  top-left of each jack, port 1 left seen from the front. Pitch nominal 14.55 mm (survey
  14.51–14.56); seated tip coordinates came out **15.04 mm/port** (replay of 10-25, 4 seats)
  — the anchors learn this, do not hard-code it.
- Breadboard (`breadboard.json`): the table board, 60 rows a–e | f–j + two ± rail pairs,
  55 × 165.1 × 8.15 mm, cross-section from the vendor CAD (`assets/models/breadboard.sab`,
  which is the 400 version: `breadboard_400.json`). Length and rail grouping of the 60-row
  board are nominal; the hole grid is exact 2.54 mm, entry funnel 1.69 → 0.45 mm in 0.6 mm.
