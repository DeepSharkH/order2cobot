# CSP overnight plan (ranked)

Branch `csp-overnight`, worktree only. Inputs: two literature dives, the profiler
(`bench/profile_model.py`), the geometry audit (`bench/geometry_audit.md`) and the baseline
(`bench/results/baseline.json`, 3 of 10 instances solved).

## Why it is slow (measured, 8 workers, Jetson under load)

1. **Presolve is redone every lazy round** and is 77-92 % of solve time. Probing alone is 55-65 %
   of that (LED: 6.5 s presolve, 0.4 s search per round).
2. **Spurious jumpers.** `stop_after_first_solution` makes the min-jumpers objective a no-op. The
   first solution comes from feasibility-jump and carries 14-176 jumpers that carry no current.
   Their casings collide, giving 50-1127 cuts per round, so the loop never settles. The RC filter
   needs 0 jumpers and still ends UNKNOWN.
3. **Point cuts.** Each cut forbids one exact placement pair, and the solver shifts the same
   conflict one column or 30 rows away (R1/D1 dodged 3 times at about 7 s each).
4. **Topology.** Every kit part with spacing 1-2 must stand along the rows, and only the 4-pitch
   jumper crosses the channel. So p3, p4 and p5 need at least 1, 1 and 2 jumpers. Every
   jumper-needing case is UNKNOWN today.

## Targets and the gate every change must pass

- **Targets** (`bench.py --time-limit 120`): p1, p2, led_circuit, rc_low_pass and
  parallel_resistors in ≤ 10 s; p3 and p4 in ≤ 30 s; p5 in ≤ 60 s; parallel_leds_5 solved
  (stretch).
- **Gate:**
  - Every solved instance shows `ver = yes`: the union-find connectivity check plus
    `assembly_plan.steps` reporting no collisions.
  - No instance that solved before regresses to UNKNOWN.
  - `map_circuit(circuit, inventory, board, time_limit, casings, gripper)` and the CLI
    (`--output`, `--time-limit`) are unchanged.
  - mapping.json keeps `placements[id, part_id, orientation_deg, center_mm, casing_id]`,
    `assembly_order`, `pin_to_hole`, `manual_connections`, `jumpers`, `gripper` and `status`.
  - `verify_assembly` and `verify_connections` still run on every returned layout.
  - Timing claims for p1 and p2 use the median of 3 runs (load noise is about ±30 %).
- **Safety argument for P1-P10:** they change only how the search finds a layout.
  `relation()` and both verifiers are untouched and run on the output, so no unsafe layout can be
  returned. Only the G items change what "safe" means; they are judged separately below.

## Loop protocol

- One candidate per commit on `csp-overnight`, then `bench.py --label <id>`, then
  `git add -f bench/results/<id>.json`.
- Keep the commit if the gate passes and the target metric improves; otherwise revert it.
- `bench.py` takes the solver lock itself, so never wrap it in `flock`. Wrap every other solver run
  with `flock …/solver.lock timeout 300 …`. Never push.

## Speed candidates (do in this order)

| id | change (mapper.py unless noted) | acceptance | expected gain | risk |
|---|---|---|---|---|
| **P1** | Solver params: `num_workers=8` (replaces deprecated `num_search_workers`; never below 4: 1 worker has no feasibility-jump and never solved anything), `cp_model_probing_level=0`, `max_presolve_iterations=1`. | p1 and led ≤ 6 s; parallel_resistors ≤ 4 s; p2 and rc OPTIMAL ≤ 15 s. | Measured: LED 27.8 → 3.0-3.8 s; rc UNKNOWN → 5-8.5 s OPTIMAL; per round 5-8 s → 0.5-2.7 s. | Probing can help hard models (pl5's first solution once came at 20.7 s instead of 5.9 s). A/B level 0 vs 1 on p3-p5 and keep the better. |
| **P2** | **Jumper budget K, staged.** Solve with `sum(wires) ≤ K` for K = 0, 1, 2, 4, … with a time slice per stage. Build no wire or flow variables at K = 0. Add "chosen wire ⇒ its edge carries ≠ 0 flow of the strip's net" (no dangling wires). Inner solves are pure feasibility with no objective. `status` = OPTIMAL only if every smaller K was proven INFEASIBLE, else FEASIBLE. | p2 and rc return 0 jumpers. p3 and p4 solve. Unit check: removing any single returned jumper makes `verify_connections` fail (no useless wires). | Removes the jumper thrash. Model 12.9k → 3.9k vars (LED), 14.2k → 5.2k (RC) at K = 0. Measured rc without jumpers: 2.1 s with P1. | Circuits that need jumpers pay the K = 0 slice. A "minimal" claim only holds after INFEASIBLE proofs, never after timeouts. |
| **P3** | **Row window, centred, widening.** Generate part, wire and manual-pin candidates only in rows [c−H/2, c+H/2), c = board centre. Start H0 = clamp(1.5 × Σ casing length per half / pitch, 12, 24). Stage schedule: (H0, K = 0, 1, 2) → (2·H0, K ≤ 4) → (full board, K ≤ 2·nets). Being centred, it needs no post-shift. | All p1-p5 layouts lie in the window. pl5 gets its first candidate in < 5 s. No regressions. | Measured: rc without jumpers 24.6 → 3.6 s at 16 rows (rounds 6 s → 0.6 s); pl5 first candidate 1.9 s at 24 rows vs none in 65 s at 64. Also more compact layouts: less reach, better camera view. | Too small a window gives INFEASIBLE or a timeout, so widening is required. Use slices, not INFEASIBLE proofs (CP-SAT proves packing infeasibility poorly). |
| **P4** | **Drop the order ints** (2,526 ints with domain 0..2525). `constrain_assembly` adds only `forbidden` cuts. After a solve, build the "A's fingers hit B" digraph over the selection and sort it topologically. Tie-break: jumpers, then slim parts, then LED/RGB last, then by row. Cycles of 3 or more get `add_bool_or([~x for x in cycle])` and a re-solve (2-cycles are already `forbidden`). | `assembly_order` passes `verify_assembly` and bench `ver`. Unit test: a constructed 3-cycle produces exactly one nogood. | Model shrinks by about 2.5k ints and order constraints. Cuts become plain clauses. The order becomes deterministic and readable. | Low. Each 3-cycle costs one extra round (expected rare). |
| **P5** | **Lift every cut to all translations** (geometry.py). Cache `relation()` by (casing_a, ori_a, casing_b, ori_b, Δcx, Δcy) in µm. When a selected pair is `forbidden`, add `a + b ≤ 1` for every candidate pair with the same key, including wire candidates. Forbidden is owner-independent, so this is exact. | p1-p4 need ≤ 3 rounds. Unit test: every lifted pair gives `relation() == 'forbidden'`. | Iterations drop from one per placement conflict to one per conflict type. pl5: 17+ non-converging rounds → est. 5-10. | Constraint count grows with the window (bounded by P3). Post per-candidate cliques if it gets large. |
| **P6** | **Eager forbidden cliques between robot-placed parts** (no-fit table built from `relation()` itself). Once per run, tabulate the hole offsets within reach (≈ 10 pitches) that are `forbidden` per (casing pair, orientation pair). For each candidate a and partner part B, post `add_at_most_one({a} ∪ {b ∈ B : forbidden(a, b)})`. This is a clique because B is exactly-one. Jumpers stay lazy (P5). | Bench: part-part cuts per round = 0, rounds ≤ 2 on p1-p5. Randomized test: clique membership == `relation()` for 10⁵ pairs. | The lazy loop is left with jumper conflicts and rare cycles only. Profile: forbidden cuts are 90-95 % of all cuts. About 450 candidates × (P−1) cliques per part at H = 24. | Medium: more constraints, so more presolve. Measure presolve with P1 on. It is exact by construction because it reuses `relation()`. |
| **P7** | **Warm start:** after each round, `clear_hints` then `add_hint` for the previous selection, with `repair_hint=True`. | No regression. Layouts stay local between rounds (log the centre drift). | Moderate: fewer and more local rounds. | None (hints never change feasibility). |
| **P8** | **Manual pins:** candidates only inside the window. Same-net manual pins get increasing hole index (symmetry breaking; p5's GND has 3 interchangeable pins). | No regression. Fewer vars (640 → about 240 per pin). | Small to moderate (rc has 2,560 manual-pin bools). | Low. |
| **P9** | **Deadline guard:** do not start a round when the time left is less than the last round's presolve time. The error names the stage reached (K, H, rounds). | rc no longer burns its last 5 s inside presolve. | Up to 1 round saved; clearer failures. | None. |
| **P10** | (optional) **Identical-branch symmetry:** components with the same part_id and the same net incidence up to private nets get `y(R_i) ≤ y(R_{i+1})` on the centre expression. Behind a flag. | p3 and pl5 faster with the flag; never INFEASIBLE where it was feasible without it. | Up to k! fewer equivalent layouts (k = 2 in p3, 5 in pl5). | A wrong detection can only lose solutions, never return an unsafe one. |
| P11 | (only if p5 or pl5 still fail) Two-level: assign nets to strips plus jumper links in a small model first, then place parts in columns, with the full model as fallback. | pl5 solved ≤ 120 s. | Turns the multi-pin-net case from search-limited into two small models (estimate). | L effort; capacity heuristics. |

## Robustness candidates (small; they make outputs buildable)

| id | change | type | acceptance | risk |
|---|---|---|---|---|
| **R1** | Manual-pin keep-out: a manual hole may not lie under any casing body. Model the hole as a 1 × 1 mm box (optionally 2.6 × 2.6 for a Dupont housing) in the body check. A body covers 2 holes beyond each lead in its column. | mapper bug (README: "accessibility not verified") | Unit test: a hole under a body is rejected. Bench shows no regression. | Tightens the model slightly. |
| **R2** | `board_connections.json`: rows 1-**60**. The table board is a 60-row board (user-confirmed per the CAD guidance notes); the model has 64, so rows 61-64 do not exist. | input bug | No output hole has a row > 60. The dashboard still renders. | Confirm with the user. P3 centring already keeps layouts away from the ends. |
| R3 | Uncased-part error lists every uncased part at once and suggests `placement_mode: "manual"` or a kit alternative. | input issue | pnp_transistor_switch errors with the full list. | None. |

## Geometry candidates (judge separately; numbers from `geometry_audit.md`)

The audit mostly contradicts "parts are modelled too fat":
- Casings are straight 20 mm prisms: width_x × 4.0 is real at every height.
- The finger is slightly too thin: 2.0 × 5.0 is the very tip, and it is 2.26 × 5.23 at 6 mm up.
- The **only real over-fattening is the LED 6.2 mm band**, applied over the full casing width.

| id | change | effect over 246,922 pair placements | safety |
|---|---|---|---|
| **G1** | **Real LED protrusion replaces the 6.2 band.** gripper.json `casing_protrusions_mm` = {led_s1: x ±2.1, y ±2.9, z 5.00-12.59; led_s2 5.27-12.86; led_s4 7.81-15.40; rgb_led_s1 6.16-13.75; rgb_led_s2 9.97-17.56}. It lives in gripper.json, where the overrides live today, so mapping.json carries it. `body` stays the 4.0 casing box. `relation()`: protrusion vs body or protrusion = forbidden; finger vs protrusion only if the protrusion top > the gripper's `grip_z`. If `assembled_thickness_overrides_mm` is present (old mappings), the old band applies unchanged. | forbidden 24,450 → 22,494 (−8 %); ordered 5,016 → 3,780 (−25 %). All 3,736 changes are LED/RGB pairs. An LED 2 holes from any casing (staggered) goes from forbidden to allowed. A part 3 holes from led_s1/s2 or rgb_s1 goes from ordered to independent, which removes order cycles in LED circuits. | Sound if the grip height holds (see U2). The flange gap at 2 holes is **0.18 mm nominal**, so check one printed LED-beside-casing pair. Acceptance: the unit test reproduces the audit counts, no non-LED relation changes, and the old gripper dict gives identical relations. |
| **G2** | **New 30 mm finger envelope:** `finger_thickness_mm` 2.5 (t(6) = 2.26 + 0.15 cap tilt + 0.1 tolerance), `finger_width_mm` 5.5, `opening_per_finger_mm` 0.5 (jaw command ≈ 1.0, tip travels ≈ 0.5 after bending), new `grip_tip_z_mm` 14.0. Per casing, `grip_z` = max(14, protrusion top + 0.5): led_s4 15.9. Reject rgb_led_s2 (1.9 mm contact) unless overridden. | Zero relation changes for non-LED pairs on the 2.54 grid versus today. This is a correctness update, not a relaxation. | Makes today's model honest: 2.0 × 5.0 is only safe for grips with ≤ 10.3 mm of finger beside neighbours. Valid **only** for the new fingers with the tip ≥ 14 mm above the board. |
| G3 | `gripper_fullopen.json` profile (`opening_per_finger_mm` 20.3), passed with `--gripper`, which describes what the bridge can execute today. | forbidden ≥ 47,930 (×2.0), ordered ≥ 28,864 (×5.8): many circuits may go UNKNOWN. | This is the only profile that is safe with today's full-open release. Not the default (the user asked for the relaxed model), but it is recorded in mapping.gripper so the executor can check it. |
| G4 | Body margin stays 0. Measure 2-3 printed casings; if width_x > nominal + 0.09, set 0.2. That moves only 7 of 301 arrangements (the 0.18-0.23 mm ones) one hole further apart. | none now | Tolerance-critical arrangements are listed in the audit. |
| — | Out of scope (needs a reprint or robot support): narrower casings (14 mm floor + 2.65 walls → would save about 2 rows per in-line neighbour), and gripping across the casing ends (needs a 90° wrist turn, past the ±0.8 rad cable limit). | | |

### Unsafe if ignored (flags)

- **U1 Release opening.** Every profile except G3 (today's 1.0 and v2's 0.5) assumes a partial
  opening. The bridge can only open the Hand-E fully (each tip moves about 20.3 mm), which can
  knock installed neighbours. Use G3, or get a Hand-E position command (about 10 counts ≈ 1 mm).
- **U2 Grip height.** G1 and G2 assume the robot grips with the tip at ≥ `grip_z` (≤ 6 mm of finger
  beside neighbours). Deeper, the finger is 3.34 mm thick at 15 mm up and the model is
  optimistic. The executor must read `mapping.gripper.grip_tip_z_mm`.
- **U3 Grip force.** The smart skill closes at 100 % (185 N, about 217 MPa): the PLA fingers break.
  Use 20-40 N. Not a mapper change, but G2 assumes it.
- **U4** Neither profile covers the stock fingertips (about 8 × 24 mm).
- **U5** Do not shrink casing width/thickness or the finger below 2.0 × 5.0: the audit shows those
  values are real.
- **U6 Polarized parts at 90° vs 270°** may need a 180° wrist turn from the single storage pick
  pose, past the ±0.8 rad cable limit. The mapper picks either. Unverified; ask the user before
  adding `allowed_orientations_deg`.

## Docs (last step)

Rewrite README "Solver" (staged K/window, cliques plus lifted cuts, topological order, status
meaning) and "Geometry" (G1/G2, caveats U1-U6), and add a 1-screen FINDINGS section with the
before/after bench table. Merge geometry.py and gripper.json together: until then, main's
dashboard draws v2 LED casings 4.0 mm thick with no protrusion (display only).

## Edge cases found

| case | type | action |
|---|---|---|
| Spurious jumpers (only `labels ≥ 0` is required, no flow) plus the objective/stop-first clash | mapper bug | P2 |
| Exact-pair cuts dodged by shifting; presolve redone per round; last round dies in presolve | mapper perf bug | P1, P5, P6, P9 |
| 1 worker never finds a solution (no feasibility-jump) | robustness | P1 (≥ 4 workers) |
| Spacing-1/2 parts must stand along rows: p3 cycle, p4 collector row between B and E, p5 RGB anodes → ≥ 1, 1, 2 jumpers | model limit (inherent) | P2. The netlist agent could prefer `_gap_1` variants to skip a row. |
| Supply nets with 6 pins (pl5): a half-strip fits ≤ 2 casings with fingers → chained jumpers | model limit | P2/P3, P11; rails |
| Power rails not in `board_connections.json` | model limit | later: needs rail offsets from the CAD (maybe off-grid) and dashboard hole names |
| 64 modelled rows vs 60 physical | input bug | R2 |
| Manual holes can land under a casing overhang or in a 2 mm gap between 20 mm casings | mapper bug / model limit | R1 |
| Manual pins as 640 bools each, interchangeable within a net | perf | P8 |
| Uncased kit parts: electrolytic cap, switch_spst/spdt, pushbutton, potentiometer, diode (buzzers excluded) → ERROR | input / kit gap | R3; netlist agent avoids them or marks them manual |
| Substitutions (330 → 220 Ω, S8050 → generic NPN), unknown LED colour or logic voltage | input issue | netlist agent records assumptions |
| Pin order trusted from the inventory (S8050 is E-B-C, real RGB is often R-K-G-B) | input issue: wrong wiring, not a collision | verify the part before building |
| Casing overhang past the board ends allowed; board-edge keep-outs not modelled | model limit | P3 centring avoids it |
| Unconnected pins reserve a whole strip each | by design (safe) | none |
| Full-open-only release, 100 % grip force, rgb_led_s2 1.9 mm contact | hardware / CAD | G3, U1-U3, G2 reject |
| Polarized 90°/270° vs the wrist-3 limit | unverified execution limit | U6, ask |
| Dashboard re-derives shapes from `mapping.gripper` with its own geometry.py | contract | G1 keeps the old key path; merge both files together |
