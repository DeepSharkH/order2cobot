# CSP overnight log: one entry per candidate, newest last

## P1 solver parameters - KEPT (cbaac8d)
- change: `num_workers=8` (replaces the deprecated `num_search_workers`), `cp_model_probing_level=0`, `max_presolve_iterations=1`; profile_model.py `--workers` now sets `num_workers`.
- A/B at 60 s, s per lazy round: probing 0 + 1 iteration 1.4-3.7, probing 1 2.5-10.0, probing 0 + 3 iterations 2.3-5.4. None of the three solved p3-p5, so the fastest round won.
- bench p1 vs baseline (wall s, it; fast instances are the median of 3 runs: p1, p1_rep2, p1_rep3):
  - p1 OPTIMAL 27.8 → 7.2 s, it 4 → 4 (2-4), 0 jumpers; led_circuit 28.1 → 7.5 s, it 4 → 2-5
  - parallel_resistors 8.6 → 6.1 s (runs: 5.1, 6.1, 11.8 with 11 rounds), 0 jumpers
  - p2 UNKNOWN → OPTIMAL 6.4 s, it 15 → 3; rc_low_pass UNKNOWN → OPTIMAL 6.2 s, it 14 → 3; both 0 jumpers
  - p3/p4/p5 still UNKNOWN, but 54/35/26 rounds instead of 9/8/7; parallel_leds_5 still UNKNOWN (26 s per round, about as before); others unchanged
- why kept: all solved instances verify, nothing regressed, rounds are 4-5× faster (6.5-17 s → 1.0-4.6 s). PLAN thresholds: p2/rc ≤ 15 s met; p1/led ≤ 6 s and parallel_resistors ≤ 4 s missed because of the lazy-round count (2-11 rounds at about 1.0-1.4 s each), which is P5/P6's job.
- surprising: parallel_leds_5 once returned after 134.7 s (limit 120): its solves summed to 131 s. Two re-runs took 123.8 and 123.4 s and a profiled run 124.5 s, so this looks like a CP-SAT overrun under load. P9 (deadline guard) should look at it.

## P2 staged jumper budget, no dangling jumpers - KEPT (91f0606)
- change: `JUMPER_STAGES` = K 0, 1, 2, 3, 4, then any number. Each stage gets an equal share of the time left. K = 0 builds no wire or flow variables, collision cuts replay into later stages, and there is no objective. A chosen wire must carry nonzero flow of its strip's net. From K = 3 on, flow also runs uphill in a strip potential, so it cannot circulate. Unneeded jumpers are pruned before verify. `OPTIMAL` means every smaller K was proven INFEASIBLE.
- bench p2 vs p1. Fast instances are the median of 3 runs (p2, p2_rep2, p2_rep3), all OPTIMAL with 0 jumpers and ver = yes:
  - p1 7.2 → 4.1 s (it 4-6); led_circuit 7.5 → 3.5 s (it 3-6)
  - p2 6.4 → 2.2 s, it 3 → 1; rc_low_pass 6.2 → 2.1 s, it 3 → 1; parallel_resistors 6.1 → 1.6 s, it 1
  - p3/p4/p5/parallel_leds_5 still UNKNOWN (45/25/13/6 solves). K = 0 is proven INFEASIBLE in 2-5 s (pl5 22 s). At K = 1, p3/p4 layouts carry 1 jumper, but the point cuts never converge. pl5 now returns at 121.7 s (was 134.7). Others unchanged.
- why kept: the gate passes, and the P1 thresholds are now met (p1/led ≤ 6 s, parallel_resistors ≤ 4 s). P2's own "p3 and p4 solve" is not met, which is P5/P6's job.
- surprising: with nonzero flow alone, the any-K stage picked 100-180 wires in circulating cycles; the potential brought that down to 1-5. Potentials (even a direction bool replacing `flow != 0`) made K = 1 rounds 1.5-2× slower, so they start at K = 3 (a cycle needs 3 wires). Fixed cumulative slices gave p5's unprovable K = 1 stage 45 s, hence equal shares. p2_try_fast is an intermediate run without potentials.

## P3 row window, centred, widening - KEPT (bdad767)
- change: `STAGES` pairs each budget with a row window centred on the board: K 0, 1, 2, 4 in the first window (1.5× casing length per board half, 12-24 rows), K ≤ 4 in twice that, then any K on the whole board. Candidates outside the window are never built. Only whole-board INFEASIBLE proofs raise the OPTIMAL bound.
- diagonal (window widens with K), not nested: the nested K-major schedule (18 stages) cut every slice to about 7.5 s. p3-p5 never converged in it, and pl5 could not refute K = 0 at 48 or 64 rows in 7 s. The diagonal keeps 6 stages, and each stage's search space contains the previous one's.
- bench p3 vs p2. Fast instances are the median of 3 runs (p3, p3_rep2, p3_rep3) at load 16-21, against 14-16 for p2:
  - p1 4.1 → 1.6 s; p2 2.2 → 1.4 s; led_circuit 3.5 → 2.4 s (it 3-16); parallel_resistors 1.6 → 1.3 s; rc_low_pass 2.1 → 1.4 s
  - all OPTIMAL with 0 jumpers and ver = yes. Every layout lies inside the 12-row window (rows 27-38 of 64) and spans 2-3 rows (27..29 to 33..35).
  - p3/p4/p5/parallel_leds_5 still UNKNOWN (106/85/34/7 solves, against 45/25/13/6). Rounds cost 2-3× less, but point cuts still do not converge (p3 at K = 1 in 17 rows: 39 rounds in 30 s). Others unchanged.
- pl5: its 24-row K ≤ 4 stage gets a first candidate after 1.5 s, where 48 or 64 rows give none in 30 s. But that stage only starts about 50 s into the run, after the K = 1 and 2 stages, which cannot be proven infeasible.
- why kept: the gate passes, and the fast instances are 1.2-2.6× faster with compact, centred layouts.
- trade-off: on the 64-row board, a layout with jumpers is now FEASIBLE, never OPTIMAL. A whole-board K = 0 proof would cost 2-6 s for p3-p5, and could not be completed in 7 s for pl5.

## P4 drop the order ints; topological assembly order - KEPT (ca67270)
- change: no `order_*` ints or precedence constraints. `constrain_assembly` cuts forbidden pairs, then sorts the "A's fingers hit B, so A first" digraph with `graphlib`; ready steps go jumpers, slim casings, then LEDs, each by row. A cycle (3 or more steps) becomes one `add_bool_or` nogood. Models are 12-14 % smaller (p4_npn K = 1 window 4995 → 4394 vars, whole board 20672 → 18144).
- bench p4 vs p3. Fast instances are the median of 3 runs (p4, p4_rep2, p4_rep3), all OPTIMAL with 0 jumpers and ver = yes:
  - p1 1.6 → 1.5 s; p2 1.4 → 1.7 s; led_circuit 2.4 → 1.5 s (it 8/1/3); parallel_resistors 1.3 → 1.3 s; rc_low_pass 1.4 → 2.4 s (it 7/1/6)
  - p3/p4/p5/parallel_leds_5 still UNKNOWN: 76/45/29/7 solves at load 22. p4_heapq (an earlier hand-written sort, same cuts and order) gave 90/73/34/7 at load 16.5, against p3's 106/85/34/7. Others unchanged.
- A/B against the P3 code (same load, alternating runs): p4_npn did 74 old vs 81 new solves in 120 s, and K = 1 rounds took 0.8-1.4 s in both. So the order ints cost no measurable time per round. Round medians over 8 runs each (old → new): rc_low_pass 1 → 2.5, p2 1 → 1, led_circuit 8.5 → 2, p1 3.5 → 4.5 (125 vs 133 rounds in total). Which first layout CP-SAT returns depends on the model, so rc lost its lucky 1-round layout and led gained one. Every run stayed under 5 s.
- why kept: the gate passes. The model is smaller, every lazy cut is a clause, and the order is deterministic and readable (led_circuit gives R1, D1). Old order-only rounds were rare (1 in 9 fast runs, 2 of 74 in p4_npn) because forbidden pairs usually came in the same round. After P6 they would be the only lazy rounds left; now they cost none. Cost: geometry.py has 26 more lines, and there is no speed gain for now.

## R2 board has 63 rows (real board) - KEPT (7e6ba73)
- change: cherry-pick of a96089b (orders-overnight): board_connections.json drops row 64 (2 strips, 1 E-F pair), `rows.last` 63; E-F channel unchanged (still to be measured). test_mapper_csp: first window is now rows 26-37 of 63 (25 spare above, 26 below: the best an even window gets on an odd board); new test checks rows 1-63, 630 holes, 63 E-F pairs.
- bench r2_63rows vs p4. Fast instances are the median of 3 runs (r2_63rows, _rep2, _rep3) at load 12-21, against 16-22 for p4, all OPTIMAL with 0 jumpers and ver = yes:
  - p1 1.5 → 1.4 s; p2 1.7 → 1.4 s; led_circuit 1.5 → 1.7 s; parallel_resistors 1.3 → 1.5 s; rc_low_pass 2.4 → 1.5 s: noise
  - every layout lies in rows 26-35 (window 26-37); no hole past row 37, none in row 64
  - p3/p4/p5/parallel_leds_5 still UNKNOWN: 116/99/26/7 solves (p4: 76/45/29/7; load 12-15 at their start, so not comparable). Others unchanged.
- why kept: data fix confirmed by the user; the gate passes and nothing regressed. Becomes the new reference.

## P5 lift every cut to all translations - REVERTED (n/a; wire-key replay fix kept, 69e4a43)
- change tried: relation() cached by (kind A, kind B, centre offset in µm), kind = the boxes around the centre (a symmetric casing at 90° and 270° is one kind). A forbidden pair became one at-most-one per centre over the two cells (a cell's candidates share holes, so exact), wires included. Its unit test passed: every lifted pair is forbidden or shares a hole, and every pair with a lifted key is cut. Patch: scratchpad p5/p5_lift_full.patch.
- bench p5 vs r2_63rows. Fast instances are the median of 3 runs (p5, _rep2, _rep3) at load 11-23, all OPTIMAL with 0 jumpers and ver = yes:
  - p1 1.4 → 1.9 s, it 2 → 5 (7/5/2); led_circuit 1.7 → 2.3 s, it 2 → 7 (8/4/7); rc_low_pass 1.5 → 1.8 s; p2 and parallel_resistors 1 round
  - p3/p4/p5/parallel_leds_5 still UNKNOWN: 27/47/27/7 solves (r2: 116/99/26/7). Others unchanged.
- why reverted: target missed. Nothing converged, and rounds got slower. Each round finds 6-8 new conflict keys, but p3's 12-row window has 1,127 forbidden (kind pair, offset) keys for the resistor, LED and 4-pitch jumper alone (3,059 by casing and orientation). Each key costs 60-140 clauses per window, and replay lifts every key again: on p3, the K = 1 window went from 6.8k to 16.4k constraints (2.1 s per round, was about 1 s), the 2× window had 48k (12.7 s) and the whole board 97k (22 s per round).
- kept: replay keys a wire by its holes (P3 follow-up). `jumper:N` depends on the window, so widening used to drop the wire cuts. test_replay_finds_wires_by_holes fails on the old key. Bench p5_wirekeys (load 19-22): the same 5 OPTIMAL and verified, p3/p4/p5/pl5 UNKNOWN (81/92/34/7 solves).
- surprising: p5_wirekeys ran r2's exact code path for p1 and drew 8 rounds, so fast-instance round counts of 1-8 are first-layout luck. P6 needs the eager per-candidate × partner-part cliques; per-key cells are too many constraints.

## P6 eager no-fit cliques against robot-placed parts - KEPT (eaa5d8d)
- change: each stage model posts, per candidate and per other robot part B, `add_at_most_one(candidate + B's candidates that relation() forbids with it)`: a clique, because B is exactly-one. Part pairs go from the lower owner, wires against every part; wire-wire pairs and order cycles stay lazy. Forbidden centre offsets are tabulated per pair of kinds with relation() itself (µm, cached across stages). Posting stops at the stage end, so the deadline holds (pl5 and p3 at `--time-limit 30`: 32.0 and 31.3 s, start-up included). The randomized test checks clique membership == relation() on > 10^5 pairs, at margin 0 and 0.3.
- bench p6 vs r2_63rows. Fast instances are the median of 3 runs (p6, _rep2, _rep3) at load 14-19, against 12-21; all OPTIMAL, 0 jumpers, ver = yes:
  - rounds: p1 2/1/6 → 1/1/1, led_circuit 2/6/2 → 1/1/1, rc_low_pass 1/1/4 → 1/1/1; p2 and parallel_resistors 1 → 1. Worst wall 2.4-2.6 → 1.6 s; medians 1.4-1.7 → 1.4-1.5 s (noise). One-round solves pay 0.1-0.2 s for the cliques (map_circuit 0.13-0.20 → 0.22-0.38 s).
  - p3/p4/p5/parallel_leds_5 still UNKNOWN, with 6/7/6/6 solves instead of 116/99/26/7: one per stage, no thrash. Others unchanged.
- why kept: the gate passes, part-part cuts per round are 0, and every fast instance takes 1 round. Missed: rounds ≤ 2 on p3-p5, which now find no layout at all.
- surprising: exact models settle the window stages. K = 1 in the 12-row window is proven INFEASIBLE for p3/p4/p5 in 18/12/19 s, and pl5's K = 0 in 6.5 s (was 22 s). But K = 2 and 4 find nothing in their 24 s. Clique literals: 0.13-0.33M in the 12-row window, 1.4M on the whole board (p3-p5), 6.7M for pl5 (10 s build). Part-only cliques (the PLAN variant, wires lazy) bring the thrash back: p3 K = 2 and p4 K = 1 find layouts in 2-7 s, with 2-4 new wire-part cuts every round.
- next: with 240 s on the whole board, p3 got its first exact layout at 109 s, but with 122 colliding wires (485 wire-wire cuts). So the open problems are first-solution search (P10 symmetry; probing back on, since presolve now runs once per stage) and the wire count in the any-K stage.

## S1 bench on the real stock (Jan's c77b847) - baseline_stock (39bd642)
- change: the inventory is the real stock (mostly 1 piece per part; LEDs red/green 4-pitch and yellow 1-pitch, no RGB, no 220 Ω, no transistor; jumpers 1× s1, 2× s2, 1× s3, 1× s4) and the supply is fixed at C1 (+) / H1 (−). bench.py died on a KeyError. p1/p2 take stock part ids; p3 = two 330 Ω + red and green LED; p5 = three inputs, each driving its own LED (no RGB in stock); parallel_leds_5 → parallel_leds_3 (3 LEDs in stock); p4 dropped (no transistor).
- baseline_stock at 120 s (load 9-10): p1 FEASIBLE 2.2 s, 1 jumper (the C1/H1 supply forces one even for one LED); p2 OPTIMAL 1.5 s; led_circuit FEASIBLE 2.2 s, 1 jumper; parallel_resistors 1.3 s, rc_low_pass 1.4 s OPTIMAL; p3, p5, parallel_leds_3 UNKNOWN (121.6-121.8 s, 6 solves). p5 re-run on clean HEAD (baseline_stock_p5): the same.

## P13 stock jumpers as pieces: every wire collision eager - KEPT
- change: with every jumper part stocked (int quantity), each piece is an owner like a part: one at-most-one over its placements, so no_fit_cliques covers wire-part and wire-wire pairs (only order cycles stay lazy). Identical pieces are ordered (second used only after the first, at a later placement). Unlimited jumpers (the test fixture) keep the old one-owner-per-wire path.
- effect: proofs become possible. p3 is proven INFEASIBLE on the whole board in 80 s (6 solves, the any-K stage 28.6 s, 13.9k vars / 72k constraints), where the lazy model returned UNKNOWN. p1/led_circuit 2.2 → 1.3 s alone, 2.4 s in the bench at load 19.
- why kept: gate passes (pieces_capacity: all five fast instances verified, 1.3-2.4 s at load 19-20 vs 9-10), and UNKNOWN turns into a proof.

## P14 strip capacity bound: 3 robot pins on one net is refused at once - KEPT
- finding: two casings with a pin in the same 5-hole strip need ≥ 3 columns between them (resistor, jumper, LED; LED-LED 4), and a casing bridging the E-F channel leaves room only for one at A or B. So a strip half holds at most 2 casing pins. A net on k + 1 strips needs ≥ k jumpers, which fill 2k of the 2(k + 1) slots: **a net never takes more than 2 robot-placed pins, whatever the jumper count.** Rigid jumpers add exactly the room they use up.
- change: before any search, `strip_capacity()` solves the exact max-packing of one middle strip per board half (pairwise relation(), tiny CP-SAT, ~0.5 s); if it is ≤ 2, a net with > 2 robot-placed pins raises INFEASIBLE with the net named. Skipped when a part has two pins on one net (it would join strips itself).
- bench pieces_capacity vs baseline_stock: p5 UNKNOWN 121.6 s → INFEASIBLE 1.8 s (GND: 3 LED cathodes); parallel_leds_3 UNKNOWN 121.8 s → INFEASIBLE 1.6 s (VCC 3 resistors, GND 3 cathodes); p3 → INFEASIBLE 80.1 s (P13; its nets have 2 robot pins each, so the bound does not apply). Fast five unchanged.
- what 3 per strip would need (offset 2 = columns A, C, E): casings ≤ 2.5 mm thick with today's 2 mm finger and 0.5 mm opening, or 3.0 mm with a 1.5 mm finger. Not possible for 5 mm LEDs.

## W1 what-ifs on p3 / p5 / parallel_leds_3 (sandbox copies, nothing committed)
- p3 with 0.5 mm opening, without the 6.2 mm LED band, and with both: still proven INFEASIBLE (81-83 s). The geometry relaxations of geometry_audit.md do not rescue it.
- p3 with the supply moved to C32/H32 (centred window): INFEASIBLE up to K = 4 in the 12-row window, UNKNOWN beyond. The row-1 supply is not the only problem: every net of p3 fills each strip it touches.
- p3 with 10 of each jumper: UNKNOWN (766k constraints in the whole-board stage).
- hand-placed wires (no casing, no grip, only two holes; operator plugs them after the robot, like the supply), lengths 1-8 along a row or column, normal stages: p3 FEASIBLE with 4 hand wires (48 s), p5 FEASIBLE with 2 (53 s), parallel_leds_3 UNKNOWN at 120 s (a single whole-board stage found one with 46 wires in 55 s; needs a wire objective or tighter windows).
