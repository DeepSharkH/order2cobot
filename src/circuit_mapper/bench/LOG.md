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
