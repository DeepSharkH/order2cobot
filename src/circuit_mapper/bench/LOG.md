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
