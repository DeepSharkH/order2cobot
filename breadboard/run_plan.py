"""Executor for an approved Circuit Assembly plan (skeleton).

  python -m breadboard.run_plan --dry-run      # targets of every step of the current plan, no robot

Reads data/circuit_designs/current.json -> plan.json (the CSP solver's verified assembly_order;
follow it exactly, never reorder) and reports progress to the dashboard
(POST /api/circuit/<id>/run: step, step_status, phase in pick/move/insert/verify, note), which
the Circuit Assembly build view shows.

Per step (TODOs mark what is not built yet):
  1. pick     TODO(pick): storage pose per casing (taught workspace poses, calibration.yaml) and
              the grasp (Hand-E via the bridge smart skills: full stroke, see breadboard/NOTES.md).
  2. grasp    TODO(grasp calibration): breadboard.grasp_calib.measure -> TCP for this grasp.
  3. move     move_checked to SAFE over the target (Placement.flange_target(holes, above=SAFE),
              in targets(); move_checked keeps its 0.10 m clearance), then a straight movel down
              to ABOVE inside the insert step; the wrist-3 cable limit applies (memory
              ur3-cable-limits). TODO(flip): symmetric parts take Placement.flip_for(holes),
              polarised ones (LED, BJT) keep the plan's pin order (needs polarity in the plan).
  4. insert   TODO(insert): straight z only (user rule): spiral search inside the hole funnel,
              then urcore seat_z (z-only force cap) and release; never x/y force mode.
  5. verify   TODO(verify): seat depth from seat_z, then camera / continuity check.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

from breadboard.board import Board
from breadboard.calibration import Placement

ROOT = Path(__file__).resolve().parent.parent
DESIGNS = ROOT / 'data' / 'circuit_designs'
APP, CLIENT = 'breadboard_insert', 'breadboard-insert'   # own bridge app/client (move tier granted once)
SAFE = 0.10                                                # m above the seated pose: move_checked target
ABOVE = 0.03                                               # m above the seated pose: start of the straight descent


def current_plan():
    if not (DESIGNS / 'current.json').exists():
        sys.exit('no approved plan: press "Start assembly" in the Circuit Assembly tab first')
    cur = json.loads((DESIGNS / 'current.json').read_text())
    return json.loads(Path(cur['plan']).read_text())


def targets(plan, placement):
    """Per step: holes, the flange poses SAFE and ABOVE over the seated target."""
    return [{'index': s['index'], 'item': s['item'], 'holes': s['holes'], 'casing': s['casing'],
             'flange_safe': placement.flange_target(s['holes'], above=SAFE),
             'flange_above': placement.flange_target(s['holes'], above=ABOVE)} for s in plan['steps']]


def run(plan):
    raise NotImplementedError('TODO(executor): pick -> grasp calibration -> move -> insert -> verify per step')


def main():
    ap = argparse.ArgumentParser(prog='python -m breadboard.run_plan')
    ap.add_argument('--dry-run', action='store_true', help='print the targets, no robot')
    a = ap.parse_args()
    plan = current_plan()
    missing = Board().check_plan(plan)
    if missing:
        sys.exit(f'holes not on the CAD board: {missing} (board model TODO in breadboard/board.py)')
    if a.dry_run:
        for t in targets(plan, Placement()):
            print(t['index'], t['item'], t['holes'], np.round(t['flange_above'][:3], 4))
        return
    run(plan)


if __name__ == '__main__':
    main()
