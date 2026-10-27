"""The physical breadboard: hole id -> position in the board frame (mm), from the CAD hole table.

Source: dashboard/web/models/cad/breadboard.json (assets/models/cad/breadboard.py), frame =
centre of the top face, x across the columns a -> j, y toward row 1, z up. The solver
(src/circuit_mapper/board_connections.json) names the same terminal holes 'A1'..'J64'; the plan
only carries hole ids, and their positions always come from here, never from the plan's
center_mm (that is in the solver's pitch geometry).

TODO(board model, user 2026-10-27): the real board has **63 rows** (user-checked). The
committed CAD (60 rows) is wrong, the solver's 64 too; the e1->f1 channel is still to be
measured (CAD 7.62 mm = BB830, solver 10.16 mm = F at 8 pitches). In order, by hand:
  1. CAD: assets/models/cad/breadboard.py -> 63 rows and the measured channel, regenerate
     dashboard/web/models/cad/breadboard.{json,glb} (an uncommitted 63-row update exists in the
     main checkout: take that one, do not redo it).
  2. Solver: src/circuit_mapper/board_connections.json -> rows 1..63 and the channel
     (branch csp-overnight queues the 63 rows); simulation/world.yaml to match.
  3. Corner annotations: the corner holes named in the CAD / dashboard (A1, J1, A63, J63) and
     the corners taught in configs/breadboard/calibration.yaml must use the fixed hole table.
  4. Hole -> base transform: teach the corner inserts on the real board, `teach fit`, check the
     residual (a wrong channel shows as ~2.5 mm on the f-j side); re-fit after any CAD change.
Until then `check_plan` refuses holes the CAD does not have (rows 61-63 today).
"""
import json
import re
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
CAD = ROOT / 'dashboard' / 'web' / 'models' / 'cad' / 'breadboard.json'

_TERMINAL = re.compile(r'^([A-Ja-j])(\d+)$')


def load(path=CAD):
    return json.loads(Path(path).read_text())


def cad_name(hole):
    """Solver id ('E12') -> CAD feature name ('e12'); rail names ('L+1') pass through."""
    m = _TERMINAL.match(hole)
    return f'{m.group(1).lower()}{int(m.group(2))}' if m else hole


class Board:
    def __init__(self, spec=None):
        self.spec = spec or load()
        self.holes = {k: np.array(v, float) for k, v in self.spec['holes'].items()}
        self.rows = self.spec['rows']

    def hole(self, hole):
        """Board-frame position (mm) of a hole given as solver id or CAD name."""
        name = cad_name(hole)
        if name not in self.holes:
            raise KeyError(f'{hole}: not on the CAD board ({self.rows} rows)')
        return self.holes[name]

    def part(self, holes):
        """Where a centre-gripped part goes: the midpoint of its holes (board mm, on the top
        face) and the direction from its first to its last hole (board frame, unit)."""
        P = np.array([self.hole(h) for h in holes])
        d = P[-1] - P[0]
        return P.mean(0), d / np.linalg.norm(d) if np.linalg.norm(d) > 0 else np.array([1.0, 0, 0])

    def check_plan(self, plan):
        """Holes of the plan that the CAD board does not have (empty = all placeable)."""
        return [h for s in plan['steps'] for h in s['holes'] if cad_name(h) not in self.holes]
