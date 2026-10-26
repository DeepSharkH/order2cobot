"""Protoboard holes from the CAD: labels, nets, placements, fit (agents/vision/board_grid.py)."""

from collections import Counter
from pathlib import Path

import numpy as np
import pytest

from agents.vision.board_grid import PITCH, BoardGrid

STEP = Path(__file__).resolve().parents[2] / "tmp/HalfBreadboard.STEP"
pytestmark = pytest.mark.skipif(not STEP.exists(), reason="half-breadboard STEP not present")
L = 0.08301


@pytest.fixture(scope="module")
def grid() -> BoardGrid:
    return BoardGrid(str(STEP), L, 2)


def test_holes_and_nets(grid):
    assert Counter(h.kind for h in grid.holes) == {"terminal": 600, "rail": 200}
    assert len(grid.index) == 800
    assert Counter(Counter(h.net for h in grid.holes).values()) == {5: 120, 25: 8}
    a1, a2 = np.array(grid["H1:A1"].uv), np.array(grid["H1:A2"].uv)
    assert np.linalg.norm(a2 - a1) == pytest.approx(PITCH, abs=1e-5)
    assert grid["H1:A1"].net == grid["H1:E1"].net != grid["H1:F1"].net


def test_orientation_relabels_only(grid):
    flipped = grid.relabel(flip_u=True, flip_v=True)
    assert np.allclose(np.sort(flipped.uv, axis=0), np.sort(grid.uv, axis=0))
    # turned half a turn: H1:A1 of one is H2:J30 of the other, at the same place
    assert np.allclose(flipped["H1:A1"].uv, grid["H2:J30"].uv)


def test_placements_respect_nets_and_gaps(grid):
    pl = grid.placements([-PITCH, PITCH])  # jumper_s2
    sets = {frozenset(p["holes"]) for p in pl}
    assert frozenset({"H1:A1", "H1:A3"}) in sets
    assert frozenset({"H1:A1", "H1:C1"}) not in sets  # same strip: shorts the part
    assert not any(p["holes"][0][:2] != p["holes"][1][:2] for p in pl)  # never across the halves
    assert all(p["residual"] < 0.0003 for p in pl)
    taken = grid.placements([-PITCH, PITCH], occupied=["H1:A3"])
    assert frozenset({"H1:A1", "H1:A3"}) not in {frozenset(p["holes"]) for p in taken}


def test_fit_recovers_offset_and_index(grid):
    rng = np.random.default_rng(1)
    seen = grid.uv[rng.choice(len(grid.uv), 150, replace=False)]
    true = np.array([0.0031, -0.0018, np.radians(0.6)])  # > 1 pitch along u: index must be found
    c, s = np.cos(true[2]), np.sin(true[2])
    shifted = (seen - true[:2]) @ np.array([[c, -s], [s, c]])  # where the grid was believed to be
    shifted += rng.normal(0, 0.00008, shifted.shape)
    r = grid.fit(shifted)
    assert r["matched"] > 0.95
    assert r["rms"] < 0.00015
    assert r["runner_up_score"] < r["score"] - 0.04  # one pitch off: the board ends and gaps
    # every hole back where it really is (turned or not: the pattern is symmetric)
    placed = grid.place_fitted(r)
    truth = (grid.uv - true[:2]) @ np.array([[c, -s], [s, c]])
    d = np.linalg.norm(placed[:, None] - truth[None], axis=2).min(1)
    assert d.max() < 0.0002
