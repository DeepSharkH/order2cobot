"""Solver-side checks of the CP-SAT mapper (parameters, search behaviour)."""

import sys
from pathlib import Path

from ortools.sat.python import cp_model

ROOT = Path(__file__).parents[1] / "src/circuit_mapper"
sys.path.insert(0, str(ROOT))
import mapper  # noqa: E402


def small_case():
    board = mapper.read_json(ROOT / "board_connections.json")
    board["conductive_groups"] = board["conductive_groups"][:4]
    inventory = mapper.read_json(ROOT / "example_components.json")
    circuit = {
        "circuit_id": "parallel_resistors",
        "components": [
            {"id": "R1", "part_id": "resistor_1kohm_gap_0"},
            {"id": "R2", "part_id": "resistor_1kohm_gap_0"},
        ],
        "nets": [{"id": "a", "pins": ["R1.1", "R2.1"]}, {"id": "b", "pins": ["R1.2", "R2.2"]}],
    }
    return circuit, inventory, board


FIELDS = (
    "num_workers",
    "num_search_workers",
    "cp_model_probing_level",
    "max_presolve_iterations",
    "stop_after_first_solution",
    "max_time_in_seconds",
)


def test_solver_parameters(monkeypatch):
    seen = []
    original = cp_model.CpSolver.solve

    def recording_solve(self, *args, **kwargs):
        seen.append({name: getattr(self.parameters, name) for name in FIELDS})
        return original(self, *args, **kwargs)

    monkeypatch.setattr(cp_model.CpSolver, "solve", recording_solve)
    circuit, inventory, board = small_case()
    result = mapper.map_circuit(circuit, inventory, board, time_limit=10)
    assert set(result["assembly_order"]) == {"R1", "R2"}
    assert seen
    for params in seen:
        # 1 worker has no feasibility-jump; the deprecated field stays unset.
        assert params["num_workers"] >= 4 and params["num_search_workers"] == 0
        assert params["cp_model_probing_level"] == 0
        assert params["max_presolve_iterations"] == 1
        assert params["stop_after_first_solution"]
        assert 0 < params["max_time_in_seconds"] <= 10
