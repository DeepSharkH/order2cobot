"""Solver-side checks of the CP-SAT mapper (parameters, search behaviour)."""

import json
import sys
from pathlib import Path

import pytest
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


def six_pin_case():
    """Net A has six pins but a strip only five holes: exactly one jumper is needed."""
    board = mapper.read_json(ROOT / "board_connections.json")
    board["conductive_groups"] = board["conductive_groups"][:16]
    inventory = mapper.read_json(ROOT / "example_components.json")
    connectors = [{"id": f"J{i}", "part_id": "power_connector_2pin"} for i in (1, 2, 3)]
    circuit = {
        "circuit_id": "six_pin_net",
        "components": [{"id": "R1", "part_id": "resistor_220ohm_gap_0"}, *connectors],
        "nets": [
            {
                "id": "A",
                "pins": ["R1.1", "J1.positive", "J1.negative", "J2.positive", "J2.negative"]
                + ["J3.positive"],
            },
            {"id": "B", "pins": ["R1.2", "J3.negative"]},
        ],
    }
    return circuit, inventory, board


def has_wires(model):
    return any(v.name.startswith(("wire_", "link_", "flow_")) for v in model.proto.variables)


def assert_every_jumper_needed(result, nets, groups):
    for jumper in result["jumpers"]:
        rest = [j for j in result["jumpers"] if j is not jumper]
        with pytest.raises(ValueError, match="disconnected"):
            mapper.verify_connections({**result, "jumpers": rest}, nets, groups)


def test_zero_jumper_stage_builds_no_wires(monkeypatch):
    models = []
    original = cp_model.CpSolver.solve

    def recording_solve(self, model, *args, **kwargs):
        models.append(has_wires(model))
        return original(self, model, *args, **kwargs)

    monkeypatch.setattr(cp_model.CpSolver, "solve", recording_solve)
    circuit, inventory, board = small_case()
    result = mapper.map_circuit(circuit, inventory, board, time_limit=10)
    assert result["status"] == "OPTIMAL" and result["jumpers"] == []
    assert models and not any(models)


def test_minimal_jumpers_are_optimal_and_needed():
    circuit, inventory, board = six_pin_case()
    result = mapper.map_circuit(circuit, inventory, board, time_limit=20)
    assert result["status"] == "OPTIMAL" and len(result["jumpers"]) == 1
    assert result["jumpers"][0]["id"] in result["assembly_order"]
    _, groups = mapper.board_geometry(board)
    assert_every_jumper_needed(result, circuit["nets"], groups)


def test_unproven_smaller_budget_is_only_feasible(monkeypatch):
    original = cp_model.CpSolver.solve

    def zero_stage_times_out(self, model, *args, **kwargs):
        if not has_wires(model):
            return cp_model.UNKNOWN
        return original(self, model, *args, **kwargs)

    monkeypatch.setattr(cp_model.CpSolver, "solve", zero_stage_times_out)
    circuit, inventory, board = six_pin_case()
    result = mapper.map_circuit(circuit, inventory, board, time_limit=20)
    assert result["status"] == "FEASIBLE" and len(result["jumpers"]) == 1


def test_any_budget_stage_returns_only_needed_jumpers(monkeypatch):
    seen = []
    original = cp_model.CpSolver.solve

    def recording_solve(self, model, *args, **kwargs):
        seen.append(any(v.name.startswith("potential_") for v in model.proto.variables))
        return original(self, model, *args, **kwargs)

    monkeypatch.setattr(cp_model.CpSolver, "solve", recording_solve)
    monkeypatch.setattr(mapper, "JUMPER_STAGES", (None,))
    circuit, inventory, board = six_pin_case()
    result = mapper.map_circuit(circuit, inventory, board, time_limit=20)
    # Nothing smaller was tried, so nothing is proven.
    assert result["status"] == "FEASIBLE" and result["jumpers"]
    assert seen and all(seen)
    _, groups = mapper.board_geometry(board)
    assert_every_jumper_needed(result, circuit["nets"], groups)


def test_bench_jumpers_are_all_needed():
    """Committed bench layouts: removing any one returned jumper disconnects a net."""
    _, groups = mapper.board_geometry(mapper.read_json(ROOT / "board_connections.json"))
    rows = [
        row
        for path in sorted((ROOT / "bench/results").glob("*.json"))
        for row in json.loads(path.read_text())["instances"]
        if row.get("mapping", {}).get("jumpers")
    ]
    if not rows:
        pytest.skip("no bench result uses jumpers")
    for row in rows:
        circuit = mapper.read_json(ROOT / row["file"])
        nets = circuit["nets"] + [
            {"id": f"UNCONNECTED:{pin}", "pins": [pin]}
            for pin in circuit.get("unconnected_pins", [])
        ]
        mapper.verify_connections(row["mapping"], nets, groups)
        assert_every_jumper_needed(row["mapping"], nets, groups)
