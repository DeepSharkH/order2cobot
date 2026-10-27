"""Solver-side checks of the CP-SAT mapper (parameters, search behaviour)."""

import json
import random
import sys
from pathlib import Path

import pytest
from ortools.sat.python import cp_model

ROOT = Path(__file__).parents[2] / "src/circuit_mapper"
sys.path.insert(0, str(ROOT))
import geometry  # noqa: E402
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
    monkeypatch.setattr(mapper, "STAGES", ((None, None),))
    circuit, inventory, board = six_pin_case()
    result = mapper.map_circuit(circuit, inventory, board, time_limit=20)
    # Nothing smaller was tried, so nothing is proven.
    assert result["status"] == "FEASIBLE" and result["jumpers"]
    assert seen and all(seen)
    _, groups = mapper.board_geometry(board)
    assert_every_jumper_needed(result, circuit["nets"], groups)


def record_models(monkeypatch):
    models = []
    original = cp_model.CpSolver.solve

    def recording_solve(self, model, *args, **kwargs):
        models.append(model)
        return original(self, model, *args, **kwargs)

    monkeypatch.setattr(cp_model.CpSolver, "solve", recording_solve)
    return models


def used_rows(result):
    holes = [*result["pin_to_hole"].values(), *result["manual_connections"].values()]
    return {int(hole[1:]) for jumper in result["jumpers"] for hole in jumper["pins"].values()} | {
        int(hole[1:]) for hole in holes
    }


def test_first_window_is_centred(monkeypatch):
    """Two 14 mm casings give the 12-row minimum: rows 26-37 of 63, 25 spare above, 26 below."""
    models = record_models(monkeypatch)
    board = mapper.read_json(ROOT / "board_connections.json")
    inventory = mapper.read_json(ROOT / "example_components.json")
    circuit = mapper.read_json(ROOT / "bench/circuits/p1_red_led_1k_series.json")
    result = mapper.map_circuit(circuit, inventory, board, time_limit=20)
    window = set(range(26, 38))
    assert result["status"] == "OPTIMAL" and used_rows(result) <= window
    # Candidates, not just the answer, stay inside: 12 rows of 10 holes per manual pin.
    manual = [v.name for v in models[0].proto.variables if v.name.startswith("manual_")]
    assert len(manual) == 2 * 12 * 10
    assert {int(name.rsplit("_", 1)[1][1:]) for name in manual} == window


def test_board_has_63_rows():
    """The real board ends at row 63: two 5-hole strips and one E-F pair per row."""
    board = mapper.read_json(ROOT / "board_connections.json")
    positions, _ = mapper.board_geometry(board)
    assert {int(hole[1:]) for hole in positions} == set(range(1, 64))
    assert len(positions) == 63 * 10 and len(board["center_gap_pairs"]) == 63


def test_too_small_window_widens(monkeypatch):
    models = record_models(monkeypatch)
    monkeypatch.setattr(mapper, "FIRST_WINDOW_ROWS", (1, 1))
    circuit, inventory, board = small_case()
    result = mapper.map_circuit(circuit, inventory, board, time_limit=20)
    # One row holds no standing resistor: that stage has no options instead of an error.
    assert not any(v.name.startswith("R1_") for v in models[0].proto.variables)
    assert set(result["assembly_order"]) >= {"R1", "R2"} and used_rows(result) == {1, 2}


def test_window_infeasibility_is_no_proof(monkeypatch):
    """K = 0 fails in the 4-row window, but only a whole-board proof makes 1 jumper optimal."""
    monkeypatch.setattr(mapper, "FIRST_WINDOW_ROWS", (4, 4))
    circuit, inventory, board = six_pin_case()
    result = mapper.map_circuit(circuit, inventory, board, time_limit=20)
    assert result["status"] == "FEASIBLE" and len(result["jumpers"]) == 1
    assert used_rows(result) <= {3, 4, 5, 6}


def box_step(model, name, body, finger):
    """A step with a box body and one finger box; relation() reads nothing else."""
    shape = {"body": body, "fingers": [finger]}
    return {"id": name, "owner": 0, "chosen": model.new_bool_var(name), "shape": shape}


def test_precedence_cycle_gets_one_nogood():
    """A's fingers hit B, B's hit C and C's hit A: no order exists, so one nogood cuts it."""
    model = cp_model.CpModel()
    boxes = [(0, 0, 1, 1), (10, 0, 11, 1), (20, 0, 21, 1)]
    a, b, c = (box_step(model, n, boxes[i], boxes[(i + 1) % 3]) for i, n in enumerate("ABC"))
    assert geometry.relation(a["shape"], b["shape"], 0) == "a_first"
    assert geometry.relation(c["shape"], a["shape"], 0) == "a_first"
    known = set()
    assert geometry.constrain_assembly(model, [a, b, c], known, 0) is None
    (nogood,) = model.proto.constraints
    # A negated literal is -index - 1 in the proto.
    assert sorted(nogood.bool_or.literals) == sorted(-s["chosen"].index - 1 for s in (a, b, c))
    # The same cycle again (a replayed layout) adds nothing.
    assert geometry.constrain_assembly(model, [c, a, b], known, 0) is None
    assert len(model.proto.constraints) == 1
    # Without C nothing cycles: A goes first, and that order verifies.
    order = geometry.constrain_assembly(model, [b, a], known, 0)
    assert order == [a, b] and len(model.proto.constraints) == 1
    geometry.verify_assembly(order, 0)


def test_ready_steps_follow_priority():
    model = cp_model.CpModel()
    far = [
        box_step(model, n, (30 * i, 0, 30 * i + 1, 1), (30 * i, 5, 30 * i + 1, 6))
        for i, n in enumerate("XYZ")
    ]
    order = geometry.constrain_assembly(model, far, set(), 0, lambda s: "ZXY".index(s["id"]))
    assert [s["id"] for s in order] == ["Z", "X", "Y"]


def test_no_order_variables(monkeypatch):
    models = record_models(monkeypatch)
    circuit, inventory, board = six_pin_case()
    result = mapper.map_circuit(circuit, inventory, board, time_limit=20)
    # The returned order passed verify_assembly inside map_circuit.
    assert set(result["assembly_order"]) == {p["id"] for p in result["placements"]}
    assert models and not any(
        v.name.startswith("order_") for model in models for v in model.proto.variables
    )


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


def test_replay_finds_wires_by_holes(monkeypatch):
    """A wire's ID depends on the window; the wider stage still replays the wire's cuts."""
    monkeypatch.setattr(mapper, "STAGES", ((1, 1), (2, 1)))
    monkeypatch.setattr(mapper, "FIRST_WINDOW_ROWS", (4, 4))
    checked, replays = [], []
    original_check = mapper.constrain_assembly
    original_solve = cp_model.CpSolver.solve

    def check(model, selected, known, margin, priority=None):
        if priority is None:
            replays.append(selected)
        else:
            checked.append(selected)
            if len(checked) == 1:
                return None  # As if it collided: the stage solves again.
        return original_check(model, selected, known, margin, priority)

    def solve(self, model, *args, **kwargs):
        if len(checked) == 1 and not replays:
            return cp_model.UNKNOWN  # The first stage runs out of time.
        return original_solve(self, model, *args, **kwargs)

    monkeypatch.setattr(mapper, "constrain_assembly", check)
    monkeypatch.setattr(cp_model.CpSolver, "solve", solve)
    circuit, inventory, board = six_pin_case()
    result = mapper.map_circuit(circuit, inventory, board, time_limit=20)
    assert len(result["jumpers"]) == 1
    first, (replayed,) = checked[0], replays
    assert any(step["id"].startswith("jumper:") for step in first)
    assert sorted(s["holes"] for s in replayed) == sorted(s["holes"] for s in first)


def window_candidates(model, part_ids, rows):
    """Candidates of the mapper's form in rows: one owner per part, then one per wire
    (shortest and longest jumper)."""
    board = mapper.read_json(ROOT / "board_connections.json")
    inventory = mapper.read_json(ROOT / "example_components.json")["components"]
    parts = {p["part_id"]: p for p in inventory}
    casings = mapper.read_json(mapper.DEFAULT_CASINGS)
    gripper = mapper.read_json(ROOT / "gripper.json")
    pitch = board["geometry"]["hole_pitch_mm"]
    positions, _ = mapper.board_geometry(board)
    inside = {hole: xy for hole, xy in positions.items() if int(hole[1:]) in rows}
    candidates = []
    for owner, part_id in enumerate(part_ids + ["jumper_rigid_1_pitch", "jumper_rigid_4_pitch"]):
        part = parts[part_id]
        casing = geometry.casing_for(part, casings, pitch)
        for holes in mapper.placements(part, inside):
            wire = part["type"] == "jumper"
            candidates.append(
                {
                    "owner": len(part_ids) + len(candidates) if wire else owner,
                    "chosen": model.new_bool_var(f"c{len(candidates)}"),
                    "shape": geometry.placement_geometry(holes, positions, casing, gripper, pitch),
                }
            )
    return candidates


@pytest.mark.parametrize("margin", [0, 0.3])
def test_no_fit_cliques_are_relation(margin):
    """Randomized: each clique holds x and exactly the candidates of one part that relation()
    forbids with x. Pairs of parts are covered from the lower owner, wires against every part."""
    rng = random.Random(6)
    model = cp_model.CpModel()
    parts = ["resistor_220ohm_gap_0", "led_red_gap_0", "led_red_gap_0", "transistor_npn_generic"]
    parts.append("led_rgb_common_cathode_gap_0")
    candidates = window_candidates(model, parts, range(26, 34))
    owners = {c["chosen"].index: c["owner"] for c in candidates}
    by_part = [[c for c in candidates if c["owner"] == part] for part in range(len(parts))]
    found = {}
    for x, *rest in geometry.no_fit_cliques(candidates, len(parts), margin, {}):
        (part,) = {owners[b.index] for b in rest}
        assert (x.index, part) not in found
        found[x.index, part] = {b.index for b in rest}
    pairs = 0
    for x in rng.sample(candidates, 250):
        own = x["owner"]
        for part in range(own + 1 if own < len(parts) else 0, len(parts)):
            pairs += len(by_part[part])
            expected = {
                b["chosen"].index
                for b in by_part[part]
                if geometry.relation(x["shape"], b["shape"], margin) == "forbidden"
            }
            assert found.get((x["chosen"].index, part), set()) == expected
    assert pairs > 10**5 and len(found) > 1000


def test_part_collisions_are_eager(monkeypatch):
    """Two parts and no wires: the cliques leave the lazy loop nothing, so one solve."""
    models = record_models(monkeypatch)
    board = mapper.read_json(ROOT / "board_connections.json")
    inventory = mapper.read_json(ROOT / "example_components.json")
    circuit = mapper.read_json(ROOT / "bench/circuits/p1_red_led_1k_series.json")
    result = mapper.map_circuit(circuit, inventory, board, time_limit=20)
    assert result["status"] == "OPTIMAL" and len(models) == 1
