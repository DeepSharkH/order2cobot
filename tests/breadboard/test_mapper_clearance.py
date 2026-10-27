"""Focused geometry, ordering, and small end-to-end checks."""

import sys
from pathlib import Path

import pytest
from ortools.sat.python import cp_model

ROOT = Path(__file__).parents[2] / "src/circuit_mapper"
sys.path.insert(0, str(ROOT))
import geometry  # noqa: E402
import mapper  # noqa: E402

GRIPPER = mapper.read_json(ROOT / "gripper.json")
CASINGS = mapper.read_json(mapper.DEFAULT_CASINGS)


@pytest.mark.parametrize(
    "direction, angle", [((1, 0), 0), ((0, 1), 90), ((-1, 0), 180), ((0, -1), 270)]
)
def test_rotations_and_open_finger_span(direction, angle):
    casing = next(c for c in CASINGS["casings"] if c["casing"] == "resistor_s1")
    dx, dy = direction
    positions = {"a": (-1.27 * dx, -1.27 * dy), "b": (1.27 * dx, 1.27 * dy)}
    shape = geometry.placement_geometry(("a", "b"), positions, casing, GRIPPER, 1)
    assert shape["orientation_deg"] == angle
    assert shape["center_mm"] == pytest.approx([0, 0])
    axis = 1 if angle in (0, 180) else 0
    assert max(f[axis + 2] for f in shape["fingers"]) == pytest.approx(5)
    assert min(f[axis] for f in shape["fingers"]) == pytest.approx(-5)


def shape_at(width, x, y, opening=1):
    casing = {"casing": "fixture", "width_x": width, "thickness_y": 4, "exit_x": [-1, 1]}
    return geometry.placement_geometry(
        ("a", "b"),
        {"a": (x - 1, y), "b": (x + 1, y)},
        casing,
        {**GRIPPER, "opening_per_finger_mm": opening},
        1,
    )


def test_opening_requires_order_and_wrong_order_is_rejected():
    a = shape_at(24, 0, 0)
    b = shape_at(14, 10, 6.5)
    assert geometry.relation(a, b, 0) == "b_first"
    assert geometry.relation(shape_at(24, 0, 0, 0), shape_at(14, 10, 6.5, 0), 0) == "independent"
    model = cp_model.CpModel()
    present = [model.new_bool_var(str(i)) for i in range(2)]
    orders = [model.new_int_var(0, 1, f"order{i}") for i in range(2)]
    selected = [
        {"id": str(i), "owner": i, "chosen": present[i], "shape": shape}
        for i, shape in enumerate([a, b])
    ]
    for p in present:
        model.add(p == 1)
    geometry.constrain_assembly(model, selected, orders, set(), 0)
    solver = cp_model.CpSolver()
    assert solver.solve(model) == cp_model.OPTIMAL
    assert solver.value(orders[1]) < solver.value(orders[0])
    geometry.verify_assembly(list(reversed(selected)), 0)
    with pytest.raises(ValueError, match="collision"):
        geometry.verify_assembly(selected, 0)


def test_permanent_overlap_and_mutual_blocking():
    a = shape_at(14, 0, 0)
    assert geometry.relation(a, shape_at(14, 0, 3), 0) == "forbidden"
    assert geometry.relation(a, shape_at(14, 0, 6.5), 0) == "forbidden"


def test_parallel_resistors_and_missing_connector_geometry():
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
    result = mapper.map_circuit(circuit, inventory, board, time_limit=10)
    assert set(result["assembly_order"]) == {"R1", "R2"}
    positions, groups = mapper.board_geometry(board)
    mapper.verify_connections(result, circuit["nets"], groups)
    for part in inventory["components"]:
        if part["part_id"] == "power_connector_2pin":
            part.pop("placement_mode", None)
    with pytest.raises(ValueError, match="Missing supported casing geometry.*power_connector"):
        mapper.map_circuit(mapper.read_json(ROOT / "examples/led_circuit.json"), inventory, board)


def test_manual_power_pins_have_no_body_and_remain_connected():
    board = mapper.read_json(ROOT / "board_connections.json")
    board["conductive_groups"] = board["conductive_groups"][:6]
    inventory = mapper.read_json(ROOT / "example_components.json")
    circuit = mapper.read_json(ROOT / "examples/led_circuit.json")
    result = mapper.map_circuit(circuit, inventory, board, time_limit=15)
    assert set(result["manual_connections"]) == {"J_POWER.positive", "J_POWER.negative"}
    assert "J_POWER" not in result["assembly_order"]
    assert all(p["id"] != "J_POWER" for p in result["placements"])
    _, groups = mapper.board_geometry(board)
    mapper.verify_connections(result, circuit["nets"], groups)
