"""Finite stock applies to requested parts and automatically routed wires."""

import json
from pathlib import Path

import pytest
from ortools.sat.python import cp_model

from src.circuit_mapper.inventory import checked_parts, limit_jumpers

ROOT = Path(__file__).resolve().parents[2] / "src/circuit_mapper"


def test_current_inventory():
    inventory = json.loads((ROOT / "example_components.json").read_text())
    parts = checked_parts({"components": []}, inventory)
    assert sum(p.get("quantity", 0) for p in parts.values()) == 21
    assert not any(p["type"] == "bjt" for p in parts.values())
    for pid in ("capacitor_ceramic_100nf", "resistor_1kohm_gap_3", "jumper_rigid_2_pitch"):
        assert parts[pid]["quantity"] == 2
    circuit = {
        "components": [{"id": f"C{i}", "part_id": "capacitor_ceramic_100nf"} for i in range(3)]
    }
    with pytest.raises(ValueError, match="Insufficient stock"):
        checked_parts(circuit, inventory)


def test_routed_jumpers_cannot_exceed_stock():
    model = cp_model.CpModel()
    part = {"part_id": "wire", "quantity": 1}
    choices = [model.new_bool_var(f"w{i}") for i in range(2)]
    limit_jumpers(model, [(c, part, ()) for c in choices], {"wire": part})
    model.add(sum(choices) == 2)
    assert cp_model.CpSolver().solve(model) == cp_model.INFEASIBLE
