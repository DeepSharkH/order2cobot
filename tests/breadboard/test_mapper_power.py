import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).parents[2] / "src/circuit_mapper"
sys.path.insert(0, str(ROOT))
import mapper  # noqa: E402
from power import fixed_power_connections  # noqa: E402


def test_fixed_power_and_top_window():
    board = mapper.read_json(ROOT / "board_connections.json")
    inventory = mapper.read_json(ROOT / "example_components.json")
    inventory["components"].append(
        {"part_id": "test_resistor_s4", "type": "resistor", "pins": ["1", "2"], "pin_spacing": 4}
    )
    circuit = {
        "circuit_id": "anchored",
        "components": [
            {"id": "SUPPLY", "part_id": "power_connector_2pin", "role": "power_supply"},
            {"id": "R1", "part_id": "test_resistor_s4"},
        ],
        "nets": [
            {"id": "VCC", "pins": ["SUPPLY.positive", "R1.1"]},
            {"id": "GND", "pins": ["SUPPLY.negative", "R1.2"]},
        ],
    }
    result = mapper.map_circuit(circuit, inventory, board, time_limit=10)
    assert result["manual_connections"] == {"SUPPLY.positive": "C1", "SUPPLY.negative": "H1"}
    assert result["pin_to_hole"] == {"R1.1": "E1", "R1.2": "F1"}
    assert result["jumpers"] == []


def test_signal_connectors_are_not_power():
    circuit = mapper.read_json(ROOT / "examples/rc_low_pass.json")
    assert fixed_power_connections(circuit, set(), {}, {}, {}) == {}


@pytest.mark.parametrize("fault", ["missing_hole", "shorted_net", "not_manual"])
def test_invalid_power_configuration(fault):
    circuit = {"components": [{"id": "J_POWER"}]}
    pins = {"J_POWER.positive", "J_POWER.negative"}
    positions = {"C1": (2, 0), "H1": (10, 0)}
    nets = {"J_POWER.positive": 0, "J_POWER.negative": 1}
    if fault == "missing_hole":
        positions.pop("C1")
    elif fault == "shorted_net":
        nets["J_POWER.negative"] = 0
    else:
        pins.clear()
    with pytest.raises(ValueError):
        fixed_power_connections(circuit, pins, positions, {"C1": 0, "H1": 1}, nets)
