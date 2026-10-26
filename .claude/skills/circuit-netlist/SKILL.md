---
name: circuit-netlist
description: Turn a circuit described in natural language (or a sketchy netlist, a list of "X connects to Y", a textbook circuit name) into the circuit JSON that src/circuit_mapper/mapper.py takes as input - picks exact inventory parts, builds the nets, records assumptions and open questions, then checks the file by running the mapper. Use when asked to write/create/convert a circuit for the circuit mapper, describe a breadboard circuit as JSON, or make a netlist for the breadboard.
---

# Circuit netlist — skill

Goal: one valid `circuit.json` that `src/circuit_mapper/mapper.py` accepts, describing **exactly**
the circuit the user meant, with every interpretation written down. You decide *what connects to
what*; the mapper decides *which holes and jumpers*. Never put hole positions or jumpers in the file.

- Output: `src/circuit_mapper/circuits/<circuit_id>.json`, unless the user names a path.
- Inventory: `src/circuit_mapper/example_components.json`, unless the user names another
  (then pass it to the mapper with `--inventory` too).
- Reference outputs: `src/circuit_mapper/examples/*.json` (LED, RC low-pass, PNP switch). Match
  their style. `src/circuit_mapper/README.md` lists the mapper's model assumptions.

## 1. Read the inventory (every time, it changes)

```bash
python3 -c "
import json,sys
for c in json.load(open(sys.argv[1]))['components']:
    print(c['part_id'], c['type'], c['pins'], 'spacing', c['pin_spacing'], c.get('parameters', {}))
" src/circuit_mapper/example_components.json
```

Only `part_id`s from this list are valid. A wrong one fails in the mapper with only the quoted
id as its message (e.g. `'pushbutton'`).

## 2. Translate the description

**Components → `components`.** One entry per physical part: `{"id": ..., "part_id": ...}`.
- IDs: `R1, R2…` resistors, `C1…` capacitors, `D1…` LEDs and diodes, `Q1…` transistors,
  `SW1…` switches/buttons, `RV1…` potentiometers, `J_POWER` for the supply, `J_IN`/`J_OUT` for
  signal connections. A role name (`R_BASE`, `R_PULLUP`, `R_LED`) is better when the user names
  roles. IDs are unique, contain no `.`, and match any names the user used ("R3" stays `R3`).
- Pick the exact inventory part by type and value. Variants: `_gap_0` (leads in adjacent
  holes, `pin_spacing` 1) is the default; use `_gap_1` only when the user asks for one hole
  between the leads. The mapper never switches variants.
- **Value or part not in the inventory** (330 Ω, 1 µF, Zener, buzzer, 555, op-amp, any DIP IC):
  never substitute silently. Ask the user (AskUserQuestion) and offer the nearest inventory parts.
  If they accept a substitute, write it in `assumptions`. If it stays open, write the file with the
  question in `unresolved_questions`. The mapper refuses such a file, which is intended. DIP ICs
  can't be expressed at all: the mapper only places parts whose pins are in a straight line.
  Say so. Don't edit the inventory unless the user asks you to.
- **Power and outside connections.** The board has no power rails and nothing gets power unless a
  connector brings it. A battery, bench supply, USB, or "5 V and ground" becomes one
  `power_connector_2pin` called `J_POWER` (`positive` = supply +, `negative` = GND). A signal
  source or measurement point (function generator, scope, Arduino pin) becomes another
  `power_connector_2pin` (`positive` = signal, `negative` = ground), as in `rc_low_pass.json`.
  Two-pin connectors only. A dual (±) supply needs two connectors that share GND.
- **Jumpers / wires**: never list them. "Wire from A to B" just means A and B are on the same net.

**Nodes → `nets`.** One net per electrical node, and a node is everything joined by wires.
- Walk the description and merge every pin that is said to be connected, directly or through a
  wire. "In series" means consecutive parts share one net. "In parallel" means both ends share
  nets. "To ground" puts the pin on `GND`, and "to +5 V/the battery +" puts it on `VCC`.
- Names: `VCC`, `GND` for the supply rails (keep `VCC` whatever the voltage); short UPPER_SNAKE
  names for the other nodes (`LED_ANODE`, `BASE`, `OUTPUT`, `SWITCHED_VCC`). Unique, and never
  starting with `UNCONNECTED:` (the mapper reserves that prefix).
- Pin references are `<component id>.<pin name>`, with the pin names copied from the inventory:

| type | pins | orientation rule |
|---|---|---|
| resistor, ceramic capacitor, switch_spst, pushbutton | `1`, `2` | symmetric; put `1` on the supply/input side for readability |
| led, diode | `anode`, `cathode` | anode on the higher-potential side; cathode (flat/stripe) toward GND |
| capacitor_electrolytic | `positive`, `negative` | positive on the higher-potential node |
| rgb_led (common cathode) | `red_anode`, `green_anode`, `blue_anode`, `common_cathode` | common_cathode to GND |
| bjt NPN | `base`, `collector`, `emitter` | low-side switch: emitter GND, collector to load, base via resistor |
| bjt PNP | `base`, `collector`, `emitter` | high-side switch: emitter VCC, collector to load |
| potentiometer | `end_1`, `wiper`, `end_2` | divider: ends to VCC/GND, wiper is the output |
| switch_spdt | `common`, `throw_1`, `throw_2` | |
| power_connector_2pin | `positive`, `negative` | |

- **Every pin exactly once**: either in one net or in `unconnected_pins`. Unused pins
  (a pot's spare end in rheostat use, an unused SPDT throw, an unused RGB colour) go in
  `unconnected_pins`, not in a one-pin net. Say why in `assumptions`.

**Text fields.**
- `circuit_id`: snake_case, the same as the file name.
- `description`: 1–2 sentences on what the circuit does and its main path, written so someone
  could check the nets against it.
- `assumptions`: every interpretation you made. Supply voltage (if not given, assume an external
  regulated 5 V DC supply on `J_POWER` and say so), substitutions, variant choice when it wasn't
  given, unused pins, switch states ("a switch's terminals stay separate nets; contact state is not
  modeled"). For transistors, add "Generic part: verify actual package pin order before building."
- `unresolved_questions`: `[]` when everything is settled.

## 3. When to ask, when to assume

Ask (AskUserQuestion, batch all questions into one call) when the answer changes the
topology or the parts: which node a part connects to, a value missing from the inventory, polarity
that changes function, a part the description mentions only vaguely ("some resistor"; if there's
an obvious default, e.g. 220 Ω–1 kΩ for an LED at 5 V, offer it as the recommended option).

Assume and record in `assumptions`: supply voltage, `_gap_0` variant, symmetric pin order,
ID and net names, connectors for outside sources.

Don't fix the circuit silently. If something looks electrically wrong (LED across the supply
with no resistor, supply shorted by a closed switch, diode or electrolytic reversed, base without
a resistor, a floating input), point it out. Encode what the user confirms and note the concern
in `assumptions`. The mapper doesn't simulate anything.

## 4. Write the file

Same layout as the examples: 2-space indent, keys in this order:

```json
{
  "schema_version": "1.0",
  "circuit_id": "<snake_case>",
  "description": "...",
  "components": [{"id": "J_POWER", "part_id": "power_connector_2pin"}, ...],
  "nets": [{"id": "VCC", "pins": ["J_POWER.positive", "R1.1"]}, ...],
  "unconnected_pins": [],
  "assumptions": ["..."],
  "unresolved_questions": []
}
```

## 5. Check

1. **Re-trace by hand.** For each component, write one line naming the net on each of its pins
   (`D1: anode=LED_ANODE, cathode=GND`) and compare it with the user's sentences. This catches
   swapped polarity and merged nodes, which no tool will.
2. **Run the mapper** (skip if `unresolved_questions` is non-empty; it refuses those). ortools isn't
   a project dependency, so add it for this run only:

   ```bash
   uv run --with ortools python src/circuit_mapper/mapper.py src/circuit_mapper/circuits/<id>.json \
     --output <scratchpad or /tmp>/<id>_mapping.json \
     && jq '{status, jumpers: (.jumpers | length)}' <scratchpad or /tmp>/<id>_mapping.json
   ```

   It usually runs for the whole time limit (default 30 s), so the JSON errors below show up
   immediately but a solve takes the full 30 s. On the result:
   - `OPTIMAL` / `FEASIBLE`: the file is valid and the circuit fits the breadboard. A
     `FEASIBLE` jumper count is often far from minimal. That's a solver budget issue, not a
     netlist issue.
   - `Unknown or multiply assigned pin: X`, `Every component pin must occur…`, `Net IDs must be
     unique`, `Empty net`, or just a quoted id like `'pushbutton'` (unknown part_id): the JSON is
     wrong. Fix it and rerun.
   - `No mapping returned: UNKNOWN`: timeout. Rerun with `--time-limit 60` before concluding anything.
   - `No mapping returned: INFEASIBLE`: the JSON is valid but the circuit doesn't fit this board
     model. Report it; don't rearrange the circuit to make it fit.

## 6. Report

Tell the user: the file path, a compact net list (`VCC: J_POWER.positive, R1.1`…), the
assumptions you made, any open questions, and the mapper status with its jumper count.
Mention the mapping file's path if they want the hole placement. To show the placement, render it
with `python3 src/circuit_mapper/visualize.py <circuit.json> <mapping.json> --out <dir>`, which
writes `<dir>/index.html`.
