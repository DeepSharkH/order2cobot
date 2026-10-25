# Circuit mapper

Run from the `order2cobot` directory:

```bash
uv run python src/circuit_mapper/mapper.py src/circuit_mapper/examples/led_circuit.json --output mapping.json
```

Use `rc_low_pass.json` or `pnp_transistor_switch.json` for the other examples.
`--inventory` and `--board` select alternative JSON files. `--time-limit 60`
changes the solver's time budget (default: 30 seconds, excluding model construction).
Without `--output`, the mapping is printed to stdout.

The result contains `pin_to_hole` for circuit components and a `jumpers` list
with part IDs, endpoint holes, and net IDs. Jumper terminals occupy holes too.

## How it works

1. Generate legal horizontal/vertical placements from each part's pin order
   and spacing, including reverse orientations.
2. Choose one placement per component. Give each conductive strip one net
   label and allow at most one terminal in each hole.
3. Choose jumper placements from the inventory. Require a connected path for
   each net using flow constraints; different nets cannot share a strip.
4. Minimize the number of jumpers. Independently reconstruct the final
   connectivity to check for shorts and missing connections.

OR-Tools CP-SAT does the search. `OPTIMAL` means the jumper count is proven
minimal under this model. `FEASIBLE` means the wiring is valid but the count
may improve with more time. `UNKNOWN` is a timeout without a solution, not
proof that the circuit cannot fit. `INFEASIBLE` means no valid layout exists
under these constraints.

## Prototype assumptions

- Pins form a straight line with uniform positive integer `pin_spacing`.
  The inventory's `pins` order defines their physical order.
- The circuit selects exact inventory variants; the mapper does not switch
  resistor/LED spacing variants automatically.
- All board holes are initially empty. Jumpers are unlimited and added by
  the mapper, so do not include jumper components in the circuit netlist.
- Every pin occurs once in `nets` or `unconnected_pins`. Unconnected pins are
  isolated from all other pins. Connectors represent external attachment points.
- Only board strips and jumpers establish direct electrical connectivity.
  Resistors, capacitors, transistors, and switch states are not simulated.
- Body size, wire crossings, lead collisions, electrical ratings, and assembly
  order are not modeled. This is a wiring layout, not a collision-free assembly
  plan. Verify actual package pin order before building.
