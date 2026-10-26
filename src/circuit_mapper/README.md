# Circuit mapper

From the `order2cobot` directory:

```bash
uv run python src/circuit_mapper/mapper.py src/circuit_mapper/examples/led_circuit.json --output mapping.json
```

Power/signal connector pins are manual connections, not robot-placed parts.
The LED and RC examples use these virtual input/output connections. The
transistor example still requires casing geometry for its pushbutton. Missing
geometry for a robot-placed part produces an explicit error; it is never
silently treated as having zero size. The parallel-resistor example is also
available as a small geometry demonstration.

Options: `--inventory`, `--board`, `--casings`, `--gripper`, `--time-limit` (30
seconds by default, excluding initial model construction), and `--output`.
Without `--output`, JSON is printed to stdout. The old `--clearance` option
has been replaced by the millimeter-based geometry model.

## Manual connections

`placement_mode: "manual"` on an inventory part or circuit instance makes each
pin an independent manual connection point. The inventory's
`power_connector_2pin` uses this mode. Each pin reserves a free hole on its net,
but no connector body, pin-spacing constraint, finger clearance, or robot
assembly step is generated. Add the external wiring by hand after robot
assembly. Accessibility and clearance for this manual wiring are not verified.

## Geometry and motion assumptions

- Board coordinates are converted using `geometry.hole_pitch_mm` (2.54 mm,
  matching the CAD source). The central trench is four pitches wide.
- `assets/models/component_casings/casings.json` provides assembled casing
  dimensions and lead exits, rather than print-oriented STL bounds.
- Inventory parts match `<type>_s<pin_spacing>`; ceramic capacitors use
  `capacitor_ceramic`. An explicit `casing_id` on a part overrides that lookup.
  Buzzers are excluded. Pin order is the inventory order and must match the
  actual part's wiring through its casing.
- Casings remain on the board. Their permanent footprint cannot overlap any
  other installed casing, including jumper casings.
- Fingers grip across the casing thickness. Each finger is 2 mm thick and
  5 mm wide, centered along the pin line, opening outward by 1 mm before
  vertical withdrawal. A 4 mm casing gives a 10 mm outside opening span.
- The robot approaches and withdraws vertically at fixed XY. Rotation and
  lateral travel happen only above all installed parts, so no turning sweep
  is reserved at board height. Safe lift height and robot motion are not
  calculated or executed by this mapper.
- Orientations are 0, 90, 180, and 270 degrees. Zero follows increasing columns;
  90 follows increasing rows. Pins, casing, and finger envelopes rotate together.

`gripper.json` contains the dimensions, grip offset, and additional safety
margin. Margin defaults to zero (nominal dimensions, not calibrated tolerances)
and is applied once between shapes. LED/RGB body thickness overrides of 6.2 mm
conservatively include the protruding body/pocket envelope from `casings.py`;
the casing alone is only 4 mm thick. These are CAD design bounds, not measured
manufacturing tolerances.

This is a conservative 2D projection: shapes at different heights cannot pass
over one another while working near the board. It does not model the rest of
the gripper, fixtures, board-edge keepouts, cables, or electrical behavior.
Casing overhang beyond the grid is allowed. External robot calibration and
physical verification are still necessary before executing a layout.

## Solver

1. Enumerate straight-line pin placements in all four directions and rigid
   jumper options. Enforce hole occupancy and circuit net connectivity.
2. Solve a candidate layout, then check permanent footprints and swept finger
   rectangles for the selected placements.
3. For overlapping bodies or mutually blocking grips, forbid that placement
   pair. If inserting A would hit an installed B, require A before B.
4. Solve again with these conditional constraints. Cyclic order requirements
   force another layout. Return only after independently checking the final
   assembly order and electrical connectivity.

Pair constraints are added as needed instead of constructing millions of
unused combinations. Each solve stops at its first feasible candidate to leave
time for collision checking. The objective favors fewer jumpers but returned
`FEASIBLE` layouts are not guaranteed to minimize them. `OPTIMAL` means the
objective was proven optimal for the model; a valid solution of the relaxed
model also meets the physical constraints after verification. `UNKNOWN` means
no verified assembly was found before the search ended, not infeasibility.
`INFEASIBLE` means the accumulated necessary constraints admit no layout.

## Output

- `pin_to_hole`: robot-placed component instance pins mapped to board holes.
- `manual_connections`: virtual input/output pins mapped to reserved holes for
  later hand wiring. These are included in electrical connectivity verification.
- `jumpers`: generated jumper IDs, part IDs, pin holes, and net IDs.
- `placements`: component/jumper IDs, casing IDs, orientation, and casing-center
  coordinates in millimeters relative to the board coordinate origin.
- `assembly_order`: those IDs in insertion order, including jumpers.
- `gripper`: the geometry settings used for this result.

Jumper IDs use the reserved prefix `jumper:`. Temporary finger regions may
occupy the same space at different times; they do not need to avoid one another.
They must avoid the permanent bodies of all earlier installed parts. Every
part uses the exact inventory variant selected by the circuit; variant choice
is not automatic.
