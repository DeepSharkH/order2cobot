"""Place straight-line components and route their nets with rigid jumpers."""

import argparse
import json
from collections import defaultdict
from pathlib import Path
from time import monotonic

from ortools.sat.python import cp_model

if __package__:
    from .geometry import casing_for, constrain_assembly, placement_geometry, verify_assembly
else:
    from geometry import casing_for, constrain_assembly, placement_geometry, verify_assembly

HERE = Path(__file__).parent
DEFAULT_CASINGS = HERE.parents[1] / "assets/models/component_casings/casings.json"


def read_json(path):
    return json.loads(Path(path).read_text())


def board_geometry(board):
    geometry = board["geometry"]
    rows = geometry["rows"]
    positions, groups = {}, {}
    for group_index, group in enumerate(board["conductive_groups"]):
        for hole in group["holes"]:
            if hole in groups:
                raise ValueError(f"Hole belongs to multiple groups: {hole}")
            positions[hole] = (
                geometry["column_positions"][hole[0]],
                rows["first_position"] + (int(hole[1:]) - rows["first"]) * rows["spacing"],
            )
            groups[hole] = group_index
    return positions, groups


def placements(part, positions):
    """Generate pin tuples in pin-list order, in all four axis directions."""
    spacing = part.get("pin_spacing")
    if type(spacing) is not int or spacing < 1:
        raise ValueError(f"{part['part_id']}: pin_spacing must be a positive integer")
    by_position = {xy: hole for hole, xy in positions.items()}
    seen = set()
    for x, y in positions.values():
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            holes = tuple(
                by_position.get((x + i * spacing * dx, y + i * spacing * dy))
                for i in range(len(part["pins"]))
            )
            if None not in holes and holes not in seen:
                seen.add(holes)
                yield holes


def map_circuit(circuit, inventory, board, time_limit=30, casings=None, gripper=None):
    casings = read_json(DEFAULT_CASINGS) if casings is None else casings
    gripper = read_json(HERE / "gripper.json") if gripper is None else gripper
    pitch = board["geometry"]["hole_pitch_mm"]
    if pitch <= 0 or time_limit <= 0:
        raise ValueError("Board pitch and time limit must be positive")
    if (
        min(gripper["finger_thickness_mm"], gripper["finger_width_mm"]) <= 0
        or min(gripper["opening_per_finger_mm"], gripper["safety_margin_mm"]) < 0
    ):
        raise ValueError("Invalid gripper dimensions or margin")
    parts = {part["part_id"]: part for part in inventory["components"]}
    instances = {c["id"]: parts[c["part_id"]] for c in circuit["components"]}
    if any(cid.startswith("jumper:") for cid in instances):
        raise ValueError("The instance ID prefix jumper: is reserved for generated wires")
    if len(instances) != len(circuit["components"]):
        raise ValueError("Component instance IDs must be unique")
    expected = {f"{cid}.{pin}" for cid, part in instances.items() for pin in part["pins"]}
    nets = list(circuit["nets"])
    # An intentionally unused pin must not accidentally short another net.
    nets += [
        {"id": f"UNCONNECTED:{pin}", "pins": [pin]} for pin in circuit.get("unconnected_pins", [])
    ]
    pin_net = {}
    for n, net in enumerate(nets):
        if not net["pins"]:
            raise ValueError(f"Empty net: {net['id']}")
        for pin in net["pins"]:
            if pin not in expected or pin in pin_net:
                raise ValueError(f"Unknown or multiply assigned pin: {pin}")
            pin_net[pin] = n
    if set(pin_net) != expected or not expected:
        raise ValueError("Every component pin must occur in one net or unconnected_pins")
    if len({net["id"] for net in nets}) != len(nets):
        raise ValueError("Net IDs must be unique")
    if circuit.get("unresolved_questions"):
        raise ValueError("Resolve circuit questions before mapping")
    for cid, part in instances.items():
        if part["type"] == "jumper":
            raise ValueError(
                "Leave jumpers out of the circuit; the mapper adds them during routing"
            )
        for connected in part.get("connected_pin_groups", []):
            if len({pin_net[f"{cid}.{pin}"] for pin in connected}) > 1:
                raise ValueError(f"{cid}: permanently connected pins assigned to different nets")

    manual_ids = {
        c["id"]
        for c in circuit["components"]
        if c.get("placement_mode", instances[c["id"]].get("placement_mode")) == "manual"
    }
    manual_pins = {f"{cid}.{pin}" for cid in manual_ids for pin in instances[cid]["pins"]}
    physical = {cid: part for cid, part in instances.items() if cid not in manual_ids}
    used_parts = {p["part_id"]: p for p in physical.values()}
    used_parts.update({pid: p for pid, p in parts.items() if p["type"] == "jumper"})
    shapes = {pid: casing_for(part, casings, pitch) for pid, part in used_parts.items()}
    positions, groups = board_geometry(board)
    group_count = len(board["conductive_groups"])
    model = cp_model.CpModel()
    # Each conductive strip can belong to only one net (-1 means unused).
    labels = [model.new_int_var(-1, len(nets) - 1, f"net_{g}") for g in range(group_count)]
    candidates = []
    occupancy = defaultdict(list)
    terminals, roots = defaultdict(list), defaultdict(list)
    choices = {}
    for owner, (cid, part) in enumerate(physical.items()):
        options = []
        for holes in placements(part, positions):
            assignments = [
                (pin_net[f"{cid}.{pin}"], groups[hole])
                for pin, hole in zip(part["pins"], holes, strict=True)
            ]
            if any(g == h and n != m for n, g in assignments for m, h in assignments):
                continue  # This placement would short two distinct nets inside a strip.
            chosen = model.new_bool_var(f"{cid}_{len(options)}")
            options.append((chosen, holes))
            candidates.append(
                {
                    "chosen": chosen,
                    "owner": owner,
                    "id": cid,
                    "part_id": part["part_id"],
                    "holes": holes,
                    "shape": placement_geometry(
                        holes, positions, shapes[part["part_id"]], gripper, pitch
                    ),
                }
            )
            for hole in holes:
                occupancy[hole].append(chosen)
            for pin, (n, g) in zip(part["pins"], assignments, strict=True):
                model.add(labels[g] == n).only_enforce_if(chosen)
                terminals[n, g].append(chosen)
                if f"{cid}.{pin}" == nets[n]["pins"][0]:
                    roots[n, g].append(chosen)
        if not options:
            raise ValueError(f"No legal placements for {cid}")
        model.add_exactly_one(chosen for chosen, _ in options)
        choices[cid] = options

    # Manual connections are individual reserved holes, without a body or grip.
    manual_choices = {}
    for pin in sorted(manual_pins):
        n = pin_net[pin]
        options = []
        for hole, g in groups.items():
            chosen = model.new_bool_var(f"manual_{pin}_{hole}")
            options.append((chosen, hole))
            occupancy[hole].append(chosen)
            model.add(labels[g] == n).only_enforce_if(chosen)
            terminals[n, g].append(chosen)
            if pin == nets[n]["pins"][0]:
                roots[n, g].append(chosen)
        model.add_exactly_one(chosen for chosen, _ in options)
        manual_choices[pin] = options

    wires, edges = [], defaultdict(list)
    seen_pairs = set()
    for part in parts.values():
        if part["type"] != "jumper":
            continue
        if len(part["pins"]) != 2:
            raise ValueError("Routing requires two-pin jumper parts")
        for holes in placements(part, positions):
            pair = tuple(sorted(holes))
            if pair in seen_pairs or groups[holes[0]] == groups[holes[1]]:
                continue
            seen_pairs.add(pair)
            chosen = model.new_bool_var(f"wire_{len(wires)}")
            candidates.append(
                {
                    "chosen": chosen,
                    "owner": len(physical) + len(wires),
                    "id": f"jumper:{len(wires) + 1}",
                    "part_id": part["part_id"],
                    "holes": holes,
                    "shape": placement_geometry(
                        holes, positions, shapes[part["part_id"]], gripper, pitch
                    ),
                }
            )
            for hole in holes:
                occupancy[hole].append(chosen)
            wires.append((chosen, part, holes))
            a, b = sorted((groups[holes[0]], groups[holes[1]]))
            edges[a, b].append(chosen)
            model.add(labels[a] == labels[b]).only_enforce_if(chosen)
            model.add(labels[a] >= 0).only_enforce_if(chosen)
    for occupants in occupancy.values():
        model.add_at_most_one(occupants)
    owner_count = len(physical) + len(wires)
    orders = [model.new_int_var(0, owner_count - 1, f"order_{i}") for i in range(owner_count)]

    # Send abstract flow from each net's first pin to all its pins.
    # Flow can cross strips only on installed jumpers carrying that net.
    balance = defaultdict(list)
    for (a, b), edge_candidates in edges.items():
        linked = model.new_bool_var(f"link_{a}_{b}")
        model.add(sum(edge_candidates) == linked)  # Parallel wires between strips are redundant.
        for n, net in enumerate(nets):
            capacity = len(net["pins"])
            active = model.new_bool_var(f"active_{n}_{a}_{b}")
            model.add(active <= linked)
            model.add(labels[a] == n).only_enforce_if(active)
            flow = model.new_int_var(-capacity, capacity, f"flow_{n}_{a}_{b}")
            model.add(flow <= capacity * active)
            model.add(flow >= -capacity * active)
            balance[n, a].append(flow)
            balance[n, b].append(-flow)
    for n, net in enumerate(nets):
        for g in range(group_count):
            model.add(
                sum(balance[n, g]) == len(net["pins"]) * sum(roots[n, g]) - sum(terminals[n, g])
            )

    model.minimize(sum(chosen for chosen, _, _ in wires))
    solver = cp_model.CpSolver()
    # Keep at least 4 workers: 1 has no feasibility-jump and never found a layout.
    solver.parameters.num_workers = 8
    # Presolve reruns every lazy round and probing dominated it.
    solver.parameters.cp_model_probing_level = 0
    solver.parameters.max_presolve_iterations = 1
    solver.parameters.stop_after_first_solution = True
    deadline = monotonic() + time_limit
    known = set()
    while True:
        remaining = deadline - monotonic()
        if remaining <= 0:
            raise ValueError(
                "UNKNOWN: time limit reached before a collision-free assembly was found"
            )
        solver.parameters.max_time_in_seconds = remaining
        status = solver.solve(model)
        if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
            reason = (
                "No layout satisfies the part, wiring, and assembly constraints."
                if status == cp_model.INFEASIBLE
                else "No verified layout found within the search limit; not proof of infeasibility."
            )
            raise ValueError(f"No mapping returned: {solver.status_name(status)}. {reason}")
        selected = [c for c in candidates if solver.value(c["chosen"])]
        if not constrain_assembly(model, selected, orders, known, gripper["safety_margin_mm"]):
            break
    selected.sort(key=lambda c: (solver.value(orders[c["owner"]]), c["owner"]))
    verify_assembly(selected, gripper["safety_margin_mm"])
    pin_to_hole = {}
    for cid, options in choices.items():
        holes = next(holes for chosen, holes in options if solver.value(chosen))
        pin_to_hole.update(
            {f"{cid}.{pin}": hole for pin, hole in zip(instances[cid]["pins"], holes, strict=True)}
        )
    jumpers = []
    for wire_index, (chosen, part, holes) in enumerate(wires):
        if solver.value(chosen):
            jumpers.append(
                {
                    "id": f"jumper:{wire_index + 1}",
                    "part_id": part["part_id"],
                    "pins": dict(zip(part["pins"], holes, strict=True)),
                    "net": nets[solver.value(labels[groups[holes[0]]])]["id"],
                }
            )
    result = {
        "circuit_id": circuit["circuit_id"],
        "board_id": board["board_id"],
        "status": solver.status_name(status),
        "geometry_model": "2d_casing_and_opening_fingers",
        "gripper": gripper,
        "placements": [
            {
                "id": c["id"],
                "part_id": c["part_id"],
                "orientation_deg": c["shape"]["orientation_deg"],
                "center_mm": c["shape"]["center_mm"],
                "casing_id": c["shape"]["casing_id"],
            }
            for c in selected
        ],
        "assembly_order": [c["id"] for c in selected],
        "pin_to_hole": pin_to_hole,
        "manual_connections": {
            pin: next(h for chosen, h in options if solver.value(chosen))
            for pin, options in manual_choices.items()
        },
        "jumpers": jumpers,
    }
    verify_connections(result, nets, groups)
    return result


def verify_connections(result, nets, groups):
    """Independently reconstruct conductor connectivity before returning a layout."""
    parent = {g: g for g in groups.values()}

    def find(g):
        while parent[g] != g:
            g = parent[g]
        return g

    pin_holes = {**result["pin_to_hole"], **result.get("manual_connections", {})}
    occupied = list(pin_holes.values())
    for jumper in result["jumpers"]:
        a, b = jumper["pins"].values()
        occupied.extend((a, b))
        parent[find(groups[a])] = find(groups[b])
    if len(occupied) != len(set(occupied)):
        raise ValueError("Invalid result: a hole is occupied more than once")
    owners = {}
    for net in nets:
        connected = {find(groups[pin_holes[pin]]) for pin in net["pins"]}
        if len(connected) != 1:
            raise ValueError(f"Invalid result: disconnected net {net['id']}")
        group = connected.pop()
        if group in owners:
            raise ValueError(f"Invalid result: short between {owners[group]} and {net['id']}")
        owners[group] = net["id"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("circuit", type=Path)
    parser.add_argument("--inventory", type=Path, default=HERE / "example_components.json")
    parser.add_argument("--board", type=Path, default=HERE / "board_connections.json")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--time-limit", type=float, default=30)
    parser.add_argument("--casings", type=Path, default=DEFAULT_CASINGS)
    parser.add_argument("--gripper", type=Path, default=HERE / "gripper.json")
    args = parser.parse_args()
    if args.time_limit <= 0:
        parser.error("--time-limit must be positive")
    try:
        result = map_circuit(
            read_json(args.circuit),
            read_json(args.inventory),
            read_json(args.board),
            args.time_limit,
            read_json(args.casings),
            read_json(args.gripper),
        )
    except (ValueError, KeyError, OSError) as error:
        parser.exit(1, f"{error}\n")
    output = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.write_text(output)
    else:
        print(output, end="")


if __name__ == "__main__":
    main()
