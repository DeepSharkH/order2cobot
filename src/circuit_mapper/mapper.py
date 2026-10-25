"""Place straight-line components and route their nets with rigid jumpers."""

import argparse
import json
from collections import defaultdict
from pathlib import Path

from ortools.sat.python import cp_model

HERE = Path(__file__).parent


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


def map_circuit(circuit, inventory, board, time_limit=30):
    parts = {part["part_id"]: part for part in inventory["components"]}
    instances = {c["id"]: parts[c["part_id"]] for c in circuit["components"]}
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

    positions, groups = board_geometry(board)
    group_count = len(board["conductive_groups"])
    model = cp_model.CpModel()
    # Each conductive strip can belong to only one net (-1 means unused).
    labels = [model.new_int_var(-1, len(nets) - 1, f"net_{g}") for g in range(group_count)]
    occupancy = defaultdict(list)
    terminals, roots = defaultdict(list), defaultdict(list)
    choices = {}
    for cid, part in instances.items():
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
            for pin, hole, (n, g) in zip(part["pins"], holes, assignments, strict=True):
                occupancy[hole].append(chosen)
                model.add(labels[g] == n).only_enforce_if(chosen)
                terminals[n, g].append(chosen)
                if f"{cid}.{pin}" == nets[n]["pins"][0]:
                    roots[n, g].append(chosen)
        if not options:
            raise ValueError(f"No legal placements for {cid}")
        model.add_exactly_one(chosen for chosen, _ in options)
        choices[cid] = options

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
            wires.append((chosen, part, holes))
            a, b = sorted((groups[holes[0]], groups[holes[1]]))
            edges[a, b].append(chosen)
            model.add(labels[a] == labels[b]).only_enforce_if(chosen)
            model.add(labels[a] >= 0).only_enforce_if(chosen)
            for hole in holes:
                occupancy[hole].append(chosen)
    for occupants in occupancy.values():
        model.add_at_most_one(occupants)

    # Send abstract flow from each net's first pin to all its pins.
    # Flow can cross strips only on installed jumpers carrying that net.
    balance = defaultdict(list)
    for (a, b), candidates in edges.items():
        linked = model.new_bool_var(f"link_{a}_{b}")
        model.add(sum(candidates) == linked)  # Parallel wires between strips are redundant.
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
    solver.parameters.max_time_in_seconds = time_limit
    solver.parameters.num_search_workers = 8
    status = solver.solve(model)
    if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        raise ValueError(
            f"No mapping returned: {solver.status_name(status)}. "
            "UNKNOWN means the time limit was reached without a solution."
        )
    pin_to_hole = {}
    for cid, options in choices.items():
        holes = next(holes for chosen, holes in options if solver.value(chosen))
        pin_to_hole.update(
            {f"{cid}.{pin}": hole for pin, hole in zip(instances[cid]["pins"], holes, strict=True)}
        )
    jumpers = []
    for chosen, part, holes in wires:
        if solver.value(chosen):
            jumpers.append(
                {
                    "part_id": part["part_id"],
                    "pins": dict(zip(part["pins"], holes, strict=True)),
                    "net": nets[solver.value(labels[groups[holes[0]]])]["id"],
                }
            )
    result = {
        "circuit_id": circuit["circuit_id"],
        "board_id": board["board_id"],
        "status": solver.status_name(status),
        "pin_to_hole": pin_to_hole,
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

    occupied = list(result["pin_to_hole"].values())
    for jumper in result["jumpers"]:
        a, b = jumper["pins"].values()
        occupied.extend((a, b))
        parent[find(groups[a])] = find(groups[b])
    if len(occupied) != len(set(occupied)):
        raise ValueError("Invalid result: a hole is occupied more than once")
    owners = {}
    for net in nets:
        connected = {find(groups[result["pin_to_hole"][pin]]) for pin in net["pins"]}
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
    args = parser.parse_args()
    if args.time_limit <= 0:
        parser.error("--time-limit must be positive")
    try:
        result = map_circuit(
            read_json(args.circuit),
            read_json(args.inventory),
            read_json(args.board),
            args.time_limit,
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
