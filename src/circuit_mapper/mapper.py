"""Place straight-line components and route their nets with rigid jumpers."""

import argparse
import json
from collections import defaultdict
from itertools import combinations
from math import ceil
from pathlib import Path
from time import monotonic

from ortools.sat.python import cp_model

if __package__:
    from .geometry import (
        casing_for,
        constrain_assembly,
        no_fit_cliques,
        placement_geometry,
        relation,
        verify_assembly,
    )
    from .inventory import checked_parts, limit_jumpers
    from .power import fixed_power_connections
else:
    from geometry import (
        casing_for,
        constrain_assembly,
        no_fit_cliques,
        placement_geometry,
        relation,
        verify_assembly,
    )
    from inventory import checked_parts, limit_jumpers
    from power import fixed_power_connections

HERE = Path(__file__).parent
SOLVERS = ("v2", "v1", "v3")  # v2: this file (10-27 overnight work); v1: mapper_v1.py (before it); v3: mapper_v3.py
DEFAULT_CASINGS = HERE.parents[1] / "assets/models/component_casings/casings.json"
# (row window, jumper budget) per stage. The window is a multiple of the first window's
# rows, centred on the board (None: every row); a None budget allows any number of jumpers.
# Each stage's layouts include the previous stage's. Each stage gets an equal share of the
# time left, so one that cannot finish never starves the later ones.
STAGES = ((1, 0), (1, 1), (1, 2), (1, 4), (2, 4), (None, None))
# Rows of the first window: 1.5x the robot parts' casing length per board half, clamped.
FIRST_WINDOW_ROWS = (12, 24)


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


def strip_capacity(parts, positions, groups, shape, margin):
    """Most casing pins that fit together in one strip, over a middle strip of each board half
    (an edge strip has a subset of its placements). Pairwise no-fit only, so an upper bound."""
    rows = sorted({y for _, y in positions.values()})
    mid = rows[len(rows) // 2]
    reach = max(part["pin_spacing"] * (len(part["pins"]) - 1) for part in parts)
    near = {hole: xy for hole, xy in positions.items() if abs(xy[1] - mid) <= reach}
    best = 0
    for column in ("A", "J"):
        strip = groups[f"{column}{int(mid) + 1}"]
        model = cp_model.CpModel()
        options, occupancy = [], defaultdict(list)
        for part in parts:
            for holes in placements(part, near):
                inside = sum(groups[hole] == strip for hole in holes)
                if inside != 1:
                    continue  # Two pins in one strip short them (no part here joins a net).
                chosen = model.new_bool_var("")
                options.append((chosen, shape(part, holes)))
                for hole in holes:
                    occupancy[hole].append(chosen)
        for occupants in occupancy.values():
            model.add_at_most_one(occupants)
        for (a, sa), (b, sb) in combinations(options, 2):
            if relation(sa, sb, margin) == "forbidden":
                model.add_bool_or([~a, ~b])
        model.maximize(sum(chosen for chosen, _ in options))
        solver = cp_model.CpSolver()
        solver.parameters.max_time_in_seconds = 10
        if solver.solve(model) != cp_model.OPTIMAL:
            return None
        best = max(best, round(solver.objective_value))
    return best


def step_key(candidate):
    """A wire's ID depends on the window, so replay finds a wire by its holes."""
    if candidate["id"].startswith("jumper:"):
        return "jumper:", tuple(sorted(candidate["holes"]))
    return candidate["id"], candidate["holes"]


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
    parts = checked_parts(circuit, inventory)
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
    fixed_power = fixed_power_connections(circuit, manual_pins, positions, groups, pin_net)
    group_count = len(board["conductive_groups"])
    margin = gripper["safety_margin_mm"]
    jumper_parts = [part for part in parts.values() if part["type"] == "jumper"]
    if any(len(part["pins"]) != 2 for part in jumper_parts):
        raise ValueError("Routing requires two-pin jumper parts")
    stocked = all(type(part.get("quantity")) is int for part in jumper_parts)
    pieces = [p for p in jumper_parts for _ in range(p["quantity"] if stocked else 1)]
    shape_cache = {}
    rules = {}  # relation() by kinds and centre offset, for every stage

    def shape(part, holes):
        key = part["part_id"], holes
        if key not in shape_cache:
            shape_cache[key] = placement_geometry(
                holes, positions, shapes[part["part_id"]], gripper, pitch
            )
        return shape_cache[key]

    # Strips joined by k jumpers have k + 1 strips' room and the jumpers fill 2k of it, so with
    # room for 2 casing pins per strip a net never takes more than 2 robot-placed pins. (Not
    # for parts that join a net's strips themselves; a bodyless wire would need the same care.)
    robot_pins = defaultdict(int)
    for cid, part in physical.items():
        for pin in part["pins"]:
            robot_pins[pin_net[f"{cid}.{pin}"]] += 1
    crowded = [n for n, count in robot_pins.items() if count > 2]
    joins_net = any(
        len({pin_net[f"{cid}.{pin}"] for pin in part["pins"]}) < len(part["pins"])
        for cid, part in physical.items()
    )
    if crowded and not joins_net:
        capacity = strip_capacity(
            list(used_parts.values()), positions, groups, shape, margin
        )
        if capacity is not None and capacity <= 2:
            raise ValueError(
                "No mapping returned: INFEASIBLE. "
                + ", ".join(f"net {nets[n]['id']} has {robot_pins[n]} robot-placed pins" for n in crowded)
                + f"; at most {capacity} casings fit in one 5-hole strip, and every rigid jumper"
                " fills the room it adds, so no number of jumpers connects more than 2."
                " Give each net at most 2 robot-placed parts (join the rest with hand-wired links)."
            )

    def build(budget, inside, until):
        """One stage's model: at most budget jumpers (None: any number, 0: no wire variables),
        with every part, wire and manual pin in the holes of inside."""
        model = cp_model.CpModel()
        # Each conductive strip can belong to only one net (-1 means unused).
        labels = [model.new_int_var(-1, len(nets) - 1, f"net_{g}") for g in range(group_count)]
        candidates = []
        occupancy = defaultdict(list)
        terminals, roots = defaultdict(list), defaultdict(list)
        choices = {}
        for owner, (cid, part) in enumerate(physical.items()):
            options = []
            for holes in placements(part, inside):
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
                        "shape": shape(part, holes),
                    }
                )
                for hole in holes:
                    occupancy[hole].append(chosen)
                for pin, (n, g) in zip(part["pins"], assignments, strict=True):
                    model.add(labels[g] == n).only_enforce_if(chosen)
                    terminals[n, g].append(chosen)
                    if f"{cid}.{pin}" == nets[n]["pins"][0]:
                        roots[n, g].append(chosen)
            if not options and len(inside) == len(positions):
                raise ValueError(f"No legal placements for {cid}")
            # In a window, no options only makes this stage infeasible.
            model.add_exactly_one(chosen for chosen, _ in options)
            choices[cid] = options

        # Manual connections are individual reserved holes, without a body or grip.
        manual_choices = {}
        for pin in sorted(manual_pins):
            n = pin_net[pin]
            options = []
            for hole in [fixed_power[pin]] if pin in fixed_power else inside:
                g = groups[hole]
                chosen = model.new_bool_var(f"manual_{pin}_{hole}")
                options.append((chosen, hole))
                occupancy[hole].append(chosen)
                model.add(labels[g] == n).only_enforce_if(chosen)
                terminals[n, g].append(chosen)
                if pin == nets[n]["pins"][0]:
                    roots[n, g].append(chosen)
            model.add_exactly_one(chosen for chosen, _ in options)
            manual_choices[pin] = options

        # A jumper in stock is that many pieces, each placed at most once, so a piece is an
        # owner like a part and its collisions are eager cliques too. Unlimited jumpers stay
        # one owner per wire, and wire-wire pairs lazy.
        wires, edges = [], defaultdict(list)
        previous = {}  # part_id: (used, position) of the piece before, for symmetry breaking
        for piece, part in enumerate(pieces if budget != 0 else ()):
            seen_pairs, used = set(), []
            for holes in placements(part, inside):
                pair = tuple(sorted(holes))
                if pair in seen_pairs or groups[holes[0]] == groups[holes[1]]:
                    continue
                seen_pairs.add(pair)
                chosen = model.new_bool_var(f"wire_{len(wires)}")
                used.append(chosen)
                candidates.append(
                    {
                        "chosen": chosen,
                        "owner": len(physical) + (piece if stocked else len(wires)),
                        "id": f"jumper:{len(wires) + 1}",
                        "part_id": part["part_id"],
                        "holes": holes,
                        "shape": shape(part, holes),
                    }
                )
                for hole in holes:
                    occupancy[hole].append(chosen)
                wires.append((chosen, part, holes))
                a, b = sorted((groups[holes[0]], groups[holes[1]]))
                edges[a, b].append(chosen)
                model.add(labels[a] == labels[b]).only_enforce_if(chosen)
                model.add(labels[a] >= 0).only_enforce_if(chosen)
            if stocked and used:
                model.add_at_most_one(used)
                # Identical pieces: the second one is used only after the first, further on.
                on = model.new_bool_var(f"piece_{piece}")
                model.add(sum(used) == on)
                at = model.new_int_var(0, len(used), f"piece_{piece}_at")
                model.add(at == sum(i * c for i, c in enumerate(used, 1)))
                if part["part_id"] in previous:
                    on_before, at_before = previous[part["part_id"]]
                    model.add_implication(on, on_before)
                    model.add(at > at_before).only_enforce_if(on)
                previous[part["part_id"]] = on, at
        limit_jumpers(model, wires, parts)
        if budget is not None and wires:
            model.add(sum(chosen for chosen, _, _ in wires) <= budget)
        for occupants in occupancy.values():
            model.add_at_most_one(occupants)
        # Collisions with a part or a stock piece are excluded up front; only wire pairs of
        # unlimited jumpers and cycles stay lazy. The lazy cuts are exact anyway, so posting
        # stops when the stage's time is up.
        owners = len(physical) + (len(pieces) if stocked else 0)
        for clique in no_fit_cliques(candidates, owners, margin, rules):
            if monotonic() > until:
                break
            model.add_at_most_one(clique)

        # Send abstract flow from each net's first pin to all its pins.
        # Flow can cross strips only on installed jumpers carrying that net.
        # An installed wire carries nonzero flow, so none dangles. A wire cycle takes three
        # wires (parallel wires are excluded); from that budget on, flow only runs to a
        # higher strip potential, so it cannot circulate around a cycle either.
        balance = defaultdict(list)
        strips = {g for edge in edges for g in edge} if budget is None or budget >= 3 else ()
        potential = {g: model.new_int_var(0, len(strips), f"potential_{g}") for g in strips}
        for (a, b), edge_candidates in edges.items():
            linked = model.new_bool_var(f"link_{a}_{b}")
            # Parallel wires between strips are redundant.
            model.add(sum(edge_candidates) == linked)
            if potential:
                forward = model.new_bool_var(f"forward_{a}_{b}")
                model.add(potential[b] > potential[a]).only_enforce_if(linked, forward)
                model.add(potential[a] > potential[b]).only_enforce_if(linked, ~forward)
            actives = []
            for n, net in enumerate(nets):
                capacity = len(net["pins"])
                active = model.new_bool_var(f"active_{n}_{a}_{b}")
                actives.append(active)
                model.add(labels[a] == n).only_enforce_if(active)
                flow = model.new_int_var(-capacity, capacity, f"flow_{n}_{a}_{b}")
                model.add(flow <= capacity * active)
                model.add(flow >= -capacity * active)
                model.add(flow != 0).only_enforce_if(active)
                if potential:
                    model.add(flow >= 0).only_enforce_if(forward)
                    model.add(flow <= 0).only_enforce_if(~forward)
                balance[n, a].append(flow)
                balance[n, b].append(-flow)
            model.add(sum(actives) == linked)
        for n, net in enumerate(nets):
            for g in {groups[hole] for hole in inside}:
                model.add(
                    sum(balance[n, g]) == len(net["pins"]) * sum(roots[n, g]) - sum(terminals[n, g])
                )
        return model, labels, candidates, choices, manual_choices, wires

    def priority(step):
        """Among steps ready to insert: jumpers, slim casings, LEDs (flange band) last; by row."""
        kind = used_parts[step["part_id"]]["type"]
        rank = 0 if kind == "jumper" else 2 if kind in ("led", "rgb_led") else 1
        return rank, step["shape"]["center_mm"][1], step["id"]

    rows = sorted({y for _, y in positions.values()})
    length = sum(shapes[part["part_id"]]["width_x"] for part in physical.values()) / 2
    first = min(max(ceil(1.5 * length / pitch), FIRST_WINDOW_ROWS[0]), FIRST_WINDOW_ROWS[1])

    def window(width):
        """Widen from row 1 for fixed power; otherwise stay centered on the board."""
        size = len(rows) if width is None else min(width * first, len(rows))
        start = 0 if fixed_power else (len(rows) - size) // 2
        kept = set(rows[start : start + size])
        return {hole: xy for hole, xy in positions.items() if xy[1] in kept}

    solver = cp_model.CpSolver()
    # Keep at least 4 workers: 1 has no feasibility-jump and never found a layout.
    solver.parameters.num_workers = 8
    # Presolve reruns every lazy round and probing dominated it.
    solver.parameters.cp_model_probing_level = 0
    solver.parameters.max_presolve_iterations = 1
    # Pure feasibility: the stage budget, not an objective, keeps jumpers few.
    solver.parameters.stop_after_first_solution = True
    deadline = monotonic() + time_limit
    checked = []  # Every checked layout; its cuts are valid in every later stage.
    lower = 0  # Fewer jumpers than this were proven INFEASIBLE on the whole board.
    for stage, (width, budget) in enumerate(STAGES):
        now = monotonic()
        if now >= deadline:
            status = cp_model.UNKNOWN
            continue
        stage_end = now + (deadline - now) / (len(STAGES) - stage)
        inside = window(width)
        model, labels, candidates, choices, manual_choices, wires = build(budget, inside, stage_end)
        by_key = {step_key(c): c for c in candidates}
        known = set()
        for keys in checked:
            replay = [by_key[k] for k in keys if k in by_key]
            constrain_assembly(model, replay, known, margin)
        while True:
            remaining = stage_end - monotonic()
            if remaining <= 0:
                status = cp_model.UNKNOWN
                break
            solver.parameters.max_time_in_seconds = remaining
            status = solver.solve(model)
            if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
                break
            selected = [c for c in candidates if solver.value(c["chosen"])]
            checked.append([step_key(c) for c in selected])
            order = constrain_assembly(model, selected, known, margin, priority)
            if order is not None:
                break
        if status in (cp_model.OPTIMAL, cp_model.FEASIBLE):
            break
        if status == cp_model.INFEASIBLE and budget is not None and len(inside) == len(positions):
            # Lazy cuts are valid constraints, so this is a proof under the cuts too. A
            # window proves nothing about the rows outside it.
            lower = budget + 1
    else:
        reason = (
            "No layout satisfies the part, wiring, and assembly constraints."
            if status == cp_model.INFEASIBLE
            else "No verified layout found within the search limit; not proof of infeasibility."
        )
        raise ValueError(f"No mapping returned: {solver.status_name(status)}. {reason}")
    pin_to_hole = {}
    for cid, options in choices.items():
        holes = next(holes for chosen, holes in options if solver.value(chosen))
        pin_to_hole.update(
            {f"{cid}.{pin}": hole for pin, hole in zip(instances[cid]["pins"], holes, strict=True)}
        )
    manual_connections = {
        pin: next(h for chosen, h in options if solver.value(chosen))
        for pin, options in manual_choices.items()
    }
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
    # Flow may split over two paths into one strip; drop every jumper no net needs.
    wiring = {"pin_to_hole": pin_to_hole, "manual_connections": manual_connections}
    for jumper in list(jumpers):
        rest = [j for j in jumpers if j is not jumper]
        try:
            verify_connections({**wiring, "jumpers": rest}, nets, groups)
        except ValueError:
            continue
        jumpers = rest
    kept = {j["id"] for j in jumpers}
    selected = [c for c in order if c["owner"] < len(physical) or c["id"] in kept]
    verify_assembly(selected, margin)
    result = {
        "circuit_id": circuit["circuit_id"],
        "board_id": board["board_id"],
        # OPTIMAL: every smaller jumper count was proven INFEASIBLE.
        "status": "OPTIMAL" if len(jumpers) <= lower else "FEASIBLE",
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
        "manual_connections": manual_connections,
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
    parser.add_argument(
        "--solver",
        choices=SOLVERS,
        default="v2",
        help="v2: this file; v1: mapper_v1.py, the old one; v3: mapper_v3.py (topology, then geometry)",
    )
    parser.add_argument("--hand-wires", type=int, default=0, help="v3 only: hand wires the operator may plug")
    args = parser.parse_args()
    if args.time_limit <= 0:
        parser.error("--time-limit must be positive")
    if args.solver == "v1":
        if __package__:
            from .mapper_v1 import map_circuit as solve
        else:
            from mapper_v1 import map_circuit as solve
    elif args.solver == "v3":
        if __package__:
            from .mapper_v3 import map_circuit as solve3
        else:
            from mapper_v3 import map_circuit as solve3

        def solve(*a):
            return solve3(*a, hand_wires=args.hand_wires)
    else:
        solve = map_circuit
    try:
        result = solve(
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
