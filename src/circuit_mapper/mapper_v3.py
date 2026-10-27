"""Breadboard layout in two levels: topology first, then the exact geometry (v3).

Topology. Every robot part and every stocked jumper piece is an *element*. The strips its pins
land in are its *route*: a straight line of holes fixes them (rows r and r + s on one board
half, or one row across the E-F channel), and only the column is left open. A small CP-SAT
model picks one route per part, at most one per jumper piece and one strip per hand-wired pin:
  - a strip carries at most one net; a route puts each pin on its net's strip;
  - every net's strips are joined by its jumpers into one tree (each strip but the one holding
    the net's first pin has a parent strip, one level lower, through a chosen jumper);
  - a strip takes no more pins than holes, nor more casings than fit in it;
  - two routes that no columns can make fit together are never chosen together (exact, from
    relation(); the same at every row, so tabulated once per casing pair and row offset);
  - fewest jumpers, then fewest rows.

Geometry. With the routes fixed, a second small model picks the columns and the hand-wired
holes: one pin per hole, no forbidden pair, and an assembly order in which no fingers hit an
earlier body. If none exists, CP-SAT names a core of elements that cannot go together. The core
depends only on the casings and the strips, so it is forbidden for every element of those
casings, in either pin order, at every row shift and mirrored to the other board half; then
the topology is solved again.

The topology starts on a window of rows (from row 1 with the fixed supply, else centred) and
widens when the window is proven too small. Every returned layout passes verify_connections
and verify_assembly, as v2's do.

Compared with v2 (mapper.py): no flow model (a parent/level tree instead), no stages of
jumper budgets (the objective counts the jumpers), no candidate-level clique tables (pairs are
decided per route, columns only in the small geometry model) and no lazy collision rounds (the
geometry model is exact; only its cores come back).
"""

import math
import os
import sys
import threading
from collections import defaultdict
from itertools import combinations
from time import monotonic

from ortools.sat.python import cp_model

if __package__:
    from .geometry import casing_for, constrain_assembly, no_fit_cliques, placement_geometry, relation, verify_assembly
    from .inventory import checked_parts
    from .mapper import DEFAULT_CASINGS, HERE, board_geometry, placements, read_json, strip_capacity, verify_connections
    from .power import fixed_power_connections
else:
    from geometry import casing_for, constrain_assembly, no_fit_cliques, placement_geometry, relation, verify_assembly
    from inventory import checked_parts
    from mapper import DEFAULT_CASINGS, HERE, board_geometry, placements, read_json, strip_capacity, verify_connections
    from power import fixed_power_connections

UNSTOCKED_PIECES = 4      # pieces per jumper part without a quantity
HAND_REACH = 8            # rows a hand wire may span
HAND_COST, JUMPER_COST = 10000, 1000   # objective: hand wires, then robot jumpers, then rows
WINDOWS = (20, 36, None)  # rows of the topology windows, widest last (None: the whole board)
WORKERS = 8
ROBOT_SHARE = 0.25         # of a window's time, for layouts without hand wires (when they are allowed)
HAND_SHARE = 0.5           # of what is left, for each next hand-wire count
ROBOT_CAP_S = 8.0          # robot-only layouts that exist show up fast; past this, try hand wires
FAST_CAP_S = 4.0           # the jumper-free model first: most small circuits need none
IMPROVE_S = 5.0            # improving stops after this many seconds without a better topology
COLUMNS = os.environ.get('V3_COLUMNS', '1') == '1'   # columns in the topology model
DEBUG = bool(os.environ.get("V3_DEBUG"))


def checked_circuit(circuit, inventory):
    """The circuit's parts, nets and pins, checked as v2 does."""
    parts = checked_parts(circuit, inventory)
    instances = {c["id"]: parts[c["part_id"]] for c in circuit["components"]}
    if any(cid.startswith("jumper:") for cid in instances):
        raise ValueError("The instance ID prefix jumper: is reserved for generated wires")
    if len(instances) != len(circuit["components"]):
        raise ValueError("Component instance IDs must be unique")
    expected = {f"{cid}.{pin}" for cid, part in instances.items() for pin in part["pins"]}
    nets = list(circuit["nets"]) + [
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
            raise ValueError("Leave jumpers out of the circuit; the mapper adds them during routing")
        for connected in part.get("connected_pin_groups", []):
            if len({pin_net[f"{cid}.{pin}"] for pin in connected}) > 1:
                raise ValueError(f"{cid}: permanently connected pins assigned to different nets")
    manual_ids = {
        c["id"] for c in circuit["components"]
        if c.get("placement_mode", instances[c["id"]].get("placement_mode")) == "manual"
    }
    return parts, instances, nets, pin_net, manual_ids


class Board:
    """Strips, their rows and halves, and the routes of each casing on them."""

    def __init__(self, board, positions, groups):
        self.positions, self.groups = positions, groups
        self.pitch = board["geometry"]["hole_pitch_mm"]
        self.hole_at = {xy: h for h, xy in positions.items()}
        self.holes = defaultdict(list)
        for hole, g in groups.items():
            self.holes[g].append(hole)
        self.strips = sorted(self.holes)
        self.row = {g: min(int(h[1:]) for h in self.holes[g]) for g in self.strips}
        self.left = {g: self.holes[g][0][0] in "ABCDE" for g in self.strips}
        self.at = {(self.left[g], self.row[g]): g for g in self.strips}
        self.rows = sorted(set(self.row.values()))

    def place(self, g):
        return self.left[g], self.row[g]


def map_circuit(circuit, inventory, board, time_limit=30, casings=None, gripper=None, hand_wires=0):
    t_end = monotonic() + time_limit
    casings = read_json(DEFAULT_CASINGS) if casings is None else casings
    gripper = read_json(HERE / "gripper.json") if gripper is None else gripper
    pitch = board["geometry"]["hole_pitch_mm"]
    if pitch <= 0 or time_limit <= 0:
        raise ValueError("Board pitch and time limit must be positive")
    margin = gripper["safety_margin_mm"]
    parts, instances, nets, pin_net, manual_ids = checked_circuit(circuit, inventory)
    manual_pins = sorted(f"{cid}.{pin}" for cid in manual_ids for pin in instances[cid]["pins"])
    physical = {cid: part for cid, part in instances.items() if cid not in manual_ids}
    positions, groups = board_geometry(board)
    fixed_power = fixed_power_connections(circuit, set(manual_pins), positions, groups, pin_net)
    B = Board(board, positions, groups)

    # ── elements: robot parts, then jumper pieces ───────────────────────────
    jumper_parts = [p for p in parts.values() if p["type"] == "jumper"]
    if any(len(p["pins"]) != 2 for p in jumper_parts):
        raise ValueError("Routing requires two-pin jumper parts")
    elements = [{"id": cid, "part": part, "pins": [f"{cid}.{p}" for p in part["pins"]]}
                for cid, part in physical.items()]
    for part in jumper_parts:
        q = part.get("quantity")
        for k in range(q if isinstance(q, int) else UNSTOCKED_PIECES):
            elements.append({"id": f"piece:{part['part_id']}:{k}", "part": part, "pins": None})
    used_parts = {e["part"]["part_id"]: e["part"] for e in elements}
    shapes = {pid: casing_for(p, casings, pitch) for pid, p in used_parts.items()}
    casing_of = {pid: shapes[pid]["casing"] for pid in used_parts}
    geo_cache = {}

    def shape(pid, holes):
        if (pid, holes) not in geo_cache:
            geo_cache[pid, holes] = placement_geometry(holes, positions, shapes[pid], gripper, pitch)
        return geo_cache[pid, holes]

    # routes: a part's strips in pin order, a jumper's as a pair in either order; each route
    # keeps its hole tuples (one per column)
    routes_of = {}
    for pid, part in used_parts.items():
        routes = defaultdict(list)
        for holes in placements(part, positions):
            key = tuple(groups[h] for h in holes)
            if len(set(key)) < len(key):
                continue        # two pins in one strip: a short for a part, nothing for a jumper
            if part["type"] == "jumper":
                key = tuple(sorted(key))
                if holes[::-1] in routes[key]:
                    continue
            routes[key].append(holes)
        routes_of[pid] = dict(routes)
    for e in elements:
        e["routes"] = routes_of[e["part"]["part_id"]]
    # hand wires (opt-in): no casing, plugged by the operator after the robot, between any two
    # strips at most HAND_REACH rows apart (a wire bends); holes picked with the columns
    hand_routes = {(a, b): [] for a in B.strips for b in B.strips if a < b and abs(B.row[a] - B.row[b]) <= HAND_REACH}
    for k in range(hand_wires):
        elements.append({"id": f"hand:{k}", "part": None, "pins": None, "hand": True, "routes": hand_routes})

    # A route's geometry: its casing and the set of its strips (casings are symmetric under a
    # half turn). shift() and mirror() move it; geo() of a route is its key.
    def geo(c, route):
        return c, frozenset(B.place(g) for g in route)

    routes_by_geo = defaultdict(list)      # geo key -> [(pid, route)]
    for pid, routes in routes_of.items():
        for route in routes:
            routes_by_geo[geo(casing_of[pid], route)].append((pid, route))

    pid_of = {}
    for pid in used_parts:
        pid_of.setdefault(casing_of[pid], pid)
    ext = {}            # how far along the rows a casing's body or fingers reach past its pins (mm)
    for pid in used_parts:
        c, far = casing_of[pid], 0.0
        for options in routes_of[pid].values():
            sh = shape(pid, options[0])
            ys = [positions[h][1] * pitch for h in options[0]]     # rows run along y
            for box in (sh["body"], *sh["fingers"]):
                far = max(far, min(ys) - box[1], box[3] - max(ys))
        ext[c] = max(ext.get(c, 0.0), far)

    # ── pairwise: routes that no columns make fit together (exact) ──────────
    def fits(pa, ra, pb, rb):
        for ha in routes_of[pa][ra]:
            for hb in routes_of[pb][rb]:
                if not set(ha) & set(hb) and relation(shape(pa, ha), shape(pb, hb), margin) != "forbidden":
                    return True
        return False

    def shape_of(route):
        """Strips relative to the route's first row, and that row."""
        base = min(B.row[g] for g in route)
        return tuple((B.left[g], B.row[g] - base) for g in route), base

    by_shape = {}
    for pid, routes in routes_of.items():
        for route in routes:
            rel, base = shape_of(route)
            by_shape[pid, rel, base] = route
    conflicts = {}

    def conflicts_of(pa, pb):
        """For each route of part pa, the routes of part pb that cannot go with it: tested
        once per shape pair and row offset in the middle of the board, the same everywhere."""
        if (pa, pb) in conflicts:
            return conflicts[pa, pb]
        reach = ext[casing_of[pa]] + ext[casing_of[pb]] + margin
        rels_a = {shape_of(r)[0] for r in routes_of[pa]}
        rels_b = {shape_of(r)[0] for r in routes_of[pb]}
        mid = B.rows[len(B.rows) // 2]
        dmax = int(reach // pitch) + 12
        bad = defaultdict(list)
        for rel_a in rels_a:
            ra = by_shape.get((pa, rel_a, mid))
            if ra is None:
                continue
            hi_a = mid + max(r for _, r in rel_a)
            for rel_b in rels_b:
                span_b = max(r for _, r in rel_b)
                for d in range(-dmax, dmax + 1):
                    lo_b = mid + d
                    if max(0, max(mid, lo_b) - min(hi_a, lo_b + span_b)) * pitch >= reach:
                        continue        # farther apart along the rows than anything reaches
                    rb = by_shape.get((pb, rel_b, lo_b))
                    if rb is not None and not fits(pa, ra, pb, rb):
                        bad[rel_a].append((rel_b, d))
        out = {}
        for ra in routes_of[pa]:
            rel_a, base = shape_of(ra)
            out[ra] = [rb for rel_b, d in bad[rel_a] if (rb := by_shape.get((pb, rel_b, base + d))) is not None]
        conflicts[pa, pb] = out
        return out

    capacity = None
    if len(physical) >= 2:
        capacity = strip_capacity(list(used_parts.values()), positions, groups,
                                  lambda part, holes: shape(part["part_id"], holes), margin)

    # ── how many hand wires the strips force (v2's capacity bound) ──────────
    # A strip holds at most `capacity` robot casings. A net on s strips joined by j robot jumpers
    # and h hand wires has s = 1 + j + h, room for capacity * s casing pins, and the jumpers
    # take 2j of it: k robot pins fit only if k <= capacity * (1 + h) (with capacity 2).
    robot_pins = defaultdict(int)
    for cid, part in physical.items():
        for pin in part["pins"]:
            robot_pins[pin_net[f"{cid}.{pin}"]] += 1
    joins_net = any(len({pin_net[f"{cid}.{pin}"] for pin in part["pins"]}) < len(part["pins"])
                    for cid, part in physical.items())
    need_hands = 0
    if capacity is not None and capacity <= 2 and not joins_net:
        need_hands = sum(max(0, -(-(k - capacity) // capacity)) for k in robot_pins.values())
    crowded = [n for n, k in robot_pins.items() if capacity and k > capacity]
    if need_hands and not hand_wires:
        raise ValueError(
            "No mapping returned: INFEASIBLE. "
            + ", ".join(f"net {nets[n]['id']} has {robot_pins[n]} robot-placed pins" for n in crowded)
            + f"; at most {capacity} casings fit in one 5-hole strip, and every rigid jumper fills the room it"
            " adds, so no number of jumpers connects more than 2. Give each net at most 2 robot-placed parts"
            " (join the rest with hand-wired links), or allow hand wires.")
    if need_hands > hand_wires:
        raise ValueError(f"No mapping returned: INFEASIBLE. These nets need at least {need_hands} hand wires; "
                         f"{hand_wires} are allowed.")

    patterns = []        # geometric cores: normalized (casing, strips) keys, valid in every model
    exact = []           # other cores: [((element id, route), ...), ((hand pin, strip), ...)]

    def window_rows(size):
        if size is None or size >= len(B.rows):
            return set(B.rows)
        start = 0 if fixed_power else (len(B.rows) - size) // 2
        return set(B.rows[start:start + size])

    stats = {"rounds": 0, "nogoods": 0, "models": 0, "phase": None}

    def build(els, rows):
        stats["models"] += 1
        model = Topology(B, rows, els, casing_of, nets, pin_net, manual_pins, fixed_power, capacity, conflicts_of,
                         shape if COLUMNS else None, margin)
        for pattern in patterns:
            model.forbid_pattern(pattern)
        index = {e["id"]: i for i, e in enumerate(els)}
        for routes, hands in exact:
            if all(eid in index for eid, _ in routes):
                model.forbid_exact([(index[eid], r) for eid, r in routes], hands)
        return model

    def attempt(model, els, deadline, improve=IMPROVE_S, hint=None):
        """Topology and geometry until a layout, a proof that none exists, or the deadline:
        ("found", (layout, chosen, pins_at, bound)) | ("infeasible", None) | ("unknown", None)."""
        while True:
            left = min(deadline, t_end) - monotonic()
            if left <= 0:
                return "unknown", None
            status, solution = model.solve(left, improve, hint)
            stats["rounds"] += 1
            if status != "found":
                return status, None
            chosen, pins_at, bound = solution
            layout, core = geometry(els, chosen, pins_at, fixed_power, B, shape, margin, t_end)
            if layout is not None:
                return "found", (layout, chosen, pins_at, bound)
            stats["nogoods"] += 1
            picked = [(i, r) for i, r in chosen if i in core]
            hands = [(p, g) for p, g in pins_at.items() if p in core]
            if DEBUG:
                print("core", [(els[i]["id"], [B.holes[g][0] for g in r]) for i, r in picked],
                      {p: B.holes[g][0] for p, g in hands}, file=sys.stderr)
            keys = [geo(casing_of[els[i]["part"]["part_id"]], r) for i, r in picked if els[i]["part"]]
            if hands or not picked or len(set(keys)) < len(picked):
                exact.append(([(els[i]["id"], r) for i, r in picked], hands))
                model.forbid_exact(picked, hands)
            else:
                pattern = normalize(keys)
                patterns.append(pattern)
                model.forbid_pattern(pattern)

    def hands_in(els, chosen):
        return sum(1 for i, _ in chosen if not els[i]["part"])

    found = None         # (els, layout, chosen, pins_at, bound, optimal)
    # 1. no jumpers, no hand wires: the easy circuits, on a small model
    parts_only = [e for e in elements if e["pins"] is not None]
    if not need_hands:
        model = build(parts_only, window_rows(WINDOWS[0]))
        model.allow_hands(0)
        status, sol = attempt(model, parts_only, monotonic() + FAST_CAP_S, improve=1.0)
        stats["phase"] = "no jumpers"
        if status == "found":
            found = (parts_only, *sol, True)         # 0 jumpers: nothing has fewer
    # 2. robot jumpers, no hand wires
    robot = [e for e in elements if e["part"]]
    if found is None and not need_hands:
        for size in WINDOWS:
            rows = window_rows(size)
            model = build(robot, rows)
            model.allow_hands(0)
            budget = t_end - monotonic()
            deadline = monotonic() + (min(budget * ROBOT_SHARE, ROBOT_CAP_S) if hand_wires else budget / 2)
            if size is None:
                deadline = t_end if not hand_wires else deadline
            status, sol = attempt(model, robot, deadline)
            stats["phase"] = "robot"
            if status == "found":
                # OPTIMAL: on the whole board the topology bound proves the jumper count minimal
                found = (robot, *sol, len(rows) == len(B.rows) and sol[3] >= 0)
                break
            if status == "unknown" or monotonic() >= t_end:
                break
        if found is None and not hand_wires:
            raise ValueError("No mapping returned: " + ("INFEASIBLE. No layout satisfies the part, wiring, and "
                             "assembly constraints." if status == "infeasible" else "UNKNOWN. No verified layout "
                             "found within the search limit; not proof of infeasibility."))
    # 3. hand wires: a first layout with up to all of them, then fewer, one at a time, from the best
    if found is None:
        model = build(elements, window_rows(WINDOWS[0]))
        model.allow_hands(hand_wires)
        status, sol = attempt(model, elements, t_end, improve=0.0)
        stats["phase"] = "hand"
        if status != "found":
            model = build(elements, window_rows(None))
            model.allow_hands(hand_wires)
            status, sol = attempt(model, elements, t_end, improve=0.0)
        if status != "found":
            raise ValueError("No mapping returned: " + ("INFEASIBLE. No layout, even with hand wires."
                             if status == "infeasible" else "UNKNOWN. No verified layout found within the search "
                             "limit, even with hand wires; not proof of infeasibility."))
        # Fewer wires, lowest count first, each try hinted from the best layout so far. `proven`:
        # fewer are impossible (the capacity bound, or a proof below); the robot-only step already
        # failed, so the first count tried is 1.
        best, proven = sol, need_hands
        cap = max(need_hands, 1)
        while cap < hands_in(elements, best[1]) and monotonic() < t_end:
            model.allow_hands(cap)
            left = t_end - monotonic()
            status, sol = attempt(model, elements, monotonic() + left * HAND_SHARE, improve=0.0,
                                  hint=(best[1], best[2]))
            if status == "found":
                best = sol
                break
            if status == "infeasible" and proven == cap:
                proven = cap + 1
            cap += 1
        # then the fewest robot jumpers with that many hand wires
        h = hands_in(elements, best[1])

        def robot_jumpers(chosen):
            return sum(1 for i, _ in chosen if elements[i]["part"] and elements[i]["pins"] is None)

        if robot_jumpers(best[1]) and monotonic() < t_end:
            model.allow_hands(h)
            status, sol = attempt(model, elements, t_end, improve=IMPROVE_S, hint=(best[1], best[2]))
            if status == "found" and hands_in(elements, sol[1]) <= h and robot_jumpers(sol[1]) < robot_jumpers(best[1]):
                best = sol
        jumpers_used = robot_jumpers(best[1])
        found = (elements, *best, h <= proven and jumpers_used == 0)
    els, layout, chosen, pins_at, bound, optimal = found
    elements = els


    # ── the result, in v2's format ──────────────────────────────────────────
    placed, pin_to_hole, manual_connections = layout
    net_of_strip = {}
    for pin, hole in {**pin_to_hole, **manual_connections}.items():
        net_of_strip[groups[hole]] = nets[pin_net[pin]]["id"]
    jumpers_out, steps = [], []
    for owner, (i, holes) in enumerate(placed):
        e = elements[i]
        if not e["part"]:           # a hand wire: drawn and checked like a jumper, no robot step
            jumpers_out.append({"id": f"wire:{sum(1 for j in jumpers_out if j.get('hand')) + 1}", "part_id": "hand_wire",
                                "hand": True, "pins": {"a": holes[0], "b": holes[1]}, "net": None,
                                "_strips": [groups[h] for h in holes]})
            continue
        pid = e["part"]["part_id"]
        sid = e["id"]
        if e["pins"] is None:
            sid = f"jumper:{sum(1 for j in jumpers_out if not j.get('hand')) + 1}"
            jumpers_out.append({"id": sid, "part_id": pid, "pins": dict(zip(e["part"]["pins"], holes, strict=True)),
                                "net": None, "_strips": [groups[h] for h in holes]})
        steps.append({"id": sid, "part_id": pid, "holes": holes, "shape": shape(pid, holes), "owner": owner, "chosen": None})
    # a jumper's net: the net of the strips it joins (through other jumpers if need be)
    for _ in jumpers_out:
        for j in jumpers_out:
            a, b = j["_strips"]
            net = net_of_strip.get(a) or net_of_strip.get(b)
            if net:
                net_of_strip[a] = net_of_strip[b] = j["net"] = net
    for j in jumpers_out:
        j.pop("_strips")

    def priority(step):
        kind = used_parts[step["part_id"]]["type"]
        return (0 if kind == "jumper" else 2 if kind in ("led", "rgb_led") else 1), step["shape"]["center_mm"][1], step["id"]

    order = constrain_assembly(None, steps, set(), margin, priority)
    if order is None:
        raise ValueError("Internal error: the geometry model returned a colliding layout")
    verify_assembly(order, margin)
    result = {
        "circuit_id": circuit["circuit_id"],
        "board_id": board["board_id"],
        # OPTIMAL: on the whole board, no layout has fewer jumpers
        # OPTIMAL: no layout has fewer hand wires, nor (with as many) fewer robot jumpers
        "status": "OPTIMAL" if optimal else "FEASIBLE",
        "geometry_model": "2d_casing_and_opening_fingers",
        "gripper": gripper,
        "placements": [
            {"id": c["id"], "part_id": c["part_id"], "orientation_deg": c["shape"]["orientation_deg"],
             "center_mm": c["shape"]["center_mm"], "casing_id": c["shape"]["casing_id"]}
            for c in order
        ],
        "assembly_order": [c["id"] for c in order],
        "pin_to_hole": pin_to_hole,
        "manual_connections": manual_connections,
        "jumpers": jumpers_out,
        "solver": {"version": "v3", **stats},
    }
    verify_connections(result, nets, groups)
    return result


def near_holes(B, box):
    """The holes strictly inside a box (mm, x along the columns, y along the rows)."""
    lo_c, hi_c = math.floor(box[0] / B.pitch), math.ceil(box[2] / B.pitch)
    lo_r, hi_r = math.floor(box[1] / B.pitch), math.ceil(box[3] / B.pitch)
    out = []
    for col in range(lo_c, hi_c + 1):
        for row in range(lo_r, hi_r + 1):
            h = B.hole_at.get((col, row))
            if h and box[0] < col * B.pitch < box[2] and box[1] < row * B.pitch < box[3]:
                out.append(h)
    return out


def normalize(keys):
    """A core's geo keys with the first row at 0, as the smaller of itself and its mirror image."""
    base = min(r for _, places in keys for _, r in places)

    def form(flip):
        return tuple(sorted((c, tuple(sorted(((left != flip), r - base) for left, r in places))) for c, places in keys))

    return min(form(False), form(True))


class Topology:
    """The topology model on a window of rows; solve() -> ("found", (routes, hand strips, jumper
    bound)) | ("infeasible", None) | ("unknown", None)."""

    def __init__(self, B, rows, elements, casing_of, nets, pin_net, manual_pins, fixed_power, capacity, conflicts_of,
                 shape=None, margin=0.0):
        self.B, self.elements = B, elements
        m = self.m = cp_model.CpModel()
        strips = [g for g in B.strips if B.row[g] in rows]
        inside = set(strips)
        N = range(len(nets))
        y = self.y = {}
        for i, e in enumerate(elements):
            for route in e["routes"]:
                if all(g in inside for g in route):
                    y[i, route] = m.new_bool_var("")
        self.by_geo = defaultdict(list)
        of = defaultdict(list)          # element: its route literals
        for (i, route), lit in y.items():
            of[i].append(lit)
            if elements[i]["part"]:
                c = casing_of[elements[i]["part"]["part_id"]]
                self.by_geo[c, frozenset(B.place(g) for g in route)].append(lit)
        for i, e in enumerate(elements):
            (m.add_at_most_one if e["pins"] is None else m.add_exactly_one)(of[i])
        pieces = [i for i, e in enumerate(elements) if e["pins"] is None]      # jumper pieces and hand wires
        used_piece = {i: sum(of[i]) for i in pieces}

        def kind(e):
            return e["part"]["part_id"] if e["part"] else "hand"

        for a, b in zip(pieces, pieces[1:]):        # identical pieces: the second only after the first
            if kind(elements[a]) == kind(elements[b]):
                m.add(used_piece[b] <= used_piece[a])
        z = self.z = {(g, n): m.new_bool_var("") for g in strips for n in N}
        for g in strips:
            m.add_at_most_one(z[g, n] for n in N)
        pins_in, casings_in, first_in = defaultdict(list), defaultdict(list), defaultdict(list)
        for (i, route), lit in y.items():
            e = elements[i]
            if e["pins"] is None:
                a, b = route
                for n in N:
                    m.add(z[a, n] == z[b, n]).only_enforce_if(lit)
                m.add_bool_or([z[a, n] for n in N]).only_enforce_if(lit)
            else:
                for pin, g in zip(e["pins"], route, strict=True):
                    n = pin_net[pin]
                    m.add_implication(lit, z[g, n])
                    if nets[n]["pins"][0] == pin:
                        first_in[g, n].append(lit)
            for g in route:
                pins_in[g].append(lit)
                if e["part"]:
                    casings_in[g].append(lit)
        man = self.manual = {}
        for pin in manual_pins:
            n = pin_net[pin]
            options = [B.groups[fixed_power[pin]]] if pin in fixed_power else strips
            for g in options:
                man[pin, g] = lit = m.new_bool_var("")
                m.add_implication(lit, z[g, n])
                pins_in[g].append(lit)
                if nets[n]["pins"][0] == pin:
                    first_in[g, n].append(lit)
            m.add_exactly_one(man[pin, g] for g in options)
        for g in strips:
            m.add(sum(pins_in[g]) <= len(B.holes[g]))
            if capacity:
                m.add(sum(casings_in[g]) <= capacity)
        # every used strip holds its net's first pin or hangs from a parent strip one level lower
        level = {g: m.new_int_var(0, len(pieces), "") for g in strips}
        parents = defaultdict(list)
        for (i, (a, b)), lit in y.items():
            if elements[i]["pins"] is None:
                for s, t in ((a, b), (b, a)):
                    q = m.new_bool_var("")
                    m.add_implication(q, lit)
                    m.add(level[t] < level[s]).only_enforce_if(q)
                    parents[s].append(q)
        for g in strips:
            for n in N:
                m.add_bool_or([*first_in[g, n], *parents[g]]).only_enforce_if(z[g, n])
        # the columns in the same model: a placement per route and column, and every pair of
        # placements that relation() forbids as a clique (exact; only order cycles stay)
        if shape is not None:
            candidates = []
            for (i, route), lit in y.items():
                if not elements[i]["part"]:
                    continue        # a hand wire's holes are picked with the hand pins
                pid = elements[i]["part"]["part_id"]
                xs = []
                for holes in elements[i]["routes"][route]:
                    x = m.new_bool_var("")
                    xs.append(x)
                    candidates.append({"chosen": x, "owner": i, "id": elements[i]["id"], "part_id": pid,
                                       "holes": holes, "shape": shape(pid, holes)})
                m.add(sum(xs) == lit)
            # holes under a casing's body take no other pin: count them against their strip
            covered = defaultdict(list)
            for c in candidates:
                body = c["shape"]["body"]
                for h in near_holes(B, body):
                    if h not in c["holes"]:
                        covered[B.groups[h]].append(c["chosen"])
            for g in strips:
                if covered[g]:
                    m.add(sum(pins_in[g]) + sum(covered[g]) <= len(B.holes[g]))
            at = defaultdict(list)
            for c in candidates:
                for h in c["holes"]:
                    at[h].append(c["chosen"])
            for lits in at.values():
                m.add_at_most_one(lits)
            for clique in no_fit_cliques(candidates, len(elements), margin, {}):
                m.add_at_most_one(clique)
        # routes that no columns make fit together
        for (i, ei), (j, ej) in combinations(enumerate(elements), 2):
            if not (ei["part"] and ej["part"]):
                continue
            table = conflicts_of(ei["part"]["part_id"], ej["part"]["part_id"])
            for ra, bad in table.items():
                if (i, ra) not in y:
                    continue
                lits = [y[j, rb] for rb in bad if (j, rb) in y]
                if lits:
                    m.add(sum(lits) == 0).only_enforce_if(y[i, ra])
        # fewest jumpers, then fewest rows (from row 1 with the fixed supply, else the span)
        self.jumpers = sum(used_piece[i] for i in pieces if elements[i]["part"])
        self.hands = sum(used_piece[i] for i in pieces if not elements[i]["part"])
        top = m.new_int_var(0, max(rows), "top")
        low = m.new_int_var(0, max(rows), "low")
        for g in strips:
            if pins_in[g]:
                used = m.new_bool_var("")
                m.add_max_equality(used, pins_in[g])
                m.add(top >= B.row[g]).only_enforce_if(used)
                m.add(low <= B.row[g]).only_enforce_if(used)
        self.objective = HAND_COST * self.hands + JUMPER_COST * self.jumpers + (top if fixed_power else top - low)
        m.minimize(self.objective)
        self.hinted = [*y.values(), *man.values()]
        self.hand_cap = m.new_int_var(0, max(len(pieces), 1), "hand_cap")    # set per attempt
        m.add(self.hands <= self.hand_cap)
        self.solver = s = cp_model.CpSolver()
        s.parameters.num_workers = WORKERS
        # presolve, not search, was the cost: probing and repeated rounds took 4 s of 5 (p1)
        s.parameters.cp_model_probing_level = 0
        s.parameters.max_presolve_iterations = 1
        self.hint = None

    def allow_hands(self, count):
        """At most count hand wires: the cap's domain is set in place (assumptions would turn
        CP-SAT's presolve off)."""
        dom = self.m.proto.variables[self.hand_cap.index].domain
        dom[0] = dom[1] = count

    def solve(self, seconds, improve=IMPROVE_S, hint=None):
        """The first layout (near hint=(routes, hand strips) if given), then improve it until
        `improve` seconds pass without a better one (0: no improving)."""
        s, m = self.solver, self.m
        t0 = monotonic()
        s.parameters.max_time_in_seconds = seconds
        s.parameters.stop_after_first_solution = True
        m.clear_hints()
        if not improve:
            m.clear_objective()         # a plain feasibility search finds a first layout sooner
        if hint:
            routes, pins_at = hint
            on = {(i, r) for i, r in routes}
            for key, lit in self.y.items():
                m.add_hint(lit, key in on)
            for (pin, g), lit in self.manual.items():
                m.add_hint(lit, pins_at.get(pin) == g)
        status = s.solve(m)
        if not improve:
            m.minimize(self.objective)
        if status == cp_model.INFEASIBLE:
            return "infeasible", None
        if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
            return "unknown", None
        best = self._read(status if improve else cp_model.FEASIBLE)
        left = seconds - (monotonic() - t0)
        if status == cp_model.FEASIBLE and improve > 0 and left > 0.5:
            # improve from the first layout until the deadline, or until STALL_S pass without a better one
            m.clear_hints()
            for v in self.hinted:
                m.add_hint(v, s.value(v))
            s.parameters.stop_after_first_solution = False
            s.parameters.max_time_in_seconds = left
            last = [monotonic()]

            class Seen(cp_model.CpSolverSolutionCallback):
                def on_solution_callback(self):
                    last[0] = monotonic()

            done = threading.Event()

            def watch():
                while not done.wait(0.2):
                    if monotonic() - last[0] > improve:
                        s.stop_search()
                        return

            t = threading.Thread(target=watch, daemon=True)
            t.start()
            status = s.solve(m, Seen())
            done.set()
            t.join()
            if status in (cp_model.OPTIMAL, cp_model.FEASIBLE):
                best = self._read(status)
        return "found", best

    def _read(self, status):
        s = self.solver
        chosen = [(i, r) for (i, r), lit in self.y.items() if s.value(lit)]
        pins_at = {pin: g for (pin, g), lit in self.manual.items() if s.value(lit)}
        bound = int(s.best_objective_bound) if status == cp_model.OPTIMAL else -1
        return chosen, pins_at, bound

    def present(self, key):
        """A literal: some element of this casing takes a route with these strips."""
        lits = self.by_geo.get(key)
        if not lits:
            return None
        if len(lits) == 1:
            return lits[0]
        b = self.m.new_bool_var("")
        self.m.add_max_equality(b, lits)
        self.by_geo[key] = [b]
        return b

    def forbid_pattern(self, pattern):
        """No element set takes this core's routes together, at any row shift, on either half."""
        B = self.B
        span = max(r for _, places in pattern for _, r in places)
        for flip in (False, True):
            for base in B.rows:
                if base + span > B.rows[-1]:
                    break
                lits = []
                for casing, places in pattern:
                    strips = []
                    for left, r in places:
                        g = B.at.get((left != flip, base + r))
                        if g is None:
                            break
                        strips.append(B.place(g))
                    key = casing, frozenset(strips)
                    lit = self.present(key) if len(strips) == len(places) else None
                    if lit is None:
                        break
                    lits.append(lit)
                else:
                    self.m.add_bool_or([~x for x in lits])

    def forbid_exact(self, routes, hands):
        lits = [self.y[i, r] for i, r in routes if (i, r) in self.y]
        lits += [self.manual[p, g] for p, g in hands if (p, g) in self.manual]
        if len(lits) == len(routes) + len(hands):
            self.m.add_bool_or([~x for x in lits])


def geometry(elements, chosen, pins_at, fixed_power, B, shape, margin, t_end):
    """Columns for the fixed routes: ((placed, pin_to_hole, manual_connections), None), or
    (None, core) with the elements (indices) and hand pins that cannot be placed together."""
    m = cp_model.CpModel()
    opts, assume, at_hole = {}, {}, defaultdict(list)
    hand_holes = []     # (literal, hole): pins the operator plugs after the robot
    for i, route in chosen:
        assume[i] = a = m.new_bool_var(f"a{i}")
        e = elements[i]
        if not e["part"]:           # a hand wire: one free hole in each of its two strips
            ends = []
            for g in route:
                lits = []
                for h in B.holes[g]:
                    lit = m.new_bool_var("")
                    lits.append((lit, h))
                    at_hole[h].append(lit)
                    hand_holes.append((lit, h))
                m.add_exactly_one(l for l, _ in lits).only_enforce_if(a)
                ends.append(lits)
            opts[i] = ends
            continue
        lits = []
        for holes in e["routes"][route]:
            lit = m.new_bool_var("")
            lits.append((lit, holes))
            for h in holes:
                at_hole[h].append(lit)
        m.add_exactly_one(l for l, _ in lits).only_enforce_if(a)
        opts[i] = lits
    hand = {}
    for pin, g in pins_at.items():
        assume[pin] = a = m.new_bool_var(f"a{pin}")
        lits = []
        for h in ([fixed_power[pin]] if pin in fixed_power else B.holes[g]):
            lit = m.new_bool_var("")
            lits.append((lit, h))
            at_hole[h].append(lit)
            hand_holes.append((lit, h))
        m.add_exactly_one(l for l, _ in lits).only_enforce_if(a)
        hand[pin] = lits
    for lits in at_hole.values():
        m.add_at_most_one(lits)
    robot = {i: lits for i, lits in opts.items() if elements[i]["part"]}
    # a hole under a casing cannot take a hand-plugged pin
    for i, lits in robot.items():
        pid = elements[i]["part"]["part_id"]
        for a, ha in lits:
            body = shape(pid, ha)["body"]
            for b, h in hand_holes:
                x, y = (v * B.pitch for v in B.positions[h])
                if body[0] < x < body[2] and body[1] < y < body[3] and h not in ha:
                    m.add_bool_or([~a, ~b])
    order = {i: m.new_int_var(0, len(robot), "") for i in robot}
    for (i, li), (j, lj) in combinations(robot.items(), 2):
        pi, pj = elements[i]["part"]["part_id"], elements[j]["part"]["part_id"]
        for a, ha in li:
            for b, hb in lj:
                rule = relation(shape(pi, ha), shape(pj, hb), margin)
                if rule == "forbidden":
                    m.add_bool_or([~a, ~b])
                elif rule == "a_first":
                    m.add(order[i] < order[j]).only_enforce_if(a, b)
                elif rule == "b_first":
                    m.add(order[j] < order[i]).only_enforce_if(a, b)
    m.add_assumptions(list(assume.values()))
    solver = cp_model.CpSolver()
    solver.parameters.num_workers = 1        # cores need a single worker
    solver.parameters.max_time_in_seconds = max(1.0, min(20.0, t_end - monotonic()))
    status = solver.solve(m)
    if status in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        placed = []
        for i, lits in opts.items():
            if i in robot:
                placed.append((i, next(h for l, h in lits if solver.value(l))))
            else:
                placed.append((i, tuple(next(h for l, h in end if solver.value(l)) for end in lits)))
        pin_to_hole = {}
        for i, holes in placed:
            if elements[i]["pins"] is not None:
                pin_to_hole.update(dict(zip(elements[i]["pins"], holes, strict=True)))
        manual_connections = {pin: next(h for l, h in lits if solver.value(l)) for pin, lits in hand.items()}
        return (placed, pin_to_hole, manual_connections), None
    if status == cp_model.INFEASIBLE:
        names = {lit.index: k for k, lit in assume.items()}
        core = {names[x] for x in solver.sufficient_assumptions_for_infeasibility() if x in names}
        return None, core or set(assume)
    return None, set(assume)       # no answer in time: the whole topology is the nogood
