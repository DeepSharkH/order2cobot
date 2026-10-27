"""Conservative 2D casing and finger envelopes; dimensions are millimeters."""

from collections import defaultdict
from graphlib import CycleError, TopologicalSorter
from heapq import heappop, heappush
from itertools import combinations
from math import isclose


def casing_for(part, catalog, pitch):
    family = part["type"]
    if part["part_id"].startswith("capacitor_ceramic_"):
        family = "capacitor_ceramic"
    casing_id = part.get("casing_id", f"{family}_s{part['pin_spacing']}")
    casing = next((c for c in catalog["casings"] if c["casing"] == casing_id), None)
    if casing is None or family.startswith("buzzer"):
        raise ValueError(f"Missing supported casing geometry for {part['part_id']} ({casing_id})")
    exits = casing["exit_x"]
    if len(exits) != len(part["pins"]) or len(exits) < 2:
        raise ValueError(f"{casing_id}: casing and part pin counts differ")
    if any(
        not isclose(b - a, part["pin_spacing"] * pitch, abs_tol=0.001)
        for a, b in zip(exits, exits[1:], strict=False)
    ):
        raise ValueError(f"{casing_id}: casing lead spacing does not match board pitch")
    if min(casing["width_x"], casing["thickness_y"], casing["height"]) <= 0:
        raise ValueError(f"{casing_id}: casing dimensions must be positive")
    return casing


def placement_geometry(holes, positions, casing, gripper, pitch):
    first, last = positions[holes[0]], positions[holes[-1]]
    dx, dy = last[0] - first[0], last[1] - first[1]
    direction = (0 if dx == 0 else (1 if dx > 0 else -1), 0 if dy == 0 else (1 if dy > 0 else -1))
    angle = {(1, 0): 0, (0, 1): 90, (-1, 0): 180, (0, -1): 270}[direction]
    ux, uy = direction
    # The CAD origin is the casing center, not the first lead.
    cx = first[0] * pitch - casing["exit_x"][0] * ux
    cy = first[1] * pitch - casing["exit_x"][0] * uy

    def transform(box):
        x0, y0, x1, y1 = box
        points = [(cx + x * ux - y * uy, cy + x * uy + y * ux) for x in (x0, x1) for y in (y0, y1)]
        return (
            min(x for x, _ in points),
            min(y for _, y in points),
            max(x for x, _ in points),
            max(y for _, y in points),
        )

    half_x, half_y = casing["width_x"] / 2, casing["thickness_y"] / 2
    body_y = (
        max(
            casing["thickness_y"],
            gripper.get("assembled_thickness_overrides_mm", {}).get(
                casing["casing"], casing["thickness_y"]
            ),
        )
        / 2
    )
    body = transform((-half_x, -body_y, half_x, body_y))
    grip_x = gripper["grip_center_x_mm"]
    width = gripper["finger_width_mm"] / 2
    outer = half_y + gripper["finger_thickness_mm"] + gripper["opening_per_finger_mm"]
    # Union over the finger's closed-to-open motion. No rotation occurs at board height.
    fingers = [
        transform((grip_x - width, half_y, grip_x + width, outer)),
        transform((grip_x - width, -outer, grip_x + width, -half_y)),
    ]
    return {
        "orientation_deg": angle,
        "center_mm": [cx, cy],
        "casing_id": casing["casing"],
        "body": body,
        "fingers": fingers,
    }


def overlaps(a, b, margin=0):
    """Touching is permitted at zero margin; margin is applied once per pair."""
    return (
        a[0] < b[2] + margin - 1e-9
        and b[0] < a[2] + margin - 1e-9
        and a[1] < b[3] + margin - 1e-9
        and b[1] < a[3] + margin - 1e-9
    )


def relation(a, b, margin):
    """Return forbidden, A-before-B, B-before-A, or independent."""
    if overlaps(a["body"], b["body"], margin):
        return "forbidden"
    a_first = any(overlaps(finger, b["body"], margin) for finger in a["fingers"])
    b_first = any(overlaps(finger, a["body"], margin) for finger in b["fingers"])
    if a_first and b_first:
        return "forbidden"
    return "a_first" if a_first else "b_first" if b_first else "independent"


def pose(shape):
    """The boxes around the centre, and the centre, in µm: all that relation() reads.

    A casing that is symmetric under a half turn has one kind at 0 and 180 degrees.
    """
    x, y = (round(v * 1000) for v in shape["center_mm"])

    def around(box):
        return tuple(round(v * 1000) - c for v, c in zip(box, (x, y, x, y), strict=True))

    return (around(shape["body"]), tuple(sorted(map(around, shape["fingers"])))), (x, y)


def no_fit_cliques(candidates, parts, margin, rules):
    """Yield cliques of candidates that relation() forbids together.

    Owners below parts are parts that pick exactly one candidate. For each candidate x and
    each such part B after x's own (any part, for a wire): x and every candidate of B that
    x forbids. B picks one, so this is a clique. Wire-wire pairs are left to the lazy cuts.
    relation() reads only boxes, so rules caches it by both kinds and the centre offset.
    """
    poses = [pose(c["shape"]) for c in candidates]
    cells = defaultdict(lambda: defaultdict(list))  # (part, kind): centre: literals
    kinds, xs, ys = defaultdict(set), defaultdict(set), defaultdict(set)
    for c, (kind, (x, y)) in zip(candidates, poses, strict=True):
        xs[kind].add(x)
        ys[kind].add(y)
        if c["owner"] < parts:
            cells[c["owner"], kind][x, y].append(c["chosen"])
            kinds[c["owner"]].add(kind)
    # Farthest box edge from the centre, per kind: farther apart, nothing overlaps.
    reach = {
        k: [max(abs(v) for box in (k[0], *k[1]) for v in box[axis::2]) for axis in (0, 1)]
        for k in xs
    }
    offsets = {}

    def forbidden(ka, kb):
        """Centre offsets from ka to kb that relation() forbids (scale-free, so in µm)."""
        if (ka, kb) not in offsets:
            far = [reach[ka][axis] + reach[kb][axis] + margin * 1000 for axis in (0, 1)]
            dxs = {xb - xa for xa in xs[ka] for xb in xs[kb] if abs(xb - xa) < far[0]}
            dys = {yb - ya for ya in ys[ka] for yb in ys[kb] if abs(yb - ya) < far[1]}
            offsets[ka, kb] = []
            for dx in dxs:
                for dy in dys:
                    key = ka, kb, dx, dy
                    if key not in rules:
                        rules[key] = relation(shift(ka, 0, 0), shift(kb, dx, dy), margin * 1000)
                    if rules[key] == "forbidden":
                        offsets[ka, kb].append((dx, dy))
        return offsets[ka, kb]

    for c, (ka, (x, y)) in zip(candidates, poses, strict=True):
        own = c["owner"]
        for part in range(own + 1 if own < parts else 0, parts):
            clique = [c["chosen"]]
            for kb in kinds[part]:
                cell = cells[part, kb]
                for dx, dy in forbidden(ka, kb):
                    clique += cell.get((x + dx, y + dy), ())
            if len(clique) > 1:
                yield clique


def shift(kind, dx, dy):
    """A shape of this kind centred at dx, dy (µm)."""
    body, fingers = kind

    def move(box):
        return box[0] + dx, box[1] + dy, box[2] + dx, box[3] + dy

    return {"body": move(body), "fingers": [move(f) for f in fingers]}


def constrain_assembly(model, selected, known, margin, priority=None):
    """Add exact cuts for collisions found in a candidate solution, or return its order.

    Checking only selected placements avoids millions of unused pair constraints. A
    forbidden pair is cut. Otherwise "A's fingers hit B" puts A before B, and the steps
    are sorted topologically, lowest priority first among the ready ones. A precedence
    cycle (3 or more steps: a 2-cycle is forbidden) gets one nogood. With any cut this
    returns None and the solver is rerun; no unverified solution is returned.
    """
    sorter = TopologicalSorter({i: () for i in range(len(selected))})
    cuts = []
    for i, j in combinations(range(len(selected)), 2):
        rule = relation(selected[i]["shape"], selected[j]["shape"], margin)
        if rule == "forbidden":
            cuts.append((i, j))
        elif rule != "independent":
            sorter.add(*((j, i) if rule == "a_first" else (i, j)))  # (step, its predecessor)
    try:
        sorter.prepare()
    except CycleError as error:
        cuts.append(error.args[1][1:])  # The cycle lists its first step twice.
    for steps in cuts:
        literals = [selected[i]["chosen"] for i in steps]
        key = tuple(sorted(x.index for x in literals))
        if key in known:
            continue
        known.add(key)
        if len(literals) == 2:
            model.add(literals[0] + literals[1] <= 1)
        else:
            model.add_bool_or([~x for x in literals])
    if cuts:
        return None
    order, ready = [], []
    while sorter.is_active():
        for i in sorter.get_ready():
            heappush(ready, (priority(selected[i]) if priority else 0, i))
        _, i = heappop(ready)
        order.append(selected[i])
        sorter.done(i)
    return order


def verify_assembly(steps, margin):
    """Steps are in installation order; check each against all earlier bodies."""
    for earlier, later in combinations(steps, 2):
        rule = relation(earlier["shape"], later["shape"], margin)
        if rule in ("forbidden", "b_first"):
            raise ValueError(f"Invalid assembly order or collision: {earlier['id']}, {later['id']}")
