"""Conservative 2D casing and finger envelopes; dimensions are millimeters."""

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
