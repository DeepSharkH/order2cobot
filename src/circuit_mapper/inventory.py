"""Stock limits for one circuit assembly; omitted quantity means unlimited."""

from collections import Counter, defaultdict


def checked_parts(circuit, inventory):
    parts = {p["part_id"]: p for p in inventory["components"]}
    for part in parts.values():
        quantity = part.get("quantity")
        if quantity is not None and (type(quantity) is not int or quantity < 0):
            raise ValueError(f"Invalid quantity for {part['part_id']}")
    for pid, needed in Counter(c["part_id"] for c in circuit["components"]).items():
        if pid not in parts:
            raise ValueError(f"Part is not in the current inventory: {pid}")
        available = parts[pid].get("quantity")
        if available is not None and needed > available:
            raise ValueError(
                f"Insufficient stock for {pid}: requested {needed}, available {available}"
            )
    return parts


def limit_jumpers(model, wires, parts):
    choices = defaultdict(list)
    for chosen, part, _ in wires:
        choices[part["part_id"]].append(chosen)
    for pid, candidates in choices.items():
        quantity = parts[pid].get("quantity")
        if quantity is not None:
            model.add(sum(candidates) <= quantity)
