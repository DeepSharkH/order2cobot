"""Identify the manual supply and reserve its board connection holes."""


def fixed_power_connections(circuit, manual_pins, positions, groups, pin_net):
    # J_POWER is the existing circuit convention. A role supports other instance IDs.
    supplies = [
        c
        for c in circuit["components"]
        if c.get("role") == "power_supply" or (c["id"] == "J_POWER" and c.get("role") is None)
    ]
    if len(supplies) > 1:
        raise ValueError("Only one power supply can use the fixed C1/H1 connections")
    if not supplies:
        return {}
    cid = supplies[0]["id"]
    fixed = {f"{cid}.positive": "C1", f"{cid}.negative": "H1"}
    if not set(fixed) <= manual_pins:
        raise ValueError("The power supply needs manual positive and negative pins")
    if not set(fixed.values()) <= positions.keys():
        raise ValueError("Board must contain power supply holes C1 and H1")
    if groups["C1"] == groups["H1"] or pin_net[f"{cid}.positive"] == pin_net[f"{cid}.negative"]:
        raise ValueError("Positive and negative supply connections must be on separate nets/strips")
    return fixed
