#!/usr/bin/env python3
"""Build viewer.html from viewer.tpl.html, embedding the STLs, the RJ45 grip and slots.json.

Usage: python3 component_table.py && python3 build_viewer.py
"""
import base64
import json
import os

import component_table as t

HERE = os.path.dirname(os.path.abspath(__file__))
RJ = os.path.join(HERE, "..", "rj45_grip")
g = t.rj

dims = {k: getattr(t, k) for k in (
    "PITCH", "CELL", "MOD", "BORDER", "BASE_W", "BASE_T", "WIN", "CORNER_R", "PLATE_T", "LEAD_MAX", "H_MOD",
    "TOE_U", "TOE_D", "PIN_L", "PIN_AB", "PIN_CD", "PIN_CLR", "CD_CLR", "SLOT_EXTRA", "PIN_CHAMFER", "SOCKET_D",
    "SOCKET_CHAMFER", "SKIRT_T", "SKIRT_H", "M3_HEAD_D", "M3_HEAD_H", "M4_HEAD", "HOLE_D", "CSK_D", "BLIND_D",
    "ROW_PITCH", "TIP_T", "TIP_W", "BODY_W", "TIP_OPEN", "TIP_Z", "APPROACH", "RJ_T", "RJ_X", "RJ_Z", "RJ_CLR",
    "CABLE_D", "CLIP_L", "CLIP_Z")}
dims["cells"] = {n: t.cell_xy(*c) for n, c in t.CELLS.items()}
dims["cellIdx"] = t.CELLS
dims["mods"] = list(t.CELLS)
dims["ROW_Y0"] = t.ROW_Y0
dims["screws"] = t.base_screws()
dims["rj"] = {k: getattr(g, k) for k in (
    "SHELL_X0", "SHELL_X1", "PLUG_LEN", "PLUG_W", "PLUG_H", "COVER_T", "NOTCH_X0", "NOTCH_X1", "CROSS_W", "ZT", "ZH",
    "FINGERTIP_W", "FINGERTIP_T", "YA", "INSERT_LEN")}
dims["rj"]["outline"] = g.handle_outline()


def b64(path):
    with open(path, "rb") as f:
        return base64.b64encode(f.read()).decode()


with open(os.path.join(HERE, "slots.json")) as f:
    slots = json.load(f)
with open(os.path.join(HERE, "viewer.tpl.html")) as f:
    page = f.read()
stls = {
    "base": b64(os.path.join(HERE, "base.stl")),
    "mods": {n: b64(os.path.join(HERE, f"module_{n.lower()}.stl")) for n in t.CELLS},
    "shell": b64(os.path.join(RJ, "rj45_grip.stl")),
    "cover": b64(os.path.join(RJ, "rj45_grip_cover.stl")),
}
subs = {"__STL__": json.dumps(stls), "__DIMS__": json.dumps(dims), "__SLOTS__": json.dumps(slots)}
for k, v in subs.items():
    page = page.replace(k, v)
with open(os.path.join(HERE, "viewer.html"), "w") as f:
    f.write(page)
print(f"viewer.html written, {len(page) / 1e6:.1f} MB")
