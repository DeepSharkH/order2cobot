#!/usr/bin/env python3
"""Build viewer.html from viewer.tpl.html, embedding both STLs and their dimensions.

Usage: python3 rj45_grip.py && python3 build_viewer.py
"""
import base64
import json
import os

import rj45_grip as g

HERE = os.path.dirname(os.path.abspath(__file__))

# dimensions in the STL frame: x from the shell front, z from the shell bottom
sx, sz = g.SHELL_X0, 0.0
dims = {
    "L": g.SHELL_X1 - sx, "W": g.H_W, "H": g.ZH - sz, "Wall": g.H_W + 2 * g.FLARE_DY,
    "flareDX": g.FLARE_DX, "flareDY": g.FLARE_DY, "mouth": g.FUNNEL_MOUTH,
    "coverT": g.COVER_T, "coverW": 2 * g.YA, "snug": g.SNUG, "ribY": g.RIB_Y, "ribW": g.RIB_W,
    "ribX0": g.SHELL_X0 + 1 - sx, "ribX1": g.PLUG_LEN - 1 - sx, "armT": g.ARM_T, "armClr": g.ARM_CLR,
    "armX0": g.ARM_X0 - sx, "armX1": g.ARM_X1 - sx, "hookZ0": g.HOOK_Z0, "hookZ1": g.HOOK_Z1,
    "hookD": g.HOOK_D, "hookIn": g.HOOK_IN, "chamfer": g.CHAMFER,
    "shellW": 2 * g.YO, "shellH": g.ZT - sz, "crossH": g.H_HEIGHT, "crossW": g.CROSS_W,
    "legT": g.LEG_T, "leg1X0": g.LEG1_X0 - sx, "notchX0": g.NOTCH_X0 - sx,
    "notchX1": g.NOTCH_X1 - sx, "leg2X1": g.LEG2_X1 - sx,
    "pocketL": g.PLUG_LEN + g.CLR - sx, "pocketW": 2 * g.YI, "pocketH": g.ZI,
    "cableW": 2 * g.YC,
    "zipX0": g.ZIP_X0 - sx, "zipW": g.ZIP_W, "zipD": g.ZIP_D,
    "bare": g.INSERT_LEN, "faceX": g.INSERT_LEN - sx, "plugLen": g.PLUG_LEN,
    "plugW": g.PLUG_W, "plugH": g.PLUG_H, "cable": g.CABLE_W,
    "fingerW": g.FINGERTIP_W, "fingerT": g.FINGERTIP_T, "clear": g.FINGER_CLEAR,
}
dims = {k: round(v, 2) for k, v in dims.items()}



def b64(name):
    with open(os.path.join(HERE, name), "rb") as f:
        return base64.b64encode(f.read()).decode()


with open(os.path.join(HERE, "viewer.tpl.html")) as f:
    page = f.read()
page = page.replace("__STL__", b64("rj45_grip.stl")).replace("__COVER__", b64("rj45_grip_cover.stl")).replace("__DIMS__", json.dumps(dims))
with open(os.path.join(HERE, "viewer.html"), "w") as f:
    f.write(page)
print("viewer.html written", dims)
