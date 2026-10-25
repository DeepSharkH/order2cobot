#!/usr/bin/env python3
"""Build viewer.html from viewer.tpl.html: every casing's two halves and its part, in the casing frame.

Usage: python3 casings.py && python3 build_viewer.py
"""
import base64
import io
import json
import os
import struct

import numpy as np

import casings as c

HERE = os.path.dirname(os.path.abspath(__file__))


def stl_b64(m):
    mesh = m.to_mesh()
    v = np.asarray(mesh.vert_properties)[:, :3]
    t = np.asarray(mesh.tri_verts)
    a, b, d = v[t[:, 0]], v[t[:, 1]], v[t[:, 2]]
    n = np.cross(b - a, d - a)
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    rec = np.zeros(len(t), dtype=[("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")])
    rec["n"], rec["v"] = n, np.hstack([a, b, d])
    buf = io.BytesIO()
    buf.write(b" " * 80)
    buf.write(struct.pack("<I", len(t)))
    buf.write(rec.tobytes())
    return base64.b64encode(buf.getvalue()).decode()


channel, w0, _ = c.jumper_channels()
items = []
builds = [("jumper", s, c.jumper_casing(s, channel, w0), c.jumper_shape(s), 2) for s in (1, 2, 3, 4)]
builds += [(k, s, c.part_casing(body, n, s, wrap, c.EXIT_D_PART.get(k, c.EXIT_D)),
            c.part_shape(body, n, s, c.EXIT_D_PART.get(k, 0.6)), n) for k, (body, n, sp, wrap) in c.PARTS.items() for s in sp]
for kind, s, (m, w, t), (body, leads), n in builds:
    h1, h2 = c.split(m, t)
    items.append({"kind": kind, "s": s, "w": round(w, 2), "t": round(t, 2), "h": c.H, "pins": n,
                  "exits": [round(x, 3) for x in c.exits(n, s)],
                  "stl": {"h1": stl_b64(h1), "h2": stl_b64(h2), "body": stl_b64(body), "leads": stl_b64(leads)}})
with open(os.path.join(HERE, "viewer.tpl.html")) as f:
    page = f.read().replace("__ITEMS__", json.dumps(items))
with open(os.path.join(HERE, "viewer.html"), "w") as f:
    f.write(page)
print(f"viewer.html written, {len(items)} casings, {len(page) / 1e6:.1f} MB")
