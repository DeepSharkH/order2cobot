#!/usr/bin/env python3
"""RJ45 (latch removed) grip shell with a funnel H handle, plus a snap-on bottom cover.

Frame: x along the cable (x=0 is the plug tip, +x toward the cable),
y across the plug, z up (plug bottom and shell bottom at z=0, H on top).

Seen from above the H has two legs running across the plug (y), joined by a
crossbar along the cable (x). The fingertips drop into the two notches of the H
and close across y on the crossbar; the legs sit tight against both edges of
each fingertip, so insertion and extraction push on the legs, not on friction.
The leg ends flare outward, so a fingertip that closes a few mm off in x is
steered into its notch.

The plug drops into the shell from below. The cover snaps on underneath with
two hooked arms between the legs; two ribs on the cover press the plug up
against the pocket ceiling so it sits without play. The zip tie around the
tail goes round the cover too.

Needs manifold3d (installed into ./.deps):
    uv pip install --python python3 --target .deps manifold3d numpy
Usage: python3 rj45_grip.py   -> rj45_grip.stl, rj45_grip_cover.stl
"""
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".deps"))
import numpy as np  # noqa: E402
from manifold3d import CrossSection, Manifold  # noqa: E402

# ---- measure your plug and adjust ----
PLUG_W = 11.7        # plug width (y)
PLUG_H = 8.0         # plug height with latch cut off (z)
PLUG_LEN = 21.5      # plug tip to rear end of plug body (x)
INSERT_LEN = 15.0    # how much of the plug disappears into the port
CABLE_W = 6.5        # cable / boot width directly behind the plug
CLR = 0.2            # fit clearance per side
WALL = 1.6           # side wall thickness (carries the snap groove)
TOP = 1.6            # top plate thickness

# ---- H handle ----
FINGERTIP_W = 24.0   # Hand-E fingertip width (x)
FINGERTIP_T = 8.0    # Hand-E fingertip thickness (y)
FINGER_FIT = 0.4     # total play between fingertip and the two legs
H_W = 24.0           # H width across the plug (y), legs span this
LEG_T = 4.0          # leg thickness (x)
CROSS_W = H_W - 2 * FINGERTIP_T  # crossbar width (y) = grip span; notches as deep as the fingertip
H_HEIGHT = 12.0      # crossbar height above the top plate

# ---- funnel: leg ends flare away from the notch ----
FLARE_DX = 4.0       # how far each leg end bends outward (x); notch mouth = notch + 2*FLARE_DX
FLARE_DY = 5.0       # flare length beyond the leg end (y)
PORT_GAP = 1.0       # front flare tip to the port face
FINGER_CLEAR = FLARE_DX + PORT_GAP  # gap from the port face to the front leg

# ---- cable retention ----
TAIL = FLARE_DX + 5.2 # shell length behind the rear leg; zip tie sits clear of the flares
ZIP_W = 3.2          # zip-tie groove width
ZIP_D = 1.0          # zip-tie groove depth (shell)

# ---- bottom cover ----
COVER_T = 1.2        # plate thickness under the plug
SNUG = 0.4           # rib height; squeezes the plug by SNUG - CLR against the ceiling
RIB_W = 1.0          # rib width
RIB_Y = 3.0          # rib centre offset from the plug axis
ARM_T = 1.0          # snap arm thickness
ARM_CLR = 0.15       # gap between arm and shell wall
ARM_INSET = 1.0      # arm ends to the legs (x)
HOOK_Z0 = 6.0        # groove bottom = hook retaining face, above the shell bottom
HOOK_H = 1.4         # hook height
HOOK_D = 0.6         # groove depth into the wall
HOOK_IN = 0.5        # hook reach into the groove
CHAMFER = 0.6        # lead-in on the shell bottom edges

LEG1_X0 = INSERT_LEN + FINGER_CLEAR
NOTCH_X0 = LEG1_X0 + LEG_T
NOTCH_X1 = NOTCH_X0 + FINGERTIP_W + FINGER_FIT
LEG2_X0 = NOTCH_X1
LEG2_X1 = NOTCH_X1 + LEG_T
SHELL_X0 = INSERT_LEN + 0.5                    # leave the inserted part bare
SHELL_X1 = LEG2_X1 + TAIL
ZIP_X0 = LEG2_X1 + FLARE_DX + 0.8
ARM_X0, ARM_X1 = NOTCH_X0 + ARM_INSET, NOTCH_X1 - ARM_INSET

YI = PLUG_W / 2 + CLR                          # inner half-width
YO = YI + WALL                                 # outer half-width
YC = CABLE_W / 2 + CLR                         # cable channel half-width
YA = YO + ARM_CLR + ARM_T                      # cover outer half-width
ZI = PLUG_H + CLR                              # channel ceiling
ZT = ZI + TOP                                  # top of plate
ZH = ZT + H_HEIGHT                             # top of H
HOOK_Z1 = HOOK_Z0 + HOOK_H
GRIP_SPAN = CROSS_W
FUNNEL_MOUTH = NOTCH_X1 - NOTCH_X0 + 2 * FLARE_DX


def box(x0, x1, y0, y1, z0, z1):
    return Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def prism_z(pts_xy, z0, z1):
    """Vertical prism from a CCW polygon in the xy plane."""
    return CrossSection([pts_xy]).extrude(z1 - z0).translate([0, 0, z0])


def prism_x(pts_yz, x0, x1):
    """Prism along x from a CCW polygon in the yz plane."""
    m = CrossSection([pts_yz]).extrude(x1 - x0)           # (y, z, x')
    return m.transform(np.array([[0, 0, 1, x0], [1, 0, 0, 0], [0, 1, 0, 0]], dtype=float))


def handle_outline():
    """Top view of the two legs with their flared ends (list of CCW polygons)."""
    e = H_W / 2
    polys = []
    for x0, x1, s in ((LEG1_X0, NOTCH_X0, -1), (LEG2_X0, LEG2_X1, 1)):  # s: flare direction in x
        polys.append([(x0, -e), (x1, -e), (x1, e), (x0, e)])
        dx = s * FLARE_DX
        polys.append([(x0, e), (x1, e), (x1 + dx, e + FLARE_DY), (x0 + dx, e + FLARE_DY)])
        polys.append([(x0 + dx, -e - FLARE_DY), (x1 + dx, -e - FLARE_DY), (x1, -e), (x0, -e)])
    return polys


def shell():
    m = box(SHELL_X0, SHELL_X1, -YO, YO, 0, ZT)
    for p in handle_outline():
        m += prism_z(p, 0, ZH)                                   # legs to the bed: no supports
    m += box(NOTCH_X0, NOTCH_X1, -CROSS_W / 2, CROSS_W / 2, ZT, ZH)  # crossbar
    # plug pocket, open at the front and underneath
    m -= box(SHELL_X0 - 1, PLUG_LEN + CLR, -YI, YI, -1, ZI)
    # cable channel behind the plug (the step is the insertion stop)
    m -= box(PLUG_LEN + CLR, SHELL_X1 + 1, -YC, YC, -1, ZI)
    # zip-tie groove: top and both sides
    m -= box(ZIP_X0, ZIP_X0 + ZIP_W, -YO - 1, YO + 1, ZT - ZIP_D, ZT + 1)
    for s in (1, -1):
        m -= box(ZIP_X0, ZIP_X0 + ZIP_W, YO - ZIP_D, YO + 1, -1, ZT + 1).scale([1, s, 1])
        # snap groove for the cover hooks, and a lead-in chamfer on the bottom edge
        m -= box(ARM_X0 - 0.3, ARM_X1 + 0.3, YO - HOOK_D, YO + 1, HOOK_Z0, HOOK_Z1 + 0.2).scale([1, s, 1])
        m -= prism_x([(YO - CHAMFER - 1, -1), (YO + 1, -1), (YO + 1, CHAMFER + 1)],
                     ARM_X0 - 0.3, ARM_X1 + 0.3).scale([1, s, 1])
    return m


def cover():
    m = box(SHELL_X0, SHELL_X1, -YA, YA, -COVER_T, 0)
    for y in (RIB_Y, -RIB_Y):                                    # ribs under the plug body
        m += box(SHELL_X0 + 1, PLUG_LEN - 1, y - RIB_W / 2, y + RIB_W / 2, 0, SNUG)
    ya, yt = YO + ARM_CLR, YO - HOOK_IN
    for s in (1, -1):
        arm = box(ARM_X0, ARM_X1, ya, YA, -COVER_T, HOOK_Z1)
        # hook: flat retaining face at the bottom, chamfered top to ride over the wall
        arm += prism_x([(yt, HOOK_Z0), (YA - 0.01, HOOK_Z0), (YA - 0.01, HOOK_Z1), (ya, HOOK_Z1), (yt, HOOK_Z0 + 0.3)],
                       ARM_X0, ARM_X1)
        m += arm.scale([1, s, 1])
    # the zip tie also wraps the cover
    m -= box(ZIP_X0, ZIP_X0 + ZIP_W, -YA - 1, YA + 1, -COVER_T - 1, -COVER_T + 0.5)
    return m


def write_stl(path, m, offset, name):
    mesh = m.to_mesh()
    v = np.asarray(mesh.vert_properties)[:, :3] + np.asarray(offset)
    t = np.asarray(mesh.tri_verts)
    a, b, c = v[t[:, 0]], v[t[:, 1]], v[t[:, 2]]
    n = np.cross(b - a, c - a)
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    rec = np.zeros(len(t), dtype=[("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")])
    rec["n"], rec["v"] = n, np.hstack([a, b, c])
    with open(path, "wb") as f:
        f.write(name.encode().ljust(80, b" "))
        f.write(struct.pack("<I", len(t)))
        f.write(rec.tobytes())
    return len(t)


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    sh, cv = shell(), cover()
    assert (sh ^ cv).volume() < 1e-6, "cover intersects the shell"
    for name, m in (("shell", sh), ("cover", cv)):
        assert m.status().name == "NoError" and m.genus() >= 0, name
    # STL frame: x from the shell front; each part sits on its own print bed
    n1 = write_stl(os.path.join(here, "rj45_grip.stl"), sh, (-SHELL_X0, 0, 0), "rj45_grip")
    n2 = write_stl(os.path.join(here, "rj45_grip_cover.stl"), cv, (-SHELL_X0, 0, COVER_T), "rj45_grip_cover")
    print(f"rj45_grip.stl: {n1} tris, {SHELL_X1 - SHELL_X0:.1f} x {H_W + 2 * FLARE_DY:.1f} x {ZH:.1f} mm, "
          f"{sh.volume() / 1000:.1f} cm3, grip span {GRIP_SPAN:.1f}, notch {NOTCH_X1 - NOTCH_X0:.1f} "
          f"-> funnel mouth {FUNNEL_MOUTH:.1f} mm, front leg {FINGER_CLEAR:.1f} mm from port face")
    print(f"rj45_grip_cover.stl: {n2} tris, {SHELL_X1 - SHELL_X0:.1f} x {2 * YA:.1f} x {COVER_T + HOOK_Z1:.1f} mm, "
          f"{cv.volume() / 1000:.1f} cm3, plug squeeze {SNUG - CLR:.2f} mm")
