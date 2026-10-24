"""
Switch / plug geometry measured by touch (2026-10-24), in robot base coordinates.

TCP is the plug tip centre: tool (0, -0.0342, 0.156). With the tool pointing down
(rotvec [pi, 0, 0]) tip = flange + (0, +0.0342, -0.156). The plug points along
-tool_y, i.e. base +y, and the switch ports face the robot (-y).
"""
import numpy as np
from scipy.spatial.transform import Rotation as R

TCP = [0.0, -0.0342, 0.156, 0.0, 0.0, 0.0]
FLANGE_TO_TIP = np.array([0.0, 0.0342, -0.156])   # for the squared, unrotated tool

# Touch measurements (flange coordinates, unrotated tool). The contacts were made by
# the white shell / H legs on the switch's top front edge, not by the plug tip
# (rj45_grip.py: the shell starts 15.5 mm behind the tip, legs are 24 mm wide and the
# shell bottom is 0.8 mm below the plug).
SHELL_SETBACK = 0.0155         # tip -> shell front
SHELL_BELOW = 0.0008           # shell bottom under the plug bottom
H_HALF_W = 0.012               # legs, half width across the plug
EDGE_FREE_X = -0.2840 + H_HALF_W - 0.0  # legs' +x side cleared the switch's left end (plug centre x here = edge - 12 mm)
TOP_FREE_Z = 0.2289 - SHELL_BELOW       # plug bottom at the switch top
FACE_Y0, FACE_X0 = 0.2319 - SHELL_SETBACK, -0.241   # flange y with the tip on the face
FACE_SLOPE = -0.0693           # dy/dx of the front edge

PLUG_HALF_W = 0.00585
PLUG_HALF_H = 0.0040
PORT1_FROM_EDGE = 0.0267       # image estimate
PITCH = 0.01485                # image estimate
PORT_BOTTOM_BELOW_TOP = 0.0192  # image estimate, opening bottom under the switch top

YAW = np.arctan(FACE_SLOPE)                    # face direction angle about z
D = np.array([np.cos(YAW), np.sin(YAW), 0.0])  # along the face, port 1 -> 8
N = np.array([-np.sin(YAW), np.cos(YAW), 0.0])  # face normal, into the switch
ROT = (R.from_euler('z', YAW) * R.from_rotvec([np.pi, 0, 0])).as_rotvec()


def face_y(x):
    return FACE_Y0 + FACE_SLOPE * (x - FACE_X0)


def port_tip(k):
    """Estimated tip pose [x, y, z, rx, ry, rz] at port k (1..8), on the face."""
    x1 = EDGE_FREE_X + PORT1_FROM_EDGE
    p1 = np.array([x1, face_y(x1), 0.0]) + FLANGE_TO_TIP
    p1[2] = TOP_FREE_Z - PLUG_HALF_H - PORT_BOTTOM_BELOW_TOP + PLUG_HALF_H + FLANGE_TO_TIP[2]
    p = p1 + (k - 1) * PITCH * D
    return list(p) + list(ROT)


def urpose(p):
    return 'p[' + ', '.join(f'{v:.5f}' for v in p) + ']'


def urvec(v):
    return '[' + ', '.join(f'{x:.5f}' for x in v) + ']'
