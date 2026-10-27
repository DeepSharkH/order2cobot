"""Picture set of the port-1 approach for offline analysis (no contact).

Each shot saves both cameras at full resolution (colour PNG + aligned depth .npy) and a
JSON with the flange pose, joints, both camera poses (world and base), intrinsics, the
grip model and the port survey, into data/approach/<stamp>/.

  python -m porttest.approach_shots snap NAME     one shot at the current pose (no motion)
  python -m porttest.approach_shots run           survey view, then the approach waypoints

The survey view (wrist 3 ≈ -4.8) is out of the cable range for the current routing, so
its frame is copied in from the survey instead. Waypoints (flange, insertion orientation): high above port 1, then the tip 30 / 15 / 8 / 5
mm in front of the surveyed face at port 1's x/z, and ±3 mm along x at 8 mm. Straight
moves at 1 cm/s; nothing goes closer than 5 mm to the estimated face.
"""
import asyncio
import json
import sys
import time
import urllib.request
from pathlib import Path

import aiohttp
import numpy as np
import yaml

from porttest.find_port1 import ROT, TIP
from porttest.go_view import W3_REF
from porttest.touch_board import board_in_base
from porttest.urexec import URBridge, URExec, call

ROOT = Path(__file__).resolve().parent.parent
JOINTS = ('base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3')


async def _state():
    async with aiohttp.ClientSession() as s:
        async with s.ws_connect('http://localhost:8090/ws') as ws:
            async for m in ws:
                if m.type == aiohttp.WSMsgType.TEXT:
                    return json.loads(m.data)


def snap(out, name, b=None):
    b = b or URBridge(client_name='order2cobot-calib')
    time.sleep(1.0)                                     # let the arm and the depth settle
    for cam in (0, 1):
        for kind, ext in (('frame', 'png'), ('depth', 'npy')):
            try:
                data = urllib.request.urlopen(f'http://localhost:8090/api/{kind}/{cam}', timeout=10).read()
                (out / f'{name}_cam{cam}.{ext}').write_bytes(data)
            except Exception as e:
                print(f'  {kind} {cam}: {e}')
    st = asyncio.run(_state())
    p = call(b, 'robot_get_pose')['data']
    q = call(b, 'robot_get_joint_positions')['data']
    Tbw = board_in_base()
    cams = []
    for c in st['cams']:
        Tw = None if c['T_world_cam'] is None else np.array(c['T_world_cam'])
        cams.append({'name': c['name'], 'K': c['K'], 'dist': c['dist'], 'size': c['size'],
                     'T_world_cam': c['T_world_cam'], 'T_base_cam': None if Tw is None else (Tbw @ Tw).tolist(),
                     'board_seen': c['board']['seen']})
    meta = {'name': name, 'stamp': time.strftime('%Y-%m-%dT%H:%M:%S'),
            'flange': p['pose']['position'] + p['pose']['orientation'], 'tcp': p.get('tcp', {}).get('name'),
            'q': [q[k] for k in JOINTS], 'cams': cams, 'T_base_world': Tbw.tolist(),
            'grip': {'tip_in_base_insertion': TIP.tolist(), 'rot_insertion': ROT}}
    (out / f'{name}.json').write_text(json.dumps(meta, indent=1))
    print(f'{name}: flange {np.round(meta["flange"][:3], 4).tolist()}')


def waypoints():
    C = np.array(yaml.safe_load((ROOT / 'data/ports.yaml').read_text())['centre'])
    port = C[0]
    face = port - TIP                                   # flange with the tip on the face
    wp = [('high', face + [0, -0.03, 0.10])]
    for d in (0.030, 0.015, 0.008, 0.005):
        wp.append((f'front{int(d * 1000):02d}mm', face + [0, -d, 0]))
    wp += [('front08mm_xm3', face + [-0.003, -0.008, 0]), ('front08mm_xp3', face + [0.003, -0.008, 0])]
    return wp


def run(out):
    ex = URExec()
    rot = ', '.join(f'{v:.6f}' for v in ROT)
    for i, (name, f) in enumerate(waypoints()):
        if i == 0:
            ex.move_checked(list(f) + ROT, w3_ref=W3_REF, v=0.2)
        else:
            ex.run(f'movel(p[{f[0]:.5f}, {f[1]:.5f}, {f[2]:.5f}, {rot}], a=0.1, v=0.01)', timeout=120)
        snap(out, f'{i + 2:02d}_{name}', ex.b)
    p = waypoints()[0][1]
    ex.run(f'movel(p[{p[0]:.5f}, {p[1]:.5f}, {p[2]:.5f}, {rot}], a=0.2, v=0.02)', timeout=120)


if __name__ == '__main__':
    out = ROOT / 'data/approach' / (sys.argv[3] if len(sys.argv) > 3 else time.strftime('%Y%m%d-%H%M%S'))
    out.mkdir(parents=True, exist_ok=True)
    (out / 'ports.yaml').write_text((ROOT / 'data/ports.yaml').read_text())
    if sys.argv[1] == 'snap':
        snap(out, sys.argv[2])
    else:
        for name, f in waypoints():
            print(name, np.round(f, 4).tolist())
        if sys.argv[1] == 'run':
            run(out)
    print('saved in', out)
