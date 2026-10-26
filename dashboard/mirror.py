"""Read-only mirror of another dashboard that owns the cameras and the robot connection.

Only one process can open the D435, the Orbbec and the bridge client name, so a second
dashboard (e.g. another checkout, on another port) mirrors the first:

  cameras  raw full-resolution colour (/api/frame/i, no overlays) and aligned depth
           (/api/depth/i), polled at `hz`; intrinsics from the source's state
  robot    joints, DH table and mode from the source's state (its websocket, 10 Hz)
  scene    fixed-cam pose, base and gripper-cam mount follow the source's calibration

Everything downstream (overlays, clouds, board detection, objects) then runs locally on the
same data. Calibration actions are refused: calibrate on the source dashboard.

    python -m dashboard.server --port 8091 --mirror http://127.0.0.1:8090
"""

import asyncio
import io
import json
import threading
import time

import aiohttp
import cv2
import numpy as np
import requests

from . import kinematics as kin
from .cameras import Camera


class Source:
    """The source dashboard's latest state, followed over its websocket."""

    def __init__(self, url):
        self.url = url.rstrip('/')
        self.state = {}
        self.t = 0.0
        self.error = 'connecting'
        threading.Thread(target=lambda: asyncio.run(self._follow()), daemon=True, name='mirror-state').start()

    async def _follow(self):
        while True:
            try:
                async with aiohttp.ClientSession() as http, \
                        http.ws_connect(self.url + '/ws', max_msg_size=0, heartbeat=10) as ws:
                    async for msg in ws:          # binary frames and clouds are ignored
                        if msg.type == aiohttp.WSMsgType.TEXT:
                            m = json.loads(msg.data)
                            if m.get('type') == 'state':
                                self.state, self.t, self.error = m, time.time(), None
            except Exception as e:
                self.error = f'source {self.url}: {e}'[:300]
            await asyncio.sleep(1.0)

    def wait(self, timeout=15.0):
        t_end = time.time() + timeout
        while not self.state and time.time() < t_end:
            time.sleep(0.1)
        if not self.state:
            raise RuntimeError(f'no state from the source dashboard {self.url} ({self.error})')

    def fresh(self):
        return self.state if time.time() - self.t < 3.0 else {}


class MirrorCamera(Camera):
    def __init__(self, source, idx, name, hz=2.0):
        super().__init__()
        self.source, self.idx, self.name, self.hz = source, idx, name, hz
        self.intrinsics_source = 'mirror'

    def _run(self):
        http = requests.Session()
        while True:
            t0 = time.time()
            cams = self.source.fresh().get('cams') or []
            c = cams[self.idx] if len(cams) > self.idx else None
            if not c or not c.get('ok'):
                self.error = (c or {}).get('error') or self.source.error or 'no signal from the source'
                time.sleep(1.0)
                continue
            if c.get('K'):
                self.K, self.dist = np.array(c['K'], float), np.array(c.get('dist') or [0] * 5, float)
            r = http.get(f'{self.source.url}/api/frame/{self.idx}', timeout=10)
            r.raise_for_status()
            color = cv2.imdecode(np.frombuffer(r.content, np.uint8), cv2.IMREAD_COLOR)
            d = http.get(f'{self.source.url}/api/depth/{self.idx}', timeout=10)
            depth = np.load(io.BytesIO(d.content)) if d.status_code == 200 else None
            self.size = [color.shape[1], color.shape[0]]
            self._put(color, depth)
            self.error = None
            dt = time.time() - t0
            self.fps = 1.0 / max(dt, 1.0 / self.hz)
            time.sleep(max(0.0, 1.0 / self.hz - dt))


class MirrorRobot:
    """The source's robot state, with the Robot interface the dashboard uses."""

    def __init__(self, source):
        self.source = source
        self.dh = None
        self.rate = 0.0

    @property
    def error(self):
        r = self.source.fresh().get('robot')
        if r is None:
            return self.source.error or 'no state from the source'
        return None if r.get('connected') else (r.get('error') or 'source robot offline')

    @property
    def state(self):
        r = self.source.fresh().get('robot') or {}
        return {'robotmode': r.get('mode'), 'controlmode': r.get('control')}

    def joints(self):
        s = self.source.fresh()
        r = s.get('robot') or {}
        if r.get('dh_table') and r.get('dh') == 'controller':
            self.dh = np.array(r['dh_table'], float)
        self.rate = r.get('rate') or 0.0
        q = r.get('q')
        return (None if q is None else np.array(q, float)), self.source.t


def follow_calibration(app, source):
    """Keep the app's fixed-cam pose, base and gripper mount equal to the source's."""
    def loop():
        while True:
            s = source.fresh()
            cams = s.get('cams') or []
            if s.get('base'):
                app.scene['base'] = dict(s['base'])
            if cams and cams[0].get('located') is not None:
                app.scene['fcam'] = cams[0]['located']
                if cams[0].get('T_world_cam') is not None:
                    app.scene['T_world_fcam'] = cams[0]['T_world_cam']
            if len(cams) > 1 and cams[1].get('mount'):
                app.mount = kin.vec_to_T(np.array(cams[1]['mount'], float))
                app.scene['gcam'] = cams[1].get('mount_set')
            time.sleep(1.0)
    threading.Thread(target=loop, daemon=True, name='mirror-calibration').start()
