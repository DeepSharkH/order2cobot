"""Objects in the scene: locate a configured object (configs/wire/objects.yaml, e.g. the
protoboard) in the live clouds of both cameras and keep its world bounding box.

The search itself is agents/vision/locate.py: template search on a height map, an edge fit
on the cloud and, when that is not precise enough, an edge fit on the raw camera frames
(no overlays). Each camera contributes its latest cloud, placed with the pose it had when
the cloud was taken; the fixed cam's frame is trusted when the cam was located from the
board, the gripper cam's when its FK-vs-board check is within `max_check_mm`.

The last result per object is kept in memory, sent with the dashboard state, drawn into the
camera feeds, and saved to <objects_dir>/<name>.json for the agents and scripts.

GET  /api/objects                 specs and the last result of each object
GET  /api/objects/{name}          last result (404 before the first search)
POST /api/objects/{name}/locate   search now; {"images": "auto|always|never", "clouds": n}?
                                  -> result, 409 when nothing of that size is found
"""

import asyncio
import dataclasses
import json
import threading
import time
from pathlib import Path

import cv2
import numpy as np
from aiohttp import web

from agents.vision.locate import CameraView, ObjectNotFound, load_objects, locate_object


class ObjectFinder:
    def __init__(self, app, specs_file, results_dir, max_check_mm=2.0, image_width=960, clouds_per_camera=1):
        self.app = app
        self.image_width = image_width
        self.clouds_per_camera = clouds_per_camera
        self.specs = load_objects(specs_file) if Path(specs_file).exists() else {}
        self.dir = Path(results_dir)
        self.max_check_mm = max_check_mm
        self.results = {}
        self.busy = None
        self.lock = threading.Lock()
        for name in self.specs:                     # the last saved result survives a restart
            f = self.dir / f'{name}.json'
            if f.exists():
                try:
                    self.results[name] = json.loads(f.read_text())
                except (OSError, json.JSONDecodeError):
                    pass

    def state(self):
        return {'busy': self.busy,
                'specs': [{'name': s.name, 'label': s.label, 'size_mm': [round(v * 1000, 2) for v in s.size],
                           'instances': s.instances, 'images': s.images, 'mesh': s.mesh}
                          for s in self.specs.values()],
                'results': {n: _brief(r) for n, r in self.results.items()}}

    # ── inputs ──────────────────────────────────────────────────────────────
    def trust(self, i):
        """(trusted, why) for camera i's world pose."""
        if i == 0:
            located = bool(self.app.scene.get('fcam'))
            return located, 'located from the board' if located else 'not located'
        if i == 2:                              # SICK ToF: placed only by hand so far
            located = bool(self.app.scene.get('tcam'))
            return located, 'located' if located else 'unverified pose'
        chk = self.app.gripper_check()
        d = None if chk is None else chk['dpos_mm']
        return (d is not None and d < self.max_check_mm,
                f'FK vs board check {d:.1f} mm' if d is not None else 'unverified pose (board not in view)')

    def world_cloud(self, n=None, timeout_s=10.0):
        """The next `n` clouds of every placed camera in the world frame: {camera: xyz}."""
        n = n or self.clouds_per_camera
        got = [[] for _ in self.app.clouds]
        seen = [None for _ in self.app.clouds]
        t_end = time.time() + timeout_s
        while time.time() < t_end:
            for i, c in enumerate(self.app.clouds):
                if c is not None and c['T'] is not None and c['t'] != seen[i] and len(got[i]) < n:
                    seen[i] = c['t']
                    got[i].append(c)
            live = [g for i, g in enumerate(got) if self.app.clouds[i] is not None and self.app.clouds[i]['T'] is not None]
            if live and all(len(g) >= n for g in live):
                break
            time.sleep(0.05)
        per_cam = {i: np.concatenate([c['xyz'] @ c['T'][:3, :3].T + c['T'][:3, 3] for c in g]).astype(float)
                   for i, g in enumerate(got) if g}
        if not per_cam:
            raise ObjectNotFound('no point cloud yet: no camera is placed in the world or has depth')
        return per_cam

    def views(self):
        """Raw full-resolution frames with pose and intrinsics, for the image fit."""
        q, _ = self.app.robot.joints()
        out = []
        for i, cam in enumerate(self.app.cams):
            _, color, _ = cam.latest()
            T = self.app.cam_pose(i, q)
            if color is None or T is None or cam.K is None:
                continue
            trusted, why = self.trust(i)
            h, w = color.shape[:2]
            img = color[:, :, ::-1]
            if w > self.image_width:            # the edge fit was tuned on the 960 px feeds, and is 2x faster
                img = cv2.resize(img, (self.image_width, round(h * self.image_width / w)), interpolation=cv2.INTER_AREA)
            out.append(CameraView(name=cam.name, K=np.asarray(cam.K, float), dist=np.asarray(cam.dist, float).ravel(),
                                  size=(w, h), img=np.ascontiguousarray(img), T_world_cam=T, trusted=trusted, why=why))
        return out

    # ── search ──────────────────────────────────────────────────────────────
    def locate(self, name, images=None, clouds=None):
        """Blocking: search for `name` now. Returns the result, raises ObjectNotFound/KeyError."""
        spec = self.specs[name]
        if images:
            spec = dataclasses.replace(spec, images=images)
        if not self.lock.acquire(blocking=False):
            raise RuntimeError(f'busy: locating {self.busy}')
        self.busy = name
        try:
            per_cam = self.world_cloud(clouds)
            # fitted: the clouds of cameras whose pose is trusted; the others are fitted alone
            # as a cross-check (two clouds whose poses disagree would smear the edges)
            main = [i for i in per_cam if self.trust(i)[0]] or list(per_cam)
            xyz = np.concatenate([per_cam[i] for i in main])
            checks = {self.app.cams[i].name: per_cam[i] for i in per_cam if i not in main}
            cams = self.views() if spec.images != 'never' else []
            log = []
            r = locate_object(spec, xyz, cams, log=log.append, check_clouds=checks)
            r['clouds'] = [{'camera': self.app.cams[i].name, 'points': len(per_cam[i]),
                            'fitted': i in main, 'why': self.trust(i)[1]} for i in per_cam]
            r['cameras'] = [{'camera': c.name, 'trusted': c.trusted, 'why': c.why, 'used': c.used} for c in cams]
            r['log'] = log
            self.results[name] = r
            self.dir.mkdir(parents=True, exist_ok=True)
            (self.dir / f'{name}.json').write_text(json.dumps(r, indent=1) + '\n')
            return r
        finally:
            self.busy = None
            self.lock.release()

    def boxes(self):
        """(label, corners 8x3, instance corners) of every result, for the feed overlays."""
        return [(r['label'], np.array(r['box']['corners']), [np.array(b) for b in r['box'].get('instance_corners', [])])
                for r in self.results.values() if r.get('box')]


def _brief(r):
    """The result without its log, for the 10 Hz state."""
    return {k: v for k, v in r.items() if k != 'log'}


def routes(finder):
    rt = web.RouteTableDef()

    @rt.get('/api/objects')
    async def list_objects(_):
        return web.json_response(finder.state())

    @rt.get('/api/objects/{name}')
    async def get_object(request):
        r = finder.results.get(request.match_info['name'])
        if r is None:
            raise web.HTTPNotFound(text=json.dumps({'error': 'not located yet'}), content_type='application/json')
        return web.json_response(r)

    @rt.post('/api/objects/{name}/locate')
    async def post_locate(request):
        name = request.match_info['name']
        if name not in finder.specs:
            raise web.HTTPNotFound(text=json.dumps({'error': f'unknown object {name}'}), content_type='application/json')
        body = await request.json() if request.can_read_body else {}
        images = body.get('images')
        clouds = body.get('clouds')
        if clouds is not None and (not isinstance(clouds, int) or not 1 <= clouds <= 20):
            raise web.HTTPBadRequest(text=json.dumps({'error': 'clouds: 1 to 20 per camera'}),
                                     content_type='application/json')
        if images not in (None, 'auto', 'always', 'never'):
            raise web.HTTPBadRequest(text=json.dumps({'error': 'images: auto, always or never'}),
                                     content_type='application/json')
        try:
            r = await asyncio.to_thread(finder.locate, name, images, clouds)
        except ObjectNotFound as e:
            return web.json_response({'error': str(e)}, status=409)
        except RuntimeError as e:
            return web.json_response({'error': str(e)}, status=409)
        return web.json_response(r)

    return rt
