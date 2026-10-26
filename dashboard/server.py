"""
Camera + UR3e calibration dashboard — web server.

World frame = the checkerboard: origin at the board centre, x along its
columns, z up out of the table. Everything is placed in it:
  fixed cam     T_world_fcam from the D435's view of the board (Locate button)
  T Mini        T_world_tcam (Nova's heightmap frame) from its heightmap view of the board
                (locate_tof; calibration.ortho_board)
  robot base    upright, parallel to the board plane: x, y, yaw (base editor) and
                a fixed height z (9 mm below the board, from scene_solve --free-z)
  gripper cam   base · forward kinematics · mount. The mount starts from
                gripper_camera.mount in the config; "Mount from board view" sets
                it from one still view of the board given the base — no hand-eye
When the gripper cam sees the board, its board pose is compared with that chain.
Saved in configs/calibration/scene.yaml.

Serves web/ and streams to each browser over one websocket (/ws):
  text   JSON {"type": "state", ...}            robot, cameras, scene
  binary [u8 1][u8 cam_index] + JPEG            camera frame (0 fixed, 1 gripper, 2 SICK ToF if configured)
  binary [u8 2][u8 cam_index][2 pad][u32 n] + n*3 f32 xyz + n*3 u8 rgb   cloud in that camera's frame
The browser sends {"type": ...}: locate_fixed, locate_tof, mount_from_view, set_base {x, y, yaw}, set_overlays {on},
qa_test {port, phase}, qa_reset {port?}, locate_object {name, images?}.

POST /api/claude {"prompt", "session"?, "mode"?}   Claude Code CLI on the subscription (claude_chat.py)
/api/circuit...   Circuit Assembly tab: request -> questions -> circuit JSON -> CSP layout -> assembly
    order -> hand-off to the assembly pipeline and its progress (circuit_design.py; config `circuit`)

GET /api/experiments[/{run}[/trace|/videos|/video/{file}]][?src=<folder>]   agent runs (experiments.py):
    videos and transcripts; config `experiments` names the results folders to choose from
PUT /api/experiments/{run}/tags, POST|DELETE /api/experiments/{run}/comments[/{id}]
    tags and comments on a run, kept in <run>/notes.json

GET /api/objects[/{name}], POST /api/objects/{name}/locate   bounding box of a configured
    object (configs/objects.yaml, e.g. the protoboard) in the live clouds (objects.py)

GET /api/frame/0|1|2   latest full-resolution colour frame (PNG) of the fixed / gripper / ToF cam (ToF: IR intensity)
GET /api/depth/0|1|2   its aligned depth (.npy float32, metres, 0 = no reading; ToF: height in Nova's frame)

Port QA for robot scripts (blocks until the link check finishes):
  GET  /api/qa                         all 8 ports + live link
  POST /api/qa/{port}/insert|extract   -> check_link result, 200 pass / 409 fail
                                          port may be "current": the port the VLM reported
  POST /api/qa/vlm  {"port", "confidence"?, "source"?}   port seen by the gripper-camera VLM

Usage:  scripts/run_calib_dashboard.sh   ->  http://localhost:8090
        python -m dashboard.server --port 8091 --mirror http://127.0.0.1:8090
                                         a second, read-only dashboard on the same cameras (mirror.py)
"""

import argparse
import asyncio
import json
import struct
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import yaml
from aiohttp import WSMsgType, web

from agents.vision.locate import EDGES

from . import calibration as cal
from . import check_link
from . import circuit_design
from . import claude_chat
from . import experiments
from . import kinematics as kin
from . import mirror as mirror_mod
from . import objects
from .cameras import Camera, OrbbecCamera, RealSenseCamera, UVCCamera, VisionaryCamera, remove_outliers
from .ur_bridge import URBridge

cv2.setNumThreads(2)     # the detection, camera and robot threads share the CPU

ROOT = Path(__file__).resolve().parent.parent
WEB = Path(__file__).resolve().parent / 'web'


class Robot:
    """Polls the UR3e through the AI Agent Bridge (observe tier is enough)."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.lock = threading.Lock()
        self.q = None
        self.q_stamp = 0.0
        self.state = {}
        self.dh = None
        self.error = None
        self.rate = 0.0
        threading.Thread(target=self._loop, daemon=True, name='robot').start()

    def _loop(self):
        while True:
            try:
                b = URBridge(self.cfg['bridge_url'], client_name='order2cobot-dashboard')
                self.dh = kin.dh_from_bridge(b.call('robot_get_kinematic_info')['data'])
                self.error = None
                n, t0, i = 0, time.time(), 0
                while True:
                    d = b.call('robot_get_joint_positions')['data']
                    q = np.array([d[j] for j in kin.JOINT_NAMES])
                    with self.lock:
                        self.q, self.q_stamp = q, time.time()
                    if i % 20 == 0:
                        self.state = b.call('robot_get_state')['data']
                    i += 1
                    n += 1
                    if time.time() - t0 > 2:
                        self.rate, n, t0 = n / (time.time() - t0), 0, time.time()
                    time.sleep(self.cfg.get('poll_s', 0.05))
            except Exception as e:
                self.error = str(e)[:300]
                time.sleep(2.0)

    def joints(self):
        with self.lock:
            return self.q, self.q_stamp


class PortQA:
    """Switch port QA: the robot moves the cable from `iface` from port to port.
    A port passes when the link comes up at full speed and holds after insertion,
    and drops again after extraction. Results persist in results_file."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.iface = cfg.get('iface', check_link.IFACE)
        self.n = cfg.get('ports', 8)
        self.file = ROOT / cfg.get('results_file', 'data/qa/ports.json')
        self.ports = self._load()
        self.live = {}
        self.events = []
        self.busy = None
        self.message = ''
        self.vlm = None              # {'port', 'confidence', 'source', 't'} from the gripper-camera VLM
        self.lock = threading.Lock()
        threading.Thread(target=self._monitor, daemon=True, name='link').start()

    def _blank(self, i):
        return {'port': i, 'status': 'untested', 'insert': None, 'extract': None, 'attempts': 0, 'stamp': None}

    def _load(self):
        try:
            ports = json.loads(self.file.read_text())['ports']
            if len(ports) == self.n:
                return ports
        except (OSError, ValueError, KeyError):
            pass
        return [self._blank(i + 1) for i in range(self.n)]

    def _save(self):
        self.file.parent.mkdir(parents=True, exist_ok=True)
        self.file.write_text(json.dumps({'iface': self.iface, 'ports': self.ports}, indent=1))

    def _monitor(self):
        prev = None
        while True:
            try:
                s = check_link.read_link(self.iface)
                s['error'] = None
            except FileNotFoundError as e:
                s = {'iface': self.iface, 'up': False, 'error': str(e)}
            if prev is not None and s['up'] != prev:
                self.events = (self.events + [{'t': time.strftime('%H:%M:%S'), 'up': s['up'],
                                               'speed': s.get('speed')}])[-12:]
            prev = s['up']
            self.live = s
            time.sleep(0.1)

    def test(self, port, phase):
        """Run the insert (link must come up) or extract (link must drop) check on a port."""
        if not 1 <= port <= self.n or phase not in ('insert', 'extract'):
            raise ValueError(f'bad port {port} or phase {phase}')
        if not self.lock.acquire(blocking=False):
            raise RuntimeError(f'busy: {self.busy}')
        try:
            self.busy = f'port {port} {phase}'
            c = self.cfg
            res = check_link.wait_link(self.iface, up=phase == 'insert', timeout=c.get('timeout_s', 6.0),
                                       stable=c.get('stable_s', 1.0), min_speed=c.get('min_speed', 1000))
            p = self.ports[port - 1]
            p[phase] = res
            if phase == 'insert':
                p['attempts'] += 1
                p['extract'] = None
            p['stamp'] = res['stamp']
            checks = [p[k] for k in ('insert', 'extract') if p[k]]
            p['status'] = 'pass' if all(r['ok'] for r in checks) else 'fail'
            self._save()
            self.message = f'port {port} {phase}: {"pass" if res["ok"] else "FAIL"} · {res["reason"]}'
            return res
        finally:
            self.busy = None
            self.lock.release()

    def set_vlm(self, port, confidence=None, source='vlm'):
        port = int(port)
        if not 1 <= port <= self.n:
            raise ValueError(f'bad port {port}')
        self.vlm = {'port': port, 'confidence': None if confidence is None else float(confidence),
                    'source': str(source), 't': time.time()}
        return self.vlm

    def current_port(self):
        if not self.vlm:
            raise ValueError('no port reported by the VLM yet')
        return self.vlm['port']

    def reset(self, port=None):
        for i in [port] if port else range(1, self.n + 1):
            self.ports[i - 1] = self._blank(i)
        self._save()
        self.message = f'port {port} reset' if port else 'all ports reset'

    def state(self):
        vlm = self.vlm and {**self.vlm, 'age': round(time.time() - self.vlm['t'], 1)}
        return {'iface': self.iface, 'live': self.live, 'events': self.events, 'ports': self.ports,
                'busy': self.busy, 'message': self.message, 'vlm': vlm,
                'cfg': {k: self.cfg.get(k, d) for k, d in (('timeout_s', 6.0), ('stable_s', 1.0), ('min_speed', 1000))}}


def base_T(x, y, yaw):
    """Robot base on the board plane: z = 0, upright, turned yaw [rad] about z."""
    T = np.eye(4)
    c, s = np.cos(yaw), np.sin(yaw)
    T[:2, :2] = [[c, -s], [s, c]]
    T[:2, 3] = x, y
    return T


def tilt_deg(R):
    """Angle between a frame's z axis and world z."""
    return float(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))))


class App:
    def __init__(self, cfg, mirror=None):
        self.cfg = cfg
        self.mirror = mirror                         # source dashboard URL: read-only mirror (mirror.py)
        b = cfg['board']
        self.board = cal.Board(b['cols'], b['rows'], b['square'])
        self.T_world_board = kin.inv(self.board.T_board_world)
        self.scene_file = ROOT / cfg.get('scene_file', 'configs/calibration/scene.yaml')
        self.scene = (yaml.safe_load(self.scene_file.read_text()) if self.scene_file.exists() else None) or {}
        self.scene.setdefault('base', {'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0})
        self.mount = (np.array(self.scene['T_flange_gcam']) if self.scene.get('T_flange_gcam') is not None
                      else kin.vec_to_T(np.array(cfg['gripper_camera']['mount'], float)))
        if mirror:
            source = mirror_mod.Source(mirror)
            source.wait()
            self.robot = mirror_mod.MirrorRobot(source)
        else:
            self.robot = Robot(cfg['robot'])
        self.qa = PortQA(cfg.get('qa', {}))
        self.objects = objects.ObjectFinder(self, ROOT / cfg.get('objects_file', 'configs/objects.yaml'),
                                            ROOT / cfg.get('objects_dir', 'data/objects'))
        if mirror:
            hz = cfg.get('mirror_hz', 2.0)
            self.cams = [mirror_mod.MirrorCamera(source, 0, cfg['fixed_camera'].get('label', 'Fixed cam'), hz).start(),
                         mirror_mod.MirrorCamera(source, 1, cfg['gripper_camera'].get('label', 'Gripper cam'), hz).start()]
            mirror_mod.follow_calibration(self, source)
        else:
            self.cams = [_camera(RealSenseCamera, cfg['fixed_camera']),
                         _camera(_gripper_cam_class(), cfg['gripper_camera'])]
            if cfg.get('tof_camera'):            # optional; unplugged it just shows offline
                self.cams.append(_camera(VisionaryCamera, cfg['tof_camera']))
        self.det = [{} for _ in self.cams]           # latest detection per camera
        self.frames = [(0, None) for _ in self.cams]
        self.cloud_msgs = [(0, None) for _ in self.cams]
        self.clouds = [None for _ in self.cams]      # latest cloud per camera + its world pose, for objects
        self.plane = [None for _ in self.cams]       # depth on the board vs z = 0, per camera
        self.overlays = True
        self.busy = None
        self.message = ''
        for i in range(len(self.cams)):
            threading.Thread(target=self._vision_loop, args=(i,), daemon=True).start()
            threading.Thread(target=self._cloud_loop, args=(i,), daemon=True).start()

    # ── poses in the world (board) frame ──────────────────────────────────
    def T_world_base(self):
        b = self.scene['base']
        T = base_T(b['x'], b['y'], np.radians(b['yaw_deg']))
        T[2, 3] = b.get('z', 0.0)          # fixed offset below the board plane (scene_solve --free-z)
        return T

    def dh(self):
        return self.robot.dh if self.robot.dh is not None else kin.UR3E_NOMINAL_DH

    def cam_pose(self, idx, q=None):
        """T_world_cam, or None if the camera can't be placed."""
        if idx != 1:
            T = self.scene.get({0: 'T_world_fcam', 2: 'T_world_tcam'}[idx])
            return None if T is None else np.array(T)
        if q is None:
            q, _ = self.robot.joints()
        return None if q is None else self.T_world_base() @ kin.fk(q, self.dh()) @ self.mount

    def board_view(self, idx):
        """T_world_cam from this camera's live view of the board, or None."""
        p = self.det[idx].get('pnp')
        return None if p is None else self.T_world_board @ kin.inv(p[0])

    def gripper_check(self):
        """Gripper cam: FK chain vs its live board view. The base that would make
        them agree is reported too; with a right mount its z and tilt are ~0."""
        d = self.det[1]
        Tb, q = self.board_view(1), d.get('q')
        if Tb is None or q is None:
            return None
        Tf = self.cam_pose(1, q)
        dp, dr = kin.pose_error(Tf, Tb)
        Wb = Tb @ kin.inv(kin.fk(q, self.dh()) @ self.mount)     # implied T_world_base
        return {'dpos_mm': dp * 1000, 'drot_deg': np.degrees(dr),
                'base': {'x': Wb[0, 3], 'y': Wb[1, 3], 'z_mm': Wb[2, 3] * 1000,
                         'yaw_deg': np.degrees(np.arctan2(Wb[1, 0], Wb[0, 0])), 'tilt_deg': tilt_deg(Wb[:3, :3])}}

    # ── per-camera detection (rate-limited) + overlay + JPEG ──────────────
    def _detect_loop(self, idx):
        """Board detection at most detect_hz on the latest frame, in its own
        thread: a failed search (board partly hidden) is slow and must not stall
        the stream. The partial-board hint runs rarely."""
        cam = self.cams[idx]
        detect_dt = 1.0 / self.cfg.get('detect_hz', 3)
        last, t_partial = 0, 0.0
        while True:
            t0 = time.time()
            seq, img, depth = cam.latest()
            if img is not None and seq != last:
                last = seq
                q, _ = self.robot.joints()
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                if cam.orthographic:
                    self.det[idx] = {'seq': seq, 'stamp': time.time(), 'q': q, 'partial': None,
                                     **self._ortho_view(idx, gray, depth, self.det[idx].get('track'))}
                    time.sleep(max(0.01, detect_dt - (time.time() - t0)))
                    continue
                corners = self.board.detect(gray, fast=True)
                d = {'seq': seq, 'stamp': time.time(), 'q': q, 'corners': corners, 'pnp': None,
                     'partial': self.det[idx].get('partial')}
                if corners is not None:
                    d['partial'] = None
                    if cam.K is not None:
                        d['pnp'] = cal.solve_pnp(self.board, corners, cam.K, cam.dist)
                elif time.time() - t_partial > 3.0:
                    t_partial = time.time()
                    d['partial'] = self.board.detect_partial(gray)
                self.det[idx] = d
            time.sleep(max(0.01, detect_dt - (time.time() - t0)))

    def _ortho_view(self, idx, gray, depth, track=None):
        """Board on an orthographic heightmap camera: corners, pnp (T_cam_board, rms px), track."""
        cam = self.cams[idx]
        f = None if depth is None or cam.grid is None else cal.ortho_board(
            self.board, gray, depth, cam.grid, prev=track, thickness=self.cfg['board'].get('thickness', 0.0))
        return ({'corners': None, 'pnp': None, 'track': None} if f is None else
                {'corners': f['corners'], 'pnp': (f['T'], f['rms_px']), 'track': f['track']})

    def _vision_loop(self, idx):
        """Overlay + JPEG for every new frame, up to stream_hz. Overlays reuse
        the latest detection."""
        cam = self.cams[idx]
        stream_dt = 1.0 / self.cfg.get('stream_hz', 10)
        detect_dt = 1.0 / self.cfg.get('detect_hz', 3)
        last, t_stream = 0, 0.0
        threading.Thread(target=self._detect_loop, args=(idx,), daemon=True).start()
        while True:
            seq, img, depth = cam.latest()
            now = time.time()
            if img is None or seq == last or now < t_stream:
                time.sleep(0.003)
                continue
            last, t_stream = seq, max(t_stream + stream_dt, now - stream_dt)   # average stream_hz
            d = self.det[idx]
            if now - d.get('stamp', 0) > 2.5 * detect_dt + 0.5:   # drop corners that are out of date
                d = {**d, 'corners': None, 'pnp': None}
            out = cam.preview(self._draw(idx, img.copy(), d) if self.overlays else img, depth)
            scale = self.cfg.get('stream_width', 960) / out.shape[1]
            if scale < 1:
                out = cv2.resize(out, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            ok, jpg = cv2.imencode('.jpg', out, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok:
                self.frames[idx] = (seq, bytes([1, idx]) + jpg.tobytes())

    def _draw(self, idx, img, d):
        """Detected corners + board axes; then, from the scene calibration, the
        robot's joint chain (cream) and the board corners (cyan rings) — both
        should sit on the real ones."""
        cam = self.cams[idx]
        if d['corners'] is not None:
            cv2.drawChessboardCorners(img, (self.board.cols, self.board.rows),
                                      d['corners'].astype(np.float32).reshape(-1, 1, 2), True)
            if d['pnp'] is not None and cam.K is not None:
                T, _ = d['pnp']
                rv, _ = cv2.Rodrigues(T[:3, :3])
                cv2.drawFrameAxes(img, cam.K, cam.dist, rv, T[:3, 3], self.board.square * 3, 3)
        q, _ = self.robot.joints()
        Tw = self.cam_pose(idx, q)
        if Tw is None or cam.K is None:
            return img
        Ti = kin.inv(Tw)

        def project(P):
            P = (Ti[:3, :3] @ P.T).T + Ti[:3, 3]
            return cal._project(P, cam.K, cam.dist).astype(int) if np.all(P[:, 2] > 0.05) else None

        if q is not None and idx == 0:          # the gripper cam rides on the arm: no chain to see
            Tb = self.T_world_base()
            P = np.array([(Tb @ F)[:3, 3] for F in kin.fk_frames(q, self.dh())])
            uv = project(P)
            if uv is not None:
                for a, b in zip(uv[:-1], uv[1:]):
                    cv2.line(img, tuple(a), tuple(b), (218, 245, 254), 3, cv2.LINE_AA)
                for p in uv:
                    cv2.circle(img, tuple(p), 6, (60, 72, 210), -1, cv2.LINE_AA)
        Wb = self.T_world_board
        uv = project((Wb[:3, :3] @ self.board.obj.T).T + Wb[:3, 3])
        if uv is not None:
            for p in uv:
                cv2.circle(img, tuple(p), 4, (255, 220, 0), 1, cv2.LINE_AA)
        # located objects: model instances (grey) and the bounding box (brass), edges sampled
        # in 3D so they bend with the lens distortion like the real ones
        t = np.linspace(0, 1, 24)[:, None]
        for name, box, inst in self.objects.boxes():
            parts = [*((b, (119, 133, 140), 1) for b in inst if len(inst) > 1), (box, (93, 161, 201), 2)]
            for corners, colour, width in parts:
                for a, b in EDGES:
                    uv = project(corners[a] + t * (corners[b] - corners[a]))
                    if uv is not None:
                        cv2.polylines(img, [uv.reshape(-1, 1, 2).astype(np.int32)], False, colour, width, cv2.LINE_AA)
            uv = project(box[4:5])
            if uv is not None:
                cv2.putText(img, name, (int(uv[0, 0]) + 6, int(uv[0, 1]) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (93, 161, 201), 2, cv2.LINE_AA)
        return img

    def _cloud_loop(self, idx):
        cam = self.cams[idx]
        last = 0
        while True:
            time.sleep(1.0 / self.cfg.get('cloud_hz', 2))
            seq, color, depth = cam.latest()
            if depth is None or seq == last:
                continue
            last = seq
            q, _ = self.robot.joints()
            c = cam.cloud(color, depth, getattr(cam, 'cfg', {}).get('cloud_stride', self.cfg.get('cloud_stride', 4)))
            if c is None:
                continue
            xyz, rgb = remove_outliers(*c, self.cfg.get('cloud_outlier_k', 8), self.cfg.get('cloud_outlier_std', 2.0),
                                       by_depth=not cam.orthographic)
            Tw = self.cam_pose(idx, q)
            self.plane[idx] = self._board_plane(xyz, Tw)
            self.clouds[idx] = {'xyz': xyz, 'rgb': rgb, 'T': Tw, 't': time.time()}
            self.cloud_msgs[idx] = (seq, struct.pack('<BBxxI', 2, idx, len(xyz)) + xyz.tobytes() + rgb.tobytes())

    def _board_plane(self, xyz, Tw):
        """Plane fitted to the depth points over the board (in the world frame):
        height at the board centre and tilt. Both ~0 when the camera is placed right."""
        if Tw is None or len(xyz) == 0:
            return None
        P = (Tw[:3, :3] @ xyz.T).T + Tw[:3, 3]
        hx, hy = (self.board.cols + 1) * self.board.square / 2, (self.board.rows + 1) * self.board.square / 2
        P = P[(np.abs(P[:, 0]) < hx) & (np.abs(P[:, 1]) < hy)]
        if len(P) < 30:
            return None
        P = P[np.abs(P[:, 2] - np.median(P[:, 2])) < 0.02]       # the board, not the arm above it
        A = np.c_[P[:, 0], P[:, 1], np.ones(len(P))]
        (a, b, c), *_ = np.linalg.lstsq(A, P[:, 2], rcond=None)        # z = a x + b y + c
        return {'z_mm': float(c * 1000), 'tilt_deg': float(np.degrees(np.arctan(np.hypot(a, b)))),
                'rms_mm': float(np.std(P[:, 2] - A @ [a, b, c]) * 1000), 'n': int(len(P))}

    # ── actions ───────────────────────────────────────────────────────────
    def _read_only(self):
        if self.mirror:
            raise RuntimeError(f'read-only mirror of {self.mirror}: calibrate on that dashboard')

    def _save_scene(self):
        self.scene_file.parent.mkdir(parents=True, exist_ok=True)
        self.scene_file.write_text(yaml.safe_dump(self.scene, sort_keys=False))

    def _board_views(self, idx, n=8, timeout=6):
        """Full-accuracy board views of a camera: [(T_world_cam, rms_px, q)]."""
        cam = self.cams[idx]
        if (cam.K is None and not cam.orthographic) or cam.error:
            raise RuntimeError(f'{cam.name} not ready ({cam.error})')
        out, last, t0, track = [], 0, time.time(), self.det[idx].get('track')
        while len(out) < n and time.time() - t0 < timeout:
            seq, img, depth = cam.latest()
            q, _ = self.robot.joints()
            if img is not None and seq != last:
                last = seq
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                if cam.orthographic:
                    v = self._ortho_view(idx, gray, depth, track)
                    p, track = v['pnp'], v['track'] or track
                else:
                    c = self.board.detect(gray)
                    p = c is not None and cal.solve_pnp(self.board, c, cam.K, cam.dist)
                if p:
                    out.append((self.T_world_board @ kin.inv(p[0]), p[1], q))
            time.sleep(0.05)
        if len(out) < 3:
            raise RuntimeError(f'{cam.name} does not see the whole board')
        return out

    @staticmethod
    def _average(Ts):
        T = kin.mean_T(Ts)
        return T, max(kin.pose_error(T, t)[0] for t in Ts) * 1000

    def locate_fixed(self):
        """Fixed cam pose from its view of the board, averaged over several frames."""
        return self._locate(0, 'fcam', 'fixed cam')

    def locate_tof(self):
        """SICK T Mini pose (Nova's grid frame) from its heightmap view of the board."""
        if len(self.cams) < 3:
            raise RuntimeError('no tof_camera in the config')
        return self._locate(2, 'tcam', 'T Mini', timeout=15)

    def _locate(self, idx, key, label, timeout=6):
        self._read_only()
        views = self._board_views(idx, timeout=timeout)
        T, spread = self._average([v[0] for v in views])
        rms = float(np.mean([v[1] for v in views]))
        cam = self.cams[idx]
        self.scene[f'T_world_{key}'] = T.tolist()
        self.scene[key] = {'stamp': time.strftime('%Y-%m-%dT%H:%M:%S'), 'frames': len(views), 'rms_px': rms,
                           'spread_mm': float(spread), 'size': cam.size,
                           **({'K': cam.K.tolist()} if cam.K is not None else {'grid': np.asarray(cam.grid).tolist()})}
        self._save_scene()
        return f'{label} located: {len(views)} frames, {rms:.2f} px, spread {spread:.1f} mm — saved'

    def mount_from_view(self):
        """Gripper cam mount from a still view of the board, given the base:
        mount = (base · FK)^-1 · board view. Redo it after moving the base."""
        self._read_only()
        views = self._board_views(1)
        qs = np.array([v[2] for v in views])
        if np.ptp(qs, axis=0).max() > 2e-4:
            raise RuntimeError('robot moved during the capture; hold still and retry')
        Tb = self.T_world_base()
        M, spread = self._average([kin.inv(Tb @ kin.fk(q, self.dh())) @ T for T, _, q in views])
        rms = float(np.mean([v[1] for v in views]))
        self.mount = M
        self.scene['T_flange_gcam'] = M.tolist()
        self.scene['gcam'] = {'stamp': time.strftime('%Y-%m-%dT%H:%M:%S'), 'frames': len(views), 'rms_px': rms,
                              'spread_mm': float(spread), 'q': qs[0].tolist(), 'base': dict(self.scene['base'])}
        self._save_scene()
        return f'gripper cam mount set: {len(views)} frames, {rms:.2f} px, spread {spread:.1f} mm — saved'

    def set_base(self, x, y, yaw_deg):
        if self.mirror:
            return f'read-only mirror of {self.mirror}: set the base on that dashboard'
        self.scene['base'] = {**self.scene['base'], 'x': float(x), 'y': float(y), 'yaw_deg': float(yaw_deg)}
        self._save_scene()
        return f'base at {x * 1000:.0f}, {y * 1000:.0f} mm, yaw {yaw_deg:.1f}° — saved'

    # ── state for the browser ─────────────────────────────────────────────
    def state(self):
        r = self.robot
        q, q_stamp = r.joints()
        robot = {'connected': r.error is None and q is not None, 'error': r.error,
                 'mode': r.state.get('robotmode'), 'control': r.state.get('controlmode'),
                 'rate': round(r.rate, 1), 'dh': 'controller' if r.dh is not None else 'nominal'}
        robot['dh_table'] = self.dh().tolist()
        if q is not None:
            robot['q'] = q.tolist()
            robot['flange'] = kin.fk(q, self.dh()).tolist()     # in the base frame
        cams = []
        for i, c in enumerate(self.cams):
            d = self.det[i]
            info = c.info()
            info['board'] = {'seen': d.get('corners') is not None,
                             'rms': None if not d.get('pnp') else round(d['pnp'][1], 3),
                             'dist': None if not d.get('pnp') else float(np.linalg.norm(d['pnp'][0][:3, 3])),
                             'partial': d.get('partial')}
            Tw, Tv = self.cam_pose(i, q), self.board_view(i)
            info['T_world_cam'] = None if Tw is None else Tw.tolist()
            info['T_world_cam_view'] = None if Tv is None else Tv.tolist()     # from the live board view
            info['plane'] = self.plane[i]
            cams.append(info)
        Tv = self.board_view(0)
        cams[0]['live_vs_saved'] = None if Tv is None or self.scene.get('T_world_fcam') is None else dict(zip(
            ('dpos_mm', 'drot_deg'), np.multiply(kin.pose_error(np.array(self.scene['T_world_fcam']), Tv), (1000, 57.2958))))
        cams[0]['located'] = self.scene.get('fcam')
        if len(cams) > 2:
            cams[2]['located'] = self.scene.get('tcam')
        cams[1]['check'] = self.gripper_check()
        cams[1]['mount'] = kin.T_to_vec(self.mount).tolist()
        cams[1]['mount_set'] = self.scene.get('gcam')
        return {'type': 'state', 't': time.time(), 'robot': robot, 'cams': cams,
                'board': self.board.describe(), 'base': self.scene['base'],
                'T_world_base': self.T_world_base().tolist(),
                'busy': self.busy, 'message': self.message, 'overlays': self.overlays,
                'qa': self.qa.state(), 'objects': self.objects.state(), 'mirror': self.mirror}


def _camera(cls, cfg):
    """Started camera reader, or an idle placeholder when the config says enabled: false
    (keeps the camera indices; e.g. a second dashboard that must not grab the USB cameras)."""
    if cfg.get('enabled', True):
        return cls(cfg).start()
    cam = Camera()
    cam.cfg, cam.kind = cfg, cls.kind
    cam.name, cam.error = cfg.get('label', cls.__name__), 'disabled in config'
    return cam


def _gripper_cam_class():
    """Orbbec SDK (color + depth) when installed, else UVC color only."""
    try:
        import pyorbbecsdk  # noqa: F401
        return OrbbecCamera
    except ImportError:
        return UVCCamera


def _no_nan(o):
    """NaN/inf -> None, numpy scalars -> float: the browser's JSON.parse rejects
    NaN and drops the whole state."""
    if isinstance(o, (float, np.floating)):
        return float(o) if np.isfinite(o) else None
    if isinstance(o, dict):
        return {k: _no_nan(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_no_nan(v) for v in o]
    return o


async def _client_loop(ws, app, dt):
    sent = {}
    sent_cloud = {}
    t_state = 0.0
    try:
        while not ws.closed:
            # frames go out as soon as they exist; state at its own rate
            if time.time() - t_state >= dt:
                t_state = time.time()
                await ws.send_str(json.dumps(_no_nan(app.state())))
            for i, (seq, payload) in enumerate(app.frames):
                if payload is not None and sent.get(i) != seq:
                    sent[i] = seq
                    await ws.send_bytes(payload)
            for i, (seq, payload) in enumerate(app.cloud_msgs):
                if payload is not None and sent_cloud.get(i) != seq:
                    sent_cloud[i] = seq
                    await ws.send_bytes(payload)
            await asyncio.sleep(0.01)
    except (ConnectionResetError, RuntimeError):
        pass


async def _run_action(app, name, fn, *args):
    if app.busy:
        app.message = f'busy: {app.busy}'
        return
    app.busy = name
    try:
        app.message = await asyncio.get_running_loop().run_in_executor(None, fn, *args)
    except Exception as e:
        app.message = f'{name} failed: {e}'
    finally:
        app.busy = None


async def _locate_object(app, name, images=None):
    def run():
        r = app.objects.locate(name, images if images in ('auto', 'always', 'never') else None)
        p, (L, W, H) = r['box']['pose'], r['box']['size_mm']
        return (f"{r['label']}: x {p['x'] * 1000:.1f} mm, y {p['y'] * 1000:.1f} mm, yaw {p['yaw_deg']:.2f}°, "
                f"{L:.0f} x {W:.0f} x {H:.1f} mm ({r['method']}, {r['duration_s']:.0f} s)")
    if name not in app.objects.specs:
        app.message = f'unknown object {name}'
        return
    await _run_action(app, f'locating {name}', run)


async def ws_handler(request):
    ws = web.WebSocketResponse(heartbeat=10.0, max_msg_size=0)
    await ws.prepare(request)
    app = request.app['app']
    pusher = asyncio.create_task(_client_loop(ws, app, request.app['dt']))
    try:
        async for msg in ws:
            if msg.type != WSMsgType.TEXT:
                continue
            try:
                req = json.loads(msg.data)
            except json.JSONDecodeError:
                continue
            t = req.get('type')
            if t == 'locate_fixed':
                asyncio.create_task(_run_action(app, 'locate fixed cam', app.locate_fixed))
            elif t == 'locate_tof':
                asyncio.create_task(_run_action(app, 'locate T Mini', app.locate_tof))
            elif t == 'mount_from_view':
                asyncio.create_task(_run_action(app, 'gripper cam mount', app.mount_from_view))
            elif t == 'set_base':
                app.message = app.set_base(float(req['x']), float(req['y']), float(req['yaw']))
            elif t == 'set_overlays':
                app.overlays = bool(req.get('on'))
            elif t == 'qa_test':
                asyncio.create_task(_qa_test(app.qa, int(req.get('port', 0)), req.get('phase')))
            elif t == 'locate_object':
                asyncio.create_task(_locate_object(app, str(req.get('name')), req.get('images')))
            elif t == 'qa_reset':
                app.qa.reset(req.get('port'))
    finally:
        pusher.cancel()
    return ws


async def _qa_test(qa, port, phase):
    try:
        return await asyncio.get_running_loop().run_in_executor(None, qa.test, port, phase)
    except (ValueError, RuntimeError) as e:
        qa.message = str(e)


async def qa_get(request):
    return web.json_response(request.app['app'].qa.state())


async def qa_post(request):
    qa = request.app['app'].qa
    try:
        port, phase = request.match_info['port'], request.match_info['phase']
        port = qa.current_port() if port == 'current' else int(port)
        res = await asyncio.get_running_loop().run_in_executor(None, qa.test, port, phase)
    except ValueError as e:
        return web.json_response({'ok': False, 'reason': str(e)}, status=400)
    except RuntimeError as e:
        return web.json_response({'ok': False, 'reason': str(e)}, status=423)
    return web.json_response(res, status=200 if res['ok'] else 409)


async def qa_vlm(request):
    qa = request.app['app'].qa
    try:
        body = await request.json()
        return web.json_response(qa.set_vlm(body['port'], body.get('confidence'), body.get('source', 'vlm')))
    except (ValueError, KeyError, TypeError) as e:
        return web.json_response({'ok': False, 'reason': f'expected {{"port": 1-{qa.n}}}: {e}'}, status=400)


def _cam(request):
    cams, i = request.app['app'].cams, int(request.match_info['cam'])
    return cams[i] if i < len(cams) else None


async def frame_get(request):
    """Latest full-resolution colour frame of camera 0 (fixed), 1 (gripper) or 2 (ToF) as PNG."""
    cam = _cam(request)
    if cam is None:
        return web.json_response({'ok': False, 'reason': 'no such camera'}, status=404)
    _, img, _ = cam.latest()
    if img is None:
        return web.json_response({'ok': False, 'reason': cam.error or 'no frame'}, status=503)
    ok, png = cv2.imencode('.png', img)
    return web.Response(body=png.tobytes(), content_type='image/png')


async def depth_get(request):
    """Latest depth of camera 0 / 1 / 2, aligned to its colour frame: .npy float32 metres (0 = none)."""
    import io
    cam = _cam(request)
    if cam is None:
        return web.json_response({'ok': False, 'reason': 'no such camera'}, status=404)
    _, _, depth = cam.latest()
    if depth is None:
        return web.json_response({'ok': False, 'reason': cam.error or 'no depth'}, status=503)
    buf = io.BytesIO()
    np.save(buf, depth.astype(np.float32))
    return web.Response(body=buf.getvalue(), content_type='application/octet-stream')


async def index(_):
    return web.FileResponse(WEB / 'index.html')


@web.middleware
async def no_cache(request, handler):
    """Always revalidate the page and its scripts: a cached app.js from an older
    version against a newer index.html leaves the page stuck on "Connecting"."""
    resp = await handler(request)
    if isinstance(resp, web.StreamResponse) and not request.path.startswith('/api') and request.path != '/ws':
        resp.headers['Cache-Control'] = 'no-cache'
    return resp


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default=str(ROOT / 'configs' / 'calib_dashboard.yaml'))
    ap.add_argument('--port', type=int)
    ap.add_argument('--mirror', metavar='URL',
                    help='mirror the dashboard at URL (it owns the cameras and robot): read-only, see mirror.py')
    a = ap.parse_args()
    cfg = yaml.safe_load(Path(a.config).read_text())
    w = web.Application(middlewares=[no_cache])
    w['app'] = App(cfg, mirror=a.mirror)
    w['dt'] = 1.0 / cfg.get('state_hz', 10)
    w.router.add_get('/', index)
    w.router.add_get('/ws', ws_handler)
    w.router.add_get('/api/qa', qa_get)
    w.router.add_get('/api/frame/{cam:[0-9]}', frame_get)
    w.router.add_get('/api/depth/{cam:[0-9]}', depth_get)
    w.router.add_post('/api/qa/vlm', qa_vlm)
    w.router.add_post('/api/qa/{port}/{phase}', qa_post)
    w.router.add_routes(claude_chat.routes(ROOT))
    w.router.add_routes(circuit_design.routes(cfg.get('circuit')))
    sources = {n: (ROOT / e['dir'], ROOT / e['cache']) for n, e in (cfg.get('experiments') or {}).items()} or {
        'results': (ROOT / cfg.get('results_dir', 'results'), ROOT / cfg.get('experiments_cache', 'data/cache/experiments'))}
    w.router.add_routes(experiments.routes(sources))
    w.router.add_routes(objects.routes(w['app'].objects))
    w.router.add_static('/', WEB)
    port = a.port or cfg.get('port', 8090)
    print(f'calibration dashboard: http://localhost:{port}' + (f' (mirror of {a.mirror})' if a.mirror else ''))
    web.run_app(w, port=port, print=None)


if __name__ == '__main__':
    main()
