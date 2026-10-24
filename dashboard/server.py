"""
Camera + UR3e calibration dashboard — web server.

Serves web/ and streams to each browser over one websocket (/ws):
  text   JSON {"type": "state", ...}            robot, cameras, calibration
  text   JSON {"type": "result", ...}           after a solve
  binary [u8 1][u8 cam_index] + JPEG            camera frame (0 fixed, 1 gripper)
  binary [u8 2][u8 cam_index][2 pad][u32 n] + n*3 f32 xyz + n*3 u8 rgb   cloud in that camera's optical frame
The browser sends {"type": ...}: capture, delete_sample {id}, new_session,
solve {opts}, set_overlays {on}, qa_test {port, phase}, qa_reset {port?}.

Port QA for robot scripts (blocks until the link check finishes):
  GET  /api/qa                         all 8 ports + live link
  POST /api/qa/{port}/insert|extract   -> check_link result, 200 pass / 409 fail
                                          port may be "current": the port the VLM reported
  POST /api/qa/vlm  {"port", "confidence"?, "source"?}   port seen by the gripper-camera VLM

Usage:  scripts/run_calib_dashboard.sh   ->  http://localhost:8090
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

from . import base_align
from . import calibration as cal
from . import check_link
from . import kinematics as kin
from .cameras import OrbbecCamera, RealSenseCamera, UVCCamera, depth_to_cloud, remove_outliers, table_pose
from .robot_model import UR3eSurface
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
        self.pose = None
        self.state = {}
        self.status = {}
        self.dh = None
        self.error = None
        self.rate = 0.0
        threading.Thread(target=self._loop, daemon=True, name='robot').start()

    def _loop(self):
        while True:
            try:
                b = URBridge(self.cfg['bridge_url'], client_name='order2cobot-dashboard')
                self.status = b.call('bridge_my_status').get('data', {})
                self.dh = kin.dh_from_bridge(b.call('robot_get_kinematic_info')['data'])
                self.error = None
                n, t0, i = 0, time.time(), 0
                while True:
                    d = b.call('robot_get_joint_positions')['data']
                    q = np.array([d[j] for j in kin.JOINT_NAMES])
                    with self.lock:
                        self.q, self.q_stamp = q, time.time()
                    if i % 5 == 0:
                        p = b.call('robot_get_pose')['data']
                        self.pose = {'T': kin.pose_to_T(p['pose']['position'], p['pose']['orientation']),
                                     'tcp': p.get('tcp', {}).get('name'),
                                     'q': np.array([p['qNear'][j] for j in kin.JOINT_NAMES])}
                    if i % 20 == 0:
                        self.state = b.call('robot_get_state')['data']
                        self.status = b.call('bridge_my_status').get('data', {})
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


class App:
    def __init__(self, cfg):
        self.cfg = cfg
        b = cfg['board']
        self.board = cal.Board(b['cols'], b['rows'], b['square'])
        self.result_file = ROOT / cfg['result_file']
        self.result = yaml.safe_load(self.result_file.read_text()) if self.result_file.exists() else None
        self.manual_file = ROOT / cfg.get('manual_base_file', 'configs/calibration/manual_base.yaml')
        self.manual = yaml.safe_load(self.manual_file.read_text()) if self.manual_file.exists() else None
        self.robot = Robot(cfg['robot'])
        self.qa = PortQA(cfg.get('qa', {}))
        self.cams = [RealSenseCamera(cfg['fixed_camera']).start(),
                     _gripper_cam_class()(cfg['gripper_camera'], self.result_file).start()]
        self.store = cal.SampleStore(ROOT / cfg['samples_dir'])
        self.det = [{} for _ in self.cams]           # latest detection per camera
        self.frames = [(0, None) for _ in self.cams]
        self.cloud_msgs = [(0, None) for _ in self.cams]
        self.fcam_table = None   # T_world_fcam with the table on z = 0 (until calibrated)
        self.overlays = True
        self.surface = UR3eSurface()
        self._model_cache = (None, None)
        self._ovl_cache = (None, None)
        self.stream_ms = [0.0, 0.0]
        self.busy = None
        self.message = ''
        for i in range(len(self.cams)):
            threading.Thread(target=self._vision_loop, args=(i,), daemon=True).start()
        for i in range(len(self.cams)):
            threading.Thread(target=self._cloud_loop, args=(i,), daemon=True).start()

    # ── calibration-derived transforms ────────────────────────────────────
    def transforms(self):
        r = self.result or {}
        T = {k: np.array(r[k]) for k in ('T_flange_gcam', 'T_base_board', 'T_base_fcam', 'T_world_base')
             if r.get(k) is not None}
        if self.manual and self.manual.get('active'):
            Twb = np.array(self.manual['T_world_base'])
            # the fixed cam (and its cloud) stays put in the world while the base is slid
            if 'T_base_fcam' in T:
                T['T_base_fcam'] = kin.inv(Twb) @ T.get('T_world_base', np.eye(4)) @ T['T_base_fcam']
            T['T_world_base'] = Twb
        return T

    def base_source(self):
        if self.manual and self.manual.get('active'):
            return 'manual'
        return 'calibrated' if self.result and self.result.get('T_world_base') else 'none'

    def set_manual_base(self, T, active=True):
        if T is None and self.manual:
            T = self.manual['T_world_base']
        if T is None:
            return 'no manual base saved'
        self.manual = {'T_world_base': np.asarray(T, float).tolist(), 'active': bool(active),
                       'stamp': time.strftime('%Y-%m-%dT%H:%M:%S')}
        self.manual_file.parent.mkdir(parents=True, exist_ok=True)
        self.manual_file.write_text(yaml.safe_dump(self.manual, sort_keys=False))
        return 'manual base saved' if active else 'using the calibrated base'

    def dh(self):
        if self.result and self.result.get('dh_refined') and self.cfg['robot'].get('use_refined_dh', True):
            return np.array(self.result['dh_refined'])
        return self.robot.dh if self.robot.dh is not None else kin.UR3E_NOMINAL_DH

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
            seq, img, _ = cam.latest()
            if img is not None and seq != last:
                last = seq
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                corners = self.board.detect(gray, fast=True)
                d = {'seq': seq, 'stamp': time.time(), 'corners': corners, 'pnp': None,
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

    def _vision_loop(self, idx):
        """Overlay + JPEG for every new frame, up to stream_hz. Overlays reuse
        the latest detection."""
        cam = self.cams[idx]
        stream_dt = 1.0 / self.cfg.get('stream_hz', 10)
        detect_dt = 1.0 / self.cfg.get('detect_hz', 3)
        last, t_stream = 0, 0.0
        threading.Thread(target=self._detect_loop, args=(idx,), daemon=True).start()
        while True:
            seq, img, _ = cam.latest()
            now = time.time()
            if img is None or seq == last or now < t_stream:
                time.sleep(0.003)
                continue
            last, t_stream = seq, max(t_stream + stream_dt, now - stream_dt)   # average stream_hz
            d = self.det[idx]
            if now - d.get('stamp', 0) > 2.5 * detect_dt + 0.5:   # drop corners that are out of date
                d = {**d, 'corners': None, 'pnp': None}
            out = self._draw(idx, img.copy(), d) if self.overlays else img
            scale = self.cfg.get('stream_width', 960) / out.shape[1]
            if scale < 1:
                out = cv2.resize(out, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            ok, jpg = cv2.imencode('.jpg', out, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok:
                self.frames[idx] = (seq, bytes([1, idx]) + jpg.tobytes())
            self.stream_ms[idx] = 0.9 * self.stream_ms[idx] + 0.1 * (time.time() - now) * 1000

    def _cam_pose_in_base(self, idx, guess=False):
        """T_base_cam for the current robot pose, if calibrated (or, with guess,
        from the configured gripper-cam mount before calibration)."""
        T = self.transforms()
        if idx == 0:
            return T.get('T_base_fcam')
        q, _ = self.robot.joints()
        if q is None:
            return None
        X = T.get('T_flange_gcam')
        if X is None and guess:
            X = kin.vec_to_T(np.array(self.cfg['gripper_camera'].get('mount_guess', [0, 0, 0, 0, 0, 0.05]), float))
        return None if X is None else kin.fk(q, self.dh()) @ X

    def _draw(self, idx, img, d):
        cam = self.cams[idx]
        if d['corners'] is not None:
            cv2.drawChessboardCorners(img, (self.board.cols, self.board.rows),
                                      d['corners'].astype(np.float32).reshape(-1, 1, 2), True)
            if d['pnp'] is not None:
                T, _ = d['pnp']
                rv, _ = cv2.Rodrigues(T[:3, :3])
                cv2.drawFrameAxes(img, cam.K, cam.dist, rv, T[:3, 3], self.board.square * 3, 3)
        T_base_cam = self._cam_pose_in_base(idx)
        q, _ = self.robot.joints()
        if T_base_cam is None or cam.K is None:
            return img
        Ti = kin.inv(T_base_cam)
        # robot skeleton from calibrated FK (projected into this camera)
        if q is not None:
            pts = self._skeleton(q)
            P = (Ti[:3, :3] @ pts.T).T + Ti[:3, 3]
            if np.all(P[:, 2] > 0.05):
                uv = cal._project(P, cam.K, cam.dist).astype(int)
                for a, b in zip(uv[:-1], uv[1:]):
                    cv2.line(img, tuple(a), tuple(b), (218, 245, 254), 3, cv2.LINE_AA)
                for p in uv:
                    cv2.circle(img, tuple(p), 6, (60, 72, 210), -1, cv2.LINE_AA)
        # robot surface model on the fixed cam: should cover the real arm
        if idx == 0 and q is not None:
            key = (tuple(np.round(q, 3)), Ti.tobytes(), img.shape)
            if self._ovl_cache[0] != key:          # re-project only when the robot or calibration moves
                P = self._model_points(q)
                P = (Ti[:3, :3] @ P.T).T + Ti[:3, 3]
                P = P[P[:, 2] > 0.05]
                uv = cal._project(P, cam.K, cam.dist).astype(int) if len(P) else np.zeros((0, 2), int)
                h, w = img.shape[:2]
                ok = (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
                self._ovl_cache = (key, uv[ok])
            uv = self._ovl_cache[1]
            img[uv[:, 1], uv[:, 0]] = (93, 161, 201)
        # predicted board corners from the calibration (cyan): should sit on the real ones
        T = self.transforms()
        if 'T_base_board' in T:
            Y = T['T_base_board']
            P = (Ti[:3, :3] @ ((Y[:3, :3] @ self.board.obj.T).T + Y[:3, 3]).T).T + Ti[:3, 3]
            if np.all(P[:, 2] > 0.05):
                for p in cal._project(P, cam.K, cam.dist).astype(int):
                    cv2.circle(img, tuple(p), 4, (255, 220, 0), 1, cv2.LINE_AA)
        return img

    def _model_points(self, q, n=3000):
        key = tuple(np.round(q, 3))
        if self._model_cache[0] != key:
            P, _ = self.surface.points(q)
            self._model_cache = (key, P[:: max(1, len(P) // n)])
        return self._model_cache[1]

    def align_base(self):
        """Fixed camera + robot base from the D435 view of the board and the arm."""
        cam = self.cams[0]
        if cam.K is None or cam.error:
            raise RuntimeError(f'fixed camera not ready ({cam.error})')
        q1, _ = self.robot.joints()
        if q1 is None:
            raise RuntimeError('no joint data from the robot')
        depths, last, t0 = [], 0, time.time()
        while len(depths) < 10 and time.time() - t0 < 4:
            _, color, depth = cam.latest()
            seq = cam.depth_seq
            if depth is not None and seq != last:
                last = seq
                depths.append(np.where(depth > 0, depth, np.nan))
            time.sleep(0.02)
        q2, _ = self.robot.joints()
        if np.max(np.abs(q2 - q1)) > 2e-4:
            raise RuntimeError('robot moved during the capture; hold still and retry')
        with np.errstate(all='ignore'):
            depth = np.nan_to_num(np.nanmedian(np.stack(depths), axis=0))   # 10-frame median: less depth noise
        log = []
        out = base_align.align(color, depth, cam.K, cam.dist, self.board, q2, self.surface, log.append)
        np.savez_compressed(self.store.dir / f'base_align_{time.strftime("%H%M%S")}.npz',
                            color=color, depth=depth, q=q2, K=cam.K)
        r = dict(self.result or {'fits': {}, 'n_samples': 0, 'n_fixed': 0})
        for k in ('T_base_fcam', 'T_base_board', 'T_world_base'):
            r[k] = None if out[k] is None else out[k].tolist()   # no board in view: drop the stale board pose
        r['fixed_intrinsics'] = {'K': cam.K.tolist(), 'dist': cam.dist.tolist(), 'size': cam.size}
        r['base_align'] = {**out['stats'], 'stamp': time.strftime('%Y-%m-%dT%H:%M:%S'), 'q': q2.tolist()}
        r['stamp'] = r['base_align']['stamp']
        r['board'] = self.board.describe()
        r['log'] = log
        self.result_file.parent.mkdir(parents=True, exist_ok=True)
        self.result_file.write_text(yaml.safe_dump(r, sort_keys=False))
        self.result = r
        if self.manual and self.manual.get('active'):
            self.set_manual_base(None, active=False)
        st = out['stats']
        return (f'base aligned: {st["inlier_frac"] * 100:.0f}% model points within 1 cm, '
                f'rms {base_align.fmt_mm(st["inlier_rms_mm"])} mm, tilt {st["tilt_deg"]:.1f}° — saved')

    def _skeleton(self, q):
        """Joint chain for drawing: nominal DH origins (they lie on the arm, unlike
        the calibrated table's), shifted so the flange matches the calibrated FK."""
        nom = kin.fk_frames(q, kin.UR3E_NOMINAL_DH)
        off = kin.fk(q, self.dh())[:3, 3] - nom[6][:3, 3]
        return np.array([F[:3, 3] for F in nom]) + off * np.linspace(0, 1, 7)[:, None]

    def _cloud_loop(self, idx):
        cam = self.cams[idx]
        last = 0
        while True:
            time.sleep(1.0 / self.cfg.get('cloud_hz', 2))
            seq, color, depth = cam.latest()
            if depth is None or seq == last or cam.K is None:
                continue
            last = seq
            xyz, rgb = depth_to_cloud(color, depth, cam.K, self.cfg.get('cloud_stride', 4))
            xyz, rgb = remove_outliers(xyz, rgb, self.cfg.get('cloud_outlier_k', 8), self.cfg.get('cloud_outlier_std', 2.0))
            if idx == 0 and 'T_base_fcam' not in self.transforms():
                self.fcam_table = table_pose(xyz, self.fcam_table)
            self.cloud_msgs[idx] = (seq, struct.pack('<BBxxI', 2, idx, len(xyz)) + xyz.tobytes() + rgb.tobytes())

    # ── actions ───────────────────────────────────────────────────────────
    def capture(self):
        """Store a sample once the arm has stopped (joints steady for 0.3 s)."""
        q1, _ = self.robot.joints()
        if q1 is None:
            raise RuntimeError('no joint data from the robot')
        time.sleep(0.3)
        q2, _ = self.robot.joints()
        if np.max(np.abs(q2 - q1)) > 2e-4:
            raise RuntimeError('robot is moving; hold still and capture again')
        imgs = {'fixed': self.cams[0].latest()[1], 'gripper': self.cams[1].latest()[1]}
        # accurate full-resolution detection, once, on the frames taken after the robot stopped
        dets = {k: None if im is None else self.board.detect(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY))
                for k, im in imgs.items()}
        s = self.store.add(q2, imgs, dets)
        seen = [k for k in ('gripper', 'fixed') if s[f'{k}_corners']]
        return f'sample {s["id"]} stored (board seen by: {", ".join(seen) or "nobody"})'

    def solve(self, opts):
        g, f = self.cams[1], self.cams[0]
        if g.K is None:
            raise RuntimeError('gripper camera not ready')
        dh = self.robot.dh if self.robot.dh is not None else kin.UR3E_NOMINAL_DH
        res = cal.solve(self.store.samples, self.board,
                        {'K': g.K, 'dist': g.dist, 'size': g.size},
                        {'K': f.K, 'dist': f.dist, 'size': f.size} if f.K is not None else None,
                        dh, opts)
        res['session'] = str(self.store.dir.relative_to(ROOT))
        self.result_file.parent.mkdir(parents=True, exist_ok=True)
        self.result_file.write_text(yaml.safe_dump(res, sort_keys=False))
        self.result = res
        if self.manual and self.manual.get('active'):
            self.set_manual_base(None, active=False)
        if res.get('gripper_intrinsics'):
            g.load_intrinsics(res['gripper_intrinsics'])
        return res

    # ── state for the browser ─────────────────────────────────────────────
    def state(self):
        r = self.robot
        q, q_stamp = r.joints()
        T = self.transforms()
        dh = self.dh()
        robot = {'connected': r.error is None and q is not None, 'error': r.error,
                 'tier': r.status.get('tier'), 'mode': r.state.get('robotmode'),
                 'control': r.state.get('controlmode'), 'safety': r.state.get('safetymode'),
                 'rate': round(r.rate, 1), 'age': round(time.time() - q_stamp, 2) if q is not None else None}
        if q is not None:
            frames = kin.fk_frames(q, dh)
            robot['q'] = q.tolist()
            robot['flange'] = frames[-1].tolist()
            if r.pose is not None:
                ctrl = r.pose['T']
                robot['tcp'] = r.pose['tcp']
                robot['tcp_pose'] = ctrl.tolist()
                pq = r.pose['q']
                if r.dh is not None:
                    robot['fk_err_ctrl'] = kin.pose_error(kin.fk(pq, r.dh), ctrl)
                robot['fk_err_nominal'] = kin.pose_error(kin.fk(pq, kin.UR3E_NOMINAL_DH), ctrl)
                if self.result and self.result.get('dh_refined'):
                    robot['fk_err_refined'] = kin.pose_error(kin.fk(pq, np.array(self.result['dh_refined'])), ctrl)
        cams = []
        for i, c in enumerate(self.cams):
            d = self.det[i]
            info = c.info()
            info['board'] = {'seen': d.get('corners') is not None,
                             'rms': None if not d.get('pnp') else round(d['pnp'][1], 3),
                             'dist': None if not d.get('pnp') else float(np.linalg.norm(d['pnp'][0][:3, 3])),
                             'partial': d.get('partial')}
            Tc = self._cam_pose_in_base(i, guess=True)
            info['T_base_cam'] = None if Tc is None else Tc.tolist()
            info['pose_calibrated'] = self._cam_pose_in_base(i) is not None
            if i == 0 and self.fcam_table is not None:
                info['T_world_cam_table'] = self.fcam_table.tolist()
            cams.append(info)
        samples = []
        for s in self.store.samples:
            Tf = kin.fk(np.array(s['q']), dh)
            samples.append({'id': s['id'], 'gripper': bool(s.get('gripper_corners')),
                            'fixed': bool(s.get('fixed_corners')), 'flange': Tf.tolist()})
        per = {p['id']: p['rms_px'] for p in (self.result or {}).get('per_sample', [])}
        for s in samples:
            s['rms'] = per.get(s['id'])
        return {'type': 'state', 't': time.time(), 'robot': robot, 'cams': cams,
                'board': self.board.describe(), 'base_source': self.base_source(),
                'calib': {k: v.tolist() for k, v in T.items()},
                'result': None if not self.result else {k: self.result.get(k) for k in (
                    'stamp', 'n_samples', 'n_fixed', 'fits', 'dh_used', 'fk_refined_vs_controller',
                    'fk_nominal_vs_controller_mm', 'gripper_intrinsics', 'dh_delta', 'log', 'session', 'base_align')},
                'samples': samples, 'session': self.store.dir.name,
                'busy': self.busy, 'message': self.message, 'overlays': self.overlays,
                'qa': self.qa.state()}


def _gripper_cam_class():
    """Orbbec SDK (color + depth) when installed, else UVC color only."""
    try:
        import pyorbbecsdk  # noqa: F401
        return OrbbecCamera
    except ImportError:
        return UVCCamera


def _no_nan(o):
    """NaN/inf -> None: the browser's JSON.parse rejects them and drops the whole state."""
    if isinstance(o, float):
        return o if np.isfinite(o) else None
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
        out = await asyncio.get_running_loop().run_in_executor(None, fn, *args)
        app.message = out if isinstance(out, str) else (
            f'solved: gripper {out["fits"][out["dh_used"]]["gripper_rms_px"]:.2f}px'
            + (f', fixed {out["fits"][out["dh_used"]]["fixed_rms_px"]:.2f}px'
               if out['fits'][out['dh_used']]['fixed_rms_px'] is not None else '')
            + ' — saved')
    except Exception as e:
        app.message = f'{name} failed: {e}'
    finally:
        app.busy = None


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
            if t == 'capture':
                asyncio.create_task(_run_action(app, 'capture', app.capture))
            elif t == 'align_base':
                asyncio.create_task(_run_action(app, 'align base', app.align_base))
            elif t == 'solve':
                asyncio.create_task(_run_action(app, 'solve', app.solve, req.get('opts') or {}))
            elif t == 'delete_sample':
                app.store.remove(int(req['id']))
                app.message = f'sample {req["id"]} removed'
            elif t == 'new_session':
                app.store.new_session()
                app.message = f'new session {app.store.dir.name}'
            elif t == 'set_manual_base':
                app.message = app.set_manual_base(req.get('T_world_base'), req.get('active', True))
            elif t == 'set_overlays':
                app.overlays = bool(req.get('on'))
            elif t == 'qa_test':
                asyncio.create_task(_qa_test(app.qa, int(req.get('port', 0)), req.get('phase')))
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


async def index(_):
    return web.FileResponse(WEB / 'index.html')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default=str(ROOT / 'configs' / 'calib_dashboard.yaml'))
    ap.add_argument('--port', type=int)
    a = ap.parse_args()
    cfg = yaml.safe_load(Path(a.config).read_text())
    w = web.Application()
    w['app'] = App(cfg)
    w['dt'] = 1.0 / cfg.get('state_hz', 10)
    w.router.add_get('/', index)
    w.router.add_get('/ws', ws_handler)
    w.router.add_get('/api/qa', qa_get)
    w.router.add_post('/api/qa/vlm', qa_vlm)
    w.router.add_post('/api/qa/{port}/{phase}', qa_post)
    w.router.add_static('/', WEB)
    port = a.port or cfg.get('port', 8090)
    print(f'calibration dashboard: http://localhost:{port}')
    web.run_app(w, port=port, print=None)


if __name__ == '__main__':
    main()
