"""Camera readers: RealSense D435 (fixed scene camera, color + aligned depth)
and the Orbbec gripper camera (Orbbec SDK with depth, else plain UVC color). Each runs in its own
thread and keeps only the latest frame."""

import math
import threading
import time
from pathlib import Path

import cv2
import numpy as np


class Camera:
    name = ''

    def __init__(self):
        self.lock = threading.Lock()
        self.seq = 0
        self.color = None
        self.depth = None          # metres, aligned to color (or None)
        self.depth_seq = 0
        self.K = None
        self.dist = np.zeros(5)
        self.size = None
        self.fps = 0.0
        self.error = None
        self.intrinsics_source = 'none'
        self._stop = False

    def latest(self):
        with self.lock:
            return self.seq, self.color, self.depth

    def _put(self, color, depth=None):
        """New color frame; depth=None keeps the last depth (it updates slower)."""
        with self.lock:
            self.seq += 1
            self.color = color
            if depth is not None:
                self.depth = depth
                self.depth_seq += 1

    def start(self):
        threading.Thread(target=self._loop, daemon=True, name=self.name).start()
        return self

    def _loop(self):
        while not self._stop:
            try:
                self._run()
            except Exception as e:     # unplugged, busy, ... retry
                self.error = str(e)
                time.sleep(2.0)

    def info(self):
        return {'name': self.name, 'ok': self.error is None and self.color is not None,
                'error': self.error, 'size': self.size, 'fps': round(self.fps, 1),
                'K': None if self.K is None else np.asarray(self.K).tolist(),
                'intrinsics': self.intrinsics_source}


def usb_present(vendor):
    """Cheap sysfs check, so we never call into librealsense without a device:
    its device queries hold the GIL for seconds and stall every other thread."""
    for d in Path('/sys/bus/usb/devices').iterdir():
        try:
            if (d / 'idVendor').read_text().strip() == vendor:
                return True
        except OSError:
            continue
    return False


class RealSenseCamera(Camera):
    def __init__(self, cfg):
        super().__init__()
        self.cfg = cfg
        self.name = cfg.get('label', 'Fixed D435')

    def _run(self):
        if not usb_present(self.cfg.get('usb_vendor', '8086')):
            raise RuntimeError('D435 not on USB (re-plug it)')
        import pyrealsense2 as rs
        last_err = None
        for w, h, fps in [tuple(m) for m in self.cfg.get('modes', [[1280, 720, 15], [640, 480, 15]])]:
            pipe, conf = rs.pipeline(), rs.config()
            if self.cfg.get('serial'):
                conf.enable_device(str(self.cfg['serial']))
            conf.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
            conf.enable_stream(rs.stream.depth, min(w, 848) if w > 640 else w, 480 if w > 640 else h, rs.format.z16, fps)
            try:
                prof = pipe.start(conf)
                break
            except RuntimeError as e:
                last_err = e
        else:
            raise RuntimeError(f'RealSense: no stream mode worked ({last_err})')
        try:
            i = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            self.K = np.array([[i.fx, 0, i.ppx], [0, i.fy, i.ppy], [0, 0, 1]])
            self.dist = np.array(i.coeffs[:5], float)
            self.size = [i.width, i.height]
            self.intrinsics_source = 'factory'
            scale = prof.get_device().first_depth_sensor().get_depth_scale()
            align = rs.align(rs.stream.color)
            depth_dt = 1.0 / self.cfg.get('depth_hz', 5)
            self.error = None
            t0, n = time.time(), 0
            t_last = t_depth = time.time()
            while not self._stop:
                # poll + sleep instead of wait_for_frames, which blocks with the GIL held
                fr = pipe.poll_for_frames()
                if not fr:
                    if time.time() - t_last > 5:
                        raise RuntimeError('D435: no frames for 5 s')
                    time.sleep(0.005)
                    continue
                t_last = time.time()
                depth = None
                if t_last - t_depth >= depth_dt:       # aligning depth to color is the costly part
                    t_depth = t_last
                    fr = align.process(fr)
                    d = fr.get_depth_frame()
                    if d:
                        depth = np.asanyarray(d.get_data()).astype(np.float32) * scale
                c = fr.get_color_frame()
                if not c:
                    continue
                self._put(np.asanyarray(c.get_data()).copy(), depth)
                n += 1
                if time.time() - t0 > 2:
                    self.fps, t0, n = n / (time.time() - t0), time.time(), 0
        finally:
            pipe.stop()


def find_uvc(name_match):
    """First V4L2 node whose card name contains name_match and streams YUYV/MJPG."""
    for p in sorted(Path('/sys/class/video4linux').glob('video*'), key=lambda p: int(p.name[5:])):
        if name_match.lower() not in (p / 'name').read_text().lower():
            continue
        cap = cv2.VideoCapture(f'/dev/{p.name}', cv2.CAP_V4L2)
        if not cap.isOpened():
            continue
        fcc = int(cap.get(cv2.CAP_PROP_FOURCC))
        cap.release()
        if ''.join(chr((fcc >> 8 * k) & 255) for k in range(4)) in ('YUYV', 'MJPG'):
            return f'/dev/{p.name}'
    return None


class UVCCamera(Camera):
    def __init__(self, cfg):
        super().__init__()
        self.cfg = cfg
        self.name = cfg.get('label', 'Gripper cam')

    def guess_intrinsics(self):
        """Pinhole guess from the HFOV (plain UVC has no factory intrinsics)."""
        w, h = self.size
        f = w / 2 / math.tan(math.radians(self.cfg.get('hfov_deg', 86)) / 2)
        self.K = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]])
        self.dist = np.zeros(5)
        self.intrinsics_source = 'guess (hfov)'

    def _run(self):
        dev = self.cfg.get('device') or find_uvc(self.cfg.get('match', 'Orbbec'))
        if not dev:
            raise RuntimeError('gripper camera not found')
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        w, h = self.cfg.get('size', [1280, 720])
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        cap.set(cv2.CAP_PROP_FPS, self.cfg.get('fps', 15))
        try:
            ok, img = cap.read()
            if not ok:
                raise RuntimeError(f'{dev}: no frames')
            self.size = [img.shape[1], img.shape[0]]
            if self.K is None:
                self.guess_intrinsics()
            self.error = None
            t0, n = time.time(), 0
            while not self._stop:
                ok, img = cap.read()
                if not ok:
                    raise RuntimeError(f'{dev}: read failed')
                self._put(img)
                n += 1
                if time.time() - t0 > 2:
                    self.fps, t0, n = n / (time.time() - t0), time.time(), 0
        finally:
            cap.release()


class OrbbecCamera(UVCCamera):
    """Gripper Gemini 335Lg through the Orbbec SDK: color + depth aligned to
    color (every 1/depth_hz s), factory intrinsics."""

    def _laser_on(self, ob, dev):
        """Active stereo needs the IR projector: without it plain surfaces (plate, switch)
        give no depth and repetitive texture (pegboard) mismatches. Force it on, log the state."""
        P = ob.OBPropertyID
        for prop, val in (('OB_PROP_LASER_CONTROL_INT', 1), ('OB_PROP_LASER_BOOL', True)):
            pid = getattr(P, prop, None)
            try:
                if pid is not None and dev.is_property_supported(pid, ob.OBPermissionType.PERMISSION_READ_WRITE):
                    (dev.set_int_property if prop.endswith('INT') else dev.set_bool_property)(pid, val)
            except Exception as e:                       # SDK/firmware differences: keep streaming
                print(f'gripper cam: {prop} not set: {e}')
        info = []
        for prop in ('OB_PROP_LASER_CONTROL_INT', 'OB_PROP_LASER_BOOL'):
            try:
                pid = getattr(P, prop)
                info.append(f'{prop}=' + str(dev.get_int_property(pid) if prop.endswith('INT') else dev.get_bool_property(pid)))
            except Exception:
                pass
        try:
            info.append(f'preset={dev.get_current_preset_name()} of {list(dev.get_available_preset_list())}')
        except Exception:
            pass
        self.depth_settings = ', '.join(info)
        print('gripper cam depth settings:', self.depth_settings)

    def _run(self):
        import pyorbbecsdk as ob
        w, h = self.cfg.get('size', [1280, 720])
        pipe, conf = ob.Pipeline(), ob.Config()
        cp = pipe.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR).get_video_stream_profile(
            w, h, ob.OBFormat.RGB, self.cfg.get('fps', 30))
        dp = pipe.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR).get_default_video_stream_profile()
        conf.enable_stream(cp)
        conf.enable_stream(dp)
        conf.set_frame_aggregate_output_mode(ob.OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE)
        pipe.start(conf)
        self._laser_on(ob, pipe.get_device())
        try:
            align = ob.AlignFilter(align_to_stream=ob.OBStreamType.COLOR_STREAM)
            self.size = [w, h]
            p = pipe.get_camera_param()
            i, d = p.rgb_intrinsic, p.rgb_distortion
            self.K = np.array([[i.fx, 0, i.cx], [0, i.fy, i.cy], [0, 0, 1]])
            self.dist = np.array([d.k1, d.k2, d.p1, d.p2, d.k3])
            self.intrinsics_source = 'factory'
            depth_dt = 1.0 / self.cfg.get('depth_hz', 5)
            self.error = None
            t0, n, t_depth, t_last = time.time(), 0, 0.0, time.time()
            while not self._stop:
                fs = pipe.wait_for_frames(200)
                if fs is None:
                    if time.time() - t_last > 5:
                        raise RuntimeError('Gemini: no frames for 5 s')
                    continue
                t_last = time.time()
                depth = None
                if t_last - t_depth >= depth_dt:
                    t_depth = t_last
                    fa = align.process(fs)
                    df = fa and fa.as_frame_set().get_depth_frame()
                    if df is not None:
                        depth = (np.frombuffer(df.get_data(), np.uint16).reshape(df.get_height(), df.get_width())
                                 .astype(np.float32) * (df.get_depth_scale() / 1000.0))
                c = fs.get_color_frame()
                if c is None:
                    continue
                rgb = np.frombuffer(c.get_data(), np.uint8).reshape(c.get_height(), c.get_width(), 3)
                self._put(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), depth)
                n += 1
                if time.time() - t0 > 2:
                    self.fps, t0, n = n / (time.time() - t0), time.time(), 0
        finally:
            pipe.stop()


def depth_to_cloud(color, depth, K, stride=4, max_range=2.0):
    """Subsampled colored point cloud in the camera optical frame."""
    d = depth[::stride, ::stride]
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    m = (d > 0.1) & (d < max_range)
    z = d[m]
    x = (u[m] - K[0, 2]) * z / K[0, 0]
    y = (v[m] - K[1, 2]) * z / K[1, 1]
    rgb = color[::stride, ::stride][m][:, ::-1]
    return np.c_[x, y, z].astype(np.float32), np.ascontiguousarray(rgb, dtype=np.uint8)


def remove_outliers(xyz, rgb, k=8, std=2.0):
    """Statistical outlier removal: drop points whose mean distance to their k
    nearest neighbours (divided by depth, since the spacing grows with it) is
    more than std sigmas above the average. Removes flying pixels at edges."""
    if std <= 0 or len(xyz) <= k:
        return xyz, rgb
    from scipy.spatial import cKDTree
    d, _ = cKDTree(xyz).query(xyz, k + 1, workers=-1)
    score = d[:, 1:].mean(1) / xyz[:, 2]
    keep = score < score.mean() + std * score.std()
    return xyz[keep], rgb[keep]
