"""Simulated UR3e + Hand-E: joint state, FK/IK on the controller's DH table, the motion the
URScript built-ins command (movej/movel/speedl/stopl/force_mode), a F/T sensor and contact
from the world model, all stepped at one control period (DT) in simulated time. `rate` paces
simulated time against the wall clock (1 = real time, 0 = as fast as possible).

Poses are URScript poses [x, y, z, rx, ry, rz] in the base frame; the TCP set by set_tcp
persists after a program ends, as on the real controller.
"""
import threading
import time
from collections import deque

import numpy as np
from scipy.spatial.transform import Rotation as R

from dashboard import kinematics as kin

DT = 0.008                  # control period [s] (get_steptime)
ADMITTANCE = 0.002          # force mode: m/s per N of force error
TOOL_DEPTH = 0.11           # Hand-E coupler + body below the flange: must stay above the plate


class ProtectiveStop(Exception):
    pass


class ProgramStopped(Exception):
    """A stop request: unwinds the running program."""


def vec_to_T(v):
    T = np.eye(4)
    T[:3, :3] = R.from_rotvec(v[3:6]).as_matrix()
    T[:3, 3] = v[:3]
    return T


def T_to_vec(T):
    return list(T[:3, 3]) + list(R.from_matrix(T[:3, :3]).as_rotvec())


def rot_err(Ra, Rb):
    """Rotation vector taking Rb to Ra (base frame)."""
    return R.from_matrix(Ra @ Rb.T).as_rotvec()


def profile(D, v, a):
    """(duration, s(t) in [0, 1]) for a move of length D."""
    D, v, a = abs(D), max(abs(v), 1e-6), max(abs(a), 1e-6)
    if D < 1e-12:
        return 0.0, lambda t: 1.0
    ta = v / a
    if a * ta * ta >= D:
        ta = np.sqrt(D / a)
        T = 2 * ta
        return T, lambda t: (min(1.0, 0.5 * a * t * t / D) if t < ta else min(1.0, 1.0 - 0.5 * a * max(T - t, 0) ** 2 / D))
    T = D / v + ta

    def s(t):
        if t <= 0:
            return 0.0
        if t < ta:
            return 0.5 * a * t * t / D
        if t < T - ta:
            return (0.5 * a * ta * ta + v * (t - ta)) / D
        if t < T:
            return 1.0 - 0.5 * a * (T - t) ** 2 / D
        return 1.0
    return T, s


class Pacer:
    """Keeps simulated time in step with the wall clock (scaled by rate)."""

    def __init__(self, rate=1.0):
        self.rate = rate
        self.anchor = None

    def reset(self):
        self.anchor = None

    def wait(self, t_sim):
        if self.rate <= 0:
            return
        now = time.monotonic()
        if self.anchor is None:
            self.anchor = (now, t_sim)
            return
        target = self.anchor[0] + (t_sim - self.anchor[1]) / self.rate
        if target > now:
            time.sleep(target - now)
        elif now - target > 0.25:              # fell behind (CPU, pause): re-anchor
            self.anchor = (now, t_sim)


class Gripper:
    """Robotiq Hand-E, as its socket interface (port 63352) reports it: 0 = open (50 mm), 255 = closed."""

    STROKE = 0.050

    def __init__(self):
        self.vars = {'ACT': 0, 'GTO': 0, 'POS': 0, 'SPE': 255, 'FOR': 255, 'STA': 0, 'OBJ': 0, 'FLT': 0, 'PRE': 0}
        self.cur = 0.0

    def set(self, key, val):
        self.vars[key] = int(val)
        if key == 'ACT':
            self.vars['STA'] = 3 if val else 0
        if key == 'POS':
            self.vars['PRE'] = int(val)

    def get(self, key):
        if key == 'POS':
            return int(round(self.cur))
        return self.vars.get(key, 0)

    def step(self, dt):
        if not (self.vars['ACT'] and self.vars['GTO']):
            return
        target = self.vars['PRE']
        speed = (20 + 130 * self.vars['SPE'] / 255) / 1000 / self.STROKE * 255     # counts/s
        d = target - self.cur
        self.cur += np.clip(d, -speed * dt, speed * dt)
        self.vars['OBJ'] = 3 if abs(target - self.cur) < 0.5 else 0

    @property
    def width(self):
        return self.STROKE * (1 - self.cur / 255)


class SimRobot:
    def __init__(self, world, dh, rate=1.0):
        self.world = world
        self.dh = np.asarray(dh, float)
        self.lock = threading.RLock()
        self.pacer = Pacer(rate)
        self.check = lambda: None          # set by the controller: raises ProgramStopped / waits on pause
        self.on_event = lambda level, msg: None
        self.rng = np.random.default_rng(0)
        self.safety = 'NORMAL'
        self.tcp = [0.0] * 6
        self.T_tcp = np.eye(4)
        self.reset()

    # ── state ──────────────────────────────────────────────────────────
    def reset(self, q=None):
        with self.lock:
            self.q = np.array(q if q is not None else self.world.home_q, float)
            self.q_prev = self.q.copy()
            self.t = 0.0
            self.twist = np.zeros(6)          # speedl command, base frame
            self.speed_held = False           # a speed command is active this step
            self.decel = 1.0
            self.fm = None                    # force mode parameters
            self.ft_zero = np.zeros(6)
            self.F = np.zeros(6)
            self.info = None
            self.p_prev = None
            self.payload = {'mass': 0.0, 'cog': [0.0, 0.0, 0.0]}
            self.registers = {'float': {}, 'int': {}, 'bool': {}}
            self.digital = {}
            self.gripper = Gripper()
            self.safety = 'NORMAL'
            self.stop_reason = None
            self._contact()

    def flange(self, q=None):
        return kin.fk(self.q if q is None else q, self.dh)

    def tcp_T(self, q=None):
        return self.flange(q) @ self.T_tcp

    def tcp_pose(self, q=None):
        return T_to_vec(self.tcp_T(q))

    def set_tcp(self, pose):
        with self.lock:
            self.tcp = [float(v) for v in pose]
            self.T_tcp = vec_to_T(self.tcp)
            self.p_prev = None

    def force_reading(self):
        """get_tcp_force(): contact wrench on the tool (base frame) + noise - the zero offset."""
        noise = np.r_[self.rng.normal(0, self.world.noise, 3), np.zeros(3)]
        return self.F + noise - self.ft_zero

    def zero_ft(self):
        self.ft_zero = self.F + np.r_[self.rng.normal(0, self.world.noise, 3), np.zeros(3)]

    # ── kinematics ─────────────────────────────────────────────────────
    def jacobian(self, q):
        frames = kin.fk_frames(q, self.dh)
        pe = (frames[6] @ self.T_tcp)[:3, 3]
        J = np.zeros((6, 6))
        for i in range(6):
            z, o = frames[i][:3, 2], frames[i][:3, 3]
            J[:3, i] = np.cross(z, pe - o)
            J[3:, i] = z
        return J

    def ik_step(self, q, T_des, iters=2, lam=0.01):
        for _ in range(iters):
            T = self.tcp_T(q)
            e = np.r_[T_des[:3, 3] - T[:3, 3], rot_err(T_des[:3, :3], T[:3, :3])]
            if np.linalg.norm(e) < 1e-9:
                break
            J = self.jacobian(q)
            q = q + J.T @ np.linalg.solve(J @ J.T + lam * lam * np.eye(6), e)
        return q

    def ik(self, pose, q_near=None, tcp=None):
        """Joint positions reaching the TCP pose, from q_near (default: current). None if unreachable."""
        T_tcp = self.T_tcp
        if tcp is not None:
            self.T_tcp = vec_to_T(tcp)
        try:
            T_des = vec_to_T(pose)
            q = np.array(self.q if q_near is None else q_near, float)
            for _ in range(300):
                T = self.tcp_T(q)
                e = np.r_[T_des[:3, 3] - T[:3, 3], rot_err(T_des[:3, :3], T[:3, :3])]
                if np.linalg.norm(e[:3]) < 1e-7 and np.linalg.norm(e[3:]) < 1e-6:
                    return q
                J = self.jacobian(q)
                lam = 0.001 if np.linalg.norm(e) < 0.01 else 0.02
                step = J.T @ np.linalg.solve(J @ J.T + lam * lam * np.eye(6), e)
                n = np.linalg.norm(step)
                q = q + (step if n < 0.3 else step * 0.3 / n)
            return None
        finally:
            self.T_tcp = T_tcp

    # ── one control period ─────────────────────────────────────────────
    def step(self, T_des=None, q_des=None):
        self.check()
        with self.lock:
            if self.safety != 'NORMAL':
                raise ProtectiveStop(self.stop_reason or self.safety)
            if q_des is not None:
                self.q = np.array(q_des, float)
            else:
                if not self.speed_held and np.any(self.twist):        # speed command ended: ramp down
                    n = np.linalg.norm(self.twist[:3])
                    self.twist *= max(0.0, n - self.decel * DT) / n if n > 1e-12 else 0.0
                    if np.linalg.norm(self.twist) < 1e-6:
                        self.twist[:] = 0
                T_act = self.tcp_T()
                if T_des is None:
                    p_des = T_act[:3, 3] + self.twist[:3] * DT
                    R_des = R.from_rotvec(self.twist[3:] * DT).as_matrix() @ T_act[:3, :3]
                else:
                    p_des, R_des = T_des[:3, 3], T_des[:3, :3]
                if self.fm is not None:
                    p_des = self._force_mode(T_act[:3, 3], p_des)
                    R_des = T_act[:3, :3] if T_des is None and not np.any(self.twist[3:]) else R_des
                T = np.eye(4)
                T[:3, :3], T[:3, 3] = R_des, p_des
                self.q = self.ik_step(self.q, T)
            self.speed_held = False
            self.t += DT
            self.gripper.step(DT)
            self._contact()
            self._limits()
        self.pacer.wait(self.t)

    def _force_mode(self, p_act, p_des):
        """Compliant axes move at ADMITTANCE x (wrench + contact force), capped at the limits; a
        pocket's holding friction is a deadband on the force along the pocket axis (static)."""
        fm = self.fm
        Rf = fm['Rf']
        d = Rf.T @ (p_des - p_act)
        info = self.info or {}
        Fc = Rf.T @ (self.F[:3] - np.array(info.get('hold_F', [0, 0, 0])))
        hold = info.get('hold', 0.0) * np.abs(Rf.T @ np.array(info.get('hold_axis', [0, 0, 0])))
        for i in range(3):
            if fm['sel'][i]:
                f = fm['wrench'][i] + Fc[i]
                f = 0.0 if abs(f) <= hold[i] else f - np.sign(f) * hold[i]
                v = ADMITTANCE * fm['damping_gain'] * f
                d[i] = np.clip(v, -fm['limits'][i], fm['limits'][i]) * DT
        return p_act + Rf @ d

    def _contact(self):
        T = self.tcp_T()
        p = T[:3, 3]
        vel = np.zeros(3) if self.p_prev is None else (p - self.p_prev) / DT
        self.p_prev = p.copy()
        F, self.info = self.world.contact(p, vel)
        self.F = np.r_[F, 0.0, 0.0, 0.0]
        self.vel = vel

    def _limits(self):
        fn = float(np.linalg.norm(self.F[:3]))
        reason = None
        if fn > self.world.stop_N:
            reason = f'C153: protective stop, contact force {fn:.0f} N on the tool ({(self.info or {}).get("object", "?")})'
        F_ = self.flange()
        low = (F_ @ np.array([0, 0, TOOL_DEPTH, 1.0]))[2]
        if low < self.world.floor_z:
            reason = f'C153: protective stop, the gripper body hit the plate (z {low * 1000:.1f} mm)'
        if np.any(np.abs(self.q) > 2 * np.pi + 1e-6):
            reason = f'C2A: joint limit exceeded ({np.round(self.q, 3).tolist()})'
        if reason:
            self.safety, self.stop_reason = 'PROTECTIVE_STOP', reason
            self.twist[:] = 0
            self.fm = None
            self.on_event('ERROR', reason)
            raise ProtectiveStop(reason)

    # ── motion primitives (blocking, in simulated time) ────────────────
    def wait(self, duration):
        for _ in range(max(0, int(round(duration / DT)))):
            self.step()

    def movej(self, q1, a=1.4, v=1.05, t=0.0):
        self.twist[:] = 0
        q0 = self.q.copy()
        q1 = np.array(q1, float)
        D = float(np.max(np.abs(q1 - q0)))
        T, s = profile(D, v, a)
        if t and t > T:
            T, s = t, (lambda tt, T=t: min(1.0, tt / T))
        n = int(np.ceil(T / DT))
        for k in range(1, n + 1):
            self.step(q_des=q0 + s(k * DT) * (q1 - q0))
        if n == 0:
            self.step(q_des=q1)

    def movel(self, pose, a=1.2, v=0.25, t=0.0):
        self.twist[:] = 0
        T0, T1 = self.tcp_T(), vec_to_T(pose)
        dp = T1[:3, 3] - T0[:3, 3]
        rv = rot_err(T1[:3, :3], T0[:3, :3])
        Tl, sl = profile(np.linalg.norm(dp), v, a)
        Tr, sr = profile(np.linalg.norm(rv), 1.05, 1.4)
        T, s = (Tl, sl) if Tl >= Tr else (Tr, sr)
        if t and t > T:
            T, s = t, (lambda tt, T=t: min(1.0, tt / T))
        R0 = T0[:3, :3]
        n = int(np.ceil(T / DT))
        for k in range(1, n + 1):
            f = s(k * DT)
            Td = np.eye(4)
            Td[:3, 3] = T0[:3, 3] + f * dp
            Td[:3, :3] = R.from_rotvec(f * rv).as_matrix() @ R0
            self.step(T_des=Td)
        for _ in range(3):                    # settle on the target
            self.step(T_des=T1)

    def speedl(self, xd, a, t):
        xd = np.array(list(xd) + [0.0] * (6 - len(xd)), float)
        self.decel = max(a, 1e-3)
        for _ in range(max(1, int(round(t / DT)))):
            dv = xd - self.twist
            n = np.linalg.norm(dv[:3])
            lim = a * DT
            if n > lim:
                dv[:3] *= lim / n
            self.twist = self.twist + dv
            self.speed_held = True
            self.step()

    def stopl(self, a):
        self.decel = max(a, 1e-3)
        while np.any(self.twist):
            self.step()

    def force_mode(self, frame, sel, wrench, ftype, limits):
        self.fm = {'Rf': vec_to_T(frame)[:3, :3], 'sel': [int(bool(s)) for s in sel],
                   'wrench': np.array(wrench, float), 'limits': np.array(limits, float),
                   'type': int(ftype), 'damping_gain': 1.0}

    def end_force_mode(self):
        self.fm = None

    def program_ended(self):
        with self.lock:
            self.twist[:] = 0
            self.fm = None

    # ── state for the viewer ───────────────────────────────────────────
    def snapshot(self):
        with self.lock:
            fl = self.flange()
            info = dict(self.info) if self.info else None
            if info:
                info = {k: (round(v * 1000, 3) if k in ('pen', 'depth', 'u', 'v') else v) for k, v in info.items()}
            return {'q': self.q.tolist(), 'flange': fl.tolist(), 'tcp': self.tcp, 'tcp_pose': T_to_vec(fl @ self.T_tcp),
                    'force': self.F.tolist(), 'force_reading': (self.F - self.ft_zero).tolist(),
                    'force_mode': None if self.fm is None else {
                        'sel': self.fm['sel'], 'wrench': self.fm['wrench'].tolist(), 'limits': self.fm['limits'].tolist()},
                    'speed': float(np.linalg.norm(getattr(self, 'vel', np.zeros(3)))),
                    'contact': info, 'gripper': {'width': self.gripper.width, 'pos': self.gripper.get('POS'),
                                                 'active': bool(self.gripper.vars['ACT'])},
                    'safety': self.safety, 'stop_reason': self.stop_reason, 't': self.t,
                    'registers': {k: dict(v) for k, v in self.registers.items()}}


class Trail:
    """Recent TCP positions for the viewer."""

    def __init__(self, n=4000):
        self.pts = deque(maxlen=n)
        self.n = 0                     # points ever added: the viewer asks for those after its last

    def add(self, p):
        if not self.pts or np.linalg.norm(np.subtract(p, self.pts[-1][1])) > 2e-4:
            self.n += 1
            self.pts.append((self.n, [round(float(x), 5) for x in p]))

    def since(self, k):
        return [p for i, p in list(self.pts) if i > k]

    def clear(self):
        self.pts.clear()
