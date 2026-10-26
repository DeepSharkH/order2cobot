"""Simulated PolyScope X controller: applications with program trees, the loaded program,
play/pause/stop, robot/safety/control modes and the controller log (psx_logs).

The program store starts with the real robot's application list (names and ids from
simulation/reference/bridge_capture.json) and persists to data/sim/programs.json, so ids
such as rj45_qa = 10 match the robot and an upload survives a simulator restart.
"""
import json
import secrets
import threading
import time
import uuid
from collections import deque
from pathlib import Path

from simulation import urscript
from simulation.robot import ProgramStopped, ProtectiveStop, Trail

ROOT = Path(__file__).resolve().parent.parent
CAPTURE = Path(__file__).with_name('reference') / 'bridge_capture.json'
STORE = ROOT / 'data' / 'sim' / 'programs.json'


def new_guid():
    return str(uuid.uuid4())


def node(type_, params=None, children=None):
    return {'guid': new_guid(), 'type': type_, 'params': params or {}, 'children': children or []}


def app_skeleton(name):
    return node('ur-program', {'name': name}, [
        node('ur-modules'), node('ur-functions', children=[node('ur-function')]), node('ur-before-start'),
        node('ur-configuration'), node('ur-status'), node('ur-code', {'loopForever': False})])


def outline(n, depth=0, out=None):
    out = [] if out is None else out
    params = ', '.join(f'{k}={json.dumps(v)}' for k, v in n['params'].items())
    out.append(f'{"  " * depth}{n["type"]}{f" ({params})" if params else ""} [{n["guid"]}]')
    for c in n['children']:
        outline(c, depth + 1, out)
    return out


def find(n, guid, parent=None):
    if n['guid'] == guid:
        return n, parent
    for c in n['children']:
        r = find(c, guid, n)
        if r[0]:
            return r
    return None, None


class Controller:
    def __init__(self, robot, tier='move', controlmode='REMOTE'):
        self.robot = robot
        self.lock = threading.RLock()
        self.tier = tier
        self.robotmode, self.controlmode, self.operationalmode = 'RUNNING', controlmode, 'AUTOMATIC'
        self.state = 'STOPPED'
        self.loaded = None
        self.apps = {}
        self.log_lines = deque(maxlen=5000)
        self.events = deque(maxlen=2000)
        self.seq = 0
        self.run_name = None
        self.script = ''
        self.script_hash = ''
        self.runtime = None
        self.error = None
        self.stop_req = False
        self.pause = threading.Event()
        self.pause.set()
        self.thread = None
        self.trail = Trail()
        self._last_ms = 0
        robot.check = self._check
        robot.on_event = lambda level, msg: self.log(level, 'SAFETY', msg)
        self._load_store()
        self.log('INFO', 'ROBOTINTERFACE', 'order2cobot simulator: controller booted, robot RUNNING')
        self.log('INFO', 'SAFETY', f'safety mode NORMAL, control mode {controlmode}')

    # ── store ──────────────────────────────────────────────────────────
    def _load_store(self):
        if STORE.exists():
            d = json.loads(STORE.read_text())
            self.apps, self.loaded = d['apps'], d.get('loaded')
            return
        progs = []
        if CAPTURE.exists():
            cap = json.loads(CAPTURE.read_text())
            progs = cap['calls']['program_list']['structuredContent']['data']['programs']
        for p in progs:
            self.apps[str(p['id'])] = {'id': str(p['id']), 'name': p['name'], 'robotType': p.get('robotType', 'UR3'),
                                       'root': app_skeleton(p['name'])}
        self._save()

    def _save(self):
        STORE.parent.mkdir(parents=True, exist_ok=True)
        STORE.write_text(json.dumps({'apps': self.apps, 'loaded': self.loaded}))

    # ── log / events ───────────────────────────────────────────────────
    def log(self, level, source, message):
        with self.lock:
            ms = max(urscript.wall_ms(), self._last_ms + 1)     # strictly increasing, like the readback expects
            self._last_ms = ms
            self.log_lines.append({'ts': str(ms), 'level': level, 'source': source, 'message': message})
        self.event('log', message, level=level)

    def event(self, kind, text, **extra):
        with self.lock:
            self.seq += 1
            self.events.append({'seq': self.seq, 't': time.time(), 'kind': kind, 'text': text, **extra})

    # ── programs ───────────────────────────────────────────────────────
    def app(self, app_id=None, name=None):
        if app_id is not None and str(app_id) in self.apps:
            return self.apps[str(app_id)]
        if name is not None:
            return next((a for a in self.apps.values() if a['name'] == name), None)
        return None

    def create(self, name, robot_type='UR3'):
        with self.lock:
            new_id = str(max([int(k) for k in self.apps] + [-1]) + 1)
            self.apps[new_id] = {'id': new_id, 'name': name, 'robotType': robot_type, 'root': app_skeleton(name)}
            self._save()
            return self.apps[new_id]

    def mutate(self):
        self._save()

    def program_text(self, app):
        """The script the controller runs: the ur-script expressions of ur-code, in order."""
        parts = []
        code = next(c for c in app['root']['children'] if c['type'] == 'ur-code')

        def walk(n):
            for c in n['children']:
                if c['type'] == 'ur-script':
                    parts.append(str(c['params'].get('expression', '')))
                elif c['type'] in ('ur-folder', 'ur-sequence'):
                    walk(c)
                else:
                    self.log('WARNING', 'SIM', f'node type {c["type"]} is not simulated, skipped')
        walk(code)
        return '\n'.join(parts) + '\n', bool(code['params'].get('loopForever'))

    def load(self, app):
        with self.lock:
            if self.state != 'STOPPED':
                raise RuntimeError('program_running: stop the running program first')
            self.loaded = app['id']
            self._save()

    def motion_blocker(self):
        if self.controlmode != 'REMOTE':
            return 'remote_control_required: controlmode=LOCAL (need REMOTE - switch the pendant to Remote Control)'
        if self.robotmode != 'RUNNING':
            return f'robot_not_running: robotmode={self.robotmode} (need RUNNING - power on + release brakes)'
        if self.robot.safety != 'NORMAL':
            return f'safety_not_normal: safetymode={self.robot.safety} (unlock the protective stop first)'
        return None

    def play(self):
        with self.lock:
            if self.loaded is None:
                raise RuntimeError('no_program_loaded: load a program first (program_load)')
            if self.state == 'PLAYING':
                raise RuntimeError('program_running: already playing')
            if self.state == 'PAUSED':
                return self.resume()
            block = self.motion_blocker()
            if block:
                raise PermissionError(block)
            app = self.apps[self.loaded]
            text, loop = self.program_text(app)
            self.script = text
            self.script_hash = f'{hash(text) & 0xffffffff:08x}'
            self.run_name = app['name'] + secrets.token_hex(2)
            self.stop_req, self.error = False, None
            rt = urscript.Runtime(self.robot, self.log, self.event, self._check)
            self.runtime = rt
            try:
                prepared = urscript.prepare(text, rt)
            except urscript.URScriptError as e:
                self._error(e)
                raise RuntimeError(f'program_compile_error: {self.error}') from None
            self.pause.set()
            self.state = 'PLAYING'
            self.thread = threading.Thread(target=self._run, args=(rt, prepared, loop), daemon=True, name='urscript')
            self.thread.start()
        return {'state': 'PLAYING'}

    def stop(self):
        with self.lock:
            if self.state == 'STOPPED':
                raise RuntimeError('program_not_running: 409 nothing to stop')
            self.stop_req = True
            self.pause.set()
        if self.thread:
            self.thread.join(timeout=5)
        return {'state': 'STOPPED'}

    def pause_(self):
        with self.lock:
            if self.state != 'PLAYING':
                raise RuntimeError('program_not_running: nothing to pause')
            self.pause.clear()
            self.state = 'PAUSED'
            self.log('INFO', 'RTMACHINE', 'PROGRAM_XXX_PAUSED ' + json.dumps({'name': self.run_name}, separators=(',', ':')))
        return {'state': 'PAUSED'}

    def resume(self):
        with self.lock:
            if self.state != 'PAUSED':
                raise RuntimeError('program_not_paused')
            self.state = 'PLAYING'
            self.robot.pacer.reset()
            self.pause.set()
        return {'state': 'PLAYING'}

    def _check(self):
        if self.stop_req:
            raise ProgramStopped()
        if not self.pause.is_set():
            self.pause.wait()
            if self.stop_req:
                raise ProgramStopped()
        if self.robot.t - getattr(self, '_trail_t', -1) >= 0.04:
            self._trail_t = self.robot.t
            self.trail.add(self.robot.tcp_T()[:3, 3])

    def _run(self, rt, prepared, loop):
        name = json.dumps({'name': self.run_name}, separators=(',', ':'))
        code, ns, lmap = prepared
        self.robot.pacer.reset()
        self.log('INFO', 'RTMACHINE', f'PROGRAM_XXX_STARTED {name}')
        try:
            while True:
                urscript.run(code, dict(ns), lmap, rt)
                if not loop:
                    break
        except (urscript.Halt, ProgramStopped):
            pass
        except ProtectiveStop as e:
            self.error = str(e)
        except urscript.URScriptError as e:
            self._error(e)
        except Exception as e:  # noqa: BLE001 - a simulator bug must not kill the controller
            self._error(urscript.URScriptError('runtime_error', f'simulator: {type(e).__name__}: {e}', rt.line))
        finally:
            rt.sockets.close_all()
            self.robot.program_ended()
            self.log('INFO', 'RTMACHINE', f'PROGRAM_XXX_STOPPED {name}')
            with self.lock:
                self.state = 'STOPPED'
                self.stop_req = False

    def _error(self, e):
        self.error = f'line {e.line}: {e.kind}: {e.msg}'
        self.log('WARNING', 'RTMACHINE', f'com_ur_controller_runtime_exceptions.{e.kind} ' + json.dumps(
            {'parameter2': e.msg, 'parameter1': str(e.line)}, separators=(',', ':')))

    # ── robot state actions ────────────────────────────────────────────
    def set_robot_state(self, action):
        if action == 'POWER_OFF':
            self.robotmode = 'POWER_OFF'
        elif action == 'POWER_ON':
            self.robotmode = 'IDLE'
        elif action == 'BRAKE_RELEASE':
            self.robotmode = 'RUNNING'
        elif action in ('UNLOCK_PROTECTIVE_STOP', 'RESTART_SAFETY'):
            with self.robot.lock:
                self.robot.safety, self.robot.stop_reason = 'NORMAL', None
            self.log('INFO', 'SAFETY', f'{action.lower()}: safety mode NORMAL')
        return {'robotmode': self.robotmode, 'safetymode': self.robot.safety}

    def robot_state(self):
        block = self.motion_blocker()
        return {'robotmode': self.robotmode, 'safetymode': self.robot.safety, 'controlmode': self.controlmode,
                'operationalmode': self.operationalmode, 'apiVersion': '3.1.6', 'source': 'simulator',
                'remoteControlEnabled': self.controlmode == 'REMOTE', 'motionRunnable': block is None,
                'motionRunnableReason': None if block is None else f'program_run play would be refused: {block}'}

    def snapshot(self):
        app = self.apps.get(self.loaded) if self.loaded else None
        return {'state': self.state, 'loaded': app['name'] if app else None, 'app_id': self.loaded,
                'run_name': self.run_name, 'line': self.runtime.line if self.runtime else 0,
                'hash': self.script_hash, 'error': self.error, 'tier': self.tier,
                'robotmode': self.robotmode, 'controlmode': self.controlmode, 'seq': self.seq}
