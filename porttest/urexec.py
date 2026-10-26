"""
Run URScript steps on the UR3e through the AI Agent Bridge.

One application (rj45_qa) holds a single ur-script node. Each step rewrites that
node, loads the app and plays it, then waits for STOPPED. All motion stays on the
bridge's gated path (move tier + Remote Control), so the operator can pause it.
"""
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from dashboard.ur_bridge import URBridge, BridgeError  # noqa: E402

import yaml  # noqa: E402

from porttest.urlib import LIB  # noqa: E402
from porttest.urscript_lint import lint  # noqa: E402

APP = 'rj45_qa'
ROOT = Path(__file__).resolve().parent.parent
STATE = ROOT / 'data' / 'urexec.json'
CLIENT = 'rj45-skill'      # own name: the dashboard's session (order2cobot-calib) no longer drops ours
GRIP = yaml.safe_load((ROOT / 'configs' / 'grip.yaml').read_text())

# Hand-E (~1 kg) + UR camera mount with the Gemini (480 g) + plug shell; CoG ~6 cm out.
# set_tcp: inside a step, get_actual_tcp_pose() and movel targets are tool (e.g. plug-tip) poses.
PAYLOAD = 'set_target_payload(1.5, [0.0, 0.0, 0.06])\n'


def preamble(tcp):
    return PAYLOAD + f'set_tcp(p{[round(v, 5) for v in tcp]})\n' + LIB


PREAMBLE = preamble(GRIP['tcp_tip'])


def call(b, tool, **args):
    """URBridge.call, but tool args may include 'name', and one reconnect when the
    session was dropped (the dashboard shares this client name)."""
    try:
        res = b._post('tools/call', {'name': tool, 'arguments': args})
        if res.get('isError') and 'unauthorized' in json.dumps(res.get('structuredContent') or res.get('content')):
            raise BridgeError('unauthorized')
    except (BridgeError, Exception) as e:
        if 'unauthorized' not in str(e) and '401' not in str(e) and '404' not in str(e):
            raise
        b.initialize()
        res = b._post('tools/call', {'name': tool, 'arguments': args})
    txt = '\n'.join(c.get('text', '') for c in res.get('content', []) if c.get('type') == 'text')
    out = res.get('structuredContent') or json.loads(txt or 'null')
    if res.get('isError'):
        raise BridgeError(out)
    return out


def _guid(line):
    return line.rsplit('[', 1)[1].rstrip(']')


class URExec:
    """One bridge client and one app per skill: defaults are the rj45-insert ones. `grip` is the
    tool model (its `tcp_key` entry is the step TCP, `collision_points` the clearance points)."""

    def __init__(self, app=APP, client=CLIENT, grip=None, tcp_key='tcp_tip'):
        self.app_name, self.grip = app, grip or GRIP
        self.tcp = self.grip[tcp_key]
        self.preamble = preamble(self.tcp)
        t = time.monotonic()
        self.b = URBridge(client_name=client)
        self.init_timing = {'initialize': round(time.monotonic() - t, 3)}
        self.app = self.guid = None
        t = time.monotonic()
        self._ensure_app()
        self.init_timing['ensure_app'] = round(time.monotonic() - t, 3)

    def _ensure_app(self):
        apps = call(self.b, 'program_list')['data']['programs']
        app = next((a for a in apps if a['name'] == self.app_name), None)
        if app is None:
            r = call(self.b, 'program_create', name=self.app_name, robotType='UR3', confirm=True)
            print('created', json.dumps(r.get('data'))[:400])
            self.app = str(r['data'].get('appId') or r['data'].get('id'))
        else:
            self.app = app['id']
        tree = call(self.b, 'program_get', appId=self.app)['data']
        script = next((l for l in tree['tree'] if 'ur-script' in l), None)
        if script is None:
            code = next(l for l in tree['tree'] if 'ur-code' in l)
            r = call(self.b, 'program_add_node', appId=self.app, programId=self.app, parentGuid=_guid(code),
                            type='ur-script', parameters={'expression': f'textmsg("{self.app_name} ready")'}, confirm=True)
            print('added', json.dumps(r.get('data'))[:400])
            tree = call(self.b, 'program_get', appId=self.app)['data']
            script = next(l for l in tree['tree'] if 'ur-script' in l)
        self.guid = _guid(script)
        loaded = call(self.b, 'program_get_loaded')['data']
        if str(loaded.get('appId')) != str(self.app):
            call(self.b, 'program_load', appId=self.app, confirm=True)
        state = STATE if self.app_name == APP else STATE.with_name(f'urexec_{self.app_name}.json')
        state.parent.mkdir(parents=True, exist_ok=True)
        state.write_text(json.dumps({'appId': self.app, 'scriptGuid': self.guid}))

    def _log_list(self):
        d = call(self.b, 'psx_logs')['data']
        return d['lines']

    def run(self, script, timeout=60.0, preamble=True):
        """Play one URScript step and block until it stops.

        Returns (msgs, pose): msgs are the step's textmsg() texts, read back from the
        controller log, which is also how the step reports results."""
        # restore the flange TCP at the end: set_tcp persists and the controller is shared
        expr = (self.preamble if preamble else '') + script + ('\ntel(0, 0)\nset_tcp(p[0, 0, 0, 0, 0, 0])\n' if preamble else '')
        tm = {'t0': time.monotonic(), 'marks': [], 'play': [], 'chars': len(expr), 'lines': expr.count('\n') + 1}
        mark = lambda name: tm['marks'].append((name, round(time.monotonic() - tm['t0'], 3)))  # noqa: E731
        problems = lint(expr)
        mark('lint')
        if problems:
            raise BridgeError('urscript_lint: ' + '; '.join(problems))
        r = call(self.b, 'program_update_node', appId=self.app, programId=self.app, guid=self.guid,
                 parameters={'expression': expr}, confirm=True)
        mark('update_node')
        v = (r.get('data') or {}).get('validation') or {}
        if v and not v.get('valid', True):
            raise BridgeError(f'script invalid: {v}')
        # the controller is shared: someone else may have loaded their app since our last step,
        # and play would then start their program
        if str(call(self.b, 'program_get_loaded')['data'].get('appId')) != str(self.app):
            call(self.b, 'program_load', appId=self.app, confirm=True)
            time.sleep(2)
        mark('get_loaded')
        base = max(int(l['ts']) for l in self._log_list())
        mark('psx_base')
        for attempt in range(4):
            ta = time.monotonic()
            try:
                call(self.b, 'program_run', action='play', confirm=True)
                tm['play'].append(('ok', round(time.monotonic() - ta, 3)))
                break
            except BridgeError as err:
                kind = 'not_ready' if 'program_not_ready' in str(err) else 'timeout' if 'timeout' in str(err) else 'error'
                tm['play'].append((kind, round(time.monotonic() - ta, 3)))
                if 'program_not_ready' in str(err) and attempt < 3:   # still generating code after the edit
                    time.sleep(2)
                    continue
                if 'timeout' not in str(err):     # the PUT often hangs although the program started
                    raise
                break
        mark('play_returned')
        t0 = time.monotonic()
        msgs, started, done = [], False, False
        while not done:
            new = [l for l in self._log_list() if int(l['ts']) > base]   # same-ms entries arrive together
            base = max([base] + [int(l['ts']) for l in new])
            for l in new:
                m = l['message']
                if ('PROGRAM_XXX_STARTED' in m or 'PROGRAM_XXX_STOPPED' in m) and self.app_name not in m:
                    continue        # another client's program
                if 'PROGRAM_XXX_STARTED' in m:
                    started = True
                    mark('started_seen')
                    tm['started_ctrl_ts'] = int(l['ts'])
                elif m.startswith('text_msg'):
                    msgs.append(json.loads(m.split(' ', 1)[1])['text'])
                elif 'PROGRAM_XXX_STOPPED' in m and started:
                    done = True
                    mark('stopped_seen')
                    tm['stopped_ctrl_ts'] = int(l['ts'])
                elif l['level'] in ('WARNING', 'ERROR', 'FAULT') or 'protective_stop' in m:
                    msgs.append('!! ' + m)
            if not done and time.monotonic() - t0 > timeout:
                self.stop()
                raise TimeoutError(f'step did not finish in {timeout:.0f} s, stopped ({msgs})')
            time.sleep(0.2)
        rs = call(self.b, 'robot_get_state')['data']
        if rs['safetymode'] != 'NORMAL':
            raise BridgeError(f'safety mode {rs["safetymode"]} after step: {msgs}')
        out = (msgs, self.pose())
        mark('post')
        self._log_timing(tm)
        return out

    def _log_timing(self, tm):
        """Append this step's bridge timing (s from the start of run()) to data/bridge_timing.jsonl."""
        rec = {'wall': time.time(), 'chars': tm['chars'], 'lines': tm['lines'], 'marks': tm['marks'],
               'play': tm['play'], 'started_ctrl_ts': tm.get('started_ctrl_ts'),
               'stopped_ctrl_ts': tm.get('stopped_ctrl_ts')}
        if getattr(self, 'init_timing', None):
            rec['init'], self.init_timing = self.init_timing, None
        try:
            with open(ROOT / 'data' / 'bridge_timing.jsonl', 'a') as f:
                f.write(json.dumps(rec) + '\n')
        except OSError:
            pass

    def move_checked(self, pose, w3_ref=None, w3_tol=0.8, min_elbow=0.5, min_clear=0.10, v=0.25, floor=0.0):
        """Joint move to `pose` (flange, base frame) after IK and safety checks; raises if any fails.

        Checks: bent elbow (|q2| >= min_elbow, a straight elbow faulted the shoulder), wrist 3
        within w3_tol of w3_ref (the camera cable winds up), every collision point of the grip
        model at least min_clear above `floor` (base z of the surface below: plate, board top)."""
        import numpy as np
        from scipy.spatial.transform import Rotation as R
        J = ('base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3')
        q0 = [call(self.b, 'robot_get_joint_positions')['data'][k] for k in J]
        w3_ref = q0[5] if w3_ref is None else w3_ref
        ik_pose = self._in_active_tcp(pose, q0)
        r = call(self.b, 'robot_inverse_kinematics', pose={'position': list(ik_pose[:3]), 'orientation': list(ik_pose[3:])},
                 qNear=q0)['data']
        q = [float(r[k]) for k in J]
        Rm = R.from_rotvec(pose[3:]).as_matrix()
        low = min((np.array(pose[:3]) + Rm @ np.array(o))[2] for o in self.grip['collision_points'].values()) - 0.005 - floor
        problems = []
        if abs(q[2]) < min_elbow:
            problems.append(f'elbow {q[2]:.2f} rad too straight')
        if abs(q[5] - w3_ref) > w3_tol:
            problems.append(f'wrist3 {q[5]:.2f} vs ref {w3_ref:.2f}')
        if low < min_clear:
            problems.append(f'tool only {low * 100:.1f} cm above the floor ({floor * 1000:.1f} mm)')
        if problems:
            raise BridgeError('move rejected: ' + '; '.join(problems))
        self.run(f'movej({[round(v_, 5) for v_ in q]}, a=0.4, v={v})', timeout=150)
        return q

    def _in_active_tcp(self, pose, q0):
        """The bridge IK (and robot_get_pose) work in the controller's active TCP, which is the
        step TCP after any step (the preamble's set_tcp persists). Detect which TCP is active by
        comparing the reported pose with nominal FK, and convert the flange `pose` to it."""
        import numpy as np
        from scipy.spatial.transform import Rotation as R
        from dashboard import kinematics as kin
        F0 = kin.fk(q0, kin.UR3E_NOMINAL_DH)
        rep = np.array(call(self.b, 'robot_get_pose')['data']['pose']['position'])
        t = np.array(self.tcp[:3])
        if np.linalg.norm(rep - F0[:3, 3]) < 0.02:
            return list(pose)
        if np.linalg.norm(rep - (F0[:3, :3] @ t + F0[:3, 3])) < 0.02:
            Rf = R.from_rotvec(pose[3:])
            Rt = Rf * R.from_rotvec(self.tcp[3:6])
            return list(np.array(pose[:3]) + Rf.as_matrix() @ t) + list(Rt.as_rotvec())
        raise BridgeError(f'active TCP unknown: reported {np.round(rep, 4)} vs flange FK {np.round(F0[:3, 3], 4)}')

    def stop(self):
        try:
            call(self.b, 'program_run', action='stop', confirm=True)
        except BridgeError as e:        # 409 when it already stopped
            print('stop:', str(e)[:120])

    def pose(self):
        p = call(self.b, 'robot_get_pose')['data']
        return p['pose']['position'] + p['pose']['orientation'], p['qNear']


if __name__ == '__main__':
    ex = URExec()
    print('app', ex.app, 'script node', ex.guid)
    t = time.monotonic()
    print(ex.run(sys.argv[1] if len(sys.argv) > 1 else 'textmsg("rj45_qa noop")'), f'{time.monotonic()-t:.1f}s')
