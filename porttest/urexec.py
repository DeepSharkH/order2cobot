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

from porttest.urlib import LIB  # noqa: E402

APP = 'rj45_qa'
STATE = Path(__file__).resolve().parent.parent / 'data' / 'urexec.json'

# Hand-E (~1 kg) + UR camera mount with the Gemini (480 g) + plug shell; CoG ~6 cm out
PREAMBLE = 'set_target_payload(1.5, [0.0, 0.0, 0.06])\n' + LIB


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
    def __init__(self):
        self.b = URBridge(client_name='order2cobot-calib')
        self.app = self.guid = None
        self._ensure_app()

    def _ensure_app(self):
        apps = call(self.b, 'program_list')['data']['programs']
        app = next((a for a in apps if a['name'] == APP), None)
        if app is None:
            r = call(self.b, 'program_create', name=APP, robotType='UR3', confirm=True)
            print('created', json.dumps(r.get('data'))[:400])
            self.app = str(r['data'].get('appId') or r['data'].get('id'))
        else:
            self.app = app['id']
        tree = call(self.b, 'program_get', appId=self.app)['data']
        script = next((l for l in tree['tree'] if 'ur-script' in l), None)
        if script is None:
            code = next(l for l in tree['tree'] if 'ur-code' in l)
            r = call(self.b, 'program_add_node', appId=self.app, programId=self.app, parentGuid=_guid(code),
                            type='ur-script', parameters={'expression': 'textmsg("rj45_qa ready")'}, confirm=True)
            print('added', json.dumps(r.get('data'))[:400])
            tree = call(self.b, 'program_get', appId=self.app)['data']
            script = next(l for l in tree['tree'] if 'ur-script' in l)
        self.guid = _guid(script)
        loaded = call(self.b, 'program_get_loaded')['data']
        if str(loaded.get('appId')) != str(self.app):
            call(self.b, 'program_load', appId=self.app, confirm=True)
        STATE.parent.mkdir(parents=True, exist_ok=True)
        STATE.write_text(json.dumps({'appId': self.app, 'scriptGuid': self.guid}))

    def _log_list(self):
        d = call(self.b, 'psx_logs')['data']
        return d['lines']

    def run(self, script, timeout=60.0, preamble=True):
        """Play one URScript step and block until it stops.

        Returns (msgs, pose): msgs are the step's textmsg() texts, read back from the
        controller log, which is also how the step reports results."""
        expr = (PREAMBLE if preamble else '') + script
        r = call(self.b, 'program_update_node', appId=self.app, programId=self.app, guid=self.guid,
                 parameters={'expression': expr}, confirm=True)
        v = (r.get('data') or {}).get('validation') or {}
        if v and not v.get('valid', True):
            raise BridgeError(f'script invalid: {v}')
        base = max(int(l['ts']) for l in self._log_list())
        for attempt in range(4):
            try:
                call(self.b, 'program_run', action='play', confirm=True)
                break
            except BridgeError as err:
                if 'program_not_ready' in str(err) and attempt < 3:   # still generating code after the edit
                    time.sleep(2)
                    continue
                if 'timeout' not in str(err):     # the PUT often hangs although the program started
                    raise
                break
        t0 = time.monotonic()
        msgs, started, done = [], False, False
        while not done:
            new = [l for l in self._log_list() if int(l['ts']) > base]   # same-ms entries arrive together
            base = max([base] + [int(l['ts']) for l in new])
            for l in new:
                m = l['message']
                if 'PROGRAM_XXX_STARTED' in m:
                    started = True
                elif m.startswith('text_msg'):
                    msgs.append(json.loads(m.split(' ', 1)[1])['text'])
                elif 'PROGRAM_XXX_STOPPED' in m and started:
                    done = True
                elif l['level'] in ('WARNING', 'ERROR', 'FAULT') or 'protective_stop' in m:
                    msgs.append('!! ' + m)
            if not done and time.monotonic() - t0 > timeout:
                self.stop()
                raise TimeoutError(f'step did not finish in {timeout:.0f} s, stopped ({msgs})')
            time.sleep(0.2)
        rs = call(self.b, 'robot_get_state')['data']
        if rs['safetymode'] != 'NORMAL':
            raise BridgeError(f'safety mode {rs["safetymode"]} after step: {msgs}')
        return msgs, self.pose()

    def move_checked(self, pose, w3_ref=None, w3_tol=0.8, min_elbow=0.5, min_clear=0.10, v=0.25):
        """Joint move to `pose` (flange, base frame) after IK and safety checks; raises if any fails.

        Checks: bent elbow (|q2| >= min_elbow, a straight elbow faulted the shoulder), wrist 3
        within w3_tol of w3_ref (the camera cable winds up), plug tip and shell rear at least
        min_clear above the plate (the cable droops behind the shell)."""
        import numpy as np
        from scipy.spatial.transform import Rotation as R
        J = ('base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3')
        q0 = [call(self.b, 'robot_get_joint_positions')['data'][k] for k in J]
        w3_ref = q0[5] if w3_ref is None else w3_ref
        r = call(self.b, 'robot_inverse_kinematics', pose={'position': list(pose[:3]), 'orientation': list(pose[3:])},
                 qNear=q0)['data']
        q = [float(r[k]) for k in J]
        Rm = R.from_rotvec(pose[3:]).as_matrix()
        low = min((np.array(pose[:3]) + Rm @ np.array(o))[2] for o in ((0, 0.035, 0.19), (0, -0.022, 0.19))) - 0.015
        problems = []
        if abs(q[2]) < min_elbow:
            problems.append(f'elbow {q[2]:.2f} rad too straight')
        if abs(q[5] - w3_ref) > w3_tol:
            problems.append(f'wrist3 {q[5]:.2f} vs ref {w3_ref:.2f}')
        if low < min_clear:
            problems.append(f'plug/cable only {low * 100:.1f} cm above the plate')
        if problems:
            raise BridgeError('move rejected: ' + '; '.join(problems))
        self.run(f'movej({[round(v_, 5) for v_ in q]}, a=0.4, v={v})', timeout=150)
        return q

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
