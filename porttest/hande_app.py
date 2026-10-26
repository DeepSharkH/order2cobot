"""Hand-E open/close through the Robotiq URCap's program node (skill protoboard-wire).

URScript cannot reach the gripper on this PolyScope X (the :63352 socket times out, the URCap
adds no rq_* script functions), but its `robotiq-gripper-move-program` node works the way the
operator's open_gripper/close_gripper apps use it. One app per setting holds one such node
(position/speed/force in percent; position 0 = open 50 mm, 100 = closed); a call loads it and plays it. The motion app has to be loaded again afterwards (URExec.run does
that itself).

  python -m porttest.hande_app <position> [speed] [force]
"""
import json
import sys
import time
import urllib.request
import ssl

from porttest.urexec import BridgeError, URExec, _guid, call

STROKE = 0.050  # opening at position 0


def position_for(width):
    """Node position (%) for an opening `width` (m)."""
    return max(0, min(100, round((1 - width / STROKE) * 100)))


class HandE:
    """The bridge cannot edit a URCap node once it exists ("not found among owned nodes"), but
    parameters given when the node is added stick: one app per (position, speed, force),
    created on first use."""

    def __init__(self, b, prefix='pgrip2', template='close_gripper',
                 robot_api='https://universal-robots/universal-robots/robot-api', robot_ip='192.168.2.12'):
        self.b, self.prefix, self.template, self.robot_api, self.robot_ip = b, prefix, template, robot_api, robot_ip

    @staticmethod
    def _dao(position, speed, force):
        return {'gripperIds': [1], 'position': int(position), 'speed': int(speed), 'force': int(force),
                'isBlocking': True, 'isRobotCompliant': False}

    def _app(self, position, speed, force):
        """App id for this setting. A new app has no gripper configured on PolyScope X ("Cannot
        find gripper with requested ID"), so it is a copy of the operator's `template` app (whose
        application holds the Hand-E) with the node's dao and the names changed, imported."""
        name = f'{self.prefix}_{position}_{speed}_{force}'
        apps = call(self.b, 'program_list')['data']['programs']
        app = next((a for a in apps if a['name'] == name), None)
        if app is not None:
            return app['id']
        ctx = ssl.create_default_context()
        ctx.check_hostname, ctx.verify_mode = False, ssl.CERT_NONE
        req = urllib.request.Request(f'{self.robot_api}/programs/v1/{self.template}', headers={'Accept': '*/*'})
        doc = json.loads(urllib.request.urlopen(req, context=ctx, timeout=20).read())

        def walk(n):
            cn = n.get('contributedNode', {})
            if cn.get('type') == 'robotiq-gripper-move-program':
                cn['parameters']['dao'].update(position=int(position), speed=int(speed), force=int(force))
            for c in n.get('children', []):
                walk(c)
        walk(doc['program']['programContent'])
        doc['program']['programContent']['contributedNode']['parameters']['name'] = name
        doc['application']['applicationInfo']['name'] = name
        doc['program'].pop('id', None)       # the template's ids would collide (program_exists)
        doc['application'].pop('id', None)
        # artifact_put fails on this host (no staging dir): serve the file once over HTTP instead
        import http.server
        import socket
        import tempfile
        import threading
        d = tempfile.mkdtemp()
        with open(f'{d}/{name}.urpx', 'w') as f:
            json.dump(doc, f)
        s_ = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s_.connect((self.robot_ip, 80))
        ip = s_.getsockname()[0]
        s_.close()
        handler = lambda *a: http.server.SimpleHTTPRequestHandler(*a, directory=d)  # noqa: E731
        srv = http.server.ThreadingHTTPServer((ip, 0), handler)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        try:
            call(self.b, 'program_import', source={'url': f'http://{ip}:{srv.server_port}/{name}.urpx'}, confirm=True)
        finally:
            srv.shutdown()
        apps = call(self.b, 'program_list')['data']['programs']
        return next(a for a in apps if a['name'] == name)['id']

    def move(self, position, speed=20, force=20, timeout=30.0):
        """Run the node once; returns the seconds it took (blocking node: the fingers stopped)."""
        aid = self._app(position, speed, force)
        if str(call(self.b, 'program_get_loaded')['data'].get('appId')) != str(aid):
            call(self.b, 'program_load', appId=aid, confirm=True)
            time.sleep(2)
        t0 = time.monotonic()
        for attempt in range(4):
            try:
                call(self.b, 'program_run', action='play', confirm=True)
                break
            except BridgeError as err:
                if 'program_not_ready' in str(err) and attempt < 3:
                    time.sleep(2)
                    continue
                if 'timeout' not in str(err):
                    raise
                break
        time.sleep(1.0)
        while time.monotonic() - t0 < timeout:
            if call(self.b, 'program_get_state')['data']['state'] == 'STOPPED':
                return time.monotonic() - t0
            time.sleep(0.3)
        call(self.b, 'program_run', action='stop', confirm=True)
        raise TimeoutError(f'gripper move did not finish in {timeout:.0f} s')


if __name__ == '__main__':
    ex = URExec(app='protoboard_wire2', client='protoboard-wire')
    a = [int(x) for x in sys.argv[1:4]]
    print('took', round(HandE(ex.b).move(*a), 1), 's')
