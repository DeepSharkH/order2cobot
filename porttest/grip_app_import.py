"""Create a motion app that can drive the Hand-E from URScript (skill protoboard-wire).

PolyScope X keeps the gripper setup in each application, and only an app with the Hand-E set up
gets the Robotiq rq_* functions in its preamble. This copies the operator's close_gripper app
(Robot API .urpx), drops its gripper move node and its ids, renames it and imports it; URExec
then adds its own ur-script node, whose steps can call e.g.
rq_move_and_wait_norm_multi_grippers(<0-100>, [9]).

  python -m porttest.grip_app_import protoboard_wire3
"""
import http.server
import json
import socket
import ssl
import sys
import tempfile
import threading
import urllib.request

from dashboard.ur_bridge import URBridge
from porttest.urexec import call

ROBOT_API = 'https://universal-robots/universal-robots/robot-api'
ROBOT_IP = '192.168.2.12'


def make(name, template='close_gripper', client='protoboard-wire'):
    b = URBridge(client_name=client)
    b.initialize()
    if any(a['name'] == name for a in call(b, 'program_list')['data']['programs']):
        return 'exists'
    ctx = ssl.create_default_context()
    ctx.check_hostname, ctx.verify_mode = False, ssl.CERT_NONE
    req = urllib.request.Request(f'{ROBOT_API}/programs/v1/{template}', headers={'Accept': '*/*'})
    doc = json.loads(urllib.request.urlopen(req, context=ctx, timeout=20).read())
    prog = doc['program']

    def strip(n):
        n['children'] = [c for c in n.get('children', [])
                         if c.get('contributedNode', {}).get('type') != 'robotiq-gripper-move-program']
        for c in n['children']:
            strip(c)
    strip(prog['programContent'])
    prog['programContent']['contributedNode']['parameters']['name'] = name
    prog['programInformation']['name'] = name
    doc['application']['applicationInfo']['name'] = name
    for k in ('id', 'urscript'):
        prog.pop(k, None)
    doc['application'].pop('id', None)
    # fresh guids: the controller also matches programs by them (program_exists otherwise)
    import re
    import uuid
    text = json.dumps(doc)
    ids = set(re.findall(r'[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}', text))
    for g in ids:
        text = text.replace(g, str(uuid.uuid4()))
    doc = json.loads(text)
    # artifact_put fails on this bridge host (no staging dir): serve the file once over HTTP
    d = tempfile.mkdtemp()
    with open(f'{d}/{name}.urpx', 'w') as f:
        json.dump(doc, f)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect((ROBOT_IP, 80))
    ip = s.getsockname()[0]
    s.close()
    hits = []

    class Handler(http.server.SimpleHTTPRequestHandler):
        def __init__(self, *a, **k):
            super().__init__(*a, directory=d, **k)

        def log_message(self, fmt, *args):
            hits.append(fmt % args)
    srv = http.server.ThreadingHTTPServer((ip, 0), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        r = call(b, 'program_import', source={'url': f'http://{ip}:{srv.server_port}/{name}.urpx'}, confirm=True)
    finally:
        srv.shutdown()
        print('http hits:', hits)
    return r.get('data')


if __name__ == '__main__':
    print(make(sys.argv[1] if len(sys.argv) > 1 else 'protoboard_wire3'))
