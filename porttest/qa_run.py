"""Test switch ports: find, seat (force mode), link check, extract, link check -> dashboard Port QA.

Port k is targeted at port 1's working entry pose plus the triangulated offset of port k
(data/ports.yaml, wrist-camera survey). Usage: python -m porttest.qa_run 2 3 ...
"""
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import yaml

from porttest.urexec import URExec

ROOT = Path(__file__).resolve().parent.parent
# flange pose where port 1 accepted the plug (this grip, 2026-10-25): x, face y, z
P1 = np.array([-0.4002, 0.1397, 0.2128])
ROT = '0, 3.14159265, 0'


def qa(port, phase):
    out = subprocess.run(['curl', '-s', '-m', '30', '-X', 'POST', f'http://localhost:8090/api/qa/{port}/{phase}'],
                         capture_output=True, text=True).stdout
    try:
        return json.loads(out)
    except ValueError:
        return {'ok': False, 'reason': out[:200]}


def target(k):
    C = np.array(yaml.safe_load((ROOT / 'data/ports.yaml').read_text())['centre'])
    return P1 + (C[k - 1] - C[0])


def run_port(ex, k):
    x, y, z = target(k)
    s = f'''p = get_actual_tcp_pose()
movel(p[p[0], 0.115, p[2], {ROT}], a=0.2, v=0.02)
movel(p[{x:.5f}, 0.115, {z:.5f}, {ROT}], a=0.2, v=0.02)
if grid_search(p[{x:.5f}, {y:.5f}, {z:.5f}, {ROT}], [1, 0, 0], [0, 1, 0], 5, 0.0005, -0.001, 5, 0.0005, 0.002, 0.008, 20.0):
  insert_fm(25.0, 0.012, 6.0)
end'''
    msgs, pose = ex.run(s, timeout=600)
    found = any(m.startswith('found') for m in msgs)
    print(f'port {k}: target {np.round([x, y, z], 4)}', [m for m in msgs if not m.startswith('col')])
    if not found:
        return {'port': k, 'result': 'not found'}
    ins = qa(k, 'insert')
    print(f'  insert: {ins.get("reason")}')
    ex.run('p = get_actual_tcp_pose()\nmovel(pose_add(p, p[0, -0.025, 0, 0, 0, 0]), a=0.1, v=0.003)', timeout=120)
    ext = qa(k, 'extract')
    print(f'  extract: {ext.get("reason")}')
    return {'port': k, 'insert': ins.get('ok'), 'extract': ext.get('ok')}


if __name__ == '__main__':
    ex = URExec()
    res = [run_port(ex, int(a)) for a in sys.argv[1:]]
    print(json.dumps(res))
