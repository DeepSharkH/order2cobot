"""Replace the dashboard's (possibly stale) joint values in a hand-eye session with the
joints of the commanded poses (bridge IK), then check the rotation invariant.

The dashboard polls joints over its own bridge session; when porttest reconnects with
the same client name that session drops and the dashboard keeps serving old joints.
Writes <session>/samples_cmd.json.
"""
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

from dashboard import calibration as cal
from dashboard import kinematics as kin
from porttest import dash
from porttest import handeye_capture as h
from porttest.urexec import URBridge, call

START = [-0.3263016263021121, -0.00407591624696821, 0.34857652538898026,
         2.2343027610928146, -2.212321062735938, -0.028352859460888506]
Q0 = [-0.399, -1.5542, 1.3455, -1.3796, -1.5751, 2.7343]
EXTRA = [[('dz', -0.05)], [('dz', 0.04), ('x', -10)], [('x', 9), ('y', 5)], [('z', 70)], [('z', -70), ('x', -8)]]
# sample id -> orbit steps (1: IK failed, the robot stayed at the start pose)
PLAN = {0: [], 1: [], 13: EXTRA[0], 14: EXTRA[1], 15: EXTRA[3]}
PLAN.update({k: h.POSES_ALL[k - 1] for k in range(2, 13)})
JOINTS = ('base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3')


def ang(M):
    return np.degrees(np.arccos(np.clip((np.trace(M) - 1) / 2, -1, 1)))


def main(session):
    d = Path(session)
    S = json.loads((d / 'samples.json').read_text())
    F = np.eye(4)
    F[:3, :3] = R.from_rotvec(START[3:]).as_matrix()
    F[:3, 3] = START[:3]
    P = F[:3, 3] + F[:3, 2] * 0.41
    b = URBridge()
    for s in S:
        T = h.orbit(F, P, PLAN[s['id']])
        pose = {'position': list(T[:3, 3]), 'orientation': list(R.from_matrix(T[:3, :3]).as_rotvec())}
        r = call(b, 'robot_inverse_kinematics', pose=pose, qNear=Q0)['data']
        q = r.get('jointPositions', r)
        s['q_stored'] = s['q']
        s['q'] = [float(q[k]) for k in JOINTS] if isinstance(q, dict) else [float(v) for v in q]
    (d / 'samples_cmd.json').write_text(json.dumps(S))

    dh = kin.dh_from_bridge(call(b, 'robot_get_kinematic_info')['data'])
    K = np.array(next(c for c in dash.state()['cams'] if 'Gemini' in c['name'])['K'])
    bd = cal.Board(8, 7, 0.02)
    G = [s for s in S if s.get('gripper_corners') and s['id'] != 1]
    A = [kin.fk(np.array(s['q']), dh) for s in G]
    B = [cal.solve_pnp(bd, np.array(s['gripper_corners'], np.float64).reshape(-1, 1, 2), K, np.zeros(5))[0] for s in G]
    for i in range(1, len(G)):
        fa = ang((np.linalg.inv(A[0]) @ A[i])[:3, :3])
        cb = ang((B[0] @ np.linalg.inv(B[i]))[:3, :3])
        dq = np.max(np.abs(np.array(G[i]['q_stored']) - G[i]['q']))
        print(f"{G[i]['id']:3d} flange {fa:5.1f}  cam {cb:5.1f}  stored-vs-commanded q {dq:.3f} rad")


if __name__ == '__main__':
    main(sys.argv[1])
