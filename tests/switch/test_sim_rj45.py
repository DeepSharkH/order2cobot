"""The switch / RJ45 insertion scenario on the simulator (split out of tests/simulation)."""
import numpy as np
from scipy.spatial.transform import Rotation as R

from porttest.urexec import PREAMBLE
from tests.simulation.test_simulation import make_robot, run_script


def test_rj45_port_found_engaged_seated():
    """urlib primitives as the rj45-insert skill uses them, 1 mm off the port, face touched beside it."""
    world, robot = make_robot()
    sw = next(o for o in world.objects if o.name == 'ls108g')
    port = sw.R @ sw.pockets[0] + sw.t
    rv = R.from_matrix(np.column_stack([[0, -1, 0], [-1, 0, 0], [0, 0, -1]])).as_rotvec()
    start = 'p[' + ', '.join(f'{v:.6f}' for v in list(port + [0.001, -0.0008, 0.0004]) + list(rv)) + ']'
    msgs = run_script(robot, PREAMBLE + f'''
s0 = {start}
movel(pose_add(s0, p[0.007, 0, 0, 0, 0, 0]), a=0.3, v=0.05)
face = contact([0, 1, 0], 4.0)
back_off([0, 1, 0], 0.001)
movel(s0, a=0.3, v=0.02)
ok = search_port([0, 1, 0], 4.0, face, 0.0008, [1, 0, 0], [0, 0, 1], 0.0004, 0.0015, 0.0015, 0.002)
d = engage_x([0, 1, 0], 6.0, face, 0.004, 4.0)
textmsg("wiggle pass ", wiggle_test([1, 0, 0], [0, 0, 1], 0.0008, 3.0))
dy = seat_x([0, 1, 0], 25.0, 0.012, 6.0)
textmsg("past face ", get_actual_tcp_pose()[1] - face)
''')
    assert any(m.startswith('candidate') for m in msgs), msgs
    assert 'wiggle pass True' in msgs
    assert float(msgs[-1].split()[-1]) > 0.012          # seat depth 13 mm
