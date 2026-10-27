"""Find port k by a peck grid around its estimate. Usage: python -m porttest.find_port K [nx sx nz sz]"""
import sys

from porttest import geometry as g
from porttest.urexec import URExec


def find(ex, k, nx=11, sx=0.0005, nz=11, sz=0.0004):
    c = g.port_tip(k)
    pre = list(c)
    pre[0] -= 0.02 * g.N[0]
    pre[1] -= 0.02 * g.N[1]
    script = f'''set_tcp(p{g.urvec(g.TCP)})
p = get_actual_tcp_pose()
if p[1] > {pre[1] - 0.005:.5f}:
  movel(pose_add(p, p[0, {pre[1] - 0.005:.5f} - p[1], 0, 0, 0, 0]), a=0.2, v=0.02)
end
p = get_actual_tcp_pose()
movel(p[p[0], p[1], {pre[2] + 0.004:.5f}, p[3], p[4], p[5]], a=0.2, v=0.02)
movel({g.urpose(pre)}, a=0.2, v=0.02)
grid_search({g.urpose(c)}, {g.urvec(g.D)}, {g.urvec(g.N)}, {nx}, {sx}, {-(nz - 1) / 2 * sz:.5f}, {nz}, {sz}, 0.0015, 0.005, 6.0)
'''
    return ex.run(script, timeout=900)


if __name__ == '__main__':
    k = int(sys.argv[1])
    args = [float(a) if '.' in a else int(a) for a in sys.argv[2:]]
    print('estimate', [round(v, 4) for v in g.port_tip(k)])
    print(find(URExec(), k, *args))
