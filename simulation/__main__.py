"""Start the simulated UR3e + AI Agent Bridge.

    python -m simulation                       # http://127.0.0.1:8095, real time
    python -m simulation --rate 0              # as fast as possible
    python -m simulation --truth-offset-mm 1.5 0 -0.8   # objects off their CAD guess
    python -m simulation --latency 1           # the robot's measured bridge delays

Then point any skill at it (same commands as with the robot):
    export UR_BRIDGE_URL=http://127.0.0.1:8095/universal-robots/ai-agent-bridge/mcp/v1/
and watch it in the dashboard's Simulation tab.
"""
import argparse
import json

from dashboard import kinematics as kin
from simulation.bridge import MCP_PATH, serve
from simulation.controller import CAPTURE, Controller
from simulation.robot import SimRobot
from simulation.world import WORLD_FILE, World


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', type=int, default=8095)
    ap.add_argument('--rate', type=float, default=1.0, help='simulated seconds per wall second (0 = unpaced)')
    ap.add_argument('--tier', default='move', choices=['observe', 'write', 'move'], help='access tier of every client')
    ap.add_argument('--local', action='store_true', help='start in Local control (play is refused, like the pendant)')
    ap.add_argument('--latency', type=float, default=0.0, help='scale of the measured bridge delays (1 = like the robot)')
    ap.add_argument('--world', default=str(WORLD_FILE))
    ap.add_argument('--truth-offset-mm', type=float, nargs=3, metavar=('DX', 'DY', 'DZ'),
                    help='move every object this far from its CAD pose (base frame)')
    a = ap.parse_args()

    world = World(a.world, a.truth_offset_mm)
    cap = json.loads(CAPTURE.read_text())
    dh = kin.dh_from_bridge(cap['calls']['robot_get_kinematic_info']['structuredContent']['data'])
    robot = SimRobot(world, dh, rate=a.rate)
    ctrl = Controller(robot, tier=a.tier, controlmode='LOCAL' if a.local else 'REMOTE')
    for name, s in world.sources.items():
        print(f'  {name}: {s["source"]}, truth offset {s["truth_offset_mm"]} mm')
    print(f'simulated UR3e bridge: http://127.0.0.1:{a.port}{MCP_PATH}  (tier {a.tier}, rate {a.rate})')
    print(f'  export UR_BRIDGE_URL=http://127.0.0.1:{a.port}{MCP_PATH}')
    serve(ctrl, world, a.port, a.latency)


if __name__ == '__main__':
    main()
