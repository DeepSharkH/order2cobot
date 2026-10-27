"""
URExec with the switch / RJ45 task's defaults: app rj45_qa, client rj45-skill, the RJ45 plug
tool model (configs/switch/grip_rj45.yaml) and the shared + RJ45 URScript primitives.

The executor itself is task-neutral and lives in urcore/urexec.py; everything the switch
scripts, skill_b, the simulator tests and the protoboard-wire skill import from here
(URExec, call, BridgeError, URBridge, _guid, ROOT, APP, CLIENT, GRIP, PREAMBLE, preamble)
keeps working.

  python -m porttest.urexec ['textmsg("hi")']    # one noop step on the rj45_qa app
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))   # also run as a file path
from urcore import urexec as core  # noqa: E402
from urcore.urexec import ROOT, BridgeError, URBridge, _guid, call, load_grip, urpose, urvec  # noqa: F401,E402

from porttest.urlib import LIB  # noqa: E402

APP = 'rj45_qa'
CLIENT = 'rj45-skill'      # own name: the dashboard's session (order2cobot-calib) no longer drops ours
STATE = ROOT / 'data' / 'urexec.json'
GRIP = load_grip('configs/switch/grip_rj45.yaml')
PAYLOAD = core.PAYLOAD


def preamble(tcp):
    return core.preamble(tcp, LIB, PAYLOAD)


PREAMBLE = preamble(GRIP['tcp_tip'])


class URExec(core.URExec):
    """The switch defaults; other callers (protoboard-wire) pass their own app, client, grip."""

    def __init__(self, app=APP, client=CLIENT, grip=None, tcp_key='tcp_tip'):
        super().__init__(app, client, grip or GRIP, tcp_key, lib=LIB, payload=PAYLOAD)

    def state_path(self):
        return STATE if self.app_name == APP else super().state_path()


if __name__ == '__main__':
    ex = URExec()
    print('app', ex.app, 'script node', ex.guid)
    t = time.monotonic()
    print(ex.run(sys.argv[1] if len(sys.argv) > 1 else 'textmsg("rj45_qa noop")'), f'{time.monotonic()-t:.1f}s')
