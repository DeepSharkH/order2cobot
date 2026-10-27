"""The simulated UR3e + AI Agent Bridge (simulation/): URScript compile, contact, MCP round trip."""

import asyncio
import json
import threading

import pytest
from aiohttp import web

from dashboard import kinematics as kin
from dashboard.ur_bridge import URBridge
from porttest.skill_b import DISPATCH
from porttest.urexec import PREAMBLE, call
from simulation import urscript
from simulation.bridge import MCP_PATH, Bridge
from simulation.controller import CAPTURE, Controller
from simulation.robot import SimRobot
from simulation.world import World
from urcore.urexec import preamble as core_preamble


def make_robot():
    cap = json.loads(CAPTURE.read_text())
    dh = kin.dh_from_bridge(cap['calls']['robot_get_kinematic_info']['structuredContent']['data'])
    world = World()
    return world, SimRobot(world, dh, rate=0)


def run_script(robot, script):
    logs = []
    rt = urscript.Runtime(robot, lambda lv, src, m: logs.append(m), lambda *a, **k: None, lambda: None)
    code, ns, lmap = urscript.prepare(script, rt)
    urscript.run(code, ns, lmap, rt)
    return [json.loads(m.split(' ', 1)[1])['text'] for m in logs if m.startswith('text_msg')]


def test_skill_scripts_compile():
    for script in (PREAMBLE, PREAMBLE + DISPATCH, core_preamble([0, 0, 0.2, 0, 0, 0])):
        urscript.compile_script(script)


def test_pose_literal_and_index():
    src, _, _ = urscript.compile_script('p = get_actual_tcp_pose()\nmovel(p[p[0], 1, 2, 0, 0, 0])\n')
    assert '_pose( [ p [ 0 ] , 1' in src


def test_unknown_function_is_a_compile_error():
    _, robot = make_robot()
    with pytest.raises(urscript.URScriptError) as e:
        run_script(robot, 'x = abs(-1)\n')
    assert e.value.kind == 'compile_error_name_not_found'


def test_textmsg_format():
    _, robot = make_robot()
    assert run_script(robot, 'textmsg("hit ", True)\ntextmsg(str_cat("d ", 0.0125), "")\n') == ['hit True', 'd 0.0125']


def test_bridge_roundtrip(tmp_path, monkeypatch):
    """The robot's client (dashboard/ur_bridge.py) on the simulated bridge: tools, state, a step."""
    monkeypatch.setattr('simulation.controller.STORE', tmp_path / 'programs.json')
    world, robot = make_robot()
    ctrl = Controller(robot)
    app = Bridge(ctrl).app(world)
    ready, box = threading.Event(), {}

    def serve():
        loop = asyncio.new_event_loop()
        runner = web.AppRunner(app)
        loop.run_until_complete(runner.setup())
        site = web.TCPSite(runner, '127.0.0.1', 0)
        loop.run_until_complete(site.start())
        box['port'] = site._server.sockets[0].getsockname()[1]
        ready.set()
        loop.run_forever()
    threading.Thread(target=serve, daemon=True).start()
    ready.wait(10)
    b = URBridge(f'http://127.0.0.1:{box["port"]}{MCP_PATH}', client_name='test')
    assert len(b.list_tools()) == 101
    assert b.call('robot_get_state')['data']['safetymode'] == 'NORMAL'
    app_id = call(b, 'program_create', name='sim_test', confirm=True)['data']['appId']
    tree = b.call('program_get', appId=app_id)['data']['tree']
    code = next(line for line in tree if 'ur-code' in line).rsplit('[', 1)[1].rstrip(']')
    b.call('program_add_node', appId=app_id, programId=app_id, parentGuid=code, type='ur-script',
           parameters={'expression': 'textmsg("sim ", 42)'}, confirm=True)
    b.call('program_load', appId=app_id, confirm=True)
    b.call('program_run', action='play', confirm=True)
    ctrl.thread.join(10)
    texts = [line['message'] for line in b.call('psx_logs')['data']['lines']]
    assert 'text_msg {"text":"sim 42"}' in texts
    assert any(t.startswith('PROGRAM_XXX_STOPPED') and 'sim_test' in t for t in texts)
