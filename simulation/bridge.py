"""The simulated UR AI Agent Bridge: MCP (Streamable HTTP, JSON-RPC over POST) on the same
path as the robot's, with the robot's tool list and response shapes, backed by the simulated
controller. Skills reach it exactly like the real bridge:

    UR_BRIDGE_URL=http://127.0.0.1:8095/universal-robots/ai-agent-bridge/mcp/v1/

Every call is recorded for the dashboard's Simulation tab (/sim/* below, proxied by the
dashboard as /api/sim/*):
    GET  /sim/state            robot + program state (poll)
    GET  /sim/events?since=N   bridge calls, socket traffic, controller log
    GET  /sim/trail?since=N    TCP path
    GET  /sim/world            simulated objects (true poses)
    GET  /sim/program          the script that runs / ran
    POST /sim/control          {"action": reset|stop|unlock|rate|remote|local|clear_trail, "value": ...}
"""
import asyncio
import json
import time
import uuid

import numpy as np
from aiohttp import web

from simulation.controller import CAPTURE, find, new_guid, node, outline

MCP_PATH = '/universal-robots/ai-agent-bridge/mcp/v1/'
JOINTS = ('base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3')
READ = {'robot_get_state', 'robot_get_pose', 'robot_get_joint_positions', 'robot_get_kinematic_info',
        'robot_inverse_kinematics', 'robot_forward_kinematics', 'program_list', 'program_get', 'program_get_loaded',
        'program_get_state', 'psx_logs', 'bridge_my_status', 'bridge_orient', 'bridge_get_config',
        'server_capabilities', 'program_validate', 'program_generate_script', 'mcp_get_guide',
        'program_list_node_types', 'bridge_list_connections', 'application_list_tcps'}
TIERS = {'observe': ['read'], 'write': ['read', 'write'], 'move': ['read', 'write', 'motion']}
# measured on the robot (porttest/LEARNINGS.md, 2026-10-26), scaled by --latency
LATENCY = {'initialize': 5.0, 'program_update_node': 9.0, 'program_load': 2.0, 'program_run': 1.0,
           'program_add_node': 1.0, 'program_create': 2.0}


class ToolError(Exception):
    def __init__(self, code, message):
        super().__init__(message)
        self.code, self.message = code, message


def _q(d):
    if isinstance(d, dict):
        return [float(d[k]) for k in JOINTS]
    return [float(v) for v in d]


def _summ(args):
    out = {}
    for k, v in args.items():
        if k == 'parameters' and isinstance(v, dict) and 'expression' in v:
            e = str(v['expression'])
            v = {**v, 'expression': f'<URScript {e.count(chr(10)) + 1} lines, {len(e)} chars>'}
        out[k] = v
    s = json.dumps(out)
    return s if len(s) < 400 else s[:400] + '…'


class Bridge:
    def __init__(self, ctrl, latency=0.0):
        self.ctrl, self.robot = ctrl, ctrl.robot
        self.latency = latency
        cap = json.loads(CAPTURE.read_text())
        self.tools = [{**t, 'description': '[SIMULATOR] ' + t.get('description', '')} for t in cap['tools']]
        self.tool_names = {t['name'] for t in self.tools}
        self.init_result = {**cap['initialize'],
                            'serverInfo': {'name': 'polyscopex-mcp-companion-sim', 'version': 'order2cobot-sim'},
                            'instructions': '[SIMULATOR] order2cobot simulation of the UR3e AI Agent Bridge: no robot '
                                            'moves. ' + cap['initialize'].get('instructions', '')}
        self.kin_info = cap['calls']['robot_get_kinematic_info']['structuredContent']['data']
        self.sessions = {}

    # ── MCP endpoint ───────────────────────────────────────────────────
    async def mcp(self, request):
        if request.method != 'POST':
            return web.Response(status=405)
        try:
            msg = await request.json()
        except json.JSONDecodeError:
            return web.json_response({'jsonrpc': '2.0', 'error': {'code': -32700, 'message': 'parse error'}}, status=400)
        method, mid, params = msg.get('method'), msg.get('id'), msg.get('params') or {}
        sid = request.headers.get('Mcp-Session-Id')
        if method == 'initialize':
            sid = str(uuid.uuid4())
            client = (params.get('clientInfo') or {}).get('name', '?')
            self.sessions[sid] = {'client': client, 'connectionId': str(uuid.uuid4()), 't': time.time()}
            self.ctrl.event('session', f'{client} connected', client=client)
            await self._delay('initialize')
            return web.json_response({'jsonrpc': '2.0', 'id': mid, 'result': self.init_result},
                                     headers={'Mcp-Session-Id': sid})
        if sid not in self.sessions:
            return web.json_response({'jsonrpc': '2.0', 'id': mid, 'error': {'code': -32001, 'message': 'session not found'}},
                                     status=404)
        if mid is None:                                          # notification
            return web.Response(status=202)
        if method == 'ping':
            return web.json_response({'jsonrpc': '2.0', 'id': mid, 'result': {}})
        if method == 'tools/list':
            return web.json_response({'jsonrpc': '2.0', 'id': mid, 'result': {'tools': self.tools}})
        if method == 'tools/call':
            res = await self.call(self.sessions[sid], params.get('name'), params.get('arguments') or {})
            return web.json_response({'jsonrpc': '2.0', 'id': mid, 'result': res})
        if method in ('resources/list', 'prompts/list'):
            return web.json_response({'jsonrpc': '2.0', 'id': mid, 'result': {method.split('/')[0]: []}})
        return web.json_response({'jsonrpc': '2.0', 'id': mid, 'error': {'code': -32601, 'message': f'unknown method {method}'}})

    async def _delay(self, name):
        if self.latency > 0 and name in LATENCY:
            await asyncio.sleep(LATENCY[name] * self.latency)

    async def call(self, sess, name, args):
        t0 = time.monotonic()
        kind = 'read' if name in READ else 'motion' if (name == 'program_run' and args.get('action', 'play') in ('play', 'resume')) else 'write'
        try:
            if name not in self.tool_names:
                raise ToolError('unknown_tool', f'no tool {name}')
            if kind not in TIERS[self.ctrl.tier]:
                raise ToolError('tier_insufficient', f'{name} needs the {kind} tier, this connection has {self.ctrl.tier}')
            fn = getattr(self, 't_' + name, None)
            if fn is None:
                raise ToolError('not_simulated', f'{name} is not simulated (the simulator covers motion through ur-script nodes, state and logs)')
            await self._delay(name)
            data = await asyncio.get_running_loop().run_in_executor(None, fn, {**args, '_session': sess})
            sc = {'status': 'success', 'data': data}
            err = None
        except ToolError as e:
            sc, err = {'status': 'error', 'error': {'code': e.code, 'message': e.message}}, e.code
        except (RuntimeError, PermissionError) as e:
            code = str(e).split(':', 1)[0]
            sc, err = {'status': 'error', 'error': {'code': code, 'message': str(e)}}, code
        ms = round((time.monotonic() - t0) * 1000, 1)
        sc['diagnostics'] = {'target': 'simulator', 'host': 'order2cobot-sim', 'durationMs': ms}
        self.ctrl.event('call', name, client=sess['client'], args=_summ(args), category=kind, ok=err is None,
                        error=None if err is None else sc['error']['message'][:300], ms=ms)
        return {'content': [{'type': 'text', 'text': json.dumps(sc, indent=2)}], 'structuredContent': sc,
                'isError': err is not None}

    # ── tools: bridge ──────────────────────────────────────────────────
    def t_bridge_my_status(self, args):
        s = args['_session']
        return {'tier': self.ctrl.tier, 'allowedEffects': TIERS[self.ctrl.tier], 'state': 'active', 'paused': False,
                'connectionId': s['connectionId'], 'client': s['client'], 'liveSession': True, 'connectionStatus': 'active',
                'simulator': True}

    def t_bridge_orient(self, args):
        return {'you': {'tier': self.ctrl.tier, 'allowedEffects': TIERS[self.ctrl.tier], 'target': 'simulator', 'state': 'active'},
                'robot': self.ctrl.robot_state(),
                'program': {'loaded': self.t_program_get_loaded({}), 'state': self.t_program_get_state({})},
                'note': 'order2cobot simulator: motion = one ur-script node per app, played with program_run; '
                        'results come back as textmsg() lines in psx_logs.'}

    def t_server_capabilities(self, args):
        return {'simulator': True, 'tools': len(self.tools)}

    # ── tools: robot ───────────────────────────────────────────────────
    def t_robot_get_state(self, args):
        return self.ctrl.robot_state()

    def t_robot_set_robot_state(self, args):
        return self.ctrl.set_robot_state(args['action'])

    def t_robot_get_joint_positions(self, args):
        return {**dict(zip(JOINTS, (round(float(v), 4) for v in self.robot.q), strict=True)), 'channel': 'simulator', 'hostValidated': True}

    def t_robot_get_pose(self, args):
        p = self.robot.tcp_pose()                 # in the active TCP, as on the controller
        return {'frame': 'base', 'pose': {'orientation': p[3:], 'position': p[:3]},
                'qNear': dict(zip(JOINTS, (round(float(v), 4) for v in self.robot.q), strict=True)), 'channel': 'simulator',
                'hostValidated': True}

    def t_robot_get_kinematic_info(self, args):
        return self.kin_info

    def t_robot_inverse_kinematics(self, args):
        pose = list(args['pose']['position']) + list(args['pose']['orientation'])
        q = self.robot.ik(pose, _q(args['qNear']) if args.get('qNear') is not None else None)
        if q is None:
            raise ToolError('ik_no_solution', f'no inverse kinematics solution for {pose}')
        return {**dict(zip(JOINTS, (float(v) for v in q), strict=True)), 'channel': 'simulator'}

    def t_robot_forward_kinematics(self, args):
        p = self.robot.tcp_pose(np.array(_q(args['jointPositions'])))
        return {'frame': 'base', 'pose': {'orientation': p[3:], 'position': p[:3]}, 'channel': 'simulator'}

    # ── tools: programs ────────────────────────────────────────────────
    def _app(self, args):
        app = self.ctrl.app(args.get('appId'), args.get('programName'))
        if app is None:
            raise ToolError('app_not_found', f'no application {args.get("appId") or args.get("programName")}')
        return app

    def _tree(self, app, args=None):
        return {'appId': app['id'], 'programId': app['id'], 'rootGuid': app['root']['guid'], 'tree': outline(app['root'])}

    def t_program_list(self, args):
        return {'programs': [{'id': a['id'], 'name': a['name'], 'robotType': a['robotType']} for a in self.ctrl.apps.values()]}

    def t_program_get(self, args):
        return self._tree(self._app(args))

    def t_program_create(self, args):
        if self.ctrl.app(name=args['name']):
            raise ToolError('name_taken', f'an application named {args["name"]} exists')
        app = self.ctrl.create(args['name'], args.get('robotType', 'UR3'))
        return {**self._tree(app), 'id': app['id'], 'channel': 'rest-fallback'}

    def t_program_delete(self, args):
        app = self._app(args)
        if self.ctrl.loaded == app['id']:
            raise ToolError('program_loaded', 'unload it first')
        del self.ctrl.apps[app['id']]
        self.ctrl.mutate()
        return {'deleted': app['id']}

    def t_program_add_node(self, args):
        app = self._app(args)
        parent_guid = args.get('parentGuid') or next(c['guid'] for c in app['root']['children'] if c['type'] == 'ur-code')
        parent, _ = find(app['root'], parent_guid)
        if parent is None:
            raise ToolError('node_not_found', f'no node {parent_guid}')

        def build(spec):
            n = node(spec['type'], spec.get('parameters') or {})
            n['children'] = [build(c) for c in spec.get('children') or []]
            return n
        n = build(args)
        pos = args.get('position')
        parent['children'].insert(len(parent['children']) if pos is None else int(pos), n)
        self.ctrl.mutate()
        return {'guid': n['guid'], **self._tree(app)}

    def t_program_update_node(self, args):
        app = self._app(args)
        n, _ = find(app['root'], args['guid'])
        if n is None:
            raise ToolError('node_not_found', f'no node {args["guid"]}')
        n['params'].update(args.get('parameters') or {})
        self.ctrl.mutate()
        valid = {'valid': True}
        if n['type'] == 'ur-script':
            from simulation import urscript
            try:
                urscript.compile_script(str(n['params'].get('expression', '')))
            except urscript.URScriptError as e:
                valid = {'valid': False, 'errors': [f'line {e.line}: {e.kind}: {e.msg}']}
        return {'guid': n['guid'], 'validation': valid, **self._tree(app)}

    def t_program_delete_node(self, args):
        app = self._app(args)
        n, parent = find(app['root'], args['guid'])
        if n is None or parent is None:
            raise ToolError('node_not_found', f'no node {args["guid"]}')
        parent['children'].remove(n)
        self.ctrl.mutate()
        return self._tree(app)

    def t_program_validate(self, args):
        from simulation import urscript
        app = self._app(args)
        text, _ = self.ctrl.program_text(app)
        try:
            urscript.compile_script(text)
            return {'valid': True}
        except urscript.URScriptError as e:
            return {'valid': False, 'errors': [f'line {e.line}: {e.kind}: {e.msg}']}

    def t_program_generate_script(self, args):
        return {'script': self.ctrl.program_text(self._app(args))[0]}

    def t_program_get_loaded(self, args):
        app = self.ctrl.apps.get(self.ctrl.loaded) if self.ctrl.loaded else None
        return {'loaded': app['name'] if app else None, 'appId': app['id'] if app else None,
                'programId': app['id'] if app else None, 'source': 'simulator'}

    def t_program_load(self, args):
        app = self._app(args)
        self.ctrl.load(app)
        return {'loaded': app['name'], 'appId': app['id'], 'programId': app['id'], 'confirmed': True}

    def t_program_get_state(self, args):
        return {'state': self.ctrl.state, 'apiVersion': '3.1.6', 'source': 'simulator'}

    def t_program_run(self, args):
        action = args.get('action', 'play')
        return {'play': self.ctrl.play, 'stop': self.ctrl.stop, 'pause': self.ctrl.pause_,
                'resume': self.ctrl.resume}[action]()

    def t_psx_logs(self, args):
        lines = list(self.ctrl.log_lines)
        if args.get('level'):
            lines = [ln for ln in lines if ln["level"].lower() == args["level"].lower()]
        if args.get('source'):
            lines = [ln for ln in lines if args["source"].lower() in ln["source"].lower()]
        if args.get('since'):
            from datetime import datetime
            t = datetime.fromisoformat(args['since'].replace('Z', '+00:00')).timestamp() * 1000
            lines = [ln for ln in lines if int(ln["ts"]) >= t]
        lines = lines[-int(args.get('limit') or 100):]
        return {'lines': lines, 'count': len(lines), 'source': 'simulator', 'backend': 'simulator'}

    # ── viewer API ─────────────────────────────────────────────────────
    async def sim_state(self, request):
        return web.json_response({'t': time.time(), 'rate': self.robot.pacer.rate, 'robot': self.robot.snapshot(),
                                  'program': self.ctrl.snapshot(), 'trail_n': self.ctrl.trail.n,
                                  'world': self.world_key})

    async def sim_events(self, request):
        since = int(request.query.get('since', 0))
        ev = [e for e in list(self.ctrl.events) if e['seq'] > since][-500:]
        return web.json_response({'events': ev, 'seq': self.ctrl.seq})

    async def sim_trail(self, request):
        return web.json_response({'points': self.ctrl.trail.since(int(request.query.get('since', 0))), 'n': self.ctrl.trail.n})

    async def sim_world(self, request):
        return web.json_response(self.world.describe())

    async def sim_program(self, request):
        return web.json_response({'name': self.ctrl.run_name, 'hash': self.ctrl.script_hash, 'text': self.ctrl.script})

    async def sim_control(self, request):
        req = await request.json()
        a, v = req.get('action'), req.get('value')
        loop = asyncio.get_running_loop()
        msg = 'ok'
        if a == 'stop':
            try:
                await loop.run_in_executor(None, self.ctrl.stop)
            except RuntimeError as e:
                msg = str(e)
        elif a == 'reset':
            if self.ctrl.state != 'STOPPED':
                await loop.run_in_executor(None, self.ctrl.stop)
            self.robot.reset(v)
            self.ctrl.trail.clear()
            self.ctrl.log('INFO', 'SIM', 'simulator reset to the home joints')
        elif a == 'unlock':
            self.ctrl.set_robot_state('UNLOCK_PROTECTIVE_STOP')
        elif a == 'rate':
            self.robot.pacer.rate = float(v)
            self.robot.pacer.reset()
        elif a in ('remote', 'local'):
            self.ctrl.controlmode = a.upper()
        elif a == 'clear_trail':
            self.ctrl.trail.clear()
        else:
            return web.json_response({'error': f'unknown action {a}'}, status=400)
        self.ctrl.event('sim', f'control {a}{"" if v is None else f" {v}"}: {msg}')
        return web.json_response({'ok': True, 'message': msg})

    def app(self, world):
        self.world = world
        self.world_key = new_guid()[:8]
        w = web.Application(client_max_size=64 * 1024 * 1024)
        for path in (MCP_PATH, MCP_PATH.rstrip('/')):
            w.router.add_route('*', path, self.mcp)
        w.router.add_get('/sim/state', self.sim_state)
        w.router.add_get('/sim/events', self.sim_events)
        w.router.add_get('/sim/trail', self.sim_trail)
        w.router.add_get('/sim/world', self.sim_world)
        w.router.add_get('/sim/program', self.sim_program)
        w.router.add_post('/sim/control', self.sim_control)
        return w


def serve(ctrl, world, port, latency=0.0):
    b = Bridge(ctrl, latency)
    web.run_app(b.app(world), port=port, print=None)

