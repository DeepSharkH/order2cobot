"""URScript for the simulated controller: compiled to Python, run against SimRobot.

Covers what the skills send (porttest/urlib.py LIB, porttest/skill_b.py dispatcher and the
step scripts): def/while/if/elif/else/end, return/break/continue/halt, global/local, pose
literals p[...], keyword arguments, and the built-ins in BUILTINS below. Every statement is
preceded by _L(line), which records the executing URScript line for the viewer, honours
stop/pause and catches loops that never let time pass.

Before a program runs it is checked like the controller does: syntax, then every called
name must be a built-in or defined by the script (unknown: compile_error_name_not_found;
a real URScript function the simulator lacks: not_simulated).
"""
import ast
import json
import keyword
import math
import random
import re
import socket
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

from simulation.robot import DT, ProgramStopped, T_to_vec, vec_to_T

URSCRIPT_BUILTINS = set((Path(__file__).resolve().parent.parent / 'porttest' / 'urscript_builtins.txt').read_text().split())
RENAME = {k for k in keyword.kwlist if k not in ('and', 'or', 'not', 'True', 'False', 'if', 'elif', 'else', 'while',
                                                  'def', 'return', 'break', 'continue', 'global')} | {'None'}
TOKEN = re.compile(r'\s*(?:(?P<num>(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?)|(?P<str>"[^"]*"|\'[^\']*\')'
                   r'|(?P<id>[A-Za-z_]\w*)|(?P<op>==|!=|<=|>=|[-+*/%<>=()\[\],:^]))')
MAX_STATEMENTS_PER_STEP = 200000


class URScriptError(Exception):
    """A controller-side error: kind is the runtime_exceptions key, line the URScript line."""

    def __init__(self, kind, msg, line=None):
        super().__init__(msg)
        self.kind, self.msg, self.line = kind, msg, line


class Halt(Exception):
    pass


class Pose(list):
    pass


# ── value formatting (textmsg / str_cat / to_str) ──────────────────────
def fmt_num(v):
    if isinstance(v, (bool, np.bool_)):
        return 'True' if v else 'False'
    if isinstance(v, (int, np.integer)):
        return str(int(v))
    v = float(v)
    if v == int(v) and abs(v) < 1e15:
        return f'{v:.1f}'
    return f'{v:.6g}'


def to_str(v):
    if isinstance(v, str):
        return v
    if isinstance(v, Pose):
        return 'p[' + ', '.join(fmt_num(x) for x in v) + ']'
    if isinstance(v, (list, tuple, np.ndarray)):
        return '[' + ', '.join(to_str(x) for x in v) + ']'
    return fmt_num(v)


# ── compiler ───────────────────────────────────────────────────────────
def _strip_comment(line):
    out, q = [], None
    for ch in line:
        if q:
            out.append(ch)
            if ch == q:
                q = None
        elif ch in '"\'':
            q = ch
            out.append(ch)
        elif ch == '#':
            break
        else:
            out.append(ch)
    return ''.join(out).rstrip()


def _tokens(text, line):
    toks, pos = [], 0
    text = text.rstrip()
    while pos < len(text):
        m = TOKEN.match(text, pos)
        if not m or m.end() == pos:
            raise URScriptError('syntax_error_on_line', f'unexpected {text[pos:].strip()[:20]!r}', line)
        kind = m.lastgroup
        toks.append((kind, m.group(kind)))
        pos = m.end()
        while pos < len(text) and text[pos].isspace():
            pos += 1
    return toks


def _logical_lines(script):
    """(first line number, tokens) per statement; lines join while brackets are open."""
    out, buf, start, depth = [], [], None, 0
    for n, raw in enumerate(script.splitlines(), 1):
        s = _strip_comment(raw)
        if not s.strip() and not buf:
            continue
        toks = _tokens(s, n)
        if not buf:
            start = n
        buf += toks
        depth += sum(1 for k, t in toks if t in '([') - sum(1 for k, t in toks if t in ')]')
        if depth <= 0 and buf:
            out.append((start, buf))
            buf, depth = [], 0
    if buf:
        raise URScriptError('syntax_error_on_line', 'unclosed bracket', start)
    return out


def _expr(toks):
    """Tokens -> Python source: pose literals p[...] (top-level commas) become _pose([...])."""
    toks = list(toks)
    close_pose = set()
    out = []
    i = 0
    while i < len(toks):
        kind, t = toks[i]
        if kind == 'id' and t == 'p' and i + 1 < len(toks) and toks[i + 1][1] == '[' and \
                not (out and (out[-1][0] in ('id', 'num', 'str') or out[-1][1] in (')', ']'))):
            depth, j, comma = 0, i + 1, False
            while j < len(toks):
                if toks[j][1] in '([':
                    depth += 1
                elif toks[j][1] in ')]':
                    depth -= 1
                    if depth == 0:
                        break
                elif toks[j][1] == ',' and depth == 1:
                    comma = True
                j += 1
            if comma:
                out.append(('id', '_pose('))
                close_pose.add(j)
                out.append(('op', '['))
                i += 2
                continue
        if i in close_pose:
            out.append(('op', '])'))
            i += 1
            continue
        if kind == 'id' and t in RENAME:
            t = f'_u_{t}'
        out.append((kind, t))
        i += 1
    return ' '.join(t for _, t in out)


def compile_script(script):
    """URScript -> (python source, line map {python line: urscript line})."""
    stmts = _logical_lines(script)
    # def ranges and their global declarations
    globals_in = {}
    stack = []
    for idx, (ln, toks) in enumerate(stmts):
        head = toks[0][1]
        if head in ('def', 'thread'):
            stack.append(('def', idx))
        elif head in ('while', 'if'):
            stack.append((head, idx))
        elif head == 'end':
            if not stack:
                raise URScriptError('syntax_error_on_line', 'end without block', ln)
            stack.pop()
        elif head == 'global' and any(k == 'def' for k, _ in stack):
            d = next(i for k, i in reversed(stack) if k == 'def')
            globals_in.setdefault(d, []).append(toks[1][1])
    if stack:
        raise URScriptError('syntax_error_on_line', f'missing end for {stack[-1][0]}', stmts[stack[-1][1]][0])

    lines, lmap, ind = [], {}, 0
    in_def = 0

    def emit(text, ln, dind=0):
        lines.append('    ' * (ind + dind) + text)
        lmap[len(lines)] = ln

    for idx, (ln, toks) in enumerate(stmts):
        head = toks[0][1]
        rest = toks[1:]
        if head in ('def', 'thread'):
            if head == 'thread':
                raise URScriptError('not_simulated', 'threads are not simulated', ln)
            if toks[-1][1] != ':':
                raise URScriptError('syntax_error_on_line', 'def without ":"', ln)
            emit(f'def {_expr(rest)}', ln)
            ind += 1
            in_def += 1
            if idx in globals_in:
                emit('global ' + ', '.join(sorted(set(globals_in[idx]))), ln)
            emit(f'_L({ln})', ln)
        elif head in ('while', 'if'):
            if toks[-1][1] != ':':
                raise URScriptError('syntax_error_on_line', f'{head} without ":"', ln)
            if head == 'if':
                emit(f'_L({ln})', ln)
            emit(f'{head} {_expr(rest)}', ln)
            ind += 1
            emit(f'_L({ln})', ln)
        elif head in ('elif', 'else'):
            if toks[-1][1] != ':':
                raise URScriptError('syntax_error_on_line', f'{head} without ":"', ln)
            emit(f'{head} {_expr(rest)}'.rstrip(), ln, -1)
            emit(f'_L({ln})', ln)
        elif head == 'end':
            ind -= 1
            if lines and re.match(r'\s*def ', lines[-1]):
                emit('pass', ln)
        elif head in ('global', 'local'):
            if len(rest) < 3 or rest[1][1] != '=':
                if len(rest) == 1:
                    continue
                raise URScriptError('syntax_error_on_line', f'{head} needs a value', ln)
            emit(f'_L({ln}); {_expr(rest)}', ln)
        elif head in ('run', 'join', 'kill'):
            raise URScriptError('not_simulated', 'threads are not simulated', ln)
        elif head == 'halt' and len(toks) == 1:
            emit(f'_L({ln}); _halt()', ln)
        elif head in ('enter_critical', 'exit_critical'):
            emit(f'_L({ln})', ln)
        else:
            emit(f'_L({ln}); {_expr(toks)}', ln)
        if head == 'end' and in_def and ind == 0:
            in_def -= 1
    src = '\n'.join(lines) + '\n'
    try:
        tree = ast.parse(src)
    except SyntaxError as e:
        raise URScriptError('syntax_error_on_line', e.msg, lmap.get(e.lineno)) from None
    return src, lmap, tree


def check_names(tree, lmap, known):
    """Names that are neither built-ins nor defined/assigned in the script."""
    defined = set(known) | {'True', 'False'}
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef):
            defined.add(node.name)
            defined.update(a.arg for a in node.args.args)
        elif isinstance(node, ast.Name) and isinstance(node.ctx, ast.Store):
            defined.add(node.id)
        elif isinstance(node, ast.Global):
            defined.update(node.names)
    for node in ast.walk(tree):
        if isinstance(node, ast.Name) and isinstance(node.ctx, ast.Load) and node.id not in defined:
            name = node.id[3:] if node.id.startswith('_u_') else node.id
            kind = 'not_simulated' if name in URSCRIPT_BUILTINS else 'compile_error_name_not_found'
            raise URScriptError(kind, name, lmap.get(node.lineno))


# ── runtime ────────────────────────────────────────────────────────────
def _vec(v):
    return np.array([float(x) for x in v], float)


def _pose(v):
    return Pose(float(x) for x in v)


class Sockets:
    """Named client sockets. 127.0.0.1:63352 is the Robotiq Hand-E's socket on the controller."""

    def __init__(self, rt):
        self.rt, self.s = rt, {}

    def open(self, host, port, name='socket_0', timeout=2.0):
        self.close(name)
        if int(port) == 63352 and host in ('127.0.0.1', 'localhost'):
            self.s[name] = {'kind': 'gripper', 'buf': b'', 'out': b''}
            return True
        try:
            sk = socket.create_connection((host, int(port)), timeout=float(timeout))
        except OSError as e:
            self.rt.textmsg(f'Not able to open socket {name} to host: {host} at port: {port} : {e}\n')
            return False
        sk.setblocking(False)
        self.s[name] = {'kind': 'tcp', 'sock': sk, 'buf': b''}
        self.rt.event('socket', f'{name} open {host}:{port}')
        return True

    def close(self, name='socket_0'):
        c = self.s.pop(name, None)
        if c and c['kind'] == 'tcp':
            c['sock'].close()

    def close_all(self):
        for n in list(self.s):
            self.close(n)

    def send(self, name, data):
        c = self.s.get(name)
        if not c:
            return False
        if c['kind'] == 'gripper':
            for line in data.decode(errors='replace').splitlines():
                c['out'] += (self._gripper(line.strip()) + '\n').encode()
            return True
        c['sock'].sendall(data)
        text = data.decode(errors='replace').strip()
        if text:
            self.rt.event('socket', f'{name} -> {text[:240]}')
        return True

    def _gripper(self, cmd):
        g = self.rt.robot.gripper
        parts = cmd.split()
        if not parts:
            return 'ack'
        if parts[0] == 'SET':
            for k, v in zip(parts[1::2], parts[2::2], strict=False):
                g.set(k, v)
            return 'ack'
        if parts[0] == 'GET' and len(parts) > 1:
            return f'{parts[1]} {g.get(parts[1])}'
        return 'ack'

    def _fill(self, c):
        if c['kind'] == 'gripper':
            c['buf'] += c['out']
            c['out'] = b''
            return
        try:
            while True:
                chunk = c['sock'].recv(65536)
                if not chunk:
                    break
                c['buf'] += chunk
        except (BlockingIOError, InterruptedError):
            pass

    def read_until(self, name, pred, timeout):
        """Step the robot (simulated time) until pred(buffer) gives a result, or timeout."""
        c = self.s.get(name)
        if not c:
            return None
        t_end = self.rt.robot.t + (timeout if timeout and timeout > 0 else 1e9)
        while True:
            self._fill(c)
            r = pred(c)
            if r is not None:
                return r
            if self.rt.robot.t >= t_end:
                return None
            self.rt.robot.step()

    def read_line(self, name, timeout):
        def pred(c):
            if b'\n' in c['buf']:
                line, _, c['buf'] = c['buf'].partition(b'\n')
                return line.decode(errors='replace').rstrip('\r')
        r = self.read_until(name, pred, timeout)
        if r is not None and self.s.get(name, {}).get('kind') == 'tcp':
            self.rt.event('socket', f'{name} <- {r[:240]}')
        return r


class Runtime:
    """Built-ins of one program run, bound to the robot and the controller log."""

    def __init__(self, robot, log, event, check):
        self.robot, self.log, self.event, self.check = robot, log, event, check
        self.line = 0
        self.count = 0
        self.t_seen = None
        self.sockets = Sockets(self)

    # statement hook
    def L(self, n):
        self.line = n
        if self.robot.t != self.t_seen:
            self.t_seen, self.count = self.robot.t, 0
        self.count += 1
        if self.count > MAX_STATEMENTS_PER_STEP:
            raise URScriptError('infinite_loop', 'the program ran too long without letting time pass (no sync/sleep/move in a loop)', n)
        if self.count % 2000 == 0:
            self.check()

    def textmsg(self, s1, s2=''):
        self.log('INFO', 'RTMACHINE', 'text_msg ' + json.dumps({'text': to_str(s1) + to_str(s2)}, separators=(',', ':')))

    def namespace(self):
        rb = self.robot
        rt = self

        def pose_arg(x):
            return list(x) if x is not None else None

        def movej(q, a=1.4, v=1.05, t=0, r=0):
            target = rb.ik(list(q)) if isinstance(q, Pose) else list(q)
            if target is None:
                raise URScriptError('ik_failed', f'movej: no inverse kinematics for {to_str(q)}', rt.line)
            rb.movej(target, a, v, t)

        def movel(x, a=1.2, v=0.25, t=0, r=0):
            pose = list(x) if isinstance(x, Pose) else rb.tcp_pose(np.array(x, float))
            if rb.ik(pose) is None:
                raise URScriptError('ik_failed', f'movel: no inverse kinematics for {to_str(_pose(pose))}', rt.line)
            rb.movel(pose, a, v, t)

        def movec(pose_via, pose_to, a=1.2, v=0.25, r=0, mode=0):
            movel(pose_via, a, v)
            movel(pose_to, a, v)

        def speedl(xd, a, t=DT, aRot=None):
            rb.speedl(list(xd), float(a), float(t) if t else DT)

        def speedj(qd, a, t=DT):
            n = max(1, int(round(float(t) / DT)))
            for _ in range(n):
                rb.step(q_des=rb.q + np.array(qd, float) * DT)

        def servoj(q, a=0, v=0, t=0.002, lookahead_time=0.1, gain=300):
            n = max(1, int(round(float(t) / DT)))
            q0 = rb.q.copy()
            for k in range(1, n + 1):
                rb.step(q_des=q0 + (np.array(q, float) - q0) * k / n)

        def stopl(a, aRot=None):
            rb.stopl(float(a))

        def stopj(a):
            rb.stopl(10.0)

        def force_mode(task_frame, selection_vector, wrench, type, limits):  # noqa: A002
            rb.force_mode(list(task_frame), list(selection_vector), list(wrench), type, list(limits))

        def force_mode_set_damping(damping):
            if rb.fm:
                rb.fm['damping_gain'] = max(0.05, 1.0 - float(damping))

        def force_mode_set_gain_scaling(scaling):
            if rb.fm:
                rb.fm['damping_gain'] = float(scaling)

        def sleep(t):
            rb.wait(float(t))

        def sync():
            rb.step()

        def get_inverse_kin(x, qnear=None, maxPositionError=1e-10, maxOrientationError=1e-10, tcp=None):
            q = rb.ik(list(x), None if qnear is None else list(qnear), pose_arg(tcp))
            if q is None:
                raise URScriptError('ik_failed', f'get_inverse_kin: no solution for {to_str(x)}', rt.line)
            return [float(v) for v in q]

        def get_inverse_kin_has_solution(pose, qnear=None, maxPositionError=1e-10, maxOrientationError=1e-10, tcp=None):
            return rb.ik(list(pose), None if qnear is None else list(qnear), pose_arg(tcp)) is not None

        def get_forward_kin(q=None, tcp=None):
            Tt = vec_to_T(list(tcp)) if tcp is not None else rb.T_tcp
            return _pose(T_to_vec(rb.flange(None if q is None else np.array(q, float)) @ Tt))

        def pose_trans(p_from, p_from_to):
            return _pose(T_to_vec(vec_to_T(list(p_from)) @ vec_to_T(list(p_from_to))))

        def pose_add(p1, p2):
            T1, T2 = vec_to_T(list(p1)), vec_to_T(list(p2))
            T = np.eye(4)
            T[:3, 3] = T1[:3, 3] + T2[:3, 3]
            T[:3, :3] = T2[:3, :3] @ T1[:3, :3]
            return _pose(T_to_vec(T))

        def pose_sub(p_to, p_from):
            T1, T2 = vec_to_T(list(p_to)), vec_to_T(list(p_from))
            T = np.eye(4)
            T[:3, 3] = T1[:3, 3] - T2[:3, 3]
            T[:3, :3] = T1[:3, :3] @ T2[:3, :3].T
            return _pose(T_to_vec(T))

        def pose_inv(p_from):
            T = vec_to_T(list(p_from))
            Ti = np.eye(4)
            Ti[:3, :3] = T[:3, :3].T
            Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
            return _pose(T_to_vec(Ti))

        def pose_dist(p_from, p_to):
            a, b = _vec(p_from), _vec(p_to)
            return float(np.linalg.norm(a[:3] - b[:3]) + np.linalg.norm(
                (R.from_rotvec(b[3:]) * R.from_rotvec(a[3:]).inv()).as_rotvec()))

        def point_dist(p_from, p_to):
            return float(np.linalg.norm(_vec(p_from)[:3] - _vec(p_to)[:3]))

        def interpolate_pose(p_from, p_to, alpha):
            a, b = _vec(p_from), _vec(p_to)
            Ra, Rb = R.from_rotvec(a[3:]), R.from_rotvec(b[3:])
            rv = (Rb * Ra.inv()).as_rotvec()
            return _pose(list(a[:3] + alpha * (b[:3] - a[:3])) + list((R.from_rotvec(alpha * rv) * Ra).as_rotvec()))

        def norm(a):
            if isinstance(a, Pose):
                return float(np.linalg.norm(_vec(a)[:3]))
            if isinstance(a, (list, tuple)):
                return float(np.linalg.norm(_vec(a)))
            return abs(a)

        def normalize(v):
            x = _vec(v)
            return [float(c) for c in x / np.linalg.norm(x)]

        def get_tcp_force():
            return [float(v) for v in rb.force_reading()]

        def get_actual_tcp_pose():
            return _pose(rb.tcp_pose())

        def get_actual_joint_positions():
            return [float(v) for v in rb.q]

        def get_actual_tcp_speed():
            return [float(v) for v in np.r_[rb.vel, 0, 0, 0]]

        def get_actual_joint_speeds():
            return [0.0] * 6

        def get_actual_tool_flange_pose():
            return _pose(T_to_vec(rb.flange()))

        def set_tcp(pose, tcp_name=''):
            rb.set_tcp(list(pose))

        def get_tcp_offset():
            return _pose(rb.tcp)

        def set_payload(m, cog=None):
            rb.payload = {'mass': float(m), 'cog': list(cog or rb.payload['cog'])}

        def set_target_payload(m, cog, inertia=None, transition_time=0):
            rb.payload = {'mass': float(m), 'cog': list(cog)}

        def write_reg(kind):
            def f(address, value):
                rb.registers[kind][int(address)] = value
            return f

        def read_reg(kind, default):
            def f(address):
                return rb.registers[kind].get(int(address), default)
            return f

        def set_digital(kind):
            def f(n, b):
                rb.digital[f'{kind}{int(n)}'] = bool(b)
            return f

        def get_digital(kind):
            def f(n):
                return rb.digital.get(f'{kind}{int(n)}', False)
            return f

        def popup(s, title='Popup', warning=False, error=False, blocking=False):
            rt.log('WARNING' if warning or error else 'INFO', 'RTMACHINE', 'popup ' + json.dumps({'title': to_str(title), 'text': to_str(s)}))

        def str_sub(src, index, len=None):  # noqa: A002
            return src[int(index):] if len is None else src[int(index):int(index) + int(len)]

        def socket_read_ascii_float(number, socket_name='socket_0', timeout=2):
            def pred(c):
                buf = c['buf']
                i, j = buf.find(b'('), buf.find(b')')
                if i >= 0 and j > i:
                    body = buf[i + 1:j].decode(errors='replace')
                    c['buf'] = buf[j + 1:]
                    return body
            body = rt.sockets.read_until(socket_name, pred, float(timeout))
            if body is None:
                return [0] + [0.0] * int(number)
            rt.event('socket', f'{socket_name} <- ({body[:240]})')
            try:
                vals = [float(x) for x in body.split(',') if x.strip()]
            except ValueError:
                return [0] + [0.0] * int(number)
            if len(vals) != int(number):
                return [0] + [0.0] * int(number)
            return [len(vals)] + vals

        def socket_read_line(socket_name='socket_0', timeout=2):
            r = rt.sockets.read_line(socket_name, float(timeout))
            return '' if r is None else r

        def socket_read_string(socket_name='socket_0', prefix='', suffix='', interpret_escape=False, timeout=2):
            r = rt.sockets.read_line(socket_name, float(timeout))
            return '' if r is None else r

        def socket_read_byte_list(number, socket_name='socket_0', timeout=2):
            def pred(c):
                if len(c['buf']) >= int(number):
                    out, c['buf'] = c['buf'][:int(number)], c['buf'][int(number):]
                    return list(out)
            r = rt.sockets.read_until(socket_name, pred, float(timeout))
            return [0] if r is None else [len(r)] + r

        def halt():
            raise Halt()

        ns = {
            '_pose': _pose, '_L': rt.L, '_halt': halt,
            'textmsg': rt.textmsg, 'popup': popup, 'varmsg': lambda name, value: rt.textmsg(f'{name}: ', value),
            'movej': movej, 'movel': movel, 'movep': movel, 'movec': movec, 'speedl': speedl, 'speedj': speedj,
            'servoj': servoj, 'stopl': stopl, 'stopj': stopj, 'sleep': sleep, 'sync': sync,
            'get_steptime': lambda: DT, 'force_mode': force_mode, 'end_force_mode': rb.end_force_mode,
            'force_mode_set_damping': force_mode_set_damping, 'force_mode_set_gain_scaling': force_mode_set_gain_scaling,
            'zero_ftsensor': rb.zero_ft, 'get_tcp_force': get_tcp_force,
            'get_actual_tcp_pose': get_actual_tcp_pose, 'get_target_tcp_pose': get_actual_tcp_pose,
            'get_actual_joint_positions': get_actual_joint_positions, 'get_target_joint_positions': get_actual_joint_positions,
            'get_actual_tcp_speed': get_actual_tcp_speed, 'get_target_tcp_speed': get_actual_tcp_speed,
            'get_actual_joint_speeds': get_actual_joint_speeds, 'get_actual_tool_flange_pose': get_actual_tool_flange_pose,
            'get_inverse_kin': get_inverse_kin, 'get_inverse_kin_has_solution': get_inverse_kin_has_solution,
            'get_forward_kin': get_forward_kin, 'set_tcp': set_tcp, 'get_tcp_offset': get_tcp_offset,
            'set_payload': set_payload, 'set_target_payload': set_target_payload,
            'set_payload_mass': lambda m: set_payload(m), 'set_payload_cog': lambda cog: set_payload(rb.payload['mass'], cog),
            'set_gravity': lambda d: None, 'set_tool_voltage': lambda v: None,
            'is_within_safety_limits': lambda *a: True, 'is_steady': lambda: not np.any(rb.twist),
            'get_controller_temp': lambda: 40.0, 'get_joint_temp': lambda j: 35.0, 'get_joint_torques': lambda: [0.0] * 6,
            'pose_trans': pose_trans, 'pose_add': pose_add, 'pose_sub': pose_sub, 'pose_inv': pose_inv,
            'pose_dist': pose_dist, 'point_dist': point_dist, 'interpolate_pose': interpolate_pose,
            'norm': norm, 'normalize': normalize, 'sqrt': math.sqrt, 'pow': math.pow, 'log': math.log,
            'sin': math.sin, 'cos': math.cos, 'tan': math.tan, 'asin': math.asin, 'acos': math.acos,
            'atan': math.atan, 'atan2': math.atan2, 'floor': lambda f: int(math.floor(f)),
            'ceil': lambda f: int(math.ceil(f)), 'd2r': math.radians, 'r2d': math.degrees, 'random': random.random,
            'to_num': lambda s: float(s) if '.' in str(s) or 'e' in str(s).lower() else int(s), 'to_str': to_str,
            'str_cat': lambda a, b: to_str(a) + to_str(b), 'str_len': len, 'str_sub': str_sub,
            'str_empty': lambda s: len(s) == 0, 'str_find': lambda s, t, start=0: s.find(t, int(start)),
            'length': len, 'get_list_length': len, 'size': len,
            'rpy2rotvec': lambda rpy: list(R.from_euler('xyz', list(rpy)).as_rotvec()),
            'rotvec2rpy': lambda rv: list(R.from_rotvec(list(rv)).as_euler('xyz')),
            'write_output_float_register': write_reg('float'), 'write_output_integer_register': write_reg('int'),
            'write_output_boolean_register': write_reg('bool'),
            'read_output_float_register': read_reg('float', 0.0), 'read_output_integer_register': read_reg('int', 0),
            'read_output_boolean_register': read_reg('bool', False),
            'read_input_float_register': lambda a: 0.0, 'read_input_integer_register': lambda a: 0,
            'read_input_boolean_register': lambda a: False,
            'set_standard_digital_out': set_digital('do'), 'set_tool_digital_out': set_digital('tdo'),
            'set_configurable_digital_out': set_digital('cdo'),
            'get_standard_digital_out': get_digital('do'), 'get_tool_digital_out': get_digital('tdo'),
            'get_configurable_digital_out': get_digital('cdo'),
            'get_standard_digital_in': lambda n: False, 'get_tool_digital_in': lambda n: False,
            'get_configurable_digital_in': lambda n: False, 'get_standard_analog_in': lambda n: 0.0,
            'set_standard_analog_out': lambda n, f: None,
            'socket_open': lambda address, port, socket_name='socket_0', timeout=2: rt.sockets.open(address, port, socket_name, timeout),
            'socket_close': lambda socket_name='socket_0': rt.sockets.close(socket_name),
            'socket_send_string': lambda s, socket_name='socket_0': rt.sockets.send(socket_name, to_str(s).encode()),
            'socket_send_line': lambda s, socket_name='socket_0': rt.sockets.send(socket_name, (to_str(s) + '\n').encode()),
            'socket_send_byte': lambda b, socket_name='socket_0': rt.sockets.send(socket_name, bytes([int(b) & 255])),
            'socket_read_ascii_float': socket_read_ascii_float, 'socket_read_line': socket_read_line,
            'socket_read_string': socket_read_string, 'socket_read_byte_list': socket_read_byte_list,
        }
        return ns


def prepare(script, runtime):
    """Compile and check; returns (code object, namespace, line map). Raises URScriptError."""
    src, lmap, tree = compile_script(script)
    ns = runtime.namespace()
    check_names(tree, lmap, ns)
    ns['__builtins__'] = {}
    return compile(tree, '<urscript>', 'exec'), ns, lmap


def run(code, ns, lmap, runtime):
    """Execute; maps Python errors to controller runtime errors with the URScript line."""
    try:
        exec(code, ns)  # noqa: S102 - the simulated controller's interpreter
    except (Halt, ProgramStopped):
        raise
    except URScriptError:
        raise
    except RecursionError:
        raise URScriptError('stack_overflow', 'too deep recursion', runtime.line) from None
    except ZeroDivisionError:
        raise URScriptError('division_by_zero', 'division by zero', runtime.line) from None
    except IndexError as e:
        raise URScriptError('index_out_of_bounds', str(e), runtime.line) from None
    except (TypeError, ValueError, KeyError, AttributeError) as e:
        raise URScriptError('runtime_error', f'{type(e).__name__}: {e}', runtime.line) from None
    except NameError as e:
        raise URScriptError('compile_error_name_not_found', str(e), runtime.line) from None


def wall_ms():
    return int(time.time() * 1000)
