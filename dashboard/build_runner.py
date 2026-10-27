"""
The build: a project's approved plan (data/circuit_designs/<id>/plan.json) put on the breadboard
by the robot, one pick-and-place per step in the solver's order (skills rack-pick, then
breadboard-place), until every step is done or the operator stops it.

Per step:
  check   two holes in one column, the casing known (what breadboard.place_probe accepts)
  slot    the rack slot that holds this casing (data/rack_state.json); none: the run waits
          for the operator to name one (POST .../build/slot {"slot": "R7"})
  pick    python -m breadboard.rack_pick pick R<k> --part <casing> [--run]
  place   python -m breadboard.place_probe <h1> <h2> --part <casing> --calib [--run] --note ...
Each command's output goes to <design>/build/<step>-<phase>.log. A phase passes when it exits 0
and, with motion, the result.json it wrote (data/runs/rack_pick_* / place_probe_*) has no
miss_reason, and a placing found the holes without ONE HOLE OFF. Otherwise the run pauses on
that step (retry, skip or abort). After the last step the hand-wired connections are listed
for the operator.

Not covered yet: releasing the part after the seat. place_probe leaves the Hand-E closed, so
the next motion would pull the part out; a release phase comes with the manipulation side.

Motion only with config circuit.build.motion: true. Without it every command runs in its plan
mode (lint + IK, no motion), so the whole chain can be checked before the skills are trusted.
One build at a time: the robot is one.

The run's state is run.json (circuit_design.CircuitDesigner.report's format, plus mode, need,
log): waiting -> running <-> paused / waiting_part -> done | failed | aborted. A run found
'running' with no task behind it (the dashboard restarted) reads as interrupted: resume goes on
from the first step not done.
"""

import asyncio
import json
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
RACK_STATE = ROOT / 'data' / 'rack_state.json'
RUNS = ROOT / 'data' / 'runs'
LIVE = ('running', 'waiting_part')
CONTROLS = ('pause', 'resume', 'abort', 'retry', 'skip', 'slot')


def _clip(s, n=160):
    s = ' '.join(str(s).split())
    return s if len(s) <= n else s[:n - 1] + '…'


# Jan's latest holder (All_Casings_Holder_Print.stl, 10-27 15:33): 4 columns x 5 rows of leg-hole
# pairs, column c (1..4) drilled for legs c x 2.54 mm apart; slot k: column (k-1) % 4 + 1
RACK_COLS, RACK_SLOTS = 4, 20


# ── what stands on the rack ─────────────────────────────────────────────────
# data/rack_state.json is rack-pick's file: slots {"R7": {"part": <casing>, "part_id": <inventory
# part>, "pin1": ...}}, in_gripper, pending, history. The operator says what stands where (the
# Rack card); rack-pick sets "part" to None when it takes a casing out.
def read_rack():
    try:
        return json.loads(RACK_STATE.read_text())
    except (FileNotFoundError, ValueError):
        return {'slots': {}, 'history': []}


def rack_contents():
    """{slot: {part_id, casing}} for every slot that holds a casing now."""
    out = {}
    for name, s in (read_rack().get('slots') or {}).items():
        if s and s.get('part'):
            out[name.upper()] = {'casing': s['part'], 'part_id': s.get('part_id')}
    return out


def set_rack_slot(slot, part_id, casing):
    """The operator's word on one slot: part_id + its casing, or empty (part_id None)."""
    st = read_rack()
    slots = st.setdefault('slots', {})
    s = slots.setdefault(slot, {})
    s.update(part=casing if part_id else None, part_id=part_id, set_by='operator', stamp=time.strftime('%Y-%m-%dT%H:%M:%S'))
    st.setdefault('history', []).append({'stamp': s['stamp'], 'slot': slot, 'operator': part_id or 'empty'})
    st['history'] = st['history'][-500:]
    RACK_STATE.parent.mkdir(parents=True, exist_ok=True)
    tmp = RACK_STATE.with_suffix('.tmp')
    tmp.write_text(json.dumps(st, indent=1))
    tmp.replace(RACK_STATE)


def shopping_list(sequence):
    """What the rack lacks for these steps: [{part_id, casing, need, have}] (empty: all there)."""
    need, casing = {}, {}
    for st in sequence:
        need[st['part_id']] = need.get(st['part_id'], 0) + 1
        casing[st['part_id']] = st['casing']
    have = {}
    for s in rack_contents().values():
        if s['part_id']:
            have[s['part_id']] = have.get(s['part_id'], 0) + 1
    return [{'part_id': pid, 'casing': casing[pid], 'need': n, 'have': have.get(pid, 0)}
            for pid, n in need.items() if have.get(pid, 0) < n]


def rack_slot(step, taken=()):
    """The rack slot for this step: one that holds its exact part, else (rack-pick's own entries
    without a part_id) its casing; None if no slot does."""
    slots = sorted(rack_contents().items(), key=lambda kv: int(kv[0].lstrip('R') or 0))
    for exact in (True, False):
        for name, s in slots:
            if name in taken:
                continue
            if (s['part_id'] == step['part_id']) if exact else (not s['part_id'] and s['casing'] == step['casing']):
                return name
    return None


def newest_result(prefix, since):
    """The newest data/runs/<prefix>_*/result.json written after since (rack_pick's (B) writes into
    the run dir of its (A), so the file time counts, not the dir name)."""
    best = max(((f.stat().st_mtime, f) for f in RUNS.glob(f'{prefix}_*/result.json')
                if f.stat().st_mtime >= since - 1), default=None)
    try:
        return json.loads(best[1].read_text()) if best else None
    except ValueError:
        return None


def check_result(phase, res):
    """Why a phase that exited 0 still missed, from its result.json; None if it passed."""
    if res is None:
        return 'no result.json written'
    if res.get('miss_reason'):
        return res['miss_reason']
    if phase == 'insert':
        if not res.get('found_offset_mm'):
            return 'the legs did not find the holes'
        if str(res.get('pair_check', 'ok')).startswith('ONE HOLE OFF'):
            return res['pair_check']
    return None


def rack_mismatch():
    """Why rack-pick's slot numbers would not be the dashboard's, or None: rack_pick places slot k
    by configs/breadboard/rack.yaml's geometry, the Rack card by Jan's 4 x 5 holder."""
    try:
        import yaml
        g = yaml.safe_load((ROOT / 'configs' / 'breadboard' / 'rack.yaml').read_text())['geometry']
    except Exception as e:      # noqa: BLE001
        return f'configs/breadboard/rack.yaml unreadable: {_clip(e, 80)}'
    cols = g.get('cols', 5)
    if cols != RACK_COLS:
        return (f"rack-pick's configs/breadboard/rack.yaml describes a {cols}-column rack ({g.get('source', '?')}); "
                f'the Rack card is the {RACK_COLS}-column holder: update rack.yaml before picking')
    return None


def check_step(step):
    """What placing cannot do with this step, or None."""
    holes = step.get('holes') or []
    if len(holes) != 2:
        return f"{len(holes)} legs: placing takes 2-leg casings"
    (c1, r1), (c2, r2) = [(h[0].lower(), int(h[1:])) for h in holes]
    if c1 != c2:
        return f"{holes[0]}–{holes[1]} crosses columns: placing inserts along one column"
    if not step.get('casing'):
        return 'no casing'
    return None


class BuildRunner:
    def __init__(self, designer, cfg=None):
        cfg = cfg or {}
        self.cd = designer
        self.motion = bool(cfg.get('motion', False))
        self.python = cfg.get('python') or sys.executable
        self.task = None            # the one build that may run
        self.design = None
        self.proc = None
        self.want = None            # pause | abort, asked during a command: acted on after it
        self.wake = asyncio.Event()

    # ── run.json ────────────────────────────────────────────────────────────
    def _run(self, d):
        return self.cd.run(d)

    def _save(self, d, run, note=None, level='info', item=None):
        if note:
            run['log'].append({'at': time.time(), 'level': level, 'item': item, 'text': note})
            run['log'] = run['log'][-500:]
        self.cd._save_run(d, run)

    def state(self, d):
        """The run as the browser sees it: run.json, with 'interrupted' for a live run that has no task."""
        run = self._run(d)
        if not run:
            return None
        if run['status'] in LIVE + ('waiting',) and not self.active(d) and run.get('mode'):
            run = {**run, 'status': 'interrupted'}
        run['motion'] = self.motion
        return run

    def active(self, d):
        return self.task is not None and not self.task.done() and self.design is d

    # ── start and controls ─────────────────────────────────────────────────
    def start(self, d):
        if self.task is not None and not self.task.done():
            raise RuntimeError(f'the robot is building {self.design.id}')
        run = self._run(d)
        if run is None:
            raise RuntimeError('not approved')
        run['mode'] = 'motion' if self.motion else 'plan'
        run['status'] = 'running'
        run.setdefault('started', time.time())
        self._save(d, run, f"build started ({'robot moves' if self.motion else 'plan only: no motion'})")
        self.design, self.want = d, None
        self.task = asyncio.create_task(self._loop(d))
        self.task.add_done_callback(lambda _: d.wake())
        return self.state(d)

    def control(self, d, action, body=None):
        if action not in CONTROLS:
            raise ValueError(f'action must be one of {", ".join(CONTROLS)}')
        run = self._run(d)
        if run is None:
            raise ValueError('not started')
        busy = self.active(d)
        if action == 'pause':
            if not busy:
                raise ValueError('not running')
            self.want = 'pause'
            self._save(d, run, 'pause asked: after the current command')
        elif action == 'abort':
            if busy:
                self.want = 'abort'
                self._save(d, run, 'abort asked: after the current command')
            else:
                run['status'] = 'aborted'
                self._save(d, run, 'aborted', 'warn')
        elif action in ('resume', 'retry', 'skip', 'slot'):
            if busy and action != 'slot':
                raise ValueError('already running')
            k = run.get('current')
            if action == 'skip' and k is not None:
                run['steps'][k].update(status='skipped', finished=time.time())
                self._save(d, run, f"{run['steps'][k]['item']} skipped by the operator", 'warn', run['steps'][k]['item'])
            if action == 'slot':
                slot = str((body or {}).get('slot', '')).upper()
                if not slot.startswith('R') or not slot[1:].isdigit():
                    raise ValueError('slot like R7')
                run['slot_override'] = {**run.get('slot_override', {}), str(k): slot}
                run.pop('need', None)
                self._save(d, run, f'slot {slot} named by the operator', item=run['steps'][k]['item'] if k is not None else None)
                if busy:
                    self.wake.set()
                    return self.state(d)
            if run['status'] in ('done', 'aborted') and action != 'retry':
                raise ValueError(f"the build is {run['status']}")
            return self.start(d)
        return self.state(d)

    # ── the loop ────────────────────────────────────────────────────────────
    async def _loop(self, d):
        plan = json.loads((d.dir / 'plan.json').read_text())
        try:
            for k, step in enumerate(plan['steps']):
                run = self._run(d)
                s = run['steps'][k]
                if s['status'] in ('done', 'skipped'):
                    continue
                if self.want:
                    return self._stop(d, self.want)
                run['current'] = k
                s.update(status='active', started=s.get('started') or time.time(), attempts=s.get('attempts', 0) + 1)
                self._save(d, run)
                bad = check_step(step)
                if bad:
                    return self._fail(d, k, f'cannot place: {bad}')
                bad = rack_mismatch()
                if bad:
                    return self._fail(d, k, bad)
                slot = await self._slot(d, k, step)
                if slot is None:
                    return self._stop(d, self.want or 'abort')
                note = f"project {d.id}, step {k + 1} {step['item']}, rack-pick {slot}"
                for phase, args in (
                        ('pick', ['-m', 'breadboard.rack_pick', 'pick', slot, '--part', step['casing']]),
                        ('insert', ['-m', 'breadboard.place_probe', *[h.lower() for h in step['holes']],
                                    '--part', step['casing'], '--calib', '--note', note])):
                    self._phase(d, k, phase)
                    t0 = time.time()
                    ok, last = await self._cmd(d, k, phase, args + (['--run'] if self.motion else []))
                    if ok and self.motion:
                        miss = check_result(phase, newest_result('rack_pick' if phase == 'pick' else 'place_probe', t0))
                        ok, last = miss is None, miss or last
                    if not ok:
                        return self._fail(d, k, f'{phase}: {last}')
                    if self.want == 'abort':
                        return self._stop(d, 'abort')
                run = self._run(d)
                run['steps'][k].update(status='done', phase='verify', finished=time.time(), note=last)
                self._save(d, run, f"{step['item']} in {'–'.join(step['holes'])}", item=step['item'])
            run = self._run(d)
            run['status'], run['current'] = 'done', None
            hand = plan.get('hand_wired') or {}
            self._save(d, run, 'all steps done' + (f"; by hand: {', '.join(f'{p} {h}' for p, h in hand.items())}" if hand else ''))
        except asyncio.CancelledError:
            run = self._run(d)
            run['status'] = 'paused'
            self._save(d, run, 'stopped: the dashboard is shutting down', 'warn')
            raise
        except Exception as e:      # noqa: BLE001  the run shows it
            run = self._run(d)
            run['status'] = 'failed'
            self._save(d, run, f'build error: {_clip(e)}', 'error')

    def _phase(self, d, k, phase):
        run = self._run(d)
        run['steps'][k]['phase'] = phase
        self._save(d, run)

    async def _slot(self, d, k, step):
        """The slot for this step: the operator's, else rack-pick's state; else wait for the operator."""
        while True:
            run = self._run(d)
            used = {v for kk, v in (run.get('slots_used') or {}).items() if int(kk) != k and not self.motion}
            slot = (run.get('slot_override') or {}).get(str(k)) or rack_slot(step, used)
            if slot:
                run.setdefault('slots_used', {})[str(k)] = slot      # plan mode: the rack is not emptied, so no slot twice
                self._save(d, run)
                return slot
            if self.want:
                return None
            if run.get('need') is None:
                run['status'] = 'waiting_part'
                run['need'] = {'step': k, 'casing': step['casing']}
                self._save(d, run, f"no rack slot holds {step['casing']}: name one", 'warn', step['item'])
            self.wake.clear()
            try:
                await asyncio.wait_for(self.wake.wait(), 2.0)
            except asyncio.TimeoutError:
                pass
            run = self._run(d)
            if run['status'] == 'waiting_part' and (run.get('slot_override') or {}).get(str(k)):
                run['status'] = 'running'
                self._save(d, run)

    async def _cmd(self, d, k, phase, args):
        log = d.dir / 'build' / f'{k + 1:02d}-{phase}.log'
        log.parent.mkdir(parents=True, exist_ok=True)
        with open(log, 'ab') as f:
            f.write(f"\n$ {self.python} {' '.join(args)}   # {time.strftime('%H:%M:%S')}\n".encode())
            f.flush()
            self.proc = await asyncio.create_subprocess_exec(
                self.python, '-u', *args, cwd=str(ROOT), stdout=f, stderr=asyncio.subprocess.STDOUT,
                stdin=asyncio.subprocess.DEVNULL, env={**os.environ, 'PYTHONUNBUFFERED': '1'})
            code = await self.proc.wait()
        self.proc = None
        lines = [x for x in log.read_text(errors='replace').splitlines() if x.strip()]
        return code == 0, _clip(lines[-1] if lines else f'exit {code}')

    def _fail(self, d, k, why):
        run = self._run(d)
        run['steps'][k].update(status='failed', note=_clip(why), finished=time.time())
        run['status'] = 'paused'
        self._save(d, run, why, 'error', run['steps'][k]['item'])

    def _stop(self, d, how):
        run = self._run(d)
        run['status'] = 'paused' if how == 'pause' else 'aborted'
        k = run.get('current')
        if k is not None and run['steps'][k]['status'] == 'active':
            run['steps'][k]['status'] = 'pending'
        run.pop('need', None)
        self.want = None
        self._save(d, run, 'paused' if how == 'pause' else 'aborted', 'warn')
