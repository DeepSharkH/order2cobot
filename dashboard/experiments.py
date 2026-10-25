"""
Experiments tab: the agent runs saved under results/ (scripts/run_agent.py, run_ethernet_exp.py).

A run folder is listed when it holds a transcript.json or camera videos (camera*.mp4).

GET /api/experiments                    runs, newest first: name, status, prompt, videos
GET /api/experiments/{run}              run metadata + final output (no step timeline)
GET /api/experiments/{run}/trace        the whole step timeline (text, tool calls, results)
GET /api/experiments/{run}/videos       per camera: ready / converting (progress) / recording / error;
                                        asking starts the conversion of videos not converted yet
GET /api/experiments/{run}/video/{file} the browser-playable copy (supports range requests)

The recorder writes MPEG-4 Part 2 (mp4v), which browsers do not play, so each video is
converted once to H.264 with PyAV and cached in cache_dir/<run>/; one conversion runs at a
time so the cameras and detection keep their CPU.
"""

import json
import re
import threading
import time
from collections import deque
from fractions import Fraction
from pathlib import Path

from aiohttp import web

RUN_RE = re.compile(r'^[\w.-]+$')
VIDEO_RE = re.compile(r'^camera\d+\.mp4$')
RECORDING_S = 20.0          # a video written to this recently is still being recorded
LIST_FIELDS = ('name', 'status', 'prompt', 'model', 'started_at', 'finished_at', 'error')


class Videos:
    """Converts recorded videos to H.264 in a background thread, one at a time."""

    def __init__(self, cache_dir):
        self.cache = Path(cache_dir)
        self.lock = threading.Lock()
        self.queue = deque()
        self.jobs = {}              # source path -> {'state', 'progress', 'error'}
        self.worker = None

    def target(self, src):
        return self.cache / src.parent.name / src.name

    def status(self, src):
        """State of one video; queues its conversion when it is neither converted nor recording."""
        dst = self.target(src)
        st = src.stat()
        if dst.exists() and dst.stat().st_mtime >= st.st_mtime:
            return {'state': 'ready', 'size': dst.stat().st_size}
        if time.time() - st.st_mtime < RECORDING_S:
            return {'state': 'recording'}
        with self.lock:
            job = self.jobs.get(src)
            if job is None or job['state'] == 'done':
                job = self.jobs[src] = {'state': 'queued', 'progress': 0.0, 'error': None}
                self.queue.append(src)
                if self.worker is None or not self.worker.is_alive():
                    self.worker = threading.Thread(target=self._work, daemon=True, name='video')
                    self.worker.start()
            return dict(job)

    def _work(self):
        while True:
            with self.lock:
                if not self.queue:
                    return
                src = self.queue.popleft()
                job = self.jobs[src]
                job['state'] = 'converting'
            try:
                self._convert(src, job)
                job['state'] = 'done'
            except Exception as e:
                job.update(state='error', error=str(e)[:300])

    def _convert(self, src, job):
        import av
        dst = self.target(src)
        dst.parent.mkdir(parents=True, exist_ok=True)
        tmp = dst.with_suffix('.part.mp4')
        with av.open(str(src)) as inp, av.open(str(tmp), 'w', options={'movflags': '+faststart'}) as out:
            ist = inp.streams.video[0]
            ist.thread_type = 'AUTO'
            fps = ist.average_rate or Fraction(30)
            ost = out.add_stream('libx264', rate=fps)
            ost.width, ost.height, ost.pix_fmt = ist.width, ist.height, 'yuv420p'
            ost.options = {'preset': 'veryfast', 'crf': '28'}
            ost.codec_context.thread_count = 3
            total, tb = ist.frames or 0, 1 / Fraction(fps)
            for n, frame in enumerate(inp.decode(ist)):
                frame.pts, frame.time_base = n, tb      # constant rate: frame n is at n / fps
                for p in ost.encode(frame):
                    out.mux(p)
                if total and n % 30 == 0:
                    job['progress'] = min(n / total, 1.0)
            for p in ost.encode():
                out.mux(p)
        tmp.replace(dst)


class Experiments:
    def __init__(self, results_dir, cache_dir):
        self.dir = Path(results_dir)
        self.videos = Videos(cache_dir)
        self._meta = {}             # transcript path -> (mtime, summary)

    def run_dir(self, name):
        d = self.dir / name
        if not RUN_RE.match(name) or not d.is_dir():
            raise web.HTTPNotFound(reason=f'no run {name}')
        return d

    def video_files(self, d):
        return sorted(p for p in d.iterdir() if VIDEO_RE.match(p.name))

    def transcript(self, d):
        try:
            return json.loads((d / 'transcript.json').read_text())
        except (OSError, ValueError):
            return None

    def summary(self, d):
        """Short form of a run for the grid; transcripts are re-read only when they change."""
        path = d / 'transcript.json'
        videos = self.video_files(d)
        try:
            mtime = path.stat().st_mtime
        except OSError:
            mtime = None
        if mtime is None and not videos:
            return None
        if mtime is not None and self._meta.get(path, (None,))[0] != mtime:
            t = self.transcript(d) or {}
            s = {k: t.get(k) for k in LIST_FIELDS}
            r = t.get('result') or {}
            s.update(cost=r.get('total_cost_usd'), turns=r.get('num_turns'),
                     duration_s=(r.get('duration_ms') or 0) / 1000 or None)
            self._meta[path] = (mtime, s)
        s = dict(self._meta[path][1]) if mtime is not None else {}
        recording = any(time.time() - v.stat().st_mtime < RECORDING_S for v in videos)
        s.update(run=d.name, transcript=mtime is not None, videos=[v.name for v in videos],
                 recording=recording)
        if not s.get('started_at'):
            s['started_at'] = _stamp_from_name(d.name)
        return s

    def list(self):
        runs = [self.summary(d) for d in self.dir.iterdir() if d.is_dir()] if self.dir.is_dir() else []
        return sorted((r for r in runs if r), key=lambda r: r['run'], reverse=True)


def _stamp_from_name(name):
    m = re.match(r'^(\d{4})(\d\d)(\d\d)_(\d\d)(\d\d)(\d\d)_', name)
    return f'{m[1]}-{m[2]}-{m[3]}T{m[4]}:{m[5]}:{m[6]}' if m else None


def routes(results_dir, cache_dir):
    ex = Experiments(results_dir, cache_dir)

    async def index(_):
        return web.json_response({'dir': str(ex.dir), 'runs': ex.list()})

    async def detail(request):
        d = ex.run_dir(request.match_info['run'])
        s = ex.summary(d) or {'run': d.name, 'videos': []}
        t = ex.transcript(d) or {}
        s.update(final_output=t.get('final_output'), steps=len(t.get('steps') or []),
                 recording_info=t.get('recording'), cwd=(t.get('agent_config') or {}).get('cwd'))
        return web.json_response(s)

    async def trace(request):
        t = ex.transcript(ex.run_dir(request.match_info['run']))
        if t is None:
            raise web.HTTPNotFound(reason='no transcript')
        return web.json_response({'started_at': t.get('started_at'), 'steps': t.get('steps') or []})

    async def videos(request):
        d = ex.run_dir(request.match_info['run'])
        out = {}
        for v in ex.video_files(d):
            try:
                out[v.name] = ex.videos.status(v)
            except OSError as e:
                out[v.name] = {'state': 'error', 'error': str(e)}
        return web.json_response(out)

    async def video(request):
        d = ex.run_dir(request.match_info['run'])
        name = request.match_info['file']
        dst = ex.videos.target(d / name)
        if not VIDEO_RE.match(name) or not dst.exists():
            raise web.HTTPNotFound(reason='video not converted yet')
        return web.FileResponse(dst, headers={'Cache-Control': 'no-cache'})

    return [
        web.get('/api/experiments', index),
        web.get('/api/experiments/{run}', detail),
        web.get('/api/experiments/{run}/trace', trace),
        web.get('/api/experiments/{run}/videos', videos),
        web.get('/api/experiments/{run}/video/{file}', video),
    ]
