"""
Claude tab: a thin frontend to the local Claude Code CLI.

POST /api/claude {"prompt", "session"?, "mode"?}  ->  NDJSON stream of the CLI's
stream-json events (system init carries the session_id to resume with).

Runs `claude -p` in the repo with ANTHROPIC_API_KEY removed from its
environment, so it authenticates with the logged-in subscription, not a key.
Closing the request (Stop in the browser) kills the run.
"""

import asyncio
import os
import shutil

from aiohttp import web

MODES = ('default', 'acceptEdits', 'plan')


def _env():
    env = dict(os.environ)
    for k in ('ANTHROPIC_API_KEY', 'ANTHROPIC_AUTH_TOKEN'):
        env.pop(k, None)
    return env


def routes(root):
    exe = shutil.which('claude') or os.path.expanduser('~/.local/bin/claude')

    async def post(request):
        body = await request.json()
        prompt = str(body.get('prompt', '')).strip()
        if not prompt:
            return web.json_response({'ok': False, 'reason': 'empty prompt'}, status=400)
        mode = body.get('mode') if body.get('mode') in MODES else 'default'
        args = [exe, '-p', prompt, '--output-format', 'stream-json', '--verbose', '--permission-mode', mode]
        if body.get('session'):
            args += ['--resume', str(body['session'])]
        proc = await asyncio.create_subprocess_exec(
            *args, cwd=str(root), env=_env(), stdin=asyncio.subprocess.DEVNULL,
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT, limit=16 << 20)
        resp = web.StreamResponse(headers={'Content-Type': 'application/x-ndjson', 'Cache-Control': 'no-cache'})
        await resp.prepare(request)
        try:
            async for line in proc.stdout:
                await resp.write(line)
            await proc.wait()
        finally:
            if proc.returncode is None:
                proc.kill()
                await proc.wait()
        return resp

    return [web.post('/api/claude', post)]
