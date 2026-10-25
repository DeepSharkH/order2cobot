"""Talk to the running calibration dashboard (dashboard/server.py) over its websocket."""
import asyncio
import json

import aiohttp

URL = 'http://localhost:8090/ws'


async def _session(send=None, until=None, timeout=60.0):
    async with aiohttp.ClientSession() as s:
        async with s.ws_connect(URL) as ws:
            if send:
                await ws.send_json(send)
            t_end = asyncio.get_running_loop().time() + timeout
            async for msg in ws:
                if msg.type == aiohttp.WSMsgType.TEXT:
                    m = json.loads(msg.data)
                    if until is None or until(m):
                        return m
                if asyncio.get_running_loop().time() > t_end:
                    raise TimeoutError('no matching dashboard message')


def state():
    return asyncio.run(_session(until=lambda m: m.get('type') == 'state'))


def board_seen():
    """{'fixed': bool, 'gripper': bool} from the live (fast) detection."""
    st = state()
    return {c.get('key', c.get('label')): (c.get('board') or {}).get('seen') for c in st['cams']}


def action(kind, timeout=300.0, **kw):
    """Send capture/solve/... and wait until the dashboard is idle again; returns its message."""
    seen_busy = {'v': False, 'msg': object()}

    def done(m):
        if m.get('type') == 'result':
            return True
        if m.get('type') != 'state':
            return False
        if seen_busy['msg'] is not None and not isinstance(seen_busy['msg'], str):
            seen_busy['msg'] = m.get('message') or ''      # first state: the old message
            return False
        if m.get('busy'):
            seen_busy['v'] = True
            return False
        return seen_busy['v'] or (m.get('message') or '') != seen_busy['msg']

    m = asyncio.run(_session({'type': kind, **kw}, until=done, timeout=timeout))
    return m.get('message'), m


if __name__ == '__main__':
    st = state()
    print(json.dumps({k: st[k] for k in ('busy', 'message', 'samples', 'session')}, default=str)[:600])
    for c in st['cams']:
        print(json.dumps(c)[:400])
