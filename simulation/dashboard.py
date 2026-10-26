"""Dashboard side of the simulator: the Simulation tab's files (/simweb/) and a proxy from
/api/sim/* to the simulator's /sim/* API, so the tab works on the dashboard's own origin.

    w.router.add_routes(simulation.dashboard.routes(cfg.get('simulation', {}).get('url')))
"""
import asyncio
from pathlib import Path

import aiohttp
from aiohttp import web

WEB = Path(__file__).with_name('web')
DEFAULT_URL = 'http://127.0.0.1:8095'


def routes(url=None):
    url = (url or DEFAULT_URL).rstrip('/')
    http = {}

    async def proxy(request):
        if 'c' not in http or http['c'].closed:
            http['c'] = aiohttp.ClientSession(timeout=aiohttp.ClientTimeout(total=10))
        try:
            async with http['c'].request(request.method, f'{url}/sim/{request.match_info["tail"]}',
                                         params=request.query, data=await request.read(),
                                         headers={'Content-Type': 'application/json'}) as r:
                return web.Response(body=await r.read(), status=r.status, content_type=r.content_type)
        except (aiohttp.ClientError, asyncio.TimeoutError):
            return web.json_response({'offline': True, 'url': url,
                                      'error': f'simulator not running at {url}: start it with python -m simulation'},
                                     status=503)

    return [web.route('*', '/api/sim/{tail:.*}', proxy), web.static('/simweb', WEB)]
