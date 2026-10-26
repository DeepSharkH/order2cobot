"""Run Nova's "Camera setup" on the SICK Visionary-T Mini without the web UI.

Camera setup crops the sensor image to a rectangle (sensor pixels, 512x424) and
fits the floor plane to the largest flat surface inside it; the heightmap grid
is then in that floor's frame (z = 0 on the surface) and covers only the crop.

The crop is edited in an on-device viewer ("FloorEstimationViewer"); this script
connects to the AppEngine CROWN websocket as that viewer and sends the edited
rectangle the way a mouse drag would.

    uv run python scripts/sick_camera_setup.py --show            # print current crop, save sensor image
    uv run python scripts/sick_camera_setup.py 0 99 283 388      # crop x0 y0 x1 y1 and apply
"""
import argparse
import asyncio
import base64
import json

import aiohttp
import cv2
import numpy as np

HOST = '192.168.136.1'
PROVIDER = 'ImageProviderV3SXX5_1'


class Crown:
    """CROWN JSON-over-websocket client (the protocol Nova's browser UI uses)."""

    def __init__(self, ws):
        self.ws, self.n, self.pending, self.events = ws, 0, {}, asyncio.Queue()
        asyncio.ensure_future(self._read())

    async def send(self, header, data=None):
        self.n += 1
        await self.ws.send_str(json.dumps({'header': {'clientId': self.n, **header}, 'data': data}))
        return self.n

    async def call(self, fn, data=None, timeout=30):
        fut = asyncio.get_event_loop().create_future()
        self.pending[self.n + 1] = fut
        await self.send({'type': 'FunctionCall', 'function': fn}, data)
        r = await asyncio.wait_for(fut, timeout)
        if r['header'].get('status'):
            raise RuntimeError(f'{fn}: {r["header"]}')
        return r.get('data')

    async def _read(self):
        async for m in self.ws:
            d = json.loads(m.data)
            h = d.get('header', {})
            if h.get('type') == 'FunctionReturn' and h.get('clientId') in self.pending:
                self.pending.pop(h['clientId']).set_result(d)
            else:
                await self.events.put(d)


async def open_viewer(session, viewer_id='FloorEstimationViewer'):
    c = Crown(await session.ws_connect(f'ws://{HOST}/crownJSON', max_msg_size=0))
    h = (await c.call('View/Present/create'))['handle']
    await c.send({'type': 'FreezeHandle', 'id': h['id']})
    await c.call('View/Present/setID', {'handle': h, 'id': viewer_id})
    for ev in ('OnPresentLive', 'OnPresentAssured'):
        await c.call('View/Present/register', {'handle': h, 'eventname': ev})
    return c, h


async def snapshot(c, timeout=8):
    """First presented frame: (sensor intensity image, crop rectangle)."""
    img = crop = None
    end = asyncio.get_event_loop().time() + timeout
    while (img is None or crop is None) and asyncio.get_event_loop().time() < end:
        try:
            e = await asyncio.wait_for(c.events.get(), 1)
        except asyncio.TimeoutError:
            continue
        for o in e.get('data', {}).get('viewObject', []):
            d = o['data']['data']
            if o['data']['class'] != 'View.Present.Add':
                continue
            if d.get('IconicId') == 'image':
                px = d['Iconics'][0]['data']['Data']
                img = np.frombuffer(base64.b64decode(px['Pixels']['data']), np.uint8).reshape(px['Height'], px['Width'])
            elif d.get('IconicId') == 'cropBox':
                crop = d['Iconics'][0]['data']
    return img, crop


async def main(a):
    async with aiohttp.ClientSession() as s:
        c, h = await open_viewer(s)
        await c.call(f'{PROVIDER}/saveCropSelectionRoi')   # what the UI does when the dialog opens
        img, crop = await snapshot(c)
        print('current crop', crop)
        if a.show:
            cv2.imwrite(a.out, img)
            print('sensor image ->', a.out)
            await c.call(f'{PROVIDER}/closeCameraSetup', {'applyChanges': False})
            return
        x0, y0, x1, y1 = a.box
        rect = {'cx': (x0 + x1) / 2, 'cy': (y0 + y1) / 2, 'width': x1 - x0, 'height': y1 - y0, 'orientationRad': 0}
        await c.call('View/Present/notifyIconicChangedEvent', {'handle': h, 'iconicId': 'cropBox',
                     'iconic': {'type': 'object', 'data': {'class': 'Shape.Rectangle', 'data': rect}}})
        await asyncio.sleep(1)
        ok = (await c.call(f'{PROVIDER}/closeCameraSetup', {'applyChanges': not a.dry}, timeout=60))['success']
        print('applied' if not a.dry else 'dry run', rect, 'ok' if ok else 'FAILED')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('box', nargs='*', type=float, help='crop x0 y0 x1 y1 in sensor pixels')
    p.add_argument('--show', action='store_true', help='print the crop and save the sensor image')
    p.add_argument('--dry', action='store_true', help='send the crop but cancel instead of applying')
    p.add_argument('--out', default='sick_sensor.png')
    a = p.parse_args()
    if not a.show and len(a.box) != 4:
        p.error('give x0 y0 x1 y1, or --show')
    asyncio.run(main(a))
