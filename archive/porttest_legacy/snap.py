"""Grab the latest frame of both cameras from the running dashboard (it owns the devices)."""
import asyncio
import sys
from pathlib import Path

import aiohttp


async def _grab(url, skip):
    got, seen = {}, {}
    async with aiohttp.ClientSession() as s:
        async with s.ws_connect(url) as ws:
            async for msg in ws:
                if msg.type == aiohttp.WSMsgType.BINARY and msg.data[0] == 1:
                    cam = msg.data[1]
                    seen[cam] = seen.get(cam, 0) + 1
                    if seen[cam] > skip:          # skip frames buffered before the call
                        got.setdefault(cam, bytes(msg.data[2:]))
                if len(got) == 2:
                    return got


def snap(out_dir, tag='', url='http://localhost:8090/ws', skip=3):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    paths = []
    for cam, jpg in sorted(asyncio.run(_grab(url, skip)).items()):
        p = out / f'{tag}cam{cam}.jpg'
        p.write_bytes(jpg)
        paths.append(p)
    return paths


if __name__ == '__main__':
    print(snap(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else ''))
