"""Measure the checkerboard square size with the gripper camera's depth (cloud from the dashboard)."""
import asyncio
import struct
import sys

import aiohttp
import cv2
import numpy as np

sys.path.insert(0, '.')
from porttest import dash  # noqa: E402


async def _grab(cam=1):
    color = cloud = None
    async with aiohttp.ClientSession() as s:
        async with s.ws_connect(dash.URL) as ws:
            async for msg in ws:
                if msg.type != aiohttp.WSMsgType.BINARY:
                    continue
                d = msg.data
                if d[0] == 1 and d[1] == cam:
                    color = cv2.imdecode(np.frombuffer(d[2:], np.uint8), cv2.IMREAD_COLOR)
                elif d[0] == 2 and d[1] == cam:
                    n = struct.unpack('<I', d[4:8])[0]
                    cloud = np.frombuffer(d, np.float32, n * 3, 8).reshape(-1, 3)
                if color is not None and cloud is not None:
                    return color, cloud


def main(cols=8, rows=7):
    color, cloud = asyncio.run(_grab())
    cam = next(c for c in dash.state()['cams'] if 'Gemini' in c['name'])
    K = np.array(cam['K'])
    sx = cam['size'][0] / color.shape[1]          # stream is downscaled
    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
    ok, c = cv2.findChessboardCornersSB(gray, (cols, rows))
    if not ok:
        sys.exit('board not found')
    c = c.reshape(-1, 2) * sx
    uv = (K @ cloud.T).T
    uv = uv[:, :2] / uv[:, 2:3]
    hull = cv2.convexHull(c.astype(np.float32))
    inside = np.array([cv2.pointPolygonTest(hull, (float(u), float(v)), False) >= 0 for u, v in uv])
    B = cloud[inside]
    # plane fit (least squares, one refit without outliers)
    for _ in range(2):
        cen = B.mean(0)
        nrm = np.linalg.svd(B - cen)[2][2]
        r = (B - cen) @ nrm
        B = B[np.abs(r) < 3 * r.std() + 1e-4]
    print(f'plane from {len(B)} points, rms {np.std((B - cen) @ nrm) * 1000:.2f} mm')
    rays = np.c_[(c[:, 0] - K[0, 2]) / K[0, 0], (c[:, 1] - K[1, 2]) / K[1, 1], np.ones(len(c))]
    t = (cen @ nrm) / (rays @ nrm)
    pts = rays * t[:, None]
    P = np.array(pts).reshape(rows, cols, 3)
    dx = np.linalg.norm(np.diff(P, axis=1), axis=2)
    dy = np.linalg.norm(np.diff(P, axis=0), axis=2)
    allv = np.concatenate([dx.ravel(), dy.ravel()])
    allv = allv[np.isfinite(allv)]
    # end-to-end spans average out the per-corner depth noise
    span = [np.linalg.norm(P[r, -1] - P[r, 0]) / (cols - 1) for r in range(rows)] + \
           [np.linalg.norm(P[-1, k] - P[0, k]) / (rows - 1) for k in range(cols)]
    span = np.array([s for s in span if np.isfinite(s)])
    print(f'cloud {len(cloud)} pts, board at z {np.nanmedian(P[..., 2]):.3f} m')
    print(f'neighbour squares: median {np.median(allv) * 1000:.2f} mm (n={len(allv)})')
    print(f'span-based square: {np.median(span) * 1000:.2f} mm ± {np.std(span) * 1000:.2f} (n={len(span)})')


if __name__ == '__main__':
    main()
