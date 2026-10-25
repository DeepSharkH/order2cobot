"""Gemini depth vs the board plane from the colour image (PnP, 15.2 mm squares), board pixels only."""
import asyncio

import cv2
import numpy as np

from dashboard import calibration as cal
from porttest import dash
from porttest.square_from_depth import _grab

SQUARE = 0.0152


def main():
    color, cloud = asyncio.run(_grab())
    K = np.array(next(c for c in dash.state()['cams'] if 'Gemini' in c['name'])['K'])
    sx = 1280 / color.shape[1]
    ok, c = cv2.findChessboardCornersSB(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY), (8, 7))
    if not ok:
        raise SystemExit('board not found')
    c = c.reshape(-1, 2) * sx
    T = cal.solve_pnp(cal.Board(8, 7, SQUARE), c.reshape(-1, 1, 2).astype(np.float64), K, np.zeros(5))[0]
    n, o = T[:3, 2], T[:3, 3]
    uv = (K @ cloud.T).T
    uv = uv[:, :2] / uv[:, 2:3]
    hull = cv2.convexHull(c.astype(np.float32))
    inside = np.array([cv2.pointPolygonTest(hull, (float(u), float(v)), False) >= 0 for u, v in uv])
    P = cloud[inside]
    rays = P / P[:, 2:3]
    z_exp = (o @ n) / (rays @ n)
    print(f'board (PnP) {np.linalg.norm(o):.3f} m from the camera, tilt to the optical axis '
          f'{np.degrees(np.arccos(abs(n[2]))):.1f}°')
    print(f'{inside.sum()} depth points on the board: measured z median {np.median(P[:, 2]):.3f} m, '
          f'expected {np.median(z_exp):.3f} m, diff {np.median(P[:, 2] - z_exp) * 1000:+.1f} mm '
          f'(IQR {np.percentile(P[:, 2] - z_exp, 25) * 1000:+.1f}..{np.percentile(P[:, 2] - z_exp, 75) * 1000:+.1f})')
    print(f'whole cloud: {len(cloud)} pts, z median {np.median(cloud[:, 2]):.3f} m')
    # every point against the board plane extended over the table (the board lies on it)
    R = cloud / cloud[:, 2:3]
    diff = (cloud[:, 2] - (o @ n) / (R @ n)) * 1000
    h, e = np.histogram(diff, bins=np.arange(-100, 401, 10))
    print('diff to the table plane [mm]: count')
    for k in np.argsort(h)[::-1][:6]:
        print(f'  {e[k]:+5.0f}..{e[k + 1]:+5.0f}: {h[k]}')
    ratio = cloud[:, 2] / ((o @ n) / (R @ n))
    h, e = np.histogram(ratio, bins=np.arange(0.5, 3.0, 0.02))
    k = np.argmax(h)
    sel = (ratio > e[k] - 0.06) & (ratio < e[k + 1] + 0.06)
    print(f'measured/true ratio mode {e[k]:.2f}..{e[k + 1]:.2f} ({h[k]} pts); '
          f'median around the mode {np.median(ratio[sel]):.3f} ({sel.sum()} pts)')
    near = np.abs(diff) < 15
    if near.any():
        print(f'points within ±15 mm of the plane: {near.sum()}, median diff {np.median(diff[near]):+.1f} mm')


if __name__ == '__main__':
    main()
