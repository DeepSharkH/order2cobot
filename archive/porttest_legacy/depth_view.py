"""Render the gripper camera's depth cloud as an image next to the colour frame.
Usage: python -m porttest.depth_view out.jpg [x0 y0 x1 y1 (region in 1280x720 px for stats)]"""
import asyncio
import sys

import cv2
import numpy as np

from porttest import dash
from porttest.square_from_depth import _grab


def main(out, box=None):
    color, cloud = asyncio.run(_grab())
    K = np.array(next(c for c in dash.state()['cams'] if 'Gemini' in c['name'])['K'])
    s = color.shape[1] / 1280
    uv = (K @ cloud.T).T
    uv = uv[:, :2] / uv[:, 2:3]
    z = cloud[:, 2]
    zmin, zmax = np.percentile(z, 2), np.percentile(z, 98)
    img = np.zeros_like(color)
    col = cv2.applyColorMap(np.clip((z - zmin) / (zmax - zmin) * 255, 0, 255).astype(np.uint8).reshape(-1, 1),
                            cv2.COLORMAP_TURBO).reshape(-1, 3)
    for (u, v), c in zip(uv * s, col):
        cv2.circle(img, (int(u), int(v)), 3, tuple(int(x) for x in c), -1)
    cv2.putText(img, f'near {zmin:.2f} m (blue) .. far {zmax:.2f} m (red)', (10, 30), 0, 0.8, (255, 255, 255), 2)
    if box:
        x0, y0, x1, y1 = box
        m = (uv[:, 0] >= x0) & (uv[:, 0] < x1) & (uv[:, 1] >= y0) & (uv[:, 1] < y1)
        print(f'region {box}: {m.sum()} depth points, z median {np.median(z[m]):.3f} m, '
              f'p10..p90 {np.percentile(z[m], 10):.3f}..{np.percentile(z[m], 90):.3f}' if m.any()
              else f'region {box}: no depth')
        for im in (img, color):
            cv2.rectangle(im, (int(x0 * s), int(y0 * s)), (int(x1 * s), int(y1 * s)), (255, 255, 255), 2)
    cv2.imwrite(out, np.vstack([color, img]))
    print(f'{len(cloud)} depth points ({len(cloud) / (1280 * 720 / 16) * 100:.0f}% of the stride grid)')


if __name__ == '__main__':
    main(sys.argv[1], [float(v) for v in sys.argv[2:6]] if len(sys.argv) > 5 else None)
