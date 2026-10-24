"""Find the 8 switch port openings in gripper-camera images and triangulate them in robot base.

detect(img)        -> 8 dicts (left->right): centre (u, v), bottom-centre (u, v), box
triangulate(views) -> per port the 3D centre / bottom-centre from >= 2 views (camera poses)
"""
import cv2
import numpy as np

from porttest.locate import K


def detect(img, roi=None, n=8, debug=None):
    """Dark RJ45 openings in a row. roi = (x0, y0, x1, y1) in pixels, else the whole image."""
    x0, y0, x1, y1 = roi or (0, 0, img.shape[1], img.shape[0])
    g = cv2.GaussianBlur(cv2.cvtColor(img[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY), (3, 3), 0)
    # openings are the darkest areas; the blue face is a bit brighter. Pick the threshold
    # that gives n similar boxes (a dark band along the bezel top is filtered by its shape).
    best = None
    for t in range(18, 70, 3):
        m = cv2.morphologyEx((g < t).astype(np.uint8) * 255, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        bx = [cv2.boundingRect(c) for c in cs]
        bx = [b for b in bx if b[2] > 8 and b[3] > 6 and 0.8 < b[2] / b[3] < 3.5]
        if len(bx) < n:
            continue
        med_w, med_h, med_y = (np.median([b[i] for b in bx]) for i in (2, 3, 1))
        bx = sorted(bx, key=lambda b: abs(b[2] - med_w) / med_w + abs(b[3] - med_h) / med_h
                    + abs(b[1] - med_y) / med_h)[:n]
        spread = np.std([b[2] for b in bx]) / med_w + np.std([b[3] for b in bx]) / med_h
        if best is None or spread < best[0]:
            best = (spread, t, bx)
    if best is None:
        return []
    boxes = best[2]
    boxes.sort(key=lambda b: b[0])
    out = []
    for x, y, w, h in boxes:
        out.append({'centre': (x0 + x + w / 2, y0 + y + h / 2), 'bottom': (x0 + x + w / 2, y0 + y + h),
                    'box': (x0 + x, y0 + y, w, h)})
    if debug is not None:
        d = img.copy()
        for i, p in enumerate(out):
            x, y, w, h = p['box']
            cv2.rectangle(d, (x, y), (x + w, y + h), (0, 255, 0), 1)
            cv2.putText(d, str(i + 1), (x, y - 3), 0, 0.5, (0, 255, 255), 1)
        cv2.imwrite(debug, d[max(0, y0 - 20):y1 + 20, max(0, x0 - 20):x1 + 20])
    return out


def _ray(C, uv):
    d = C[:3, :3] @ np.array([(uv[0] - K[0, 2]) / K[0, 0], (uv[1] - K[1, 2]) / K[1, 1], 1.0])
    return C[:3, 3], d / np.linalg.norm(d)


def triangulate(views, key='centre'):
    """views: [(C 4x4 camera in base, detections)] -> (n x 3 points, per-point ray rms in mm)."""
    n = min(len(d) for _, d in views)
    pts, rms = [], []
    for i in range(n):
        A, b, rays = np.zeros((3, 3)), np.zeros(3), []
        for C, det in views:
            o, d = _ray(C, det[i][key])
            P = np.eye(3) - np.outer(d, d)
            A += P
            b += P @ o
            rays.append((o, d))
        X = np.linalg.solve(A, b)
        pts.append(X)
        rms.append(np.sqrt(np.mean([np.sum(((np.eye(3) - np.outer(d, d)) @ (X - o)) ** 2) for o, d in rays])) * 1000)
    return np.array(pts), np.array(rms)
