"""Read-only shell-tag test: N gripper-cam frames -> detect ArUco 4x4_50, size in px, pose jitter.

Usage: .venv/bin/python porttest/tag_test.py [n_frames=10] [tag_mm=15] [cam=1] [tag_id=0]
Writes data/tag_test_annotated.png. Never moves the robot.
"""
import glob
import json
import os
import sys
import time
import urllib.request

import cv2
import numpy as np

N = int(sys.argv[1]) if len(sys.argv) > 1 else 10
SIZE = (float(sys.argv[2]) if len(sys.argv) > 2 else 15.0) / 1000.0
CAM = int(sys.argv[3]) if len(sys.argv) > 3 else 1
TAG = int(sys.argv[4]) if len(sys.argv) > 4 else 0
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SHOTS = os.path.join(ROOT, "data", "approach", "approach_20261025")

cam = json.load(open(sorted(glob.glob(os.path.join(SHOTS, "*.json")))[0]))["cams"][CAM]
K, dist = np.array(cam["K"]), np.array(cam["dist"])
det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
                              cv2.aruco.DetectorParameters())
params = det.getDetectorParameters()
params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
det.setDetectorParameters(params)
h = SIZE / 2
obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float32)   # IPPE_SQUARE order

sols, widths, last, misses = [], [], None, 0
for k in range(N):
    png = urllib.request.urlopen(f"http://localhost:8090/api/frame/{CAM}", timeout=10).read()
    img = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_COLOR)
    corners, ids, _ = det.detectMarkers(img)
    hit = None if ids is None else next((c for c, i in zip(corners, ids.ravel()) if i == TAG), None)
    if hit is None:
        misses += 1
        last = img
        time.sleep(0.2)
        continue
    c = hit.reshape(4, 2)
    widths.append(np.mean([np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)]))
    # a small planar tag has two near-equal pose solutions (mirror flip); keep both, pick later
    _, rv2, tv2, err2 = cv2.solvePnPGeneric(obj, c.astype(np.float32), K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    sols.append([(r.ravel(), t.ravel(), float(e)) for r, t, e in zip(rv2, tv2, np.ravel(err2))])
    rvec, tvec = rv2[0], tv2[0]
    last = img.copy()
    cv2.aruco.drawDetectedMarkers(last, [hit], np.array([[TAG]]))
    cv2.drawFrameAxes(last, K, dist, rvec, tvec, SIZE)
    time.sleep(0.2)

out = {"frames": N, "detected": N - misses, "tag_id": TAG, "tag_mm": SIZE * 1000}
def rdist(a, b):
    return np.degrees(np.linalg.norm(cv2.Rodrigues(cv2.Rodrigues(a)[0] @ cv2.Rodrigues(b)[0].T)[0]))


import yaml  # noqa: E402

SCENE = os.path.join(ROOT, "configs", "calibration", "scene.yaml")
T_fg = np.array(yaml.safe_load(open(SCENE))["T_flange_gcam"])


def axis_err(r):
    """Angle (deg) between the tag normal in the flange frame and the nearest flange axis.
    The shell's faces are square to the flange, so the true solution has a small value."""
    n = T_fg[:3, :3] @ cv2.Rodrigues(r)[0][:, 2]
    return float(np.degrees(np.arccos(min(1.0, np.max(np.abs(n)))))), np.round(n, 3).tolist()


ts, rs, flips, errs = [], [], 0, []
if sols:
    # the two IPPE solutions of the first frame; the physically right one has its normal
    # along a flange axis. Then every frame takes the solution closest to it.
    ref = min((r for r, _, _ in sols[0]), key=lambda r: axis_err(r)[0])
    out["candidates"] = [{"axis_err_deg": round(axis_err(r)[0], 1), "normal_in_flange": axis_err(r)[1]}
                         for r, _, _ in sols[0]]
    for fr in sols:
        k = int(np.argmin([rdist(r, ref) for r, _, _ in fr]))
        flips += k != 0
        rs.append(fr[k][0]); ts.append(fr[k][1] * 1000); errs.append(fr[k][2])
if ts:
    T, Rv = np.array(ts), np.array(rs)
    ang = [np.degrees(np.linalg.norm(cv2.Rodrigues(cv2.Rodrigues(r)[0] @ cv2.Rodrigues(Rv[0])[0].T)[0])) for r in Rv]
    n = cv2.Rodrigues(Rv[0])[0][:, 2]   # tag normal in the camera frame
    out.update({
        "width_px": round(float(np.mean(widths)), 1),
        "px_per_mm": round(float(np.mean(widths)) / (SIZE * 1000), 2),
        "distance_mm": round(float(np.linalg.norm(T.mean(0))), 1),
        "t_cam_mm": np.round(T.mean(0), 1).tolist(),
        "jitter_t_mm_std": np.round(T.std(0), 3).tolist(),
        "jitter_rot_deg_max": round(float(max(ang)), 3),
        "view_angle_deg": round(float(np.degrees(np.arccos(abs(n[2])))), 1),
        "frames_where_2nd_solution_chosen": flips,
        "reproj_err_px_mean": round(float(np.mean(errs)), 3),
        "mean_of_frames_t_std_mm": np.round(T.std(0) / np.sqrt(len(T)), 3).tolist(),
        "tag_centre_in_flange_mm": np.round((T_fg @ np.r_[T.mean(0) / 1000, 1])[:3] * 1000, 1).tolist(),
        "chosen_normal_in_flange": axis_err(Rv[0])[1],
    })
    # the grip is measured from the mean of N frames, so judge the mean's spread; the flip
    # must be decided clearly (chosen normal near a flange axis, the other far from one)
    ax = sorted(c["axis_err_deg"] for c in out["candidates"])
    out["verdict"] = ("PASS" if out["detected"] == N and max(out["mean_of_frames_t_std_mm"]) < 0.3
                      and ax[0] < 10 and ax[1] - ax[0] > 10 else "CHECK")
else:
    out["verdict"] = "NOT DETECTED"
cv2.imwrite(os.path.join(ROOT, "data", "tag_test_annotated.png"), last)
print(json.dumps(out, indent=1))
