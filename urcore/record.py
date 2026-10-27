"""Demo recording: both cameras (fixed D435 + gripper Gemini) from the dashboard, plus a
manifest that lines them up with the run logs, telemetry and the laptop screen capture.

  python -m urcore.record start <name>   # returns at once; recorder runs in the background
  python -m urcore.record stop           # stops, writes manifest.json, copies the run logs
  python -m urcore.record status

Writes data/recordings/<YYYYmmdd-HHMMSS>_<name>/:
  cam0_fixed.mp4, cam1_gripper.mp4   constant 10 fps, 1280x720, wall clock burned in
  cam0_fixed.csv, cam1_gripper.csv   per written frame: wall time, source frame age
  manifest.json                      start/stop wall time, frame counts, files
  run/                               copy of data/runs/current (events, tools, telemetry, trace)
data/recordings/ACTIVE holds the folder name while recording; the laptop watcher
(tools/rec_watch.py) starts/stops the screen recording from it.
Read-only for the robot: only GETs on the dashboard's /api/frame.
"""
import json
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
import urllib.request
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
RECS = ROOT / "data" / "recordings"
ACTIVE = RECS / "ACTIVE"
FPS = 10.0
CAMS = {0: "cam0_fixed", 1: "cam1_gripper"}


def grab_loop(cam, latest, stop):
    """Keep the newest decoded frame of one camera (the dashboard serves ~13 fps)."""
    import cv2
    import numpy as np
    while not stop.is_set():
        try:
            png = urllib.request.urlopen(f"http://localhost:8090/api/frame/{cam}", timeout=5).read()
            img = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                latest[cam] = (time.time(), img)
        except Exception:
            time.sleep(0.2)


def record(d):
    """Background worker: write both cameras at a constant FPS until SIGTERM."""
    import cv2
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    latest = {}
    for cam in CAMS:
        threading.Thread(target=grab_loop, args=(cam, latest, stop), daemon=True).start()
    writers, logs, counts = {}, {}, {c: 0 for c in CAMS}
    t0 = time.time()
    k = 0
    while not stop.is_set():
        tick = t0 + k / FPS
        time.sleep(max(0.0, tick - time.time()))
        now = time.time()
        for cam, name in CAMS.items():
            if cam not in latest:
                continue
            ts, img = latest[cam]
            if cam not in writers:
                h, w = img.shape[:2]
                writers[cam] = cv2.VideoWriter(str(d / f"{name}.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), FPS, (w, h))
                logs[cam] = open(d / f"{name}.csv", "w")
                logs[cam].write("frame,wall_time,frame_age_s\n")
            frame = img.copy()
            label = f"{name}  {datetime.fromtimestamp(now):%H:%M:%S.%f}"[:-3]
            cv2.putText(frame, label, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(frame, label, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
            writers[cam].write(frame)
            logs[cam].write(f"{counts[cam]},{now:.3f},{now - ts:.3f}\n")
            counts[cam] += 1
        k += 1
    for cam in writers:
        writers[cam].release()
        logs[cam].close()
    (d / "counts.json").write_text(json.dumps({CAMS[c]: n for c, n in counts.items()}))


def start(name):
    if ACTIVE.exists():
        sys.exit(f"already recording: {ACTIVE.read_text().strip()} (python -m urcore.record stop)")
    d = RECS / f"{datetime.now():%Y%m%d-%H%M%S}_{name}"
    d.mkdir(parents=True)
    log = open(d / "recorder.log", "w")
    p = subprocess.Popen([sys.executable, "-m", "urcore.record", "_worker", str(d)], cwd=ROOT,
                         stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    t = time.time()
    (d / "manifest.json").write_text(json.dumps({"name": name, "start_wall": t, "start_local": f"{datetime.fromtimestamp(t)}",
                                                 "pid": p.pid, "fps": FPS}, indent=1))
    ACTIVE.write_text(d.name + "\n")
    print(f"REC START {datetime.fromtimestamp(t):%H:%M:%S}  ->  {d}")
    print("Sync cue: clap once in view of the fixed camera (and the phone) now.")


def stop():
    if not ACTIVE.exists():
        sys.exit("not recording")
    d = RECS / ACTIVE.read_text().strip()
    ACTIVE.unlink()                                   # the laptop watcher stops the screencast on this
    m = json.loads((d / "manifest.json").read_text())
    try:
        os.kill(m["pid"], signal.SIGTERM)
        for _ in range(100):
            os.kill(m["pid"], 0)
            time.sleep(0.1)
    except ProcessLookupError:
        pass
    run = ROOT / "data" / "runs" / "current"
    if run.exists():
        shutil.copytree(run.resolve(), d / "run", dirs_exist_ok=True)
    m.update({"stop_wall": time.time(), "stop_local": f"{datetime.now()}",
              "duration_s": round(time.time() - m["start_wall"], 1),
              "frames": json.loads((d / "counts.json").read_text()) if (d / "counts.json").exists() else None,
              "run": run.resolve().name if run.exists() else None,
              "files": sorted(p.name for p in d.iterdir())})
    (d / "manifest.json").write_text(json.dumps(m, indent=1))
    print(f"REC STOP {m['duration_s']} s, frames {m['frames']}  ->  {d}")


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "status"
    if cmd == "start":
        start(sys.argv[2] if len(sys.argv) > 2 else "demo")
    elif cmd == "stop":
        stop()
    elif cmd == "_worker":
        record(Path(sys.argv[2]))
    else:
        print(f"recording: {ACTIVE.read_text().strip()}" if ACTIVE.exists() else "not recording")


if __name__ == "__main__":
    main()
