"""Record UR telemetry over RTDE (read-only, port 30004) to CSV while a skill step runs.

Records, per sample: wall time, controller time, actual and target TCP pose, measured TCP
force (actual_TCP_force), the active TCP offset, runtime/safety state, and two values the
URScript writes so they line up with the motion:
  output_double_register_24 = force limit in use [N]   write_output_float_register(24, fmax)
  output_double_register_25 = phase code               write_output_float_register(25, k)
Phase codes: 0 idle, 1 approach, 2 contact, 3 search, 4 engage, 5 wiggle, 6 seat, 7 back-off,
8 extract, 9 touch.

Never sends anything that moves the robot (RTDE outputs only, no inputs).

  python -m urcore.telemetry                  # until Ctrl-C / SIGTERM, 125 Hz, into the run dir
  python -m urcore.telemetry --hz 250 --out data/tel.csv --seconds 60
"""
import argparse
import csv
import signal
import socket
import struct
import sys
import time
from datetime import datetime
from pathlib import Path

HOST, PORT = "192.168.2.12", 30004
ROOT = Path(__file__).resolve().parent.parent
FIELDS = ["timestamp", "actual_TCP_pose", "target_TCP_pose", "actual_TCP_force", "tcp_offset",
          "runtime_state", "safety_mode", "output_double_register_24", "output_double_register_25"]
FMT = {"DOUBLE": "d", "VECTOR6D": "6d", "VECTOR3D": "3d", "UINT64": "Q", "UINT32": "I",
       "INT32": "i", "BOOL": "?", "UINT8": "B", "VECTOR6INT32": "6i", "VECTOR6UINT32": "6I"}
V, O, S, P, U, M = (ord(c) for c in "VOSPUM")


class RTDE:
    def __init__(self, host=HOST, port=PORT):
        self.s = socket.create_connection((host, port), timeout=5)
        self.buf = b""

    def send(self, cmd, payload=b""):
        self.s.sendall(struct.pack(">HB", len(payload) + 3, cmd) + payload)

    def recv(self):
        while True:
            while len(self.buf) < 3:
                self.buf += self.s.recv(4096)
            size, cmd = struct.unpack(">HB", self.buf[:3])
            while len(self.buf) < size:
                self.buf += self.s.recv(4096)
            pkt, self.buf = self.buf[3:size], self.buf[size:]
            if cmd != M:                      # skip controller text messages
                return cmd, pkt

    def setup(self, fields, hz):
        self.send(V, struct.pack(">H", 2))
        cmd, pkt = self.recv()
        if cmd != V or not pkt[0]:
            sys.exit("RTDE protocol v2 not accepted")
        self.send(O, struct.pack(">d", hz) + ",".join(fields).encode())
        cmd, pkt = self.recv()
        self.recipe = pkt[0]
        self.types = pkt[1:].decode().split(",")
        bad = [f for f, t in zip(fields, self.types) if t not in FMT]
        if bad:
            sys.exit(f"RTDE: not available: {bad} ({self.types})")
        self.fmt = ">B" + "".join(FMT[t] for t in self.types)
        self.send(S)
        cmd, pkt = self.recv()
        if cmd != S or not pkt[0]:
            sys.exit("RTDE start refused")

    def sample(self):
        while True:
            cmd, pkt = self.recv()
            if cmd == U:
                vals = struct.unpack(self.fmt, pkt)
                return vals[1:] if vals[0] == self.recipe else None

    def close(self):
        try:
            self.send(P)
        finally:
            self.s.close()


def header():
    cols = ["wall_time", "t_ctrl"]
    for name in ("act", "tgt"):
        cols += [f"{name}_{a}" for a in ("x", "y", "z", "rx", "ry", "rz")]
    cols += [f"F_{a}" for a in ("x", "y", "z", "tx", "ty", "tz")]
    cols += [f"tcp_{a}" for a in ("x", "y", "z", "rx", "ry", "rz")]
    return cols + ["runtime_state", "safety_mode", "force_limit", "phase"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hz", type=float, default=125.0)
    ap.add_argument("--out")
    ap.add_argument("--seconds", type=float, default=0, help="0 = until stopped")
    a = ap.parse_args()
    run = ROOT / "data" / "runs" / "current"
    out = Path(a.out) if a.out else (run if run.exists() else ROOT / "data") / f"telemetry_{datetime.now():%H%M%S}.csv"

    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    r = RTDE()
    r.setup(FIELDS, a.hz)
    n, t0 = 0, time.time()
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header())
        while not stop and (not a.seconds or time.time() - t0 < a.seconds):
            v = r.sample()
            if v is None:
                continue
            ts, act, tgt, force, tcp, rs, sm, flim, phase = v[0], v[1:7], v[7:13], v[13:19], v[19:25], *v[25:]
            w.writerow([f"{time.time():.4f}", f"{ts:.4f}", *(f"{x:.6f}" for x in act + tgt),
                        *(f"{x:.3f}" for x in force), *(f"{x:.5f}" for x in tcp), rs, sm,
                        f"{flim:.2f}", f"{phase:.0f}"])
            n += 1
            if n % 250 == 0:
                f.flush()
    r.close()
    print(f"telemetry: {n} samples, {time.time() - t0:.1f} s -> {out}")


if __name__ == "__main__":
    main()
