"""Option B: one fixed skill program per session, commands over a socket, one play per step.

The agent still decides every step and every step starts with a play through the bridge
(move tier + Remote Control gate). What changes: the program is uploaded ONCE per session
(program_update_node costs ~9 s, measured 2026-10-26) and each play then runs a short
sequence of primitives that the Jetson sends over a TCP socket, with results sent back.

  python -m porttest.skill_b upload          # one-time per session (or after another edit)
  python -m porttest.skill_b noop [n]        # no-motion test: n plays of harmless commands

Python:
  from porttest.skill_b import SkillB
  sk = SkillB()                               # bridge session + command server
  with sk.session() as s:                     # = one play
      face = s.contact([0, 1, 0], 3.5)        # each call = one primitive on the robot
      ...
Results come back as (value, tip_pose). Timing per session -> data/skill_b_timing.jsonl.
"""
import json
import socket
import sys
import threading
import time
from contextlib import contextmanager

from porttest.urexec import APP, PREAMBLE, ROOT, BridgeError, URExec, call
from porttest.urscript_lint import lint

HOST, PORT = "192.168.2.250", 50101       # the Jetson on the robot network
VERSION = 1.2                             # bump when the dispatcher or urlib changes
NP = 16                                   # parameters per command (plus the opcode)
OPS = {"end": 0, "movel": 1, "movej": 2, "movel_rel": 3, "creep": 4, "contact": 5, "back_off": 6,
       "slide": 7, "search_port": 8, "engage": 9, "wiggle": 10, "seat_x": 11, "guarded_axial": 12,
       "release": 13, "tel": 14, "adv_along": 15, "force": 16, "sleep": 17, "ik": 18,
       "end_force_mode": 19, "engage_x": 20}

DISPATCH = f'''
def sk_ret(v):
  socket_send_line(str_cat(str_cat(str_cat("R ", v), str_cat(" | ", get_actual_tcp_pose())), str_cat(" | ", get_tcp_force())), "sk")
end

def v3(c, i):
  return [c[i], c[i + 1], c[i + 2]]
end

ok = socket_open("{HOST}", {PORT}, "sk")
if not ok:
  textmsg("skill_b: socket_open failed")
  halt
end
socket_send_line(str_cat("HELLO ", {VERSION}), "sk")
running = True
while running:
  c = socket_read_ascii_float({NP + 1}, "sk", 60)
  if c[0] != {NP + 1}:
    textmsg(str_cat("skill_b: no command (read ", str_cat(c[0], ")")))
    running = False
  else:
    op = c[1]
    if op == 0:
      running = False
    elif op == 1:
      movel(p[c[2], c[3], c[4], c[5], c[6], c[7]], a=c[8], v=c[9])
      sk_ret(1)
    elif op == 2:
      movej([c[2], c[3], c[4], c[5], c[6], c[7]], a=c[8], v=c[9])
      sk_ret(1)
    elif op == 3:
      movel(pose_add(get_actual_tcp_pose(), p[c[2], c[3], c[4], 0, 0, 0]), a=c[5], v=c[6])
      sk_ret(1)
    elif op == 4:
      sk_ret(creep(v3(c, 2), c[5], c[6], c[7], c[8]))
    elif op == 5:
      sk_ret(contact(v3(c, 2), c[5]))
    elif op == 6:
      back_off(v3(c, 2), c[5])
      sk_ret(1)
    elif op == 7:
      sk_ret(slide_until_advance(v3(c, 2), c[5], v3(c, 6), c[9], c[10], c[11]))
    elif op == 8:
      sk_ret(search_port(v3(c, 2), c[5], c[6], c[7], v3(c, 8), v3(c, 11), c[14], c[15], c[16], c[17]))
    elif op == 9:
      sk_ret(engage(v3(c, 2), c[5], c[6], c[7], c[8]))
    elif op == 10:
      sk_ret(wiggle_test(v3(c, 2), v3(c, 5), c[8], c[9]))
    elif op == 11:
      sk_ret(seat_x(v3(c, 2), c[5], c[6], c[7]))
    elif op == 12:
      sk_ret(guarded_axial(v3(c, 2), c[5], c[6], c[7]))
    elif op == 13:
      release(v3(c, 2), c[5], c[6])
      sk_ret(1)
    elif op == 14:
      tel(c[2], c[3])
      sk_ret(1)
    elif op == 15:
      sk_ret(adv_along(v3(c, 2), c[5]))
    elif op == 16:
      sk_ret(get_tcp_force())
    elif op == 17:
      sleep(c[2])
      sk_ret(1)
    elif op == 18:
      sk_ret(get_inverse_kin(p[c[2], c[3], c[4], c[5], c[6], c[7]], qnear=get_actual_joint_positions()))
    elif op == 19:
      end_force_mode()
      sk_ret(1)
    elif op == 20:
      sk_ret(engage_x(v3(c, 2), c[5], c[6], c[7], c[8]))
    else:
      textmsg(str_cat("skill_b: unknown op ", op))
      running = False
    end
  end
end
end_force_mode()
stopl(0.5)
socket_send_line("BYE", "sk")
socket_close("sk")
'''


def skill_program():
    # PREAMBLE = payload + set_tcp(tip) + urlib; urexec.run appends tel(0,0) + set_tcp(flange)
    return DISPATCH


def _parse(v):
    v = v.strip()
    if v in ("True", "False"):
        return v == "True"
    if v.startswith("p[") or v.startswith("["):
        return [float(x) for x in v.lstrip("p[").rstrip("]").split(",") if x.strip()]
    try:
        return float(v)
    except ValueError:
        return v


TRACE_FIELDS = ("step", "cmd", "params", "result", "pose0", "pose1", "d_mm", "f_end", "t0", "dur")


class Session:
    """One play. Every command appends a trace record (step, cmd, params, result, tip pose
    before/after, displacement in mm, TCP force at the end, wall time, duration) to
    data/runs/current/trace.jsonl (or data/trace.jsonl outside a run)."""

    def __init__(self, conn, timing, step="", trace_path=None):
        self.f = conn.makefile("rw", newline="\n")
        self.timing = timing
        self.step = step
        self.trace_path = trace_path
        self.records = []
        self.pose = None

    def cmd(self, name, *params, timeout=120.0):
        vals = [float(x) for x in params]
        if len(vals) > NP:
            raise ValueError(f"{name}: {len(vals)} params > {NP}")
        vals += [0.0] * (NP - len(vals))
        t, tw = time.monotonic(), time.time()
        self.f.write("(" + ",".join([str(OPS[name])] + [f"{x:.6f}" for x in vals]) + ")\n")
        self.f.flush()
        line = self.f.readline()
        if not line.startswith("R "):
            raise BridgeError(f"skill_b {name}: unexpected reply {line!r}")
        parts = line[2:].strip().split(" | ")
        val, pose = _parse(parts[0]), _parse(parts[1])
        force = _parse(parts[2]) if len(parts) > 2 else None
        dur = round(time.monotonic() - t, 3)
        self.timing["cmds"].append((name, dur))
        p0 = self.pose
        rec = {"step": self.step, "cmd": name, "params": [round(x, 6) for x in params], "result": val,
               "pose0": p0[:3] if p0 else None, "pose1": pose[:3] if isinstance(pose, list) else None,
               "d_mm": [round((b - a) * 1000, 3) for a, b in zip(p0[:3], pose[:3])] if p0 and isinstance(pose, list) else None,
               "f_end": [round(x, 2) for x in force[:3]] if isinstance(force, list) else None,
               "t0": round(tw, 3), "dur": dur}
        self.records.append(rec)
        if self.trace_path:
            with open(self.trace_path, "a") as fh:
                fh.write(json.dumps(rec) + "\n")
        self.pose = pose if isinstance(pose, list) else self.pose
        return val, pose

    # primitives (same arguments as urlib; vectors are lists of 3)
    def movel(self, pose, a=0.2, v=0.05):
        return self.cmd("movel", *pose, a, v)

    def movej(self, q, a=0.5, v=0.3):
        return self.cmd("movej", *q, a, v)

    def movel_rel(self, d, a=0.2, v=0.03):
        return self.cmd("movel_rel", *d, a, v)

    def creep(self, d, maxd, step, fmax, v):
        return self.cmd("creep", *d, maxd, step, fmax, v)

    def contact(self, n, fn):
        return self.cmd("contact", *n, fn)

    def back_off(self, n, dist):
        return self.cmd("back_off", *n, dist)

    def slide(self, n, fn, s, maxd, adv, v):
        return self.cmd("slide", *n, fn, *s, maxd, adv, v)

    def search_port(self, n, fn, face, ain, a, b, pitch, rmax, adv, v):
        return self.cmd("search_port", *n, fn, face, ain, *a, *b, pitch, rmax, adv, v)

    def engage(self, n, fe, face, depth, tmax):
        return self.cmd("engage", *n, fe, face, depth, tmax)

    def engage_x(self, n, fe, face, depth, tmax):
        return self.cmd("engage_x", *n, fe, face, depth, tmax)

    def wiggle(self, a, b, amp, fthr):
        return self.cmd("wiggle", *a, *b, amp, fthr)

    def seat_x(self, n, fy, depth, tmax):
        return self.cmd("seat_x", *n, fy, depth, tmax)

    def guarded_axial(self, d, maxd, fmax, v):
        return self.cmd("guarded_axial", *d, maxd, fmax, v)

    def release(self, n, fmax, maxd):
        return self.cmd("release", *n, fmax, maxd)

    def tel(self, flim, phase):
        return self.cmd("tel", flim, phase)

    def adv_along(self, n, ref):
        return self.cmd("adv_along", *n, ref)

    def force(self):
        return self.cmd("force")

    def sleep(self, s):
        return self.cmd("sleep", s)

    def ik(self, pose):
        return self.cmd("ik", *pose)

    def end_force_mode(self):
        return self.cmd("end_force_mode")


class SkillB:
    def __init__(self, ex=None):
        self.ex = ex or URExec()
        self.srv = socket.socket()
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("0.0.0.0", PORT))
        self.srv.listen(1)

    def upload(self):
        """The one program edit per session (~9 s). Lint runs inside urexec.run; the upload is
        verified by a no-motion session that must answer HELLO with this VERSION."""
        expr = PREAMBLE + skill_program() + "\ntel(0, 0)\nset_tcp(p[0, 0, 0, 0, 0, 0])\n"
        problems = lint(expr)
        if problems:
            raise BridgeError("urscript_lint: " + "; ".join(problems))
        t = time.monotonic()
        call(self.ex.b, "program_update_node", appId=self.ex.app, programId=self.ex.app, guid=self.ex.guid,
             parameters={"expression": expr}, confirm=True)
        print(f"skill_b uploaded ({len(expr)} chars) in {time.monotonic() - t:.1f} s")

    def _play(self, out):
        try:
            if str(call(self.ex.b, "program_get_loaded")["data"].get("appId")) != str(self.ex.app):
                call(self.ex.b, "program_load", appId=self.ex.app, confirm=True)
                time.sleep(2)
            for attempt in range(4):
                try:
                    call(self.ex.b, "program_run", action="play", confirm=True)
                    out.append("ok")
                    return
                except BridgeError as err:
                    if "program_not_ready" in str(err) and attempt < 3:
                        time.sleep(2)
                        continue
                    out.append("timeout" if "timeout" in str(err) else f"error: {err}")
                    return
        except Exception as e:  # noqa: BLE001
            out.append(f"error: {e}")

    @contextmanager
    def session(self, connect_timeout=20.0, step=""):
        """One play of the skill program; yields a Session for any number of commands."""
        timing = {"wall": time.time(), "cmds": []}
        t0 = time.monotonic()
        play = []
        th = threading.Thread(target=self._play, args=(play,), daemon=True)
        th.start()
        self.srv.settimeout(connect_timeout)
        try:
            conn, _ = self.srv.accept()
        except socket.timeout:
            raise BridgeError(f"skill_b: robot did not connect within {connect_timeout} s (play: {play}) "
                              "- program not uploaded, not loaded, or not playing")
        conn.settimeout(120)
        timing["connect"] = round(time.monotonic() - t0, 3)
        run = ROOT / "data" / "runs" / "current"
        s = Session(conn, timing, step=step,
                    trace_path=(run if step and run.exists() else ROOT / "data") / "trace.jsonl")
        hello = s.f.readline().strip()
        if hello != f"HELLO {VERSION}" and not hello.startswith(f"HELLO {VERSION:g}"):
            s.f.write(f"({OPS['end']}" + ",0" * NP + ")\n")
            s.f.flush()
            conn.close()
            raise BridgeError(f"skill_b: robot runs another dispatcher version ({hello!r}); upload first")
        try:
            yield s
        finally:
            t = time.monotonic()
            try:
                s.f.write(f"({OPS['end']}" + ",0" * NP + ")\n")
                s.f.flush()
                bye = s.f.readline().strip()
            except OSError:
                bye = "lost"
            conn.close()
            th.join(timeout=15)
            timing.update({"end": round(time.monotonic() - t, 3), "bye": bye, "play": play,
                           "total": round(time.monotonic() - t0, 3)})
            with open(ROOT / "data" / "skill_b_timing.jsonl", "a") as f:
                f.write(json.dumps(timing) + "\n")
            self.last_timing = timing


def noop_test(n=3):
    sk = SkillB()
    sk.upload()
    for i in range(n):
        with sk.session() as s:
            r1 = s.tel(0, 0)
            r2 = s.force()
            r3 = s.sleep(0.2)
            r4 = s.ik(s.cmd("tel", 0, 0)[1])     # IK of the current tip pose: no motion
        t = sk.last_timing
        print(f"play {i}: connect {t['connect']} s, cmds {t['cmds']}, end {t['end']} s, total {t['total']} s, "
              f"play={t['play']}, bye={t['bye']}")
        print(f"   force {[round(x, 2) for x in r2[0]]}  ik {[round(x, 3) for x in r4[0]]}")


if __name__ == "__main__":
    if sys.argv[1] == "upload":
        SkillB().upload()
    elif sys.argv[1] == "noop":
        noop_test(int(sys.argv[2]) if len(sys.argv) > 2 else 3)
