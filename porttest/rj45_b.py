"""One RJ45 port with Option B (skill_b): same moves, speeds and forces as the recorded port-1
run 20261025-202013, but one upload per session and one play per step.

  python -m porttest.rj45_b port 1          # touch refs -> insert -> link -> extract -> lift
  python -m porttest.rj45_b port 1 --no-upload

Steps (each = one play): touch_reference_step (approach + face/top/left touches),
insert_step (approach, local face, search, engage, wiggle, seat_x), extract_step (pull, link
drop, lift). Run log + telemetry as with the skill; timings in data/skill_b_timing.jsonl.
"""
import json
import subprocess
import sys
import time
import urllib.request

import yaml

from porttest import skilllib
from porttest.go_view import W3_REF
from porttest.skill_b import SkillB
from porttest.urexec import ROOT, BridgeError, call

PY = sys.executable
ROT = [-2.2214, 2.2214, 0.0]
nY, nA, nB = [0, 1, 0], [1, 0, 0], [0, 0, 1]
PROD = yaml.safe_load((ROOT / "configs" / "products" / "ls108g.yaml").read_text())
# free-space (no contact) motion: default = the recorded run's speeds; --fast = just under the
# 250 mm/s tool speed limit of the safety config (exactly at the limit risks a safety stop)
SPEED = {"joint_v": 0.2, "joint_a": 0.4, "lin_v": 0.03, "lin_a": 0.2, "near_v": 0.01, "near_a": 0.1}
FAST = {"joint_v": 0.5, "joint_a": 1.0, "lin_v": 0.24, "lin_a": 0.8, "near_v": 0.24, "near_a": 0.8}


def runlog(*args):
    subprocess.run([PY, "-m", "porttest.runlog", *args], cwd=ROOT, stdout=subprocess.DEVNULL, check=False)


def qa(port, phase):
    req = urllib.request.Request(f"http://localhost:8090/api/qa/{port}/{phase}", method="POST")
    try:
        return json.loads(urllib.request.urlopen(req, timeout=30).read())
    except Exception as e:  # noqa: BLE001
        return {"ok": False, "reason": str(e)[:200]}


def tip(x, y, z):
    return [x, y, z] + ROT


def checked_movej(s, pose, v=0.2, a=0.4):
    """IK on the controller (no motion), the move_checked limits, then movej."""
    q, _ = s.ik(pose)
    problems = []
    if abs(q[2]) < 0.5:
        problems.append(f"elbow {q[2]:.2f} rad too straight")
    if abs(q[5] - W3_REF) > 0.8:
        problems.append(f"wrist3 {q[5]:.2f} vs ref {W3_REF:.2f}")
    if pose[2] < 0.10:
        problems.append(f"approach tip z {pose[2]:.3f} m below 10 cm")
    if problems:
        raise BridgeError("move rejected: " + "; ".join(problems))
    return s.movej(q, a=a, v=v)


def touch_refs(s, spot):
    """= p4s5.script: creep to the face, contact, top-edge slide, left-end slide twice."""
    checked_movej(s, tip(spot[0], spot[1], spot[2] + 0.10), v=SPEED["joint_v"], a=SPEED["joint_a"])
    s.movel(tip(*spot), a=SPEED["lin_a"], v=SPEED["lin_v"])
    h, _ = s.creep(nY, 0.025, 0.002, 4.0, 0.004)
    if not h:
        raise BridgeError("touch: no face within 25 mm")
    s.movel_rel([0, -0.001, 0], a=0.1, v=0.005)
    face, _ = s.contact(nY, 3.5)
    s.back_off(nY, 0.0015)
    _, s1 = s.tel(0, 9)
    top_ok, ptop = s.slide(nY, 3.5, nB, 0.022, 0.0015, 0.002)
    s.movel([ptop[0], s1[1] - 0.001, ptop[2]] + ROT, a=0.1, v=0.005)
    s.movel([s1[0], s1[1] - 0.001, s1[2]] + ROT, a=0.1, v=0.01)
    s.movel(s1, a=0.1, v=0.005)
    lefts = []
    for i in range(2):
        ok, pl = s.slide(nY, 3.5, [-1, 0, 0], 0.025, 0.0015, 0.002)
        lefts.append(pl[0] if ok else None)
        s.movel([pl[0], s1[1] - 0.001, pl[2]] + ROT, a=0.1, v=0.005)
        back = 0.001 if i == 0 else 0.010
        s.movel([s1[0], s1[1] - back, s1[2]] + ROT, a=0.1, v=0.01)
        if i == 0:
            s.movel(s1, a=0.1, v=0.005)
    if not top_ok or None in lefts:
        raise BridgeError(f"touch: top {top_ok}, left {lefts}")
    return {"face_y": face, "top_pass_z": ptop[2], "left_pass_x": sum(lefts) / 2, "left_spread": abs(lefts[0] - lefts[1])}


LAST = ROOT / "data" / "last_refs.json"


def touch_refs_short(s, prev):
    """Short touches from the previous run's references (same forces/speeds as touch_refs):
    descend 6 mm in front of the previous face, creep (<= 12 mm), contact; top-edge slide from
    4 mm below the expected pass (<= 8 mm); ONE left-end slide from 4 mm right of the expected
    pass (<= 8 mm). Falls back to the long slide from the face spot when a short one finds nothing."""
    spot = [prev["left_pass_x"] + 0.020, prev["face_y"] - 0.006, prev["top_pass_z"] - 0.0139]
    checked_movej(s, tip(spot[0], spot[1], spot[2] + 0.10), v=SPEED["joint_v"], a=SPEED["joint_a"])
    s.movel(tip(*spot), a=SPEED["lin_a"], v=SPEED["lin_v"])
    h, _ = s.creep(nY, 0.012, 0.002, 4.0, 0.004)
    if not h:
        raise BridgeError("short touch: no face within 12 mm of the previous face - use the long touch")
    s.movel_rel([0, -0.001, 0], a=0.1, v=0.005)
    face, _ = s.contact(nY, 3.5)
    s.back_off(nY, 0.0015)
    _, s1 = s.tel(0, 9)
    fallback = []
    # top edge
    s.movel([s1[0], s1[1], prev["top_pass_z"] - 0.004] + ROT, a=0.1, v=0.01)
    top_ok, ptop = s.slide(nY, 3.5, nB, 0.008, 0.0015, 0.002)
    s.movel([ptop[0], s1[1] - 0.001, ptop[2]] + ROT, a=0.1, v=0.005)
    s.movel([s1[0], s1[1] - 0.001, s1[2]] + ROT, a=0.1, v=0.01)
    if not top_ok:
        fallback.append("top")
        s.movel(s1, a=0.1, v=0.005)
        top_ok, ptop = s.slide(nY, 3.5, nB, 0.022, 0.0015, 0.002)
        s.movel([ptop[0], s1[1] - 0.001, ptop[2]] + ROT, a=0.1, v=0.005)
        s.movel([s1[0], s1[1] - 0.001, s1[2]] + ROT, a=0.1, v=0.01)
    # left end (one slide)
    s.movel([prev["left_pass_x"] + 0.004, s1[1] - 0.001, s1[2]] + ROT, a=0.1, v=0.01)
    s.movel([prev["left_pass_x"] + 0.004, s1[1], s1[2]] + ROT, a=0.1, v=0.005)
    left_ok, pl = s.slide(nY, 3.5, [-1, 0, 0], 0.008, 0.0015, 0.002)
    s.movel([pl[0], s1[1] - 0.001, pl[2]] + ROT, a=0.1, v=0.005)
    if not left_ok:
        fallback.append("left")
        s.movel([s1[0], s1[1] - 0.001, s1[2]] + ROT, a=0.1, v=0.01)
        s.movel(s1, a=0.1, v=0.005)
        left_ok, pl = s.slide(nY, 3.5, [-1, 0, 0], 0.025, 0.0015, 0.002)
        s.movel([pl[0], s1[1] - 0.001, pl[2]] + ROT, a=0.1, v=0.005)
    s.movel([s1[0], s1[1] - 0.010, s1[2]] + ROT, a=0.1, v=0.01)
    if not (top_ok and left_ok):
        raise BridgeError(f"short touch: top {top_ok}, left {left_ok} even after fallback")
    return {"face_y": face, "top_pass_z": ptop[2], "left_pass_x": pl[0], "left_spread": 0.0,
            "fallback": fallback, "spot": spot}


def save_refs(refs):
    LAST.write_text(json.dumps({k: refs[k] for k in ("face_y", "top_pass_z", "left_pass_x")} |
                               {"stamp": time.strftime("%Y-%m-%dT%H:%M:%S")}))


def touch_check():
    """Validation, no insertion: short touches, then the long touches, in one play; compare."""
    prev = json.loads(LAST.read_text())
    sk = SkillB()
    sk.upload()
    t = time.time()
    with sk.session() as s:
        t1 = time.time()
        short = touch_refs_short(s, prev)
        t2 = time.time()
        s.movel_rel([0, -0.005, 0], a=0.1, v=0.01)      # straight away from the face, then up,
        s.movel_rel([0, 0, 0.100], a=SPEED["lin_a"], v=SPEED["lin_v"])   # before any joint move
        spot = [short["left_pass_x"] + 0.0179 + 0.0021, short["face_y"] - 0.0158, short["top_pass_z"] - 0.0139]
        long = touch_refs(s, spot)
        t3 = time.time()
    d = {k: round((short[k] - long[k]) * 1000, 3) for k in ("face_y", "top_pass_z", "left_pass_x")}
    print(json.dumps({"prev": prev, "short": {k: round(v * 1000, 2) for k, v in short.items() if isinstance(v, float)},
                      "long": {k: round(v * 1000, 2) for k, v in long.items() if isinstance(v, float)},
                      "short_minus_long_mm": d, "fallback": short["fallback"],
                      "short_s": round(t2 - t1, 1), "long_s": round(t3 - t2, 1), "play_total_s": round(time.time() - t, 1)}, indent=1))
    ok = all(abs(v) <= 0.3 for v in d.values())
    print("VALIDATION", "PASS" if ok else "CHECK", "(|short - long| <= 0.3 mm on face, top, left)")
    if ok:
        save_refs(short)


def lib_report(s, extra=None):
    """Check this step's trace records against the skill library; log every match."""
    hits = skilllib.check(s.records, extra=extra)
    for h in hits:
        runlog("note", "observed", skilllib.describe(h)[:480])
    return {h[0]["id"] for h in hits}


def port_target(refs, n):
    dx = PROD.get("measured_from_left_x", {}).get(n, PROD["port1_from_left_pass_x"] + (n - 1) * 0.0150)
    dz = PROD.get("measured_below_top_z", {}).get(n, PROD["port1_below_top_pass_z"])
    return refs["left_pass_x"] + dx, refs["top_pass_z"] - dz


def insert(s, X, Z, refs):
    """= p1r.script: approach, local face above the port, search, engage, wiggle, seat_x."""
    yap = refs["face_y"] - 0.016
    s.tel(0, 1)
    _, q = s.tel(0, 1)
    s.movel(tip(q[0], yap, q[2]), a=SPEED["near_a"], v=SPEED["near_v"])
    s.movel(tip(X, yap, refs["top_pass_z"] - 0.0065), a=SPEED["near_a"], v=SPEED["near_v"])
    h, _ = s.creep(nY, 0.025, 0.002, 4.0, 0.004)
    if not h:
        raise BridgeError("insert: no local face within 25 mm")
    s.movel_rel([0, -0.001, 0], a=0.1, v=0.005)
    face, _ = s.contact(nY, 3.5)
    s.back_off(nY, 0.0015)
    s.tel(0, 1)
    s.movel(tip(X, face - 0.0015, Z), a=0.1, v=0.005)
    tries = []
    for attempt in range(2):
        if attempt:
            s.movel(tip(X, face - 0.0015, Z), a=0.1, v=0.005)   # retry from the computed target
        found, cand = s.search_port(nY, 4.0, face, 0.0008, nA, nB, 0.0004, 0.0015, 0.0015, 0.002)
        if not found:
            tries.append("no opening")
            s.back_off(nY, 0.0015)
            continue
        # z stiff, x compliant: the x/z-compliant engage let the tip climb 3.8-3.9 mm up an edge
        # and jam (port 2 10-25 21:10; port 1 10-26 11:23)
        e, pe = s.engage_x(nY, 11.0, face, 0.0045, 3.0)
        w, _ = s.wiggle(nA, nB, 0.0008, 4.0)
        c, _ = s.adv_along(nY, face)
        tries.append(f"cand x {cand[0]*1000:.2f} z {cand[2]*1000:.2f}, engaged {e*1000:.2f} mm, wiggle {w}, past {c*1000:.2f} mm")
        if w or c > 0.004:
            break
        s.back_off(nY, 0.002)
    else:
        return {"seated": False, "face": face, "reason": "false hit / not found", "tries": tries}
    dy, ps = s.seat_x(nY, 18.0, 0.013 - c, 5.0)
    past, _ = s.adv_along(nY, face)
    s.tel(0, 0)
    return {"seated": past > 0.010, "face": face, "candidate": cand[:3], "past_face": past, "seat_pose": ps[:3],
            "tries": tries}


def extract(s):
    """= p4s8.script + up: guarded pull 22 mm, then back 10 mm and lift 100 mm."""
    s.tel(30, 8)
    blocked, _ = s.guarded_axial([0, -1, 0], 0.022, 30.0, 0.002)
    s.tel(0, 0)
    return blocked


def main(port, upload=True, fast=False, short=False):
    if fast:
        SPEED.update(FAST)
    t_run = time.time()
    tag = "-B-fast" if fast else "-B"
    runlog("start", f"rj45-port{port}{tag}", "--goal", f"Option B{' fast' if fast else ''}: seat port {port}, "
           f"link 1000/full; free-space speeds {SPEED}")
    run_dir = ROOT / "data" / "runs" / "current"
    tel = subprocess.Popen([PY, "-m", "porttest.telemetry"], cwd=ROOT, stdout=open(run_dir / "telemetry.log", "w"))
    steps = {}
    try:
        runlog("step", "S1", "prep_step", "--intent", "bridge session, upload skill program once, preflight")
        t = time.time()
        sk = SkillB()
        st = call(sk.ex.b, "bridge_my_status")["data"]
        rs = call(sk.ex.b, "robot_get_state")["data"]
        if st.get("tier") != "move" or rs.get("safetymode") != "NORMAL":
            raise BridgeError(f"preflight: tier {st.get('tier')}, safety {rs.get('safetymode')}")
        if upload:
            sk.upload()
        steps["prep"] = time.time() - t
        for st_name in ("touch_reference_step", "insert_step"):
            ids = [f"{e['id']}({e.get('status', '?')[0]})" for e in skilllib.for_step(st_name)]
            runlog("note", "interpretation", f"library read for {st_name}: {', '.join(ids)}")
        runlog("done", "--status", "success", "--summary", f"tier move, NORMAL, uploaded={upload}")

        runlog("step", "S5", "touch_reference_step", "--intent", "one play: approach + face/top/left touches")
        t = time.time()
        r1 = PROD["touch"]
        spot = [r1["left_end_pass_x"] + 0.0179, r1["face_y"] - 0.0118, r1["top_pass_z"] - 0.0139]
        prev = json.loads(LAST.read_text()) if LAST.exists() else None
        with sk.session(step="touch_reference_step") as s:
            refs = touch_refs_short(s, prev) if short and prev else touch_refs(s, spot)
        moved = {k: round((refs[k] - prev[k]) * 1000, 2) for k in ("face_y", "top_pass_z", "left_pass_x")} if prev else {}
        hits = lib_report(s, extra={"refs_vs_last_mm": max(map(abs, moved.values())) if moved else None})
        if "switch-creep" in hits:
            msg = (f"OPERATOR: the switch moved since the last run (face {moved['face_y']:+.2f}, top {moved['top_pass_z']:+.2f}, "
                   f"left end {moved['left_pass_x']:+.2f} mm). The seats push the taped switch; proposal: a printed corner "
                   "stop / bracket against its back and left side, or clamps instead of tape. Continuing with fresh touches.")
            print(msg)
            runlog("note", "interpretation", msg)
        save_refs(refs)
        steps["touch"] = time.time() - t
        runlog("note", "observed", f"B touches: face {refs['face_y']*1000:.2f}, top {refs['top_pass_z']*1000:.2f}, "
               f"left {refs['left_pass_x']*1000:.2f} (spread {refs['left_spread']*1000:.2f}) mm; "
               f"play {sk.last_timing['connect']} s to connect")
        runlog("done", "--status", "success", "--summary", "references measured")

        X, Z = port_target(refs, port)
        runlog("step", "S7", "insert_step", "--intent", f"one play: port {port} at x {X*1000:.2f} z {Z*1000:.2f}")
        t = time.time()
        with sk.session(step="insert_step") as s:
            res = insert(s, X, Z, refs)
        lib_report(s)
        link = qa(port, "insert") if res["seated"] else {"ok": False, "reason": "not seated"}
        steps["insert"] = time.time() - t
        runlog("note", "observed", f"B insert: {json.dumps(res)[:300]}; link {link.get('ok')} {link.get('reason', '')}")
        runlog("done", "--status", "success" if link.get("ok") else "failure",
               "--summary", f"port {port} seated={res['seated']} link={link.get('ok')}")

        runlog("step", "S8", "extract_step", "--intent", "one play: guarded pull, link drop, lift")
        t = time.time()
        with sk.session(step="extract_step") as s:
            blocked = extract(s)
            drop = qa(port, "extract")
            s.movel_rel([0, -0.010, 0], a=SPEED["near_a"], v=SPEED["near_v"])
            s.movel_rel([0, 0, 0.100], a=SPEED["lin_a"], v=SPEED["lin_v"])
        steps["extract"] = time.time() - t
        runlog("note", "observed", f"B extract: blocked {blocked}, link drop {drop.get('ok')} {drop.get('reason', '')}")
        runlog("done", "--status", "success", "--summary", "extracted and lifted")
        ok = bool(link.get("ok"))
        runlog("end", "--status", "success" if ok else "failure",
               "--summary", f"Option B port {port}: link {link.get('ok')}; steps " +
               ", ".join(f"{k} {v:.1f}s" for k, v in steps.items()) + f"; total {time.time() - t_run:.1f}s")
    except Exception as e:  # noqa: BLE001
        runlog("note", "observed", f"B run aborted: {e}")
        runlog("end", "--status", "aborted", "--summary", str(e)[:200])
        raise
    finally:
        tel.terminate()
        tel.wait(timeout=10)
        runlog("trace")
        print(json.dumps({"steps_s": {k: round(v, 1) for k, v in steps.items()}, "total_s": round(time.time() - t_run, 1)}))


if __name__ == "__main__":
    if sys.argv[1] == "touchcheck":
        touch_check()
    elif sys.argv[1] == "port":
        main(int(sys.argv[2]), upload="--no-upload" not in sys.argv, fast="--fast" in sys.argv,
             short="--short" in sys.argv)
