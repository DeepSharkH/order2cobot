"""CAD-guidance convergence run on the LS108G (Option B primitives, cadguide targets).

  python -m porttest.cad_run [--ports 8] [--fast]

One process, one skill upload, one play per step:
  1. short plug-tip touches (face, top-edge pass, left-end pass) -> `cadguide register --touch`
     (anchors reset): the prior. Port 1 sits on the touched end, the CAD gives the rest.
  2. per port: target = cadguide prediction (tip coordinates), spiral radius from its 3 sigma
     (1.5-3 mm); local face touch, search, engage_x (z stiff), wiggle, seat; link check;
     anchor the tip pose at the end of the engage; extract; lift.
  3. next port = the farthest unvisited one whose 3 sigma fits the spiral (march outward
     while the pitch is still loose, jump once it is known).

Seat = guarded_axial (position-controlled push along +y, stops at 20 N reaction) + release:
z cannot sink and the force cannot run away (the 10-25 21:33 seat_x force mode pressed 70 N).
Results: data/runs/current/cad_convergence.json (+ run log, telemetry, trace).
"""
import contextlib
import io
import json
import subprocess
import sys
import time

from porttest.rj45_b import (LAST, PY, SPEED, FAST, extract, nA, nB, nY, qa, runlog, save_refs, tip,
                             touch_refs, touch_refs_short)
from porttest.skill_b import SkillB
from porttest.urexec import ROOT, BridgeError, call

sys.path.insert(0, str(ROOT))
from cadguide import __main__ as cg  # noqa: E402
from cadguide import state as cgs  # noqa: E402
from cadguide.model import CadModel  # noqa: E402

OBJ = 'ls108g'
R_MIN, R_MAX = 0.0015, 0.003      # spiral radius clamp [m]
SEAT_F, SEAT_DEPTH = 20.0, 0.013  # guarded seat: reaction cap [N], target past the face [m]


def cad(*argv):
    """Run a cadguide CLI command in-process, return its stdout."""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        cg.main(list(argv))
    return buf.getvalue()


def predict(port):
    return cg.predict(CadModel(OBJ), cgs.load(OBJ), f'port:{port}')


def next_port(done, n_ports):
    """Farthest unvisited port (from the visited ones) whose 3 sigma fits the largest spiral."""
    if not done:
        return 1
    left = [k for k in range(1, n_ports + 1) if k not in done]
    ok = [k for k in left if predict(k)['search_budget_mm'] <= R_MAX * 1000]
    pool = ok or sorted(left, key=lambda k: predict(k)['search_budget_mm'])[:1]
    return max(pool, key=lambda k: min(abs(k - d) for d in done))


def insert_cad(s, X, Z, refs, rmax):
    """rj45_b.insert with the CAD target, a sigma-sized spiral, the engage pose kept as the
    anchor, and the guarded seat."""
    yap = refs["face_y"] - 0.016
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
    tries, pe = [], None
    for attempt in range(2):
        if attempt:
            s.movel(tip(X, face - 0.0015, Z), a=0.1, v=0.005)
        found, cand = s.search_port(nY, 4.0, face, 0.0008, nA, nB, 0.0004, rmax, rmax, 0.002)
        if not found:
            tries.append("no opening")
            s.back_off(nY, 0.0015)
            continue
        e, pe = s.engage_x(nY, 11.0, face, 0.0045, 3.0)
        w, _ = s.wiggle(nA, nB, 0.0008, 4.0)
        c, _ = s.adv_along(nY, face)
        tries.append(f"cand x {cand[0]*1000:.2f} z {cand[2]*1000:.2f}, engaged {e*1000:.2f} mm, wiggle {w}, "
                     f"past {c*1000:.2f} mm")
        if w or c > 0.004:
            break
        s.back_off(nY, 0.002)
    else:
        return {"seated": False, "face": face, "reason": "false hit / not found", "tries": tries}
    s.end_force_mode()
    s.tel(SEAT_F, 6)
    hit, ps = s.guarded_axial(nY, SEAT_DEPTH + 0.0005 - c, SEAT_F, 0.003)
    s.release(nY, 3.0, 0.001)
    past, _ = s.adv_along(nY, face)
    s.tel(0, 0)
    return {"seated": past > 0.010, "face": face, "candidate": cand[:3], "engage_pose": pe[:3],
            "seat_force_cap_hit": bool(hit), "past_face": past, "tries": tries}


def main(n_ports=8, fast=False):
    if fast:
        SPEED.update(FAST)
    t_run = time.time()
    runlog("start", "rj45-cad-convergence", "--goal",
           f"CAD-guidance convergence on the taped LS108G: ports 1..{n_ports}, cadguide targets, anchors")
    run_dir = (ROOT / "data" / "runs" / "current").resolve()
    tel = subprocess.Popen([PY, "-m", "porttest.telemetry"], cwd=ROOT, stdout=open(run_dir / "telemetry.log", "w"))
    out = {"run": run_dir.name, "ports": [], "speed": dict(SPEED)}
    try:
        runlog("step", "S1", "prep", "--intent", "bridge session, preflight, one skill upload")
        sk = SkillB()
        st = call(sk.ex.b, "bridge_my_status")["data"]
        rs = call(sk.ex.b, "robot_get_state")["data"]
        if st.get("tier") != "move" or rs.get("safetymode") != "NORMAL":
            raise BridgeError(f"preflight: tier {st.get('tier')}, safety {rs.get('safetymode')}")
        sk.upload()
        runlog("done", "--status", "success", "--summary", "tier move, NORMAL, uploaded")

        runlog("step", "S4", "touch registration", "--intent", "short touches -> cadguide register --touch")
        t = time.time()
        with sk.session() as s:
            refs = touch_refs_short(s, json.loads(LAST.read_text())) if LAST.exists() else None
            if refs is None:
                raise BridgeError("no data/last_refs.json for the short touches")
        save_refs(refs)
        cad('reset', OBJ, '--anchors')
        reg = cad('register', OBJ, '--touch', str(refs['face_y']), str(refs['top_pass_z']), str(refs['left_pass_x']))
        out["touch"] = {k: round(refs[k] * 1000, 3) for k in ("face_y", "top_pass_z", "left_pass_x")}
        out["touch_s"] = round(time.time() - t, 1)
        runlog("note", "observed", f"touches {out['touch']} mm in {out['touch_s']} s; {reg.strip().splitlines()[0]}")
        runlog("done", "--status", "success", "--summary", "registered from touches")

        done = []
        while len(done) < n_ports:
            port = next_port(done, n_ports)
            p = predict(port)
            X, Z = p["entry"][0], p["entry"][2]
            rmax = min(max(p["search_budget_mm"] / 1000, R_MIN), R_MAX)
            rec = {"port": port, "pred_mm": [round(v * 1000, 2) for v in p["entry"]],
                   "sigma_mm": p["sigma_mm"]["lateral"], "budget_mm": p["search_budget_mm"],
                   "spiral_mm": round(rmax * 1000, 2), "anchors_before": len([a for a in cgs.load(OBJ)['anchors']
                                                                              if a.get('used', True)])}
            runlog("step", f"P{port}", f"port {port}", "--intent",
                   f"CAD target x {X*1000:.2f} z {Z*1000:.2f}, sigma {rec['sigma_mm']} mm, spiral {rec['spiral_mm']} mm")
            t = time.time()
            with sk.session() as s:
                res = insert_cad(s, X, Z, refs, rmax)
            link = qa(port, "insert") if res["seated"] else {"ok": False, "reason": "not seated"}
            rec["insert_s"] = round(time.time() - t, 1)
            rec["seated"], rec["link"], rec["tries"] = res["seated"], bool(link.get("ok")), res["tries"]
            if res.get("engage_pose"):
                pe = res["engage_pose"]
                rec["engage_mm"] = [round(v * 1000, 2) for v in pe]
                rec["err_x_mm"] = round((pe[0] - X) * 1000, 2)
                rec["err_z_mm"] = round((pe[2] - Z) * 1000, 2)
                rec["past_face_mm"] = round(res["past_face"] * 1000, 2)
                rec["seat_force_cap_hit"] = res["seat_force_cap_hit"]
            if rec["link"]:
                a = cad('anchor', OBJ, f'port:{port}', *map(str, pe), '--run', run_dir.name,
                        '--note', 'engage end, link up')
                rec["anchor_gates"] = [ln for ln in a.splitlines() if ln.startswith('GATE')]
                f = cgs.load(OBJ)['fit']
                rec["fit"] = {k: f[k] for k in ('anchors', 'epochs', 'residual_mm', 'yaw_deg', 'pitch_mm')}
            runlog("note", "observed", json.dumps(rec)[:900])
            t = time.time()
            with sk.session() as s:
                blocked = extract(s)
                drop = qa(port, "extract")
                s.movel_rel([0, -0.010, 0], a=SPEED["near_a"], v=SPEED["near_v"])
                s.movel_rel([0, 0, 0.100], a=SPEED["lin_a"], v=SPEED["lin_v"])
            rec["extract_s"] = round(time.time() - t, 1)
            rec["extract_blocked"], rec["link_dropped"] = bool(blocked), bool(drop.get("ok"))
            out["ports"].append(rec)
            (run_dir / "cad_convergence.json").write_text(json.dumps(out, indent=1))
            runlog("done", "--status", "success" if rec["link"] else "failure",
                   "--summary", f"port {port}: link {rec['link']}, err x {rec.get('err_x_mm')} z {rec.get('err_z_mm')} mm, "
                                f"sigma {rec['sigma_mm']}, {rec['insert_s']}+{rec['extract_s']} s")
            print(json.dumps(rec), flush=True)
            if not rec["link"]:
                raise BridgeError(f"port {port} failed ({res.get('reason', 'no link')}): stop on surprises")
            done.append(port)
        out["total_s"] = round(time.time() - t_run, 1)
        runlog("end", "--status", "success", "--summary",
               f"CAD convergence: {len(done)} ports, order {done}, total {out['total_s']} s")
    except Exception as e:  # noqa: BLE001
        out["aborted"] = str(e)[:300]
        runlog("note", "observed", f"cad_run aborted: {e}")
        runlog("end", "--status", "aborted", "--summary", str(e)[:200])
        raise
    finally:
        (run_dir / "cad_convergence.json").write_text(json.dumps(out, indent=1))
        tel.terminate()
        tel.wait(timeout=10)
        runlog("trace")


if __name__ == "__main__":
    n = int(sys.argv[sys.argv.index("--ports") + 1]) if "--ports" in sys.argv else 8
    main(n, fast="--fast" in sys.argv)
