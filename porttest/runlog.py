"""Run log for skill runs: step markers, notes, outcome -> session trace (draw.io).

Tool calls and user prompts are logged automatically by the Claude Code hook
(.claude/hooks/log_tool.py) into data/runs/current/tools.jsonl. This CLI adds what
only the agent knows: where a step starts and ends, what it assumed, what it saw,
what the operator did.

  python -m porttest.runlog start rj45-port1
  python -m porttest.runlog step S3 "Grip check" --intent "tag pose vs reference grip"
  python -m porttest.runlog note assumption "plug pins face down" --status unverified
  python -m porttest.runlog note observed "tag 58 px, axis err 7 deg"
  python -m porttest.runlog note operator "switched pendant to Remote"
  python -m porttest.runlog done --status success --summary "grip unchanged (0.3 mm)"
  python -m porttest.runlog end --status failure --summary "no link after 3 tries"
  python -m porttest.runlog trace      # -> data/runs/<run>/session_trace.json + .drawio.xml
"""
import argparse
import json
import os
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
RUNS = ROOT / "data" / "runs"
CURRENT = RUNS / "current"
STATUSES = ("success", "partial", "failure", "aborted")


def run_dir():
    if not CURRENT.exists():
        sys.exit("no active run: python -m porttest.runlog start <name>")
    return CURRENT.resolve()


def log(kind, **fields):
    rec = {"ts": time.time(), "kind": kind, **fields}
    with open(run_dir() / "events.jsonl", "a") as f:
        f.write(json.dumps(rec, ensure_ascii=False) + "\n")
    print(json.dumps(rec, ensure_ascii=False))


def read(path):
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def hhmm(ts):
    return datetime.fromtimestamp(ts).strftime("%H:%M:%S")


def build_trace(d):
    events, tools = read(d / "events.jsonl"), read(d / "tools.jsonl")
    meta = json.loads((d / "meta.json").read_text())
    steps, cur = [], None
    for e in events:
        if e["kind"] == "step":
            cur = {"id": e["id"], "title": e["title"], "t0": e["ts"], "t1": None, "intent": e.get("intent", ""),
                   "notes": [], "outcome": None}
            steps.append(cur)
        elif e["kind"] == "note" and cur:
            cur["notes"].append(e)
        elif e["kind"] == "done" and cur:
            cur["outcome"], cur["t1"] = e, e["ts"]
    end = next((e for e in events if e["kind"] == "end"), None)
    t_end = end["ts"] if end else time.time()
    for i, s in enumerate(steps):
        s["t1"] = s["t1"] or (steps[i + 1]["t0"] if i + 1 < len(steps) else t_end)

    episodes = []
    for s in steps:
        win = [t for t in tools if s["t0"] <= t["ts"] <= s["t1"]]
        prompts = [t["prompt"] for t in win if t.get("event") == "UserPromptSubmit"]
        calls = [t for t in win if t.get("event") != "UserPromptSubmit"]
        notes = s["notes"]
        o = s["outcome"] or {"status": "aborted", "summary": "step never closed"}
        bridge = sum("urexec" in (t.get("input") or "") or (t.get("tool") or "").startswith("mcp__ur3__") for t in calls)
        episodes.append({
            "title": f'{s["id"]} {s["title"]}',
            "time": f'{hhmm(s["t0"])}-{hhmm(s["t1"])} ({(s["t1"] - s["t0"]) / 60:.1f} min)',
            "trigger": "skill step",
            "ask": " / ".join(p[:200] for p in prompts) or f'(skill step {s["id"]})',
            "interpretation": " ".join([s["intent"]] + [n["text"] for n in notes if n["type"] == "interpretation"]).strip(),
            "actions": [f'{t.get("tool")}: {t.get("summary", "")[:90]}' for t in calls][:12]
                       + ([f"... +{len(calls) - 12} more tool calls"] if len(calls) > 12 else []),
            "observed": [n["text"] for n in notes if n["type"] == "observed"],
            "outcome": {"status": o["status"], "summary": o.get("summary", ""),
                        "cost": {"wall_time": f'{(s["t1"] - s["t0"]) / 60:.1f} min', "tool_calls": len(calls),
                                 "robot_calls": bridge}},
            "assumptions": [{"text": n["text"], "status": n.get("status") or "unverified"}
                            for n in notes if n["type"] == "assumption"],
            "human": " / ".join(n["text"] for n in notes if n["type"] == "operator"),
        })
    ops = sum(n["type"] == "operator" for s in steps for n in s["notes"])
    return {
        "session": {"title": f'{meta["name"]} (skill run)', "date": meta["date"],
                    "goal": meta.get("goal", ""),
                    "setup": f'run dir {d.name}; operator actions: {ops}; '
                             f'final: {end["status"] if end else "open"} - {end.get("summary", "") if end else ""}',
                    "notes": "Built by porttest.runlog from events.jsonl + tools.jsonl. "
                             "Fill 'interpretation' and 'summary' where empty before review."},
        "episodes": episodes,
        "summary": {"time_sinks": [], "failure_patterns": [], "missing_capabilities": [], "keep": []},
    }


def main():
    ap = argparse.ArgumentParser(prog="python -m porttest.runlog")
    sub = ap.add_subparsers(dest="cmd", required=True)
    a = sub.add_parser("start"); a.add_argument("name"); a.add_argument("--goal", default="")
    a = sub.add_parser("step"); a.add_argument("id"); a.add_argument("title"); a.add_argument("--intent", default="")
    a = sub.add_parser("note"); a.add_argument("type", choices=("assumption", "observed", "operator", "interpretation"))
    a.add_argument("text"); a.add_argument("--status", choices=("verified", "unverified", "false"))
    a = sub.add_parser("done"); a.add_argument("--status", choices=STATUSES, required=True); a.add_argument("--summary", default="")
    a = sub.add_parser("end"); a.add_argument("--status", choices=STATUSES, required=True); a.add_argument("--summary", default="")
    sub.add_parser("trace")
    args = ap.parse_args()

    if args.cmd == "start":
        d = RUNS / f'{datetime.now():%Y%m%d-%H%M%S}_{args.name}'
        d.mkdir(parents=True)
        (d / "meta.json").write_text(json.dumps({"name": args.name, "goal": args.goal,
                                                 "date": f"{datetime.now():%Y-%m-%d}", "started": time.time()}))
        if CURRENT.is_symlink() or CURRENT.exists():
            CURRENT.unlink()
        CURRENT.symlink_to(d.name)
        log("start", name=args.name)
        print(f"run dir: {d}")
    elif args.cmd == "step":
        log("step", id=args.id, title=args.title, intent=args.intent)
    elif args.cmd == "note":
        log("note", type=args.type, text=args.text, status=args.status)
    elif args.cmd == "done":
        log("done", status=args.status, summary=args.summary)
    elif args.cmd == "end":
        log("end", status=args.status, summary=args.summary)
    elif args.cmd == "trace":
        d = run_dir()
        out = d / "session_trace.json"
        out.write_text(json.dumps(build_trace(d), indent=2, ensure_ascii=False))
        gen = ROOT / "porttest" / "session_trace" / "session_trace_to_drawio.py"
        subprocess.run([sys.executable, str(gen), str(out), str(d / "session_trace.drawio.xml")], check=True)
        print(f"trace: {out}")


if __name__ == "__main__":
    main()
