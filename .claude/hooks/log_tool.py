#!/usr/bin/env python3
"""Claude Code hook: append every tool call and user prompt to the active run log.

Wired in .claude/settings.json for PostToolUse and UserPromptSubmit. Writes to
data/runs/current/tools.jsonl only while a run is active (porttest.runlog start).
Never blocks Claude: any error is swallowed and the hook exits 0.
"""
import json
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CURRENT = os.path.join(ROOT, "data", "runs", "current")
MAX = 4000   # chars kept per input/output field


def clip(v):
    s = v if isinstance(v, str) else json.dumps(v, ensure_ascii=False, default=str)
    return s if len(s) <= MAX else s[:MAX] + f"... [+{len(s) - MAX} chars]"


def main():
    if not os.path.isdir(CURRENT):
        return
    ev = json.load(sys.stdin)
    kind = ev.get("hook_event_name", "")
    rec = {"ts": time.time(), "event": kind, "session": ev.get("session_id")}
    if kind == "UserPromptSubmit":
        rec["prompt"] = clip(ev.get("prompt", ""))
    else:
        rec["tool"] = ev.get("tool_name")
        inp = ev.get("tool_input") or {}
        rec["summary"] = (inp.get("description") or inp.get("command") or inp.get("file_path")
                          or inp.get("skill") or json.dumps(inp, default=str))[:200]
        rec["input"] = clip(inp)
        rec["output"] = clip(ev.get("tool_response", ""))
    rec["transcript"] = ev.get("transcript_path")
    with open(os.path.join(CURRENT, "tools.jsonl"), "a") as f:
        f.write(json.dumps(rec, ensure_ascii=False) + "\n")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        pass
    sys.exit(0)
