"""Skill library: validated lessons (symptom -> cause -> fix) checked against skill_b trace records.

Entries live in skills_lib/<domain>/<id>.yaml:
  id, title, applies_to, symptom (null or a condition on trace records), cause, fix,
  auto_fix (optional, handled in code), status (proposed | validated | superseded),
  validated_by (runs that PASSED with the fix), seen_in (runs that showed the symptom), stamp

Rule: an entry becomes 'validated' only through `validate <id> <run>` with a run whose log ended
in success - ideally on a different port than the one it was learned on. Never delete entries;
mark them 'superseded' (with the replacing id in the fix text).

  python -m porttest.skilllib list [step]          # entries (optionally those applying to a step)
  python -m porttest.skilllib check <run_dir>      # symptoms matched in that run's trace.jsonl
  python -m porttest.skilllib validate <id> <run>  # add a passing run, status -> validated
"""
import json
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
LIB = ROOT / "skills_lib"
OPS = {">": lambda a, b: a > b, "<": lambda a, b: a < b, ">=": lambda a, b: a >= b,
       "<=": lambda a, b: a <= b, "==": lambda a, b: a == b}


def load(domain="rj45"):
    return {e["id"]: e | {"_file": str(f)} for f in sorted((LIB / domain).glob("*.yaml"))
            for e in [yaml.safe_load(f.read_text())]}


def for_step(step, domain="rj45"):
    """Entries to read before writing or running `step` (validated first)."""
    es = [e for e in load(domain).values() if e.get("status") != "superseded"
          and any(step in a or a in ("all robot steps", "any URScript") for a in e.get("applies_to", []))]
    return sorted(es, key=lambda e: e.get("status") != "validated")


def metric(rec, name):
    if name.startswith("d_mm."):
        d = rec.get("d_mm")
        return d["xyz".index(name[-1])] if d else None
    if name == "result_mm":
        r = rec.get("result")
        return r * 1000 if isinstance(r, (int, float)) and not isinstance(r, bool) else None
    if name == "f_end_norm":
        f = rec.get("f_end")
        return sum(x * x for x in f) ** 0.5 if f else None
    return rec.get(name)


def check(records, domain="rj45", extra=None):
    """Return [(entry, record)] for every record that matches an entry's symptom.
    `extra` holds step-level metrics computed by the caller, e.g. {'refs_vs_last_mm': 1.4}."""
    hits = []
    for e in load(domain).values():
        sym = e.get("symptom")
        if not sym or e.get("status") == "superseded":
            continue
        if "refs_vs_last_mm" in sym:
            op, v = sym["refs_vs_last_mm"]
            x = (extra or {}).get("refs_vs_last_mm")
            if x is not None and OPS[op](x, v):
                hits.append((e, {"refs_vs_last_mm": x}))
            continue
        for rec in records:
            if rec.get("cmd") not in sym.get("cmd", []):
                continue
            vals = [(metric(rec, m), op, v) for m, op, v in sym.get("all", [])]
            if vals and all(x is not None and OPS[op](x, v) for x, op, v in vals):
                hits.append((e, rec))
    return hits


def describe(hit):
    e, rec = hit
    where = f"{rec.get('step', '')}/{rec.get('cmd', '')}" if "cmd" in rec else "step"
    data = {k: rec[k] for k in ("d_mm", "result", "f_end", "refs_vs_last_mm") if k in rec}
    return f"LIB {e['id']} [{e.get('status')}] matched at {where} {json.dumps(data)} -> fix: {e['fix']}"


def validate(entry_id, run, domain="rj45"):
    f = LIB / domain / f"{entry_id}.yaml"
    e = yaml.safe_load(f.read_text())
    ev = ROOT / "data" / "runs" / run / "events.jsonl"
    end = [json.loads(l) for l in ev.read_text().splitlines() if '"kind": "end"' in l] if ev.exists() else []
    if not end or end[-1].get("status") != "success":
        sys.exit(f"refused: run {run} did not end in success ({end[-1].get('status') if end else 'no end'})")
    if run not in e.get("validated_by", []):
        e.setdefault("validated_by", []).append(run)
    e["status"] = "validated"
    f.write_text(yaml.safe_dump(e, sort_keys=False, allow_unicode=True, width=110))
    print(f"{entry_id}: validated by {e['validated_by']}")


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "list"
    if cmd == "list":
        es = for_step(sys.argv[2]) if len(sys.argv) > 2 else list(load().values())
        for e in es:
            print(f"{e.get('status', '?'):10} {e['id']:30} {e['title']}")
    elif cmd == "check":
        d = Path(sys.argv[2])
        d = d if d.is_absolute() else ROOT / "data" / "runs" / d
        recs = [json.loads(l) for l in (d / "trace.jsonl").read_text().splitlines()] if (d / "trace.jsonl").exists() else []
        hits = check(recs)
        print("\n".join(describe(h) for h in hits) or f"no known symptom in {len(recs)} trace records")
    elif cmd == "validate":
        validate(sys.argv[2], sys.argv[3])


if __name__ == "__main__":
    main()
