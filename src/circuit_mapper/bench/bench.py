"""Benchmark the circuit mapper on a set of circuit files.

Each instance runs in a fresh subprocess through mapper.map_circuit(circuit, inventory, board,
time_limit) with the default casings and gripper, while this script holds an exclusive fcntl lock
on the solver lock file (one solve at a time on the Jetson, comparable timings). Do not wrap this
script in flock(1) on the same file: it takes the lock itself, per instance, and would wait forever.

Per instance it records status or error text, wall time, model build time and number of CP-SAT
solves (measured by timing CpSolver.solve calls, so null if the mapper stops using it), jumpers,
placements, the rows the layout spans, and whether the output verifies:
  - connections: an independent union-find over the board's conductive strips and the jumpers
    (every net connected, no two nets shorted, no hole used twice, every pin placed),
  - assembly: the dashboard's own reading of the output (dashboard/assembly_plan.steps) reports no
    collision for the returned assembly_order, i.e. geometry.verify_assembly's test.
Only map_circuit's public signature and its output are used, so it keeps working as the mapper
changes.

  python src/circuit_mapper/bench/bench.py --label baseline --time-limit 120
  python src/circuit_mapper/bench/bench.py --label try1 --only p3,p4 --time-limit 60

Results: bench/results/<label>.json and a one-line-per-instance table on stdout.

Benchmark prompts (bench/circuits, as typed into the dashboard's Circuit Assembly tab), with the
part ids moved to the real stock of 10-27 (example_components.json, mostly one piece per part):
  p1 "A red LED with a 1 kΩ series resistor on a 5 V supply."               (real design, 10-26)
  p2 "An RC low-pass filter between a signal input and an output, 1 kΩ and 100 nF" (real, 10-26)
  p3 "Two LEDs in parallel, each with its own 330 Ω resistor, on 5 V"        (red + green, stock)
  p5 "Three logic inputs, each driving its own LED through its own resistor, common ground"
      (was an RGB LED; stock has none: red, green, yellow LEDs)
  circuits/parallel_leds_3: three LEDs with 1 kΩ each on 5 V (was 5; stock has 3 LEDs)
  p4 (NPN LED switch) was dropped with the stock: no transistor.
"""

import argparse
import fcntl
import importlib.util
import json
import os
import platform
import subprocess
import sys
import tempfile
import time
from pathlib import Path

BENCH = Path(__file__).resolve().parent
MAPPER_DIR = BENCH.parent
ROOT = MAPPER_DIR.parents[1]
LOCK = Path(
    "/tmp/claude-1000/-home-lab-order2cobot/b6c41912-1f5f-4780-9e03-a82e9b207ea0/scratchpad/solver.lock"
)
DEFAULT_GLOBS = ("examples/*.json", "circuits/*.json", "bench/circuits/*.json")


# ── worker (runs in the subprocess) ──────────────────────────────────────────


def _instrument_solver():
    """Time CpSolver.solve calls: the first start marks the end of model construction."""
    stats = {"first_solve_at": None, "solves": 0, "solve_s": 0.0}
    try:
        from ortools.sat.python import cp_model
    except ImportError:
        return stats
    depth = [0]

    def wrap(original):
        def timed(self, *args, **kwargs):
            if depth[0]:
                return original(self, *args, **kwargs)
            depth[0] += 1
            start = time.monotonic()
            if stats["first_solve_at"] is None:
                stats["first_solve_at"] = start
            try:
                return original(self, *args, **kwargs)
            finally:
                stats["solves"] += 1
                stats["solve_s"] += time.monotonic() - start
                depth[0] -= 1

        return timed

    for name in ("solve", "Solve"):
        if hasattr(cp_model.CpSolver, name):
            setattr(cp_model.CpSolver, name, wrap(getattr(cp_model.CpSolver, name)))
    return stats


def _status_of(error):
    text = str(error)
    for status in ("INFEASIBLE", "MODEL_INVALID", "UNKNOWN"):
        if status in text:
            return status
    return "ERROR"


def worker(circuit_path, time_limit, out_path):
    sys.path.insert(0, str(MAPPER_DIR))
    import mapper  # noqa: PLC0415  (the worktree's src/circuit_mapper/mapper.py)

    stats = _instrument_solver()
    read = lambda p: json.loads(Path(p).read_text())  # noqa: E731
    record = {}
    start = time.monotonic()
    try:
        result = mapper.map_circuit(
            read(circuit_path),
            read(MAPPER_DIR / "example_components.json"),
            read(MAPPER_DIR / "board_connections.json"),
            time_limit,
        )
        record.update(status=result.get("status", "?"), error=None, result=result)
    except Exception as error:  # noqa: BLE001  (every failure is a benchmark outcome)
        record.update(status=_status_of(error), error=f"{type(error).__name__}: {error}")
    end = time.monotonic()
    record["solve_wall_s"] = round(end - start, 3)
    if stats["first_solve_at"] is not None:
        record["build_s"] = round(stats["first_solve_at"] - start, 3)
        record["solves"] = stats["solves"]
        record["solver_s"] = round(stats["solve_s"], 3)
    Path(out_path).write_text(json.dumps(record))


# ── checks on the output (main process) ─────────────────────────────────────


def _row(hole):
    return int(hole[1:])


def check_connections(circuit, parts, board, result):
    """Independent connectivity check from the mapping output and the board's strips."""
    groups = {h: g for g, group in enumerate(board["conductive_groups"]) for h in group["holes"]}
    parent = list(range(len(board["conductive_groups"])))

    def find(g):
        while parent[g] != g:
            parent[g] = parent[parent[g]]
            g = parent[g]
        return g

    pin_holes = {**result["pin_to_hole"], **result.get("manual_connections", {})}
    expected = {
        f"{c['id']}.{pin}" for c in circuit["components"] for pin in parts[c["part_id"]]["pins"]
    }
    if set(pin_holes) != expected:
        missing = sorted(expected - set(pin_holes))
        extra = sorted(set(pin_holes) - expected)
        return f"pin set differs: missing {missing}, extra {extra}"
    occupied = list(pin_holes.values())
    for jumper in result["jumpers"]:
        a, b = jumper["pins"].values()
        occupied += [a, b]
        parent[find(groups[a])] = find(groups[b])
    if len(occupied) != len(set(occupied)):
        return "a hole is occupied more than once"
    nets = list(circuit["nets"]) + [
        {"id": f"UNCONNECTED:{p}", "pins": [p]} for p in circuit.get("unconnected_pins", [])
    ]
    owner = {}
    for net in nets:
        roots = {find(groups[pin_holes[p]]) for p in net["pins"]}
        if len(roots) != 1:
            return f"net {net['id']} is disconnected"
        root = roots.pop()
        if root in owner:
            return f"short between {owner[root]} and {net['id']}"
        owner[root] = net["id"]
    return None


def check_assembly(circuit, parts, board, result):
    """The dashboard's reading of the output: every step collision-free in assembly_order."""
    spec = importlib.util.spec_from_file_location("bench_assembly_plan", ROOT / "dashboard/assembly_plan.py")
    assembly_plan = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(assembly_plan)
    steps = assembly_plan.steps(
        circuit, result, list(parts.values()), board, assembly_plan.load_casings()
    )
    placed = {p["id"] for p in result["placements"]}
    if [s["item"] for s in steps] != result["assembly_order"] or set(result["assembly_order"]) != placed:
        return "assembly_order and placements differ"
    bad = [f"{s['item']} hits {', '.join(s['collisions'])}" for s in steps if s["collisions"]]
    return "; ".join(bad) or None


def summarize(path, circuit, parts, board, record):
    out = {
        "file": str(path.relative_to(MAPPER_DIR)),
        "name": path.stem,
        "circuit_id": circuit.get("circuit_id"),
        "robot_parts": sum(
            1 for c in circuit["components"] if parts.get(c["part_id"], {}).get("placement_mode") != "manual"
        ),
        "manual_parts": sum(
            1 for c in circuit["components"] if parts.get(c["part_id"], {}).get("placement_mode") == "manual"
        ),
        "nets": len(circuit["nets"]),
        **{k: v for k, v in record.items() if k != "result"},
    }
    result = record.get("result")
    if result is None:
        out["verified"] = False
        return out
    holes = (
        list(result["pin_to_hole"].values())
        + list(result.get("manual_connections", {}).values())
        + [h for j in result["jumpers"] for h in j["pins"].values()]
    )
    rows = [_row(h) for h in holes]
    out.update(
        jumpers=len(result["jumpers"]),
        placements=len(result["placements"]),
        row_min=min(rows),
        row_max=max(rows),
        rows_spanned=max(rows) - min(rows) + 1,
        geometry_model=result.get("geometry_model"),
    )
    problems = []
    for name, check in (("connections", check_connections), ("assembly", check_assembly)):
        try:
            problem = check(circuit, parts, board, result)
        except Exception as error:  # noqa: BLE001
            problem = f"check failed: {type(error).__name__}: {error}"
        if problem:
            problems.append(f"{name}: {problem}")
    out["verified"] = not problems
    out["verify_error"] = "; ".join(problems) or None
    out["mapping"] = result
    return out


# ── driver ───────────────────────────────────────────────────────────────────


def run_one(path, time_limit, lock_path, hard_timeout):
    with open(lock_path, "a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print(f"  waiting for solver lock {lock_path} ...", file=sys.stderr, flush=True)
            fcntl.flock(lock, fcntl.LOCK_EX)
        with tempfile.NamedTemporaryFile(suffix=".json", dir=Path(lock_path).parent, delete=False) as tmp:
            out_path = tmp.name
        start = time.monotonic()
        try:
            proc = subprocess.run(
                [sys.executable, __file__, "--_worker", str(path), "--time-limit", str(time_limit),
                 "--_out", out_path],
                cwd=ROOT, capture_output=True, text=True, timeout=hard_timeout,
            )
            wall = time.monotonic() - start
            try:
                record = json.loads(Path(out_path).read_text())
            except (OSError, json.JSONDecodeError):
                tail = (proc.stderr or proc.stdout).strip().splitlines()[-3:]
                record = {"status": "CRASH", "error": f"exit {proc.returncode}: {' | '.join(tail)}"}
        except subprocess.TimeoutExpired:
            wall = time.monotonic() - start
            record = {"status": "KILLED", "error": f"no answer within the hard timeout {hard_timeout:g} s"}
        finally:
            Path(out_path).unlink(missing_ok=True)
    record["wall_s"] = round(wall, 3)
    return record


def git_state():
    def git(*args):
        try:
            return subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True).stdout.strip()
        except OSError:
            return None

    return {"commit": git("rev-parse", "--short", "HEAD"), "dirty": bool(git("status", "--porcelain", "--untracked-files=no", "--", "src"))}


def fmt_row(r):
    span = f"{r['row_min']}..{r['row_max']}" if "row_min" in r else "-"
    build = f"{r['build_s']:.1f}" if r.get("build_s") is not None else "-"
    note = r.get("error") or r.get("verify_error") or ""
    return (
        f"{r['name'][:28]:28} {r['status'][:10]:10} {r['wall_s']:7.1f} {build:>6} "
        f"{str(r.get('solves', '-')):>3} {str(r.get('jumpers', '-')):>3} {str(r.get('placements', '-')):>3} "
        f"{span:>7} {'yes' if r['verified'] else 'no':3}  {note[:90]}"
    )


HEADER = f"{'instance':28} {'status':10} {'wall_s':>7} {'build':>6} {'it':>3} {'jmp':>3} {'plc':>3} {'rows':>7} ver  note"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("files", nargs="*", type=Path, help="circuit files (default: %s)" % ", ".join(DEFAULT_GLOBS))
    parser.add_argument("--label", default="run", help="results go to bench/results/<label>.json")
    parser.add_argument("--time-limit", type=float, default=60, help="map_circuit time_limit per instance [s]")
    parser.add_argument("--only", help="comma-separated substrings; keep instances whose file name contains one")
    parser.add_argument("--lock", type=Path, default=LOCK, help="solver lock file")
    parser.add_argument("--hard-timeout", type=float, help="kill an instance after this many s (default 2*limit+120)")
    parser.add_argument("--_worker", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--_out", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.time_limit <= 0:
        parser.error("--time-limit must be positive")
    if args._worker:
        worker(args._worker, args.time_limit, args._out)
        return

    files = [f.resolve() for f in args.files] or sorted(
        p for pattern in DEFAULT_GLOBS for p in MAPPER_DIR.glob(pattern)
    )
    if args.only:
        keys = [k for k in args.only.split(",") if k]
        files = [f for f in files if any(k in f.name for k in keys)]
    if not files:
        parser.error("no circuit files selected")
    hard_timeout = args.hard_timeout or 2 * args.time_limit + 120
    parts = {p["part_id"]: p for p in json.loads((MAPPER_DIR / "example_components.json").read_text())["components"]}
    board = json.loads((MAPPER_DIR / "board_connections.json").read_text())

    meta = {
        "label": args.label,
        "time_limit_s": args.time_limit,
        "started": time.strftime("%Y-%m-%d %H:%M:%S"),
        "host": platform.node(),
        "cpus": os.cpu_count(),
        "load_avg_at_start": [round(x, 2) for x in os.getloadavg()],
        **git_state(),
    }
    print(f"# {args.label}: {len(files)} instances, time limit {args.time_limit:g} s, "
          f"commit {meta['commit']}{' +dirty' if meta['dirty'] else ''}, load {meta['load_avg_at_start']}")
    print(HEADER, flush=True)
    rows = []
    for path in files:
        circuit = json.loads(path.read_text())
        record = run_one(path, args.time_limit, args.lock, hard_timeout)
        row = summarize(path, circuit, parts, board, record)
        rows.append(row)
        print(fmt_row(row), flush=True)
    meta["finished"] = time.strftime("%Y-%m-%d %H:%M:%S")
    meta["load_avg_at_end"] = [round(x, 2) for x in os.getloadavg()]
    out = BENCH / "results" / f"{args.label}.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps({"meta": meta, "instances": rows}, indent=1) + "\n")
    solved = sum(r["verified"] for r in rows)
    print(f"# {solved}/{len(rows)} solved and verified; results in {out.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
