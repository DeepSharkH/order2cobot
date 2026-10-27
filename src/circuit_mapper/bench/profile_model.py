"""Profile the circuit mapper's CP-SAT model without editing mapper.py / geometry.py.

Monkeypatches CpModel.add*/new_*_var (to tag every constraint with the source line that
created it), CpSolver.solve (to time each lazy round, capture the CP-SAT log and the time to
first solution) and mapper.constrain_assembly (to count the lazy collision cuts per round).

Usage (always under the solver lock, see bench/README or the task notes):
  flock <lock> python src/circuit_mapper/bench/profile_model.py \
      src/circuit_mapper/examples/led_circuit.json --time-limit 120 --workers 8 \
      --json out.json --log-dir logs/
"""

import argparse
import functools
import json
import linecache
import re
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
MAPPER_DIR = HERE.parent
sys.path.insert(0, str(MAPPER_DIR))

from ortools.sat.python import cp_model  # noqa: E402

import mapper  # noqa: E402  (imported as a top-level module: uses `from geometry import ...`)

STATE = {
    "tags": [],  # one (file, line) per constraint, in creation order
    "add_time": Counter(),  # python time spent inside model.add* per source line
    "var_count": Counter(),
    "rounds": [],
    "t_start": None,
    "t_first_solve": None,
    "geometry_time": 0.0,
    "placements_time": 0.0,
    "workers": None,
    "params": [],
    "log_dir": None,
    "label": "run",
}


def _caller():
    frame = sys._getframe(2)
    while frame is not None:
        name = Path(frame.f_code.co_filename).name
        if name in ("mapper.py", "geometry.py"):
            return name, frame.f_lineno
        frame = frame.f_back
    return "other", 0


def _code(file, line):
    path = str(MAPPER_DIR / file)
    text = linecache.getline(path, line).strip()
    extra = 1
    while text.endswith("(") and extra < 4:
        text += linecache.getline(path, line + extra).strip()
        extra += 1
    return text


FAMILIES = [
    # (file, pattern, family)
    ("mapper.py", "add_exactly_one", "exactly_one (part / manual pin choice)"),
    ("mapper.py", "labels[g] == n).only_enforce_if(chosen)", "pin->strip label (enforced)"),
    ("mapper.py", "labels[a] == labels[b]", "wire: equal labels (enforced)"),
    ("mapper.py", "labels[a] >= 0", "wire: strip used (enforced)"),
    ("mapper.py", "add_at_most_one(occupants)", "occupancy at_most_one per hole"),
    ("mapper.py", "sum(edge_candidates) == linked", "flow: link == sum(parallel wires)"),
    ("mapper.py", "sum(actives) == linked", "flow: one active net per link"),
    ("mapper.py", "potential[", "flow: strip potential order (enforced)"),
    ("mapper.py", "flow != 0", "flow: nonzero on a link (enforced)"),
    ("mapper.py", "flow >= 0", "flow: direction (enforced)"),
    ("mapper.py", "flow <= 0", "flow: direction (enforced)"),
    ("mapper.py", "<= budget", "wire budget"),
    ("mapper.py", "labels[a] == n).only_enforce_if(active)", "flow: active -> label (enforced)"),
    ("mapper.py", "flow <= capacity", "flow: capacity bound"),
    ("mapper.py", "flow >= -capacity", "flow: capacity bound"),
    ("mapper.py", "sum(balance", "flow: conservation per (net, strip)"),
    ("geometry.py", "<= 1", "lazy cut: forbidden pair"),
    ("geometry.py", "orders[", "lazy cut: order precedence (enforced)"),
]


@functools.lru_cache(maxsize=None)
def family(file, line):
    code = _code(file, line)
    for f, pattern, name in FAMILIES:
        if f == file and pattern in code:
            if pattern == "labels[g] == n).only_enforce_if(chosen)":
                # mapper.py: the manual-pin loop comes after the "Manual connections" comment
                manual = _manual_start()
                return ("manual " if manual and line > manual else "part ") + name
            return name
    return f"{file}:{line} {code[:60]}"


_MANUAL = {}


def _manual_start():
    if "line" not in _MANUAL:
        _MANUAL["line"] = 0
        for i, text in enumerate((MAPPER_DIR / "mapper.py").read_text().splitlines(), 1):
            if "Manual connections" in text:
                _MANUAL["line"] = i
                break
    return _MANUAL["line"]


def _wrap_add(name):
    original = getattr(cp_model.CpModel, name)

    def wrapper(self, *args, **kwargs):
        tag = _caller()
        t = time.perf_counter()
        result = original(self, *args, **kwargs)
        STATE["add_time"][tag] += time.perf_counter() - t
        STATE["tags"].append(tag)
        return result

    setattr(cp_model.CpModel, name, wrapper)


for _name in ("add", "add_at_most_one", "add_exactly_one", "add_bool_or", "add_implication"):
    _wrap_add(_name)


VAR_FAMILIES = {
    "net_": "strip label int (net id / -1)",
    "wire_": "wire candidate bool",
    "order_": "order int (assembly position)",
    "link_": "flow: link bool (strip pair)",
    "active_": "flow: active bool (net, strip pair)",
    "flow_": "flow: int (net, strip pair)",
    "manual_": "manual pin->hole bool",
    "potential_": "flow: strip potential int",
    "forward_": "flow: link direction bool",
}


def var_family(name):
    for prefix, label in VAR_FAMILIES.items():
        if name.startswith(prefix):
            return label
    return "part placement bool"


def model_inventory(model):
    proto = model.proto
    variables = Counter()
    domain_sizes = defaultdict(int)
    for v in proto.variables:
        fam = var_family(v.name)
        variables[fam] += 1
        dom = list(v.domain)
        domain_sizes[fam] = max(domain_sizes[fam], dom[-1] - dom[0] + 1 if dom else 0)
    constraints = Counter(family(*tag) for tag in STATE["tags"])
    return {
        "num_variables": len(proto.variables),
        "num_constraints": len(proto.constraints),
        "num_tagged_constraints": len(STATE["tags"]),
        "variables": dict(variables.most_common()),
        "max_domain_size": dict(domain_sizes),
        "constraints": dict(constraints.most_common()),
        "model_stats": model.model_stats(),
    }


_original_solve = cp_model.CpSolver.solve


class _FirstSolution(cp_model.CpSolverSolutionCallback):
    def __init__(self):
        super().__init__()
        self.first = None
        self.count = 0

    def on_solution_callback(self):
        self.count += 1
        if self.first is None:
            self.first = self.wall_time


def _parse_log(lines):
    # CP-SAT prints thousands as 10'478; drop the separators so \d+ captures whole numbers.
    text = re.sub(r"(?<=\d)'(?=\d)", "", "\n".join(lines))
    out = {}
    m = re.search(r"Starting presolve at ([\d.]+)s", text)
    out["presolve_start_s"] = float(m.group(1)) if m else None
    m = re.search(r"Starting search at ([\d.]+)s with (\d+) workers", text)
    out["search_start_s"] = float(m.group(1)) if m else None
    out["workers_reported"] = int(m.group(2)) if m else None
    sols = re.findall(r"^#(\d+)\s+([\d.]+)s best:(\S+)\s+next:\S+\s+(\S+)", text, re.M)
    if sols:
        out["first_solution_s"] = float(sols[0][1])
        out["first_solution_by"] = sols[0][3]
        out["first_solution_obj"] = sols[0][2]
    m = re.search(r"Presolved \w+ model.*?\n#Variables: (\d+)", text)
    out["presolved_vars"] = int(m.group(1)) if m else None
    presolved = re.search(r"Presolved \w+ model.*?\n(.*?)\n\n", text, re.S)
    if presolved:
        out["presolved_constraints"] = {
            k: int(v) for k, v in re.findall(r"^#(k\w+): (\d+)", presolved.group(1), re.M)
        }
    m = re.search(r"^#Model\s+([\d.]+)s var:(\d+)/(\d+) constraints:(\d+)/(\d+)", text, re.M)
    if m:
        out["loaded_model"] = {"t": float(m.group(1)), "vars": int(m.group(3)), "cons": int(m.group(5))}
    # The heaviest presolve steps, by wall time.
    steps = Counter()
    for sec, step in re.findall(r"^\s+([\d.e+-]+)s\s+[\d.e+-]+d\s+\[(\w+)\]", text, re.M):
        steps[step] += float(sec)
    out["top_presolve_steps_s"] = {k: round(v, 3) for k, v in steps.most_common(5)}
    m = re.search(r"\[Symmetry\] Symmetry computation done. time: ([\d.e+-]+)", text)
    out["symmetry_s_first"] = float(m.group(1)) if m else None
    out["symmetry_s_total"] = round(
        sum(float(x) for x in re.findall(r"Symmetry computation done. time: ([\d.e+-]+)", text)), 3
    )
    m = re.search(r"^walltime: ([\d.]+)", text, re.M)
    out["log_walltime_s"] = float(m.group(1)) if m else None
    m = re.search(r"^deterministic_time: ([\d.]+)", text, re.M)
    out["dtime"] = float(m.group(1)) if m else None
    return out


def profiled_solve(self, model, solution_callback=None):
    now = time.monotonic()
    if STATE["t_first_solve"] is None:
        STATE["t_first_solve"] = now
        STATE["inventory_round1"] = model_inventory(model)
    if STATE["workers"]:
        self.parameters.num_workers = STATE["workers"]
    for name, value in STATE["params"]:
        current = getattr(self.parameters, name)
        if isinstance(current, bool):
            value = value.lower() in ("1", "true", "yes")
        else:
            value = type(current)(value)
        if name == "max_time_in_seconds":  # a per-round cap; never extend the mapper's deadline
            value = min(value, current)
        setattr(self.parameters, name, value)
    lines = []
    self.parameters.log_search_progress = True
    self.parameters.log_to_stdout = False
    self.log_callback = lines.append
    callback = _FirstSolution()
    n_constraints = len(model.proto.constraints)
    t = time.monotonic()
    status = _original_solve(self, model, callback)
    wall = time.monotonic() - t
    rnd = {
        "round": len(STATE["rounds"]) + 1,
        "status": self.status_name(status),
        "wall_s": round(wall, 3),
        "solver_wall_s": round(self.wall_time, 3),
        "first_solution_callback_s": None if callback.first is None else round(callback.first, 3),
        "num_constraints": n_constraints,
        "conflicts": self.num_conflicts,
        "branches": self.num_branches,
        "time_limit_s": round(self.parameters.max_time_in_seconds, 2),
        **_parse_log(lines),
    }
    if status in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        rnd["objective"] = self.objective_value
    STATE["rounds"].append(rnd)
    if STATE["log_dir"]:
        path = Path(STATE["log_dir"]) / f"{STATE['label']}_round{rnd['round']}.log"
        path.write_text("\n".join(lines) + "\n")
    print(
        f"  round {rnd['round']}: {rnd['status']} wall {wall:.2f}s "
        f"presolve {rnd['presolve_start_s']}->{rnd['search_start_s']}s "
        f"first sol {rnd.get('first_solution_s')}s by {rnd.get('first_solution_by')} "
        f"cons {n_constraints}",
        file=sys.stderr,
        flush=True,
    )
    return status


cp_model.CpSolver.solve = profiled_solve

_original_constrain = mapper.constrain_assembly


def profiled_constrain(model, selected, orders, known, margin):
    before_known = len(known)
    before_tags = len(STATE["tags"])
    t = time.monotonic()
    changed = _original_constrain(model, selected, orders, known, margin)
    kinds = Counter(family(*tag) for tag in STATE["tags"][before_tags:])
    round_ = STATE["rounds"][-1]
    round_["cuts_added"] = len(known) - before_known
    round_["cut_kinds"] = dict(kinds)
    round_["constrain_s"] = round(time.monotonic() - t, 4)
    round_["selected"] = [c["id"] for c in selected]
    return changed


mapper.constrain_assembly = profiled_constrain

_original_geometry = mapper.placement_geometry


def profiled_geometry(*args, **kwargs):
    t = time.perf_counter()
    result = _original_geometry(*args, **kwargs)
    STATE["geometry_time"] += time.perf_counter() - t
    return result


mapper.placement_geometry = profiled_geometry

_original_placements = mapper.placements


def profiled_placements(part, positions):
    t = time.perf_counter()
    result = list(_original_placements(part, positions))
    STATE["placements_time"] += time.perf_counter() - t
    return iter(result)


mapper.placements = profiled_placements


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("circuit", type=Path)
    parser.add_argument("--time-limit", type=float, default=120)
    parser.add_argument("--workers", type=int, default=None, help="override num_workers")
    parser.add_argument("--json", type=Path, help="write the full stats here")
    parser.add_argument("--log-dir", type=Path, help="write each round's CP-SAT log here")
    parser.add_argument("--label", default=None)
    parser.add_argument(
        "--param", action="append", default=[], help="extra SatParameters, e.g. cp_model_probing_level=0"
    )
    args = parser.parse_args()
    STATE["workers"] = args.workers
    STATE["log_dir"] = args.log_dir
    STATE["params"] = [tuple(p.split("=", 1)) for p in args.param]
    STATE["label"] = args.label or f"{args.circuit.stem}_w{args.workers or 'default'}"
    if args.log_dir:
        args.log_dir.mkdir(parents=True, exist_ok=True)
    circuit = mapper.read_json(args.circuit)
    inventory = mapper.read_json(MAPPER_DIR / "example_components.json")
    board = mapper.read_json(MAPPER_DIR / "board_connections.json")
    STATE["t_start"] = time.monotonic()
    error = None
    result = None
    try:
        result = mapper.map_circuit(circuit, inventory, board, args.time_limit)
    except ValueError as exc:
        error = str(exc)
    t_end = time.monotonic()
    rounds = STATE["rounds"]
    solve_total = sum(r["wall_s"] for r in rounds)
    presolve_total = sum(
        (r["search_start_s"] or 0) - (r["presolve_start_s"] or 0) for r in rounds
    )
    first_sol = [r.get("first_solution_s") for r in rounds]
    summary = {
        "label": STATE["label"],
        "circuit": str(args.circuit),
        "workers": args.workers or 8,
        "params": args.param,
        "time_limit_s": args.time_limit,
        "result_status": result["status"] if result else None,
        "error": error,
        "jumpers": len(result["jumpers"]) if result else None,
        "total_s": round(t_end - STATE["t_start"], 3),
        "build_s": round((STATE["t_first_solve"] or t_end) - STATE["t_start"], 3),
        "build_geometry_s": round(STATE["geometry_time"], 3),
        "build_placements_s": round(STATE["placements_time"], 3),
        "build_add_calls_s": round(sum(STATE["add_time"].values()), 3),
        "num_rounds": len(rounds),
        "solve_total_s": round(solve_total, 3),
        "presolve_total_s": round(presolve_total, 3),
        "search_total_s": round(solve_total - presolve_total, 3),
        "constrain_total_s": round(sum(r.get("constrain_s", 0) for r in rounds), 3),
        "first_solution_per_round_s": first_sol,
        "rounds": rounds,
        "inventory_round1": STATE.get("inventory_round1"),
        "add_time_by_family_s": {
            k: round(v, 3)
            for k, v in sorted(
                (
                    (fam, sum(t for tag, t in STATE["add_time"].items() if family(*tag) == fam))
                    for fam in {family(*tag) for tag in STATE["add_time"]}
                ),
                key=lambda kv: -kv[1],
            )
        },
    }
    if args.json:
        args.json.write_text(json.dumps(summary, indent=2) + "\n")
    inv = summary["inventory_round1"] or {}
    print(f"== {summary['label']}: {summary['result_status'] or error}")
    print(
        f"total {summary['total_s']}s | build {summary['build_s']}s "
        f"(geometry {summary['build_geometry_s']}s, placements {summary['build_placements_s']}s, "
        f"model.add* {summary['build_add_calls_s']}s) | rounds {summary['num_rounds']} | "
        f"solve {summary['solve_total_s']}s (presolve {summary['presolve_total_s']}s, "
        f"search {summary['search_total_s']}s) | cuts python {summary['constrain_total_s']}s"
    )
    if inv:
        print(f"vars {inv['num_variables']}  constraints {inv['num_constraints']} "
              f"(tagged {inv['num_tagged_constraints']})")
        for k, v in inv["variables"].items():
            print(f"  var  {v:7d}  {k}  (max domain {inv['max_domain_size'][k]})")
        for k, v in inv["constraints"].items():
            print(f"  cons {v:7d}  {k}")
    for r in rounds:
        print(
            f"  r{r['round']:<3d} {r['status']:<9s} wall {r['wall_s']:7.2f}s "
            f"presolve {r['presolve_start_s']}->{r['search_start_s']}s "
            f"1st sol {r.get('first_solution_s')}s ({r.get('first_solution_by')}) "
            f"obj {r.get('objective')} cuts +{r.get('cuts_added')} {r.get('cut_kinds')} "
            f"presolved vars {r.get('presolved_vars')}"
        )


if __name__ == "__main__":
    main()
