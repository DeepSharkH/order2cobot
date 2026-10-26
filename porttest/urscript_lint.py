"""Static check of URScript before it goes to the robot: unknown functions and Python-isms.

The allowlist (urscript_builtins.txt) is every function heading of the PolyScope X 10.14
script directory (the controller runs URControl 10.14.1). Functions the script defines
itself (`def name(`) are allowed too. Not a parser: it catches the cheap mistakes that
otherwise surface as `compile_error_name_not_found` on the controller (e.g. abs()).

  python -m porttest.urscript_lint            # checks urlib.LIB
  python -m porttest.urscript_lint file.script
  from porttest.urscript_lint import lint; problems = lint(script_text)
"""
import re
import sys
from pathlib import Path

BUILTINS = set((Path(__file__).with_name("urscript_builtins.txt")).read_text().split())
# Robotiq URCap 4.2.1 functions, defined in the preamble of an app whose application has the Hand-E
# configured (seen in close_gripper's generated script; the slave id there is 9)
BUILTINS |= {"rq_wait_for_gripper_reconnection", "rq_is_gripper_connected", "rq_is_gripper_activated",
             "rq_set_speed_norm_multi_grippers", "rq_set_force_norm_multi_grippers",
             "rq_move_and_wait_norm_multi_grippers", "rq_move_norm_multi_grippers", "write_error_popup"}
KEYWORDS = {"if", "elif", "while", "return", "and", "or", "not", "def", "thread", "run", "join", "kill"}
# Python habits that are not URScript (with the URScript way)
PYTHONISMS = [
    (r"^\s*for\s", "no for-loops in URScript: use while with a counter"),
    (r"\*\*", "no ** operator: use pow(base, exponent)"),
    (r"\bNone\b", "no None"),
    (r"^\s*(import|from)\s", "no imports"),
    (r"\blambda\b", "no lambda"),
    (r"^\s*(try|except|with|class)\b", "no try/except/with/class"),
    (r"\[[^\]]*\bfor\b[^\]]*\]", "no list comprehensions"),
    (r"\bis\s+not\b|\bis\b", "no `is`: use == / !="),
]
HINTS = {"abs": "use norm(x)", "fabs": "not in the 10.14 directory: use norm(x)", "min": "write an if",
         "max": "write an if", "len": "use size(x) or x.length()", "round": "use floor(x + 0.5)",
         "range": "use while with a counter", "print": "use textmsg()", "str": "use to_str()",
         "int": "use floor() / to_num()", "float": "use to_num()", "sum": "loop with while"}


def strip(line):
    """Drop string literals and the # comment."""
    line = re.sub(r'"[^"]*"', '""', line)
    return line.split("#", 1)[0]


def lint(script):
    lines = [strip(l) for l in script.splitlines()]
    defined = {m for l in lines for m in re.findall(r"\bdef\s+([A-Za-z_]\w*)\s*\(", l)}
    allowed = BUILTINS | defined | KEYWORDS
    problems = []
    for n, l in enumerate(lines, 1):
        for name in re.findall(r"(?<![\w.])([A-Za-z_]\w*)\s*\(", l):
            if name not in allowed:
                hint = HINTS.get(name, "not a URScript built-in and not defined in this script")
                problems.append(f"line {n}: {name}() - {hint}")
        for pat, msg in PYTHONISMS:
            if re.search(pat, l):
                problems.append(f"line {n}: {msg}")
    return problems


def main():
    if len(sys.argv) > 1:
        text = Path(sys.argv[1]).read_text()
    else:
        sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
        from porttest.urlib import LIB
        text = LIB
    problems = lint(text)
    print("\n".join(problems) if problems else "urscript_lint: OK")
    sys.exit(1 if problems else 0)


if __name__ == "__main__":
    main()
