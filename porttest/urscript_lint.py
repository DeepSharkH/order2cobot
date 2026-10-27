"""Moved to urcore/urscript_lint.py (shared). `python -m porttest.urscript_lint` lints the
switch LIB (shared + RJ45 primitives)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))   # also run as a file path
from urcore.urscript_lint import BUILTINS, HINTS, KEYWORDS, PYTHONISMS, lint, strip  # noqa: F401

if __name__ == "__main__":
    from porttest.urlib import LIB
    from urcore.urscript_lint import main
    main(LIB)
