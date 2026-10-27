"""Moved to urcore/runlog.py (shared by every task). Kept so `python -m porttest.runlog` keeps working."""
import runpy
import sys
from pathlib import Path

if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))   # also run as a file path
    runpy.run_module("urcore.runlog", run_name="__main__", alter_sys=True)
