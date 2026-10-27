"""Moved to urcore/session_trace/session_trace_to_drawio.py; kept so the old path still runs."""
import runpy
from pathlib import Path

if __name__ == "__main__":
    runpy.run_path(str(Path(__file__).resolve().parents[2] / "urcore/session_trace/session_trace_to_drawio.py"),
                   run_name="__main__")
