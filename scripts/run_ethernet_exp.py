#!/usr/bin/env python3
"""Run an ethernet experiment unattended: the Claude CLI agent, shown live and recorded like
scripts/run_agent.py, while both cameras are filmed into the same results folder.

The prompt is the positional argument, or is read from --prompt-file, or from stdin:

    uv run scripts/run_ethernet_exp.py --name rj45_port1 "/rj45-insert seat port 1"
    uv run scripts/run_ethernet_exp.py --prompt-file prompts/rj45.txt --name rj45_port1

Nothing is asked during the run. configs/switch/ethernet_exp.yaml runs the CLI in the
rj45-port-qa worktree with the skill's commands allowed up front (all Bash and edits, minus a
deny list, no MCP servers), and tells the agent that no operator is available.

The cameras are read from the calibration dashboard (it owns them; see configs/vision.yaml),
so it must be running. Recording starts before the agent and stops when it ends, fails or
is interrupted; if a camera sends no frames, the run fails before the agent starts.

    results/<timestamp>_<name>/
        transcript.json  as for run_agent.py, plus "recording": per camera its file, name,
                         start time, duration and frame counts
        camera0.mp4      fixed camera
        camera1.mp4      gripper camera
        final_output.md  the final answer, or the error when the run failed
        run.log          agent and recorder log
"""

import argparse
import logging
import re
import sys
from datetime import datetime
from pathlib import Path

from dotenv import load_dotenv

from agents.cameras import CameraRecorder
from agents.claude_cli import ConsoleRenderer, load_claude_cli_config
from agents.claude_cli.session import read_prompt, run_and_record
from agents.core import ConfigError, Settings
from agents.core.config import read_yaml

DEFAULT_CONFIG = Path("configs/switch/ethernet_exp.yaml")
DEFAULT_VISION = Path("configs/vision.yaml")
DEFAULT_RESULTS = Path("results")


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("prompt", nargs="?", help="the prompt (default: --prompt-file or stdin)")
    ap.add_argument("--prompt-file", type=Path, help="read the prompt from this file")
    ap.add_argument("--name", help="run name for the results folder (default: ethernet_<time>)")
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="Claude CLI agent config")
    ap.add_argument("--model", help="model; overrides the one in --config")
    ap.add_argument("--results-dir", type=Path, default=DEFAULT_RESULTS)
    ap.add_argument(
        "--dashboard-url", help=f"camera dashboard (default: dashboard_url in {DEFAULT_VISION})"
    )
    ap.add_argument("--cameras", type=int, nargs="+", default=[0, 1], help="camera indices")
    ap.add_argument("--fps", type=float, default=30.0, help="video frame rate")
    ap.add_argument("--no-color", action="store_true", help="plain output, no ANSI colours")
    args = ap.parse_args()

    prompt, source = read_prompt(args.prompt, args.prompt_file)
    if not prompt:
        sys.exit("error: the prompt is empty")
    name = re.sub(r"\W", "_", args.name or f"ethernet_{datetime.now():%Y%m%d_%H%M%S}")

    load_dotenv()
    settings = Settings()
    # the terminal shows the session; the agent and recorder logs go to run.log only
    logging.basicConfig(level=settings.log_level, handlers=[logging.NullHandler()])
    try:
        config = load_claude_cli_config(args.config)
        dashboard_url = args.dashboard_url or read_yaml(DEFAULT_VISION)["dashboard_url"]
    except (ConfigError, KeyError) as exc:
        sys.exit(f"error: {exc}")
    if args.model:
        config = config.model_copy(update={"model": args.model})

    def recorder(run_dir: Path) -> CameraRecorder:
        return CameraRecorder(dashboard_url, run_dir, args.cameras, fps=args.fps)

    renderer = ConsoleRenderer(color=False if args.no_color else None)
    record, run_dir = run_and_record(
        config, args.config, prompt, source, name, args.results_dir, settings, renderer, recorder
    )
    for cam in (record.get("recording") or {}).get("cameras", []):
        if cam["file"]:
            print(f"video {run_dir / cam['file']} ({cam['name']}, {cam['duration_s']:.1f} s)")
    if record["error"]:
        sys.exit(f"error: {record['error']}\ntranscript in {run_dir / 'transcript.json'}")
    print(f"saved {run_dir / 'transcript.json'}")


if __name__ == "__main__":
    main()
