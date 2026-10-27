#!/usr/bin/env python3
"""Run a prompt through the Claude CLI agent, show the session live and save it under results/.

The prompt is the positional argument, or is read from --prompt-file, or from stdin:

    uv run scripts/run_agent.py "Summarise what urcore/urexec.py does"
    uv run scripts/run_agent.py --prompt-file prompts/review.txt --name review
    echo "List the configs" | uv run scripts/run_agent.py --model claude-sonnet-5-5

The terminal shows the model's text as it is written, each tool call and a short form of
its result, and the turns, time, tokens and cost at the end. Each run gets its own folder,
written even when the run fails or is interrupted:

    results/<timestamp>_<name>/
        transcript.json  run metadata, the step timeline (text and tool calls with their
                         results), the final output, usage and cost, and every CLI event
        final_output.md  the final answer, or the error when the run failed
        run.log          agent log
"""

import argparse
import logging
import re
import sys
from datetime import datetime
from pathlib import Path

from dotenv import load_dotenv

from agents.claude_cli import ConsoleRenderer, load_claude_cli_config
from agents.claude_cli.session import read_prompt, run_and_record
from agents.core import ConfigError, Settings

DEFAULT_CONFIG = Path("configs/agents/claude_cli.yaml")
DEFAULT_RESULTS = Path("results")


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("prompt", nargs="?", help="the prompt (default: --prompt-file or stdin)")
    ap.add_argument("--prompt-file", type=Path, help="read the prompt from this file")
    ap.add_argument("--name", help="run name for the results folder (default: claude_<time>)")
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="Claude CLI agent config")
    ap.add_argument("--model", help="model; overrides the one in --config")
    ap.add_argument("--results-dir", type=Path, default=DEFAULT_RESULTS)
    ap.add_argument("--no-color", action="store_true", help="plain output, no ANSI colours")
    args = ap.parse_args()

    prompt, source = read_prompt(args.prompt, args.prompt_file)
    if not prompt:
        sys.exit("error: the prompt is empty")
    name = re.sub(r"\W", "_", args.name or f"claude_{datetime.now():%Y%m%d_%H%M%S}")

    load_dotenv()
    settings = Settings()
    # the terminal shows the session; the agent log goes to run.log only
    logging.basicConfig(level=settings.log_level, handlers=[logging.NullHandler()])
    try:
        config = load_claude_cli_config(args.config)
    except ConfigError as exc:
        sys.exit(f"error: {exc}")
    if args.model:
        config = config.model_copy(update={"model": args.model})

    renderer = ConsoleRenderer(color=False if args.no_color else None)
    record, run_dir = run_and_record(
        config, args.config, prompt, source, name, args.results_dir, settings, renderer
    )
    if record["error"]:
        sys.exit(f"error: {record['error']}\ntranscript in {run_dir / 'transcript.json'}")
    print(f"\nsaved {run_dir / 'transcript.json'}")


if __name__ == "__main__":
    main()
