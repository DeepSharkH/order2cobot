#!/usr/bin/env python3
"""Generate a .urpx program with the simple agent and save it, with metadata, under results/.

The task prompt is read from --prompt-file, or from stdin when no file is given:

    echo "Build a basic pick-and-place program ..." | uv run scripts/generate_urpx.py
    uv run scripts/generate_urpx.py --prompt-file prompts/pick_place.txt --name pick_place

Each run gets its own folder, written even when the build fails:

    results/<timestamp>_<name>/
        <name>.urpx      the program (only on success)
        metadata.json    agent, model, prompt, timings, status, summary or error
        run.log          tool calls and agent log
"""

import argparse
import asyncio
import logging
import re
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Literal

from dotenv import load_dotenv

from agents.core import AgentError, AppModel, ConfigError, Settings
from agents.simple import SimpleAgentConfig, SimpleTask, SingleAgent, load_simple_config

DEFAULT_CONFIG = Path("configs/agents/simple.yaml")
DEFAULT_RESULTS = Path("results")


class RunMetadata(AppModel):
    agent: str
    status: Literal["success", "failed"]
    program_name: str
    urpx_file: str | None
    prompt: str
    prompt_source: str
    llm: dict[str, Any]
    agent_config: dict[str, Any]
    config_file: str
    git_commit: str | None
    started_at: datetime
    finished_at: datetime
    duration_s: float
    summary: str | None = None
    error: str | None = None


def read_prompt(prompt_file: Path | None) -> tuple[str, str]:
    """Return (prompt, source) from the file, or from stdin when no file is given."""
    if prompt_file:
        return prompt_file.read_text().strip(), str(prompt_file)
    if sys.stdin.isatty():
        sys.exit("error: pipe the prompt on stdin or pass --prompt-file")
    return sys.stdin.read().strip(), "stdin"


def git_commit() -> str | None:
    try:
        out = subprocess.run(
            ["git", "describe", "--always", "--dirty"], capture_output=True, text=True, check=True
        )
    except (OSError, subprocess.CalledProcessError):
        return None
    return out.stdout.strip()


def attach_run_log(path: Path) -> logging.Handler:
    handler = logging.FileHandler(path)
    handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(name)s %(message)s"))
    logging.getLogger().addHandler(handler)
    return handler


def generate(
    config: SimpleAgentConfig,
    config_file: Path,
    prompt: str,
    prompt_source: str,
    name: str,
    results_dir: Path,
    settings: Settings,
) -> tuple[RunMetadata, Path]:
    started = datetime.now(timezone.utc)
    run_dir = results_dir / f"{started:%Y%m%d_%H%M%S}_{name}"
    run_dir.mkdir(parents=True, exist_ok=False)
    output_file = run_dir / f"{name}.urpx"
    handler = attach_run_log(run_dir / "run.log")

    agent = SingleAgent(config, settings)
    summary: str | None = None
    error: str | None = None
    t0 = time.perf_counter()
    try:
        result = asyncio.run(agent.run(SimpleTask(prompt=prompt, output_file=output_file)))
        summary = result.summary
    except AgentError as exc:
        error = str(exc)
    finally:
        logging.getLogger().removeHandler(handler)
        handler.close()

    metadata = RunMetadata(
        agent=agent.name,
        status="failed" if error else "success",
        program_name=name,
        urpx_file=output_file.name if output_file.exists() else None,
        prompt=prompt,
        prompt_source=prompt_source,
        llm=config.llm.model_dump(mode="json"),
        agent_config=config.model_dump(mode="json", exclude={"llm"}),
        config_file=str(config_file),
        git_commit=git_commit(),
        started_at=started,
        finished_at=datetime.now(timezone.utc),
        duration_s=round(time.perf_counter() - t0, 1),
        summary=summary,
        error=error,
    )
    (run_dir / "metadata.json").write_text(metadata.model_dump_json(indent=2) + "\n")
    return metadata, run_dir


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "--prompt-file", type=Path, help="read the prompt from this file (default: stdin)"
    )
    ap.add_argument(
        "--name",
        help="program name on the robot; must not exist there yet (default: task_<timestamp>)",
    )
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="simple agent config")
    ap.add_argument("--llm", type=Path, help="LLM config; overrides the one in --config")
    ap.add_argument("--results-dir", type=Path, default=DEFAULT_RESULTS)
    args = ap.parse_args()

    prompt, source = read_prompt(args.prompt_file)
    if not prompt:
        sys.exit("error: the prompt is empty")
    name = re.sub(r"\W", "_", args.name or f"task_{datetime.now():%Y%m%d_%H%M%S}")

    load_dotenv()
    settings = Settings()
    logging.basicConfig(level=settings.log_level, format="%(asctime)s %(name)s %(message)s")
    try:
        config = load_simple_config(args.config, llm=args.llm)
    except ConfigError as exc:
        sys.exit(f"error: {exc}")

    metadata, run_dir = generate(
        config, args.config, prompt, source, name, args.results_dir, settings
    )
    if metadata.error:
        sys.exit(f"error: {metadata.error}\nmetadata in {run_dir}")
    print(metadata.summary)
    print(f"\nsaved {run_dir / (metadata.urpx_file or '')}")


if __name__ == "__main__":
    main()
