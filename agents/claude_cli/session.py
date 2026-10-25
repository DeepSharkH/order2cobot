"""Run one prompt through the Claude CLI agent and save the session under a results folder.

Shared by scripts/run_agent.py and scripts/run_ethernet_exp.py. Each run gets its own folder,
written even when the run fails or is interrupted:

    <results_dir>/<timestamp>_<name>/
        transcript.json  run metadata, the step timeline (text and tool calls with their
                         results), the final output, usage and cost, and every CLI event
        final_output.md  the final answer, or the error when the run failed
        run.log          agent log

A `recorder` runs alongside the agent (e.g. camera videos): it gets the run folder, and
whatever its context manager yields goes into transcript.json if it has a `summary()`. A
`RecorderError` when it starts fails the run before the agent starts.
"""

import asyncio
import json
import logging
import subprocess
import sys
from collections.abc import Callable
from contextlib import AbstractContextManager, nullcontext
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from agents.claude_cli.agent import ClaudeCLIAgent, ClaudeCLITask
from agents.claude_cli.config import ClaudeCLIAgentConfig
from agents.claude_cli.render import ConsoleRenderer
from agents.core import AgentError, Settings

Recorder = Callable[[Path], AbstractContextManager[Any]]


class RecorderError(RuntimeError):
    """A recorder could not start or failed while the agent ran."""


def read_prompt(prompt: str | None, prompt_file: Path | None) -> tuple[str, str]:
    """Return (prompt, source) from the argument, the file, or stdin."""
    if prompt:
        return prompt.strip(), "argument"
    if prompt_file:
        return prompt_file.read_text().strip(), str(prompt_file)
    if sys.stdin.isatty():
        sys.exit("error: give the prompt as an argument, with --prompt-file, or on stdin")
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


def run_and_record(
    config: ClaudeCLIAgentConfig,
    config_file: Path,
    prompt: str,
    prompt_source: str,
    name: str,
    results_dir: Path,
    settings: Settings,
    renderer: ConsoleRenderer | None = None,
    recorder: Recorder | None = None,
) -> tuple[dict[str, Any], Path]:
    started = datetime.now(timezone.utc)
    run_dir = results_dir / f"{started:%Y%m%d_%H%M%S}_{name}"
    run_dir.mkdir(parents=True, exist_ok=False)
    handler = attach_run_log(run_dir / "run.log")

    agent = ClaudeCLIAgent(config, settings, on_event=renderer)
    error: str | None = None
    recording: Any = None
    try:
        with recorder(run_dir) if recorder else nullcontext() as recording:
            asyncio.run(agent.run(ClaudeCLITask(prompt=prompt)))
    except AgentError as exc:
        error = str(exc)
    except RecorderError as exc:
        error = f"recording failed: {exc}"
    except KeyboardInterrupt:
        error = "interrupted"
    finally:
        logging.getLogger().removeHandler(handler)
        handler.close()
        record: dict[str, Any] = {
            "agent": agent.name,
            "status": "failed" if error else "success",
            "name": name,
            "prompt_source": prompt_source,
            "config_file": str(config_file),
            "agent_config": config.model_dump(mode="json"),
            "git_commit": git_commit(),
            "error": error,
        }
        if hasattr(recording, "summary"):
            record["recording"] = recording.summary()
        if agent.transcript:
            transcript = agent.transcript.model_dump(mode="json")
            transcript["cli_status"] = transcript.pop("status")
            transcript["cli_error"] = transcript.pop("error")
            record.update(transcript)
        else:  # failed before the CLI started
            record.update(prompt=prompt, started_at=started.isoformat())
        (run_dir / "transcript.json").write_text(
            json.dumps(record, indent=2, ensure_ascii=False) + "\n"
        )
        final = f"**Run failed:** {error}" if error else str(record.get("final_output") or "")
        (run_dir / "final_output.md").write_text(final.strip() + "\n")
    return record, run_dir
