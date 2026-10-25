"""Claude CLI agent: run one prompt through the logged-in `claude` CLI and record the session.

The CLI is started in print mode with `--output-format stream-json`, so every step it takes
arrives as one JSON event per line: the session setup, the model's text, each tool call and
its result, and the final result with usage and cost. The agent hands each event to an
optional `on_event` callback as it arrives (for live display) and collects the session in a
`Transcript`: all events, a step-by-step timeline with tool calls paired to their results,
and the final output. The transcript is kept on the agent even when the run fails, so the
caller can save what happened up to that point.
"""

import asyncio
import json
import os
import shutil
from collections.abc import Callable
from datetime import datetime
from pathlib import Path
from typing import Any

from pydantic import BaseModel, Field

from agents.claude_cli.config import ClaudeCLIAgentConfig
from agents.core import AppModel, BaseAgent, Settings
from agents.core.models import utc_now

EventHandler = Callable[[dict[str, Any]], None]

LINE_LIMIT = 64 * 1024 * 1024  # one event per line; tool results can be large


class ClaudeCLIError(RuntimeError):
    """The CLI could not start, or the session ended without a successful result."""


class ClaudeCLITask(AppModel):
    prompt: str = Field(min_length=1)


class ClaudeCLIResult(AppModel):
    session_id: str | None
    final_output: str
    num_turns: int | None
    duration_s: float | None
    total_cost_usd: float | None


class Transcript(BaseModel):
    """Everything one CLI session produced, in the order it arrived."""

    command: list[str]
    cwd: str
    prompt: str
    started_at: datetime = Field(default_factory=utc_now)
    finished_at: datetime | None = None
    status: str = "running"  # running, then the result subtype (success, error_max_turns, ...)
    session_id: str | None = None
    model: str | None = None
    tools: list[str] = Field(default_factory=list)
    steps: list[dict[str, Any]] = Field(default_factory=list)
    final_output: str | None = None
    result: dict[str, Any] | None = None  # the CLI's result event: usage, cost, timings
    error: str | None = None
    stderr: str = ""
    events: list[dict[str, Any]] = Field(default_factory=list)

    def add(self, event: dict[str, Any], record_partial: bool) -> None:
        kind = event.get("type")
        if kind != "stream_event" or record_partial:
            self.events.append(event)
        if kind == "system" and event.get("subtype") == "init":
            self.session_id = event.get("session_id")
            self.model = event.get("model")
            self.tools = list(event.get("tools") or [])
        elif kind == "assistant":
            self._add_assistant(event)
        elif kind == "user":
            self._add_tool_results(event)
        elif kind == "result":
            self.result = event
            self.status = event.get("subtype") or "unknown"
            self.final_output = event.get("result")

    def _add_assistant(self, event: dict[str, Any]) -> None:
        parent = event.get("parent_tool_use_id")
        for block in event.get("message", {}).get("content", []):
            step: dict[str, Any] = {"kind": block.get("type"), "at": utc_now().isoformat()}
            if parent:
                step["parent_tool_use_id"] = parent  # inside a subagent
            if block.get("type") == "text":
                step["text"] = block.get("text", "")
            elif block.get("type") == "thinking":
                step["text"] = block.get("thinking", "")
            elif block.get("type") == "tool_use":
                step.update(id=block.get("id"), name=block.get("name"), input=block.get("input"))
            else:
                step["block"] = block
            self.steps.append(step)

    def _add_tool_results(self, event: dict[str, Any]) -> None:
        content = event.get("message", {}).get("content", [])
        if not isinstance(content, list):
            return
        calls = {s.get("id"): s for s in self.steps if s["kind"] == "tool_use"}
        for block in content:
            if block.get("type") != "tool_result":
                continue
            call = calls.get(block.get("tool_use_id"))
            if call is not None:
                call["result"] = tool_result_text(block)
                call["is_error"] = bool(block.get("is_error"))


def tool_result_text(block: dict[str, Any]) -> str:
    """The text of a tool_result block, whose content is a string or a list of blocks."""
    content = block.get("content")
    if isinstance(content, str):
        return content
    parts = []
    for item in content or []:
        if item.get("type") == "text":
            parts.append(item.get("text", ""))
        else:
            parts.append(f"[{item.get('type')}]")
    return "\n".join(parts)


class ClaudeCLIAgent(BaseAgent[ClaudeCLITask, ClaudeCLIResult]):
    name = "claude_cli"

    def __init__(
        self,
        config: ClaudeCLIAgentConfig,
        settings: Settings | None = None,
        *,
        on_event: EventHandler | None = None,
        cli_path: str | None = None,
    ) -> None:
        super().__init__(settings)
        self.config = config
        self.on_event = on_event
        self.cli_path = cli_path or shutil.which("claude")
        self.transcript: Transcript | None = None

    @property
    def timeout_s(self) -> float:
        return self.config.timeout_s

    async def validate(self, data: ClaudeCLITask) -> None:
        if not self.cli_path:
            raise ClaudeCLIError(
                "the `claude` CLI is not on PATH; install it and run `claude` to log in"
            )

    def command(self, prompt: str) -> list[str]:
        c = self.config
        argv = [str(self.cli_path), "-p", prompt, "--output-format", "stream-json", "--verbose"]
        if c.stream_partial:
            argv.append("--include-partial-messages")
        if c.model:
            argv += ["--model", c.model]
        if c.effort:
            argv += ["--effort", c.effort]
        if c.max_turns:
            argv += ["--max-turns", str(c.max_turns)]
        if c.allowed_tools:
            argv += ["--allowedTools", *c.allowed_tools]
        if c.disallowed_tools:
            argv += ["--disallowedTools", *c.disallowed_tools]
        argv += ["--permission-mode", c.permission_mode]
        if c.append_system_prompt:
            argv += ["--append-system-prompt", c.append_system_prompt]
        return argv + c.extra_args

    async def execute(self, data: ClaudeCLITask) -> ClaudeCLIResult:
        argv = self.command(data.prompt)
        cwd = self.config.cwd or Path.cwd()
        transcript = self.transcript = Transcript(command=argv, cwd=str(cwd), prompt=data.prompt)
        env = dict(os.environ)
        if not self.config.use_api_key:
            env["ANTHROPIC_API_KEY"] = ""
        if self.config.path:
            env["PATH"] = os.pathsep.join([*map(str, self.config.path), env.get("PATH", "")])
        try:
            proc = await asyncio.create_subprocess_exec(
                *argv,
                cwd=cwd,
                env=env,
                stdin=asyncio.subprocess.DEVNULL,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                limit=LINE_LIMIT,
            )
        except OSError as exc:
            transcript.status, transcript.error = "failed", str(exc)
            transcript.finished_at = utc_now()
            raise ClaudeCLIError(f"could not start the claude cli: {exc}") from exc

        try:
            stderr = asyncio.create_task(self._read_stderr(proc))
            await self._read_events(proc, transcript)
            await proc.wait()
            transcript.stderr = await stderr
        except BaseException as exc:  # timeout, Ctrl-C or a failing on_event: stop the CLI
            if proc.returncode is None:
                proc.kill()
                await proc.wait()
            transcript.status = "failed" if transcript.result is None else transcript.status
            transcript.error = transcript.error or repr(exc)
            raise
        finally:
            transcript.finished_at = utc_now()

        return self._finish(transcript, proc.returncode)

    async def _read_events(self, proc: asyncio.subprocess.Process, transcript: Transcript) -> None:
        assert proc.stdout
        async for raw in proc.stdout:
            line = raw.decode(errors="replace").strip()
            if not line:
                continue
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                event = {"type": "raw", "text": line}  # not an event; keep it anyway
            if not isinstance(event, dict):
                event = {"type": "raw", "text": line}
            transcript.add(event, self.config.record_partial)
            if self.on_event:
                self.on_event(event)

    async def _read_stderr(self, proc: asyncio.subprocess.Process) -> str:
        assert proc.stderr
        lines = []
        async for raw in proc.stderr:
            line = raw.decode(errors="replace").rstrip()
            self.log.debug("claude stderr: %s", line)
            lines.append(line)
        return "\n".join(lines)

    def _finish(self, transcript: Transcript, returncode: int | None) -> ClaudeCLIResult:
        result = transcript.result
        if result is None:
            transcript.status = "failed"
            tail = transcript.stderr.strip()[-2000:] or "no output on stderr"
            transcript.error = f"the claude cli exited ({returncode}) without a result: {tail}"
            raise ClaudeCLIError(transcript.error)
        if result.get("is_error") or transcript.status != "success":
            transcript.error = f"claude cli failed ({transcript.status}): {result.get('result')}"
            raise ClaudeCLIError(transcript.error)
        duration_ms = result.get("duration_ms")
        return ClaudeCLIResult(
            session_id=transcript.session_id,
            final_output=(transcript.final_output or "").strip(),
            num_turns=result.get("num_turns"),
            duration_s=duration_ms / 1000 if duration_ms is not None else None,
            total_cost_usd=result.get("total_cost_usd"),
        )
