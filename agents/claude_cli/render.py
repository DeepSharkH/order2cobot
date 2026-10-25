"""Show a Claude CLI stream-json session live in the terminal, like the Claude apps do.

Text is printed as it is generated, each tool call as a `⏺ Tool(argument)` line once its
input is complete, its result as an indented, shortened `⎿` block, and the run's turns,
time, tokens and cost at the end. Steps inside a subagent are indented one level.
"""

import json
import os
import sys
from typing import Any, TextIO

from agents.claude_cli.agent import tool_result_text

# the argument worth showing for common tools; others show their input as compact JSON
MAIN_ARG = {
    "Bash": "command",
    "Read": "file_path",
    "Write": "file_path",
    "Edit": "file_path",
    "NotebookEdit": "notebook_path",
    "Glob": "pattern",
    "Grep": "pattern",
    "WebFetch": "url",
    "WebSearch": "query",
    "Agent": "description",
    "Task": "description",
    "Skill": "skill",
}


class ConsoleRenderer:
    def __init__(
        self, out: TextIO = sys.stdout, *, color: bool | None = None, result_lines: int = 6
    ) -> None:
        self.out = out
        if color is None:
            color = out.isatty() and "NO_COLOR" not in os.environ
        self.color = color
        self.result_lines = result_lines
        self.streamed = False  # text or thinking for the current message came as partials
        self.at_line_start = True
        self.block: str | None = None  # type of the block being streamed

    def __call__(self, event: dict[str, Any]) -> None:
        handler = getattr(self, f"_on_{event.get('type')}", None)
        if handler:
            handler(event)
        self.out.flush()

    # -- events -----------------------------------------------------------------------

    def _on_system(self, event: dict[str, Any]) -> None:
        if event.get("subtype") != "init":
            return
        servers = ", ".join(s.get("name", "?") for s in event.get("mcp_servers") or [])
        self._line(
            self._style("dim", f"● session {event.get('session_id')} · model {event.get('model')}")
        )
        self._line(
            self._style(
                "dim",
                f"  cwd {event.get('cwd')} · {len(event.get('tools') or [])} tools"
                + (f" · mcp: {servers}" if servers else ""),
            )
        )
        self._line("")

    def _on_stream_event(self, event: dict[str, Any]) -> None:
        if event.get("parent_tool_use_id"):
            return  # subagent steps are shown from their full messages
        inner = event.get("event", {})
        kind = inner.get("type")
        if kind == "message_start":
            self.streamed = False
        elif kind == "content_block_start":
            self.block = inner.get("content_block", {}).get("type")
            if self.block == "thinking":
                self._break()
                self._line(self._style("dim", "✻ Thinking…"))
        elif kind == "content_block_delta":
            delta = inner.get("delta", {})
            if delta.get("type") == "text_delta":
                if self.at_line_start and not self.streamed:
                    self._write("⏺ ")
                self.streamed = True
                self._write(delta.get("text", ""))
            elif delta.get("type") == "thinking_delta":
                self.streamed = True
                self._write(self._style("dim", delta.get("thinking", "")))
        elif kind == "content_block_stop":
            if self.block in ("text", "thinking"):
                self._break()
                self._line("")
            self.block = None

    def _on_assistant(self, event: dict[str, Any]) -> None:
        indent = "  " if event.get("parent_tool_use_id") else ""
        streamed = self.streamed and not indent
        for block in event.get("message", {}).get("content", []):
            kind = block.get("type")
            if kind == "text" and not streamed:
                self._line(indent + "⏺ " + block.get("text", "").strip())
                self._line("")
            elif kind == "thinking" and not streamed and block.get("thinking"):
                self._line(indent + self._style("dim", "✻ " + block["thinking"].strip()))
                self._line("")
            elif kind == "tool_use":
                self._break()
                name = block.get("name", "?")
                self._line(
                    f"{indent}{self._style('green', '⏺')} {self._style('bold', name)}"
                    f"({tool_argument(name, block.get('input') or {})})"
                )
        if not indent:
            self.streamed = False

    def _on_user(self, event: dict[str, Any]) -> None:
        content = event.get("message", {}).get("content")
        if not isinstance(content, list):
            return
        indent = "  " if event.get("parent_tool_use_id") else ""
        for block in content:
            if block.get("type") != "tool_result":
                continue
            text = tool_result_text(block).strip() or "(no output)"
            lines = text.splitlines()
            shown = lines[: self.result_lines]
            if len(lines) > len(shown):
                shown.append(f"… +{len(lines) - len(shown)} lines")
            style = "red" if block.get("is_error") else "dim"
            for i, line in enumerate(shown):
                prefix = "  ⎿  " if i == 0 else "     "
                self._line(indent + self._style(style, prefix + clip(line, 200)))
            self._line("")

    def _on_result(self, event: dict[str, Any]) -> None:
        self._break()
        usage = event.get("usage") or {}
        tokens_in = usage.get("input_tokens", 0) + usage.get("cache_creation_input_tokens", 0)
        status = event.get("subtype", "?")
        style = "red" if event.get("is_error") or status != "success" else "green"
        self._line(self._style("dim", "─" * 60))
        self._line(
            f"{self._style(style, status)} · {event.get('num_turns')} turns · "
            f"{(event.get('duration_ms') or 0) / 1000:.1f} s · "
            f"tokens in {tokens_in} (+{usage.get('cache_read_input_tokens', 0)} cached) "
            f"out {usage.get('output_tokens', 0)} · ${event.get('total_cost_usd') or 0:.2f}"
        )
        if event.get("is_error") and event.get("result"):
            self._line(self._style("red", str(event["result"])))

    # -- output helpers ---------------------------------------------------------------

    def _write(self, text: str) -> None:
        if text:
            self.out.write(text)
            self.at_line_start = text.endswith("\n")

    def _line(self, text: str) -> None:
        self._write(text + "\n")

    def _break(self) -> None:
        if not self.at_line_start:
            self._write("\n")

    def _style(self, style: str, text: str) -> str:
        codes = {"dim": "2", "bold": "1", "red": "31", "green": "32"}
        return f"\033[{codes[style]}m{text}\033[0m" if self.color and text else text


def tool_argument(name: str, args: dict[str, Any]) -> str:
    key = MAIN_ARG.get(name)
    if key and isinstance(args.get(key), str):
        return clip(" ".join(args[key].split()), 120)
    return clip(json.dumps(args, ensure_ascii=False), 120)


def clip(text: str, width: int) -> str:
    return text if len(text) <= width else text[: width - 1] + "…"
