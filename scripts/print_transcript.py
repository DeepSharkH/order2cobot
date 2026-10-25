#!/usr/bin/env python3
"""Print a saved Claude CLI session (transcript.json) as a readable log on stdout.

    uv run scripts/print_transcript.py results/<run>/transcript.json
    uv run scripts/print_transcript.py results/<run>                    # the run folder works too
    uv run scripts/print_transcript.py results/<run> --result-lines 20  # shorten tool output
    uv run scripts/print_transcript.py results/<run> --no-results | less -R

Prints the run header (prompt, model, times), then every step in order: Claude's text,
each tool call with its full input (the Bash command, the file read, ...) and the tool's
output, and finally the run's status, turns, tokens and cost. Times are minutes:seconds
since the run started. Thinking is only shown when the transcript kept its text.
"""

import argparse
import json
import os
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

from agents.claude_cli.render import MAIN_ARG

STYLES = {"dim": "2", "bold": "1", "red": "31", "green": "32", "cyan": "36"}


class Printer:
    def __init__(self, color: bool, result_lines: int | None, show_results: bool) -> None:
        self.color = color
        self.result_lines = result_lines
        self.show_results = show_results

    def style(self, name: str, text: str) -> str:
        return f"\033[{STYLES[name]}m{text}\033[0m" if self.color and text else text

    def header(self, d: dict[str, Any]) -> None:
        rule = self.style("dim", "─" * 72)
        print(rule)
        print(self.style("bold", f"Run {d.get('name')}") + f"  ({d.get('status')})")
        fields = [
            ("prompt", d.get("prompt")),
            ("model", d.get("model")),
            ("session", d.get("session_id")),
            ("cwd", (d.get("agent_config") or {}).get("cwd") or d.get("cwd")),
            ("git", d.get("git_commit")),
            ("started", d.get("started_at")),
            ("finished", d.get("finished_at")),
        ]
        for key, value in fields:
            if value:
                print(f"  {key:<9}{value}")
        recording = d.get("recording") or {}
        for cam in recording.get("cameras") or []:
            print(f"  {'video':<9}{cam.get('file')} ({cam.get('name')}, {cam.get('duration_s')} s)")
        print(rule)
        print()

    def step(self, step: dict[str, Any], t0: datetime | None) -> None:
        stamp = self.style("dim", f"[{elapsed(step.get('at'), t0)}]")
        kind = step.get("kind")
        if kind == "text":
            print(f"{stamp} {self.style('cyan', '⏺ Claude')}")
            print(indent(step.get("text", "").strip(), "  "))
            print()
        elif kind == "thinking":
            if step.get("text", "").strip():
                print(f"{stamp} {self.style('dim', '✻ Thinking')}")
                print(self.style("dim", indent(step["text"].strip(), "  ")))
                print()
        elif kind == "tool_use":
            self.tool(step, stamp)

    def tool(self, step: dict[str, Any], stamp: str) -> None:
        name = step.get("name", "?")
        args = dict(step.get("input") or {})
        title = args.pop("description", None)
        print(
            f"{stamp} {self.style('green', '⏺')} {self.style('bold', name)}"
            + (f"  {title}" if title else "")
        )
        key = MAIN_ARG.get(name)
        if key and isinstance(args.get(key), str):
            print(indent(args.pop(key).rstrip(), "  │ "))
        for k, v in args.items():
            text = v if isinstance(v, str) else json.dumps(v, ensure_ascii=False)
            print(indent(f"{k}: {text}".rstrip(), "  │ "))
        if self.show_results:
            self.result(step)
        print()

    def result(self, step: dict[str, Any]) -> None:
        text = result_text(step.get("result")).rstrip() or "(no output)"
        lines = text.splitlines()
        if self.result_lines is not None and len(lines) > self.result_lines:
            lines = lines[: self.result_lines] + [f"… +{len(lines) - self.result_lines} lines"]
        style = "red" if step.get("is_error") else "dim"
        for i, line in enumerate(lines):
            print(self.style(style, ("  ⎿  " if i == 0 else "     ") + line))

    def footer(self, d: dict[str, Any]) -> None:
        r = d.get("result") or {}
        usage = r.get("usage") or {}
        tokens_in = usage.get("input_tokens", 0) + usage.get("cache_creation_input_tokens", 0)
        status = r.get("subtype") or d.get("status") or "?"
        ok = status == "success" and not r.get("is_error") and not d.get("error")
        print(self.style("dim", "─" * 72))
        print(
            f"{self.style('green' if ok else 'red', status)} · {r.get('num_turns')} turns · "
            f"{(r.get('duration_ms') or 0) / 1000:.1f} s · "
            f"tokens in {tokens_in} (+{usage.get('cache_read_input_tokens', 0)} cached) "
            f"out {usage.get('output_tokens', 0)} · ${r.get('total_cost_usd') or 0:.2f}"
        )
        for key in ("error", "cli_error", "stderr"):
            if d.get(key):
                print(self.style("red", f"{key}: {d[key]}"))


def result_text(result: Any) -> str:
    """Tool results are a string, or a list of content blocks (text, images)."""
    if isinstance(result, str):
        return result
    if isinstance(result, list):
        parts = []
        for block in result:
            if isinstance(block, dict) and block.get("type") == "text":
                parts.append(block.get("text", ""))
            elif isinstance(block, dict):
                parts.append(f"[{block.get('type', '?')}]")
            else:
                parts.append(str(block))
        return "\n".join(parts)
    return "" if result is None else json.dumps(result, ensure_ascii=False, indent=2)


def parse_time(value: str | None) -> datetime | None:
    if not value:
        return None
    try:
        return datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        return None


def elapsed(at: str | None, t0: datetime | None) -> str:
    t = parse_time(at)
    if t is None or t0 is None:
        return "--:--"
    s = max(0, int((t - t0).total_seconds()))
    return f"{s // 60:02d}:{s % 60:02d}"


def indent(text: str, prefix: str) -> str:
    return "\n".join(prefix + line for line in text.splitlines())


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("transcript", type=Path, help="transcript.json, or the run folder holding it")
    ap.add_argument(
        "-n", "--result-lines", type=int, help="show at most this many lines of each tool result"
    )
    ap.add_argument("--no-results", action="store_true", help="leave out tool results")
    ap.add_argument(
        "--color", choices=["auto", "always", "never"], default="auto", help="ANSI colors"
    )
    args = ap.parse_args()

    path = args.transcript / "transcript.json" if args.transcript.is_dir() else args.transcript
    try:
        d = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        sys.exit(f"error: {path}: {exc}")

    color = args.color == "always" or (
        args.color == "auto" and sys.stdout.isatty() and "NO_COLOR" not in os.environ
    )
    p = Printer(color, args.result_lines, not args.no_results)
    t0 = parse_time(d.get("started_at"))
    p.header(d)
    for step in d.get("steps") or []:
        p.step(step, t0)
    p.footer(d)


if __name__ == "__main__":
    try:
        main()
    except BrokenPipeError:  # e.g. piped into head
        sys.stderr.close()
