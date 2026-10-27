import importlib.util
import io
import json
import sys
from pathlib import Path
from typing import Any

import pytest

from agents.claude_cli import (
    ClaudeCLIAgent,
    ClaudeCLIAgentConfig,
    ClaudeCLIError,
    ClaudeCLITask,
    ConsoleRenderer,
    load_claude_cli_config,
    session,
)
from agents.claude_cli.session import RecorderError
from agents.core import AgentError, Settings

SCRIPTS = Path(__file__).resolve().parents[2] / "scripts"

INIT = {"type": "system", "subtype": "init", "session_id": "s1", "model": "m", "tools": ["Read"]}
PARTIALS = [
    {"type": "stream_event", "event": {"type": "message_start"}},
    {
        "type": "stream_event",
        "event": {"type": "content_block_start", "content_block": {"type": "text"}},
    },
    {
        "type": "stream_event",
        "event": {"type": "content_block_delta", "delta": {"type": "text_delta", "text": "Look"}},
    },
    {"type": "stream_event", "event": {"type": "content_block_stop"}},
]
ASSISTANT = {
    "type": "assistant",
    "message": {
        "content": [
            {"type": "text", "text": "Look"},
            {"type": "tool_use", "id": "t1", "name": "Read", "input": {"file_path": "a.txt"}},
        ]
    },
}
TOOL_RESULT = {
    "type": "user",
    "message": {"content": [{"type": "tool_result", "tool_use_id": "t1", "content": "hello"}]},
}
FINAL = {"type": "assistant", "message": {"content": [{"type": "text", "text": "It says hello."}]}}
RESULT = {
    "type": "result",
    "subtype": "success",
    "is_error": False,
    "result": "It says hello.",
    "num_turns": 2,
    "duration_ms": 1500,
    "total_cost_usd": 0.01,
    "usage": {"input_tokens": 3, "output_tokens": 5},
}
SESSION = [INIT, *PARTIALS, ASSISTANT, TOOL_RESULT, FINAL, RESULT]


def fake_cli(tmp_path: Path, events: list[dict[str, Any]], exit_code: int = 0) -> str:
    """A stand-in `claude` that prints `events` as stream-json and records its argv."""
    lines = "\n".join(json.dumps(e) for e in events)
    path = tmp_path / "claude"
    path.write_text(
        f"#!{sys.executable}\n"
        "import sys, pathlib\n"
        f"pathlib.Path({str(tmp_path / 'argv.json')!r}).write_text(__import__('json').dumps(sys.argv))\n"  # noqa: E501
        f"print({lines!r})\n"
        "print('some warning', file=sys.stderr)\n"
        f"sys.exit({exit_code})\n"
    )
    path.chmod(0o755)
    return str(path)


def make_agent(tmp_path: Path, events: list[dict[str, Any]], **config: Any) -> ClaudeCLIAgent:
    cfg = ClaudeCLIAgentConfig(cwd=tmp_path, **config)
    seen: list[dict[str, Any]] = []
    agent = ClaudeCLIAgent(
        cfg, Settings(), on_event=seen.append, cli_path=fake_cli(tmp_path, events)
    )
    agent.seen = seen  # type: ignore[attr-defined]
    return agent


async def test_run_records_steps_and_result(tmp_path: Path) -> None:
    agent = make_agent(tmp_path, SESSION, model="m", allowed_tools=["Read"], max_turns=5)
    result = await agent.run(ClaudeCLITask(prompt="read a.txt"))

    assert result.final_output == "It says hello."
    assert (result.session_id, result.num_turns, result.duration_s) == ("s1", 2, 1.5)
    assert len(agent.seen) == len(SESSION)  # type: ignore[attr-defined]

    t = agent.transcript
    assert t and t.status == "success" and t.model == "m" and t.tools == ["Read"]
    assert [s["kind"] for s in t.steps] == ["text", "tool_use", "text"]
    call = t.steps[1]
    assert call["name"] == "Read" and call["result"] == "hello" and call["is_error"] is False
    assert all(e["type"] != "stream_event" for e in t.events)  # partials not recorded
    assert "some warning" in t.stderr

    argv = json.loads((tmp_path / "argv.json").read_text())
    assert argv[1:5] == ["-p", "read a.txt", "--output-format", "stream-json"]
    assert "--include-partial-messages" in argv
    assert argv[argv.index("--allowedTools") + 1] == "Read"
    assert argv[argv.index("--max-turns") + 1] == "5"


async def test_record_partial_keeps_stream_events(tmp_path: Path) -> None:
    agent = make_agent(tmp_path, SESSION, record_partial=True)
    await agent.run(ClaudeCLITask(prompt="p"))
    assert agent.transcript and len(agent.transcript.events) == len(SESSION)


async def test_error_result_fails_but_keeps_transcript(tmp_path: Path) -> None:
    error = {**RESULT, "subtype": "error_max_turns", "is_error": True, "result": None}
    agent = make_agent(tmp_path, [INIT, ASSISTANT, error])
    with pytest.raises(AgentError) as exc:
        await agent.run(ClaudeCLITask(prompt="p"))
    assert isinstance(exc.value.cause, ClaudeCLIError)
    assert agent.transcript and agent.transcript.status == "error_max_turns"
    assert agent.transcript.steps[1]["name"] == "Read"


async def test_missing_result_fails(tmp_path: Path) -> None:
    agent = make_agent(tmp_path, [INIT])
    with pytest.raises(AgentError, match="without a result: some warning"):
        await agent.run(ClaudeCLITask(prompt="p"))
    assert agent.transcript and agent.transcript.status == "failed"


def test_renderer_streams_text_and_tool_calls() -> None:
    out = io.StringIO()
    render = ConsoleRenderer(out, color=False)
    for event in SESSION:
        render(event)
    text = out.getvalue()
    assert text.count("Look") == 1  # streamed once, not repeated from the full message
    assert "⏺ Read(a.txt)" in text
    assert "⎿  hello" in text
    assert "⏺ It says hello." in text  # not streamed, so shown from the full message
    assert "success · 2 turns · 1.5 s" in text


def test_load_config_resolves_cwd(tmp_path: Path) -> None:
    (tmp_path / "c.yaml").write_text("cwd: sub\nmodel: m\n")
    config = load_claude_cli_config(tmp_path / "c.yaml")
    assert config.cwd == (tmp_path / "sub").resolve()


class FakeRecording:
    def __init__(self, run_dir: Path) -> None:
        self.run_dir = run_dir
        self.open = False

    def __enter__(self) -> "FakeRecording":
        self.open = True
        return self

    def __exit__(self, *_: Any) -> None:
        self.open = False

    def summary(self) -> dict[str, Any]:
        return {"cameras": [{"file": "camera0.mp4"}], "open": self.open}


def failing_recorder(run_dir: Path) -> Any:
    raise RecorderError("no frames from camera(s) [1]")


def record_run(
    tmp_path: Path, events: list[dict[str, Any]], monkeypatch: pytest.MonkeyPatch, **kw: Any
) -> tuple[dict[str, Any], Path]:
    monkeypatch.setattr(session, "ClaudeCLIAgent", _with_cli(fake_cli(tmp_path, events)))
    _, run_dir = session.run_and_record(
        ClaudeCLIAgentConfig(cwd=tmp_path),
        Path("c.yaml"),
        "p",
        "argument",
        "t",
        tmp_path / "results",
        Settings(),
        **kw,
    )
    return json.loads((run_dir / "transcript.json").read_text()), run_dir


@pytest.mark.parametrize("events", [SESSION, [INIT]])
def test_run_and_record_writes_transcript(
    tmp_path: Path, events: list[dict[str, Any]], monkeypatch: pytest.MonkeyPatch
) -> None:
    saved, run_dir = record_run(tmp_path, events, monkeypatch)
    final = (run_dir / "final_output.md").read_text()
    if events is SESSION:
        assert saved["status"] == "success" and saved["cli_status"] == "success"
        assert saved["final_output"] == "It says hello." and final == "It says hello.\n"
        assert [s["kind"] for s in saved["steps"]] == ["text", "tool_use", "text"]
    else:
        assert saved["status"] == "failed" and saved["error"]
        assert final.startswith("**Run failed:**")
    assert saved["prompt"] == "p" and saved["events"][0]["type"] == "system"
    assert "recording" not in saved


def test_run_and_record_saves_recording_summary(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    saved, run_dir = record_run(tmp_path, SESSION, monkeypatch, recorder=FakeRecording)
    assert saved["status"] == "success"
    assert saved["recording"] == {"cameras": [{"file": "camera0.mp4"}], "open": False}


def test_recorder_error_fails_before_the_agent(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    saved, _ = record_run(tmp_path, SESSION, monkeypatch, recorder=failing_recorder)
    assert saved["status"] == "failed" and "camera(s) [1]" in saved["error"]
    assert not (tmp_path / "argv.json").exists()  # the CLI never started


@pytest.mark.parametrize("name", ["run_agent.py", "run_ethernet_exp.py"])
def test_scripts_import(name: str) -> None:
    spec = importlib.util.spec_from_file_location(name[:-3], SCRIPTS / name)
    assert spec and spec.loader
    spec.loader.exec_module(importlib.util.module_from_spec(spec))


def _with_cli(cli: str) -> type[ClaudeCLIAgent]:
    class Agent(ClaudeCLIAgent):
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            super().__init__(*args, cli_path=cli, **kwargs)

    return Agent


async def test_path_goes_in_front_of_the_cli_path(tmp_path: Path) -> None:
    events = [INIT, RESULT]
    cli = fake_cli(tmp_path, events)
    probe = tmp_path / "claude"
    probe.write_text(
        probe.read_text().replace(
            "import sys, pathlib\n",
            f"import sys, pathlib, os\npathlib.Path({str(tmp_path / 'path.txt')!r})"
            ".write_text(os.environ['PATH'])\n",
        )
    )
    config = ClaudeCLIAgentConfig(cwd=tmp_path, path=[Path("/opt/venv/bin")])
    await ClaudeCLIAgent(config, Settings(), cli_path=cli).run(ClaudeCLITask(prompt="p"))
    assert (tmp_path / "path.txt").read_text().startswith("/opt/venv/bin:")


def test_load_config_resolves_path_entries(tmp_path: Path) -> None:
    (tmp_path / "c.yaml").write_text("path: [venv/bin, /abs/bin]\n")
    config = load_claude_cli_config(tmp_path / "c.yaml")
    assert config.path == [(tmp_path / "venv/bin").resolve(), Path("/abs/bin")]


def test_ethernet_config_loads() -> None:
    config = load_claude_cli_config(SCRIPTS.parent / "configs/switch/ethernet_exp.yaml")
    assert "Bash" in config.allowed_tools and "Bash(rm -rf *)" in config.disallowed_tools
    assert "--strict-mcp-config" in config.extra_args
