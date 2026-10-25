import importlib.util
import json
from pathlib import Path
from types import ModuleType
from typing import Any

import pytest

from agents.core import AgentError, Settings
from agents.llm import LLMConfig
from agents.robot import RobotConfig
from agents.simple import SimpleAgentConfig, SimpleTask, SimpleTaskResult

SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "generate_urpx.py"


def load_script() -> ModuleType:
    spec = importlib.util.spec_from_file_location("generate_urpx", SCRIPT)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeAgent:
    name = "simple"
    summary: str | None = "## Program\n\nBuilt `p`."

    def __init__(self, *_: Any) -> None:
        pass

    async def run(self, task: SimpleTask) -> SimpleTaskResult:
        if self.summary is None:
            raise AgentError(self.name, RuntimeError("no program named 'p'"))
        task.output_file.write_bytes(b"urpx")
        return SimpleTaskResult(
            program_name="p", output_file=task.output_file, summary=self.summary
        )


def run_generate(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    script = load_script()
    monkeypatch.setattr(script, "SingleAgent", FakeAgent)
    config = SimpleAgentConfig(robot=RobotConfig(), llm=LLMConfig(provider="openai", model="x"))
    _, run_dir = script.generate(
        config, Path("c.yaml"), "prompt", "stdin", "p", tmp_path, Settings()
    )
    return Path(run_dir)


def test_writes_final_output_on_success(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    run_dir = run_generate(tmp_path, monkeypatch)

    assert (run_dir / "final_output.md").read_text() == "## Program\n\nBuilt `p`.\n"
    assert json.loads((run_dir / "metadata.json").read_text())["summary"] == FakeAgent.summary


def test_writes_error_as_final_output_on_failure(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(FakeAgent, "summary", None)
    run_dir = run_generate(tmp_path, monkeypatch)

    text = (run_dir / "final_output.md").read_text()
    assert text.startswith("**Build failed:**")
    assert "no program named 'p'" in text
