import json
from pathlib import Path
from typing import Any

import pytest

from agents.core import AgentError
from agents.llm import LLMConfig, ToolHandler, ToolLoop, ToolSpec
from agents.robot import BLOCKED_TOOLS, RobotConfig
from agents.simple import SimpleAgentConfig, SimpleTask, SingleAgent, load_simple_config

CONFIGS = Path(__file__).resolve().parents[2] / "configs"


class FakeBridge:
    def __init__(self, tier: str = "adjust", programs: set[str] | None = None) -> None:
        self.tier = tier
        self.programs = programs if programs is not None else set()
        self.calls: list[str] = []

    async def __aenter__(self) -> "FakeBridge":
        return self

    async def __aexit__(self, *exc: object) -> None:
        pass

    async def call(self, tool: str, **args: Any) -> Any:
        assert tool == "bridge_my_status"
        effects = ["read", "write"] if self.tier == "adjust" else ["read"]
        return {"client": "test", "tier": self.tier, "allowedEffects": effects}

    async def program_names(self) -> set[str]:
        return self.programs

    async def list_tools(self) -> list[dict[str, Any]]:
        schema = {"$schema": "x", "type": "object"}
        return [
            {"name": "program_create", "description": "d", "inputSchema": schema},
            {"name": "program_run", "description": "d", "inputSchema": schema},
        ]

    async def call_raw(self, tool: str, args: dict[str, Any]) -> tuple[str, bool]:
        self.calls.append(tool)
        if tool == "program_create":
            self.programs.add(args["name"])
        return json.dumps({"status": "success"}), False

    async def download_urpx(self, name: str) -> bytes:
        return b"urpx:" + name.encode()


class ScriptedLoop(ToolLoop):
    """Calls the given tools in order, then returns a summary."""

    def __init__(self, script: list[tuple[str, dict[str, Any]]]) -> None:
        super().__init__(LLMConfig(provider="openai", model="fake"), max_turns=5)
        self.script = script
        self.offered: list[str] = []
        self.results: list[tuple[str, bool]] = []

    async def run(
        self, system: str, task: str, tools: list[ToolSpec], call_tool: ToolHandler
    ) -> str:
        self.offered = [t.name for t in tools]
        for turn, (name, args) in enumerate(self.script):
            self.results.append(await call_tool(name, args, turn))
        return "built it"


def make_agent(bridge: FakeBridge, loop: ScriptedLoop) -> SingleAgent:
    config = SimpleAgentConfig(robot=RobotConfig(), llm=LLMConfig(provider="openai", model="fake"))
    return SingleAgent(config, tool_loop=loop, bridge_factory=lambda *_: bridge)  # type: ignore[arg-type]


async def test_builds_and_downloads_program(tmp_path: Path) -> None:
    bridge = FakeBridge()
    loop = ScriptedLoop([("program_create", {"name": "pick_place"})])
    out = tmp_path / "pick-place.urpx"

    result = await make_agent(bridge, loop).run(SimpleTask(prompt="pick", output_file=out))

    assert result.program_name == "pick_place"
    assert result.summary == "built it"
    assert out.read_bytes() == b"urpx:pick_place"


async def test_blocked_tools_are_not_offered_or_relayed(tmp_path: Path) -> None:
    bridge = FakeBridge()
    loop = ScriptedLoop([("program_run", {}), ("program_create", {"name": "p"})])

    await make_agent(bridge, loop).run(SimpleTask(prompt="x", output_file=tmp_path / "p.urpx"))

    assert loop.offered == ["program_create"]
    assert loop.results[0][1] is True
    assert "program_run" not in bridge.calls


async def test_requires_adjust_access(tmp_path: Path) -> None:
    loop = ScriptedLoop([])
    with pytest.raises(AgentError, match="Adjust"):
        await make_agent(FakeBridge(tier="observe"), loop).run(
            SimpleTask(prompt="x", output_file=tmp_path / "p.urpx")
        )


async def test_refuses_existing_program_name(tmp_path: Path) -> None:
    loop = ScriptedLoop([])
    with pytest.raises(AgentError, match="already exists"):
        await make_agent(FakeBridge(programs={"p"}), loop).run(
            SimpleTask(prompt="x", output_file=tmp_path / "p.urpx")
        )


async def test_fails_when_model_creates_nothing(tmp_path: Path) -> None:
    with pytest.raises(AgentError, match="no program named"):
        await make_agent(FakeBridge(), ScriptedLoop([])).run(
            SimpleTask(prompt="x", output_file=tmp_path / "p.urpx")
        )


async def test_rejects_non_urpx_output(tmp_path: Path) -> None:
    with pytest.raises(AgentError, match=r"\.urpx"):
        await make_agent(FakeBridge(), ScriptedLoop([])).run(
            SimpleTask(prompt="x", output_file=tmp_path / "p.txt")
        )


@pytest.mark.parametrize("llm", sorted((CONFIGS / "llm").glob("*.yaml")), ids=lambda p: p.stem)
def test_shipped_configs_load(llm: Path) -> None:
    config = load_simple_config(CONFIGS / "agents" / "simple.yaml", llm=llm)
    assert config.robot.cert_path.is_absolute()
    assert config.llm.model


def test_blocked_list_covers_motion() -> None:
    assert {"program_run", "signal_set", "robot_set_robot_state"} <= BLOCKED_TOOLS


def test_default_cert_path_is_expanded() -> None:
    assert RobotConfig().cert_path.is_absolute()
