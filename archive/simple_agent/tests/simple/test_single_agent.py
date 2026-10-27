import json
from pathlib import Path
from typing import Any

import pytest

from agents.core import AgentError
from agents.llm import LLMConfig, ToolHandler, ToolLoop, ToolSpec
from agents.robot import BLOCKED_TOOLS, RobotConfig
from agents.simple import SimpleAgentConfig, SimpleTask, SingleAgent, load_simple_config

CONFIGS = Path(__file__).resolve().parents[2] / "configs"


GUID = "{:08d}-0000-0000-0000-000000000000"


class FakeBridge:
    """Just enough of the bridge: a program is a dict of section -> top-level node types."""

    def __init__(
        self, tier: str = "adjust", programs: set[str] | None = None, invalid_pushes: int = 0
    ) -> None:
        self.tier = tier
        self.programs = programs if programs is not None else set()
        self.invalid_pushes = invalid_pushes
        self.calls: list[str] = []
        self.added: dict[str, list[str]] = {}  # guid -> [section, type]
        self._next = 1

    async def __aenter__(self) -> "FakeBridge":
        return self

    async def __aexit__(self, *exc: object) -> None:
        pass

    async def call(self, tool: str, **args: Any) -> Any:
        self.calls.append(tool)
        if tool == "bridge_my_status":
            effects = ["read", "write"] if self.tier == "adjust" else ["read"]
            return {"client": "test", "tier": self.tier, "allowedEffects": effects}
        if tool == "program_list_node_types":
            return {"nodeTypes": [
                {"type": "ur-folder", "allowsChildren": True},
                {"type": "ur-comment", "allowsChildren": False},
            ]}  # fmt: skip
        if tool == "program_create":
            self.programs.add(args["name"])
            return {}
        if tool == "program_list":
            return {"programs": [{"name": n, "id": "7"} for n in self.programs]}
        if tool == "program_get":
            tree = ["ur-program [" + GUID.format(0) + "]"]
            for i, section in enumerate(("ur-modules", "ur-code"), start=1):
                tree.append(f"  {section} [{GUID.format(i * 1000)}]")
                tree += [
                    f"    {kind} [{guid}]"
                    for guid, (sec, kind) in self.added.items()
                    if sec == section
                ]
            return {"programId": "7", "tree": tree}
        if tool == "program_delete_node":
            del self.added[args["guid"]]
            return {}
        if tool == "program_validate":
            if self.invalid_pushes:
                self.invalid_pushes -= 1
                return {"valid": False, "messages": ["move has no waypoint"]}
            return {"valid": True, "messages": [], "hostValidated": True}
        return {}

    async def program_names(self) -> set[str]:
        return self.programs

    async def list_tools(self) -> list[dict[str, Any]]:
        schema = {"$schema": "x", "type": "object"}
        return [
            {"name": n, "description": "d", "inputSchema": schema}
            for n in ("program_describe_node_type", "program_add_node", "program_run")
        ]

    async def call_raw(self, tool: str, args: dict[str, Any]) -> tuple[str, bool]:
        self.calls.append(tool)
        if tool == "program_add_node":
            section = "ur-code" if args["parentGuid"] == GUID.format(2000) else "ur-modules"
            self.added[GUID.format(self._next)] = [section, args["type"]]
            self._next += 1
        return json.dumps({"status": "success"}), False

    async def download_urpx(self, name: str) -> bytes:
        return b"urpx:" + name.encode()


class ScriptedLoop(ToolLoop):
    """Each run calls the next script's tools in order, then returns a summary."""

    def __init__(self, *scripts: list[tuple[str, dict[str, Any]]]) -> None:
        super().__init__(LLMConfig(provider="openai", model="fake"), max_turns=5)
        self.scripts = list(scripts)
        self.offered: list[str] = []
        self.tasks: list[str] = []
        self.results: list[tuple[str, bool]] = []

    async def run(
        self, system: str, task: str, tools: list[ToolSpec], call_tool: ToolHandler
    ) -> str:
        self.offered = [t.name for t in tools]
        self.tasks.append(task)
        for turn, (name, args) in enumerate(self.scripts.pop(0) if self.scripts else []):
            self.results.append(await call_tool(name, args, turn))
        return "built it"


def make_agent(bridge: FakeBridge, loop: ScriptedLoop, lock_dir: Path) -> SingleAgent:
    config = SimpleAgentConfig(
        robot=RobotConfig(lock_dir=lock_dir), llm=LLMConfig(provider="openai", model="fake")
    )
    return SingleAgent(config, tool_loop=loop, bridge_factory=lambda *_: bridge)  # type: ignore[arg-type]


PROGRAM = {
    "sections": {
        "ur-code": [
            {
                "type": "ur-folder",
                "children": [{"type": "ur-comment", "parameters": {"comment": "hi"}}],
            }
        ]
    }
}
SUBMIT = ("program_submit", PROGRAM)


async def test_plans_locally_then_builds_and_downloads(tmp_path: Path) -> None:
    bridge = FakeBridge()
    loop = ScriptedLoop([("program_describe_node_type", {"type": "ur-folder"}), SUBMIT])
    out = tmp_path / "pick-place.urpx"

    result = await make_agent(bridge, loop, tmp_path).run(
        SimpleTask(prompt="pick", output_file=out)
    )

    assert result.program_name == "pick_place"
    assert result.summary == "built it"
    assert out.read_bytes() == b"urpx:pick_place"
    assert json.loads(out.with_suffix(".program.json").read_text()) == PROGRAM["sections"]
    # nothing was created on the robot until the model had finished
    assert bridge.calls.index("program_create") > bridge.calls.index("program_describe_node_type")
    assert bridge.calls.count("program_add_node") == 1


async def test_only_catalog_tools_are_offered_or_relayed(tmp_path: Path) -> None:
    bridge = FakeBridge()
    loop = ScriptedLoop([("program_run", {}), ("program_add_node", {}), SUBMIT])

    await make_agent(bridge, loop, tmp_path).run(
        SimpleTask(prompt="x", output_file=tmp_path / "p.urpx")
    )

    assert loop.offered == ["program_describe_node_type", "program_submit"]
    assert loop.results[0][1] is True
    assert loop.results[1][1] is True
    assert "program_run" not in bridge.calls
    assert bridge.calls.count("program_add_node") == 1  # only the agent's own push


async def test_submit_rejects_unknown_types_and_children(tmp_path: Path) -> None:
    bad = {"sections": {"ur-code": [
        {"type": "ur-teleport"},
        {"type": "ur-comment", "children": [{"type": "ur-comment"}]},
    ]}}  # fmt: skip
    loop = ScriptedLoop([("program_submit", bad), SUBMIT])

    await make_agent(FakeBridge(), loop, tmp_path).run(
        SimpleTask(prompt="x", output_file=tmp_path / "p.urpx")
    )

    text, failed = loop.results[0]
    assert failed
    assert "ur-teleport" in text
    assert "does not allow children" in text
    assert loop.results[1][1] is False


async def test_validation_problems_go_back_to_the_model(tmp_path: Path) -> None:
    bridge = FakeBridge(invalid_pushes=1)
    loop = ScriptedLoop([SUBMIT], [SUBMIT])

    await make_agent(bridge, loop, tmp_path).run(
        SimpleTask(prompt="x", output_file=tmp_path / "p.urpx")
    )

    assert "move has no waypoint" in loop.tasks[1]
    assert bridge.calls.count("program_create") == 1
    assert bridge.calls.count("program_delete_node") == 1  # first push cleared before the second
    assert len(bridge.added) == 1


async def test_fails_when_validation_never_passes(tmp_path: Path) -> None:
    loop = ScriptedLoop([SUBMIT], [SUBMIT], [SUBMIT])
    with pytest.raises(AgentError, match="did not validate"):
        await make_agent(FakeBridge(invalid_pushes=10), loop, tmp_path).run(
            SimpleTask(prompt="x", output_file=tmp_path / "p.urpx")
        )


async def test_requires_adjust_access(tmp_path: Path) -> None:
    loop = ScriptedLoop()
    with pytest.raises(AgentError, match="Adjust"):
        await make_agent(FakeBridge(tier="observe"), loop, tmp_path).run(
            SimpleTask(prompt="x", output_file=tmp_path / "p.urpx")
        )


async def test_refuses_existing_program_name(tmp_path: Path) -> None:
    loop = ScriptedLoop()
    with pytest.raises(AgentError, match="already exists"):
        await make_agent(FakeBridge(programs={"p"}), loop, tmp_path).run(
            SimpleTask(prompt="x", output_file=tmp_path / "p.urpx")
        )


async def test_fails_when_model_submits_nothing(tmp_path: Path) -> None:
    bridge = FakeBridge()
    with pytest.raises(AgentError, match="without an accepted program_submit"):
        await make_agent(bridge, ScriptedLoop(), tmp_path).run(
            SimpleTask(prompt="x", output_file=tmp_path / "p.urpx")
        )
    assert "program_create" not in bridge.calls


async def test_rejects_non_urpx_output(tmp_path: Path) -> None:
    with pytest.raises(AgentError, match=r"\.urpx"):
        await make_agent(FakeBridge(), ScriptedLoop(), tmp_path).run(
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


def test_claude_cli_config_builds_cli_loop() -> None:
    from agents.llm import create_tool_loop
    from agents.llm.claude_cli_loop import ClaudeCLIToolLoop

    config = load_simple_config(
        CONFIGS / "agents" / "simple.yaml", llm=CONFIGS / "llm" / "claude_cli.yaml"
    )
    assert config.llm.api_key_env is None
    assert isinstance(create_tool_loop(config.llm, 5), ClaudeCLIToolLoop)
