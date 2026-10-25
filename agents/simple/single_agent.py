"""Single agent: describe a robot task in words, an LLM builds it on the robot through the
AI Agent Bridge MCP server, and the finished program is saved as a .urpx file.

The bridge lives on the robot's private network, so this agent is the MCP client: it lists
the bridge's tools, offers the safe subset to the model, and relays every call the model
makes. The program is built and validated on the robot but never played.

Needs the 'adjust' access level for the configured client name on the pendant
(AI Agent Bridge -> Connections).
"""

import re
from collections.abc import Callable
from pathlib import Path
from typing import Any

from pydantic import Field

from agents.core import AppModel, BaseAgent, Settings
from agents.llm import LLMError, ToolLoop, create_tool_loop
from agents.robot import Bridge, BridgeError, RobotConfig, is_allowed, offered_tools
from agents.simple.config import SimpleAgentConfig

SYSTEM = """You build PolyScope X programs on a real Universal Robots arm through the AI Agent Bridge tools.

Build exactly one new program, named exactly as the user gives it, that does the requested task. You cannot run it; a person will review and run it later.

Working method:
- Start with bridge_orient and bridge_my_status, then announce your goal with bridge_report_activity so the operator sees it on the pendant. Read the program-authoring and moves-and-waypoints guides with mcp_get_guide.
- program_create, then program_load before adding nodes. Use program_describe_node_type before the first use of any node type, and build in batches with `children`.
- Base waypoints on robot_get_joint_positions / robot_get_pose and the kinematics tools. Every move needs waypoint.qNear. Keep speeds modest, and keep new points close to the current pose unless the task gives coordinates.
- Read the `validation` block after every edit and fix what it reports. Finish with a whole-program program_validate that returns hostValidated: true, and read the tree back with program_get.
- If something the task needs is impossible with these tools, or the gripper/I/O wiring is unknown, make a reasonable, clearly labelled choice (for example a ur-comment node) instead of stopping.
- End with bridge_report_activity phase "done", then a short summary: the program structure, the assumptions you made, and anything the operator must check before running it."""  # noqa: E501


class SimpleTask(AppModel):
    prompt: str = Field(min_length=1)
    output_file: Path  # its stem becomes the program name on the robot

    @property
    def program_name(self) -> str:
        return re.sub(r"\W", "_", self.output_file.stem)


class SimpleTaskResult(AppModel):
    program_name: str
    output_file: Path
    summary: str


class SimpleAgentError(RuntimeError):
    """The build could not start or did not produce the program."""


BridgeFactory = Callable[[RobotConfig, str], Bridge]


class SingleAgent(BaseAgent[SimpleTask, SimpleTaskResult]):
    name = "simple"

    def __init__(
        self,
        config: SimpleAgentConfig,
        settings: Settings | None = None,
        *,
        tool_loop: ToolLoop | None = None,
        bridge_factory: BridgeFactory = Bridge,
    ) -> None:
        super().__init__(settings)
        self.config = config
        self.tool_loop = tool_loop
        self.bridge_factory = bridge_factory

    @property
    def timeout_s(self) -> float:
        return self.config.timeout_s

    async def validate(self, data: SimpleTask) -> None:
        if data.output_file.suffix != ".urpx":
            raise SimpleAgentError(f"output file must end in .urpx: {data.output_file}")

    async def execute(self, data: SimpleTask) -> SimpleTaskResult:
        name = data.program_name
        loop = self.tool_loop or create_tool_loop(self.config.llm, self.config.max_turns)
        try:
            async with self.bridge_factory(
                self.config.robot, self.config.llm.client_name
            ) as bridge:
                await self._check_access(bridge, name)

                async def relay(tool: str, args: dict[str, Any], turn: int) -> tuple[str, bool]:
                    return await self._relay(bridge, tool, args, turn)

                tools = offered_tools(await bridge.list_tools())
                summary = await loop.run(SYSTEM, task_message(name, data.prompt), tools, relay)

                if name not in await bridge.program_names():
                    raise SimpleAgentError(
                        f"the model finished, but no program named '{name}' exists on the "
                        f"robot:\n{summary}"
                    )
                urpx = await bridge.download_urpx(name)
        except LLMError as exc:
            raise SimpleAgentError(str(exc)) from exc
        finally:
            if self.tool_loop is None:
                await loop.aclose()

        data.output_file.parent.mkdir(parents=True, exist_ok=True)
        data.output_file.write_bytes(urpx)
        return SimpleTaskResult(program_name=name, output_file=data.output_file, summary=summary)

    async def _check_access(self, bridge: Bridge, name: str) -> None:
        status = await bridge.call("bridge_my_status")
        if "write" not in status.get("allowedEffects", []):
            raise SimpleAgentError(
                f"connection '{status.get('client')}' has access level '{status.get('tier')}'; "
                "raise it to 'Adjust' on the pendant (AI Agent Bridge -> Connections)"
            )
        if name in await bridge.program_names():
            raise SimpleAgentError(
                f"a program named '{name}' already exists on the robot; choose another output name"
            )

    async def _relay(
        self, bridge: Bridge, tool: str, args: dict[str, Any], turn: int
    ) -> tuple[str, bool]:
        """Run one tool call for the model; failures become error results it can see."""
        self.log.info("[%02d] %s", turn + 1, tool)
        if not is_allowed(tool):
            return f"tool '{tool}' is not available to this agent", True
        try:
            text, failed = await bridge.call_raw(tool, args)
        except BridgeError as exc:
            text, failed = f"bridge call failed: {exc}", True
        return text or "(empty result)", failed


def task_message(name: str, prompt: str) -> str:
    return f"Program name: {name}\n\nTask:\n{prompt}"
