"""Single agent: describe a robot task in words, an LLM plans the program locally, and the
finished program is built on the robot through the AI Agent Bridge and saved as a .urpx file.

The bridge lives on the robot's private network, so this agent is the MCP client. The model
only reads from it: the node toolbox, node descriptions, guides, the arm's pose and the
kinematics (CATALOG_TOOLS). It writes the whole program tree with a local tool, so every
edit and retry stays on this machine and the draft is saved next to the .urpx. Once the model
stops, the agent creates the program on the robot in one pass and validates it; validation
problems go back to the model for a corrected draft (repair_rounds). The program is never
played.

Needs the 'adjust' access level on the pendant (AI Agent Bridge -> Connections) for every
connection name in the configured client name's pool; each run holds one free name.
"""

import json
import re
from collections.abc import Callable
from pathlib import Path
from typing import Any

from pydantic import Field

from agents.core import AppModel, BaseAgent, Settings
from agents.llm import LLMError, ToolHandler, ToolLoop, ToolSpec, create_tool_loop
from agents.robot import (
    CATALOG_TOOLS,
    Bridge,
    BridgeError,
    PushResult,
    RobotConfig,
    acquire_client_name,
    create_program,
    offered_tools,
    push_program,
)
from agents.simple.config import SimpleAgentConfig
from agents.simple.draft import SUBMIT_SPEC, SUBMIT_TOOL, ProgramDraft, Sections

SYSTEM = """You plan PolyScope X programs for a real Universal Robots arm. The AI Agent Bridge tools you have are read-only lookups; you write the program yourself, locally, with program_submit.

Plan exactly one program that does the requested task. Nothing you do here changes the robot: when you stop, your last accepted program_submit is built on the robot in one pass and validated there. You cannot run it; a person will review and run it later.

Working method:
- Read the program-authoring and moves-and-waypoints guides with mcp_get_guide, and program_list_node_types for the toolbox.
- Use program_describe_node_type before the first use of any node type, and follow its authoring hint and worked example for `parameters` and placement (e.g. modules and functions go in their own sections, not in ur-code).
- Base waypoints on robot_get_joint_positions / robot_get_pose and the kinematics tools. Every move needs waypoint.qNear. Keep speeds modest, and keep new points close to the current pose unless the task gives coordinates.
- Submit the whole tree with program_submit (sections -> nodes with type, parameters, children). Fix anything it rejects and submit again.
- If something the task needs is impossible with these nodes, or the gripper/I/O wiring is unknown, make a reasonable, clearly labelled choice (for example a ur-comment node) instead of stopping.
- After an accepted submit, end with a short summary: the program structure, the assumptions you made, and anything the operator must check before running it."""  # noqa: E501


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
        draft_file = data.output_file.with_suffix(".program.json")
        try:
            robot = self.config.robot
            async with (
                acquire_client_name(
                    self.config.llm.client_name, robot.client_pool_size, robot.lock_dir
                ) as client_name,
                self.bridge_factory(robot, client_name) as bridge,
            ):
                await self._check_access(bridge, name)
                draft = ProgramDraft(await self._node_types(bridge))

                async def relay(tool: str, args: dict[str, Any], turn: int) -> tuple[str, bool]:
                    return await self._relay(bridge, draft, tool, args, turn)

                tools = [*offered_tools(await bridge.list_tools(), CATALOG_TOOLS), SUBMIT_SPEC]
                summary = await loop.run(SYSTEM, task_message(name, data.prompt), tools, relay)
                summary, report = await self._build(
                    bridge, loop, tools, relay, draft, data, summary, draft_file
                )
                urpx = await bridge.download_urpx(name)
        except LLMError as exc:
            raise SimpleAgentError(str(exc)) from exc
        finally:
            if self.tool_loop is None:
                await loop.aclose()

        data.output_file.parent.mkdir(parents=True, exist_ok=True)
        data.output_file.write_bytes(urpx)
        if not report.host_validated:
            summary += (
                "\n\nNote: the robot validated this structurally only (hostValidated: false)."
            )
        return SimpleTaskResult(program_name=name, output_file=data.output_file, summary=summary)

    async def _build(
        self,
        bridge: Bridge,
        loop: ToolLoop,
        tools: list[ToolSpec],
        relay: ToolHandler,
        draft: ProgramDraft,
        data: SimpleTask,
        summary: str,
        draft_file: Path,
    ) -> tuple[str, PushResult]:
        """Push the model's draft to the robot; on validation problems, ask for a fixed draft."""
        name = data.program_name
        shell = None
        for attempt in range(self.config.repair_rounds + 1):
            sections = draft.sections
            if sections is None:
                raise SimpleAgentError(
                    f"the model finished without an accepted program_submit:\n{summary}"
                )
            draft_file.parent.mkdir(parents=True, exist_ok=True)
            draft_file.write_text(json.dumps(sections, indent=2) + "\n")
            self.log.info("pushing draft %d (%s) to the robot", attempt + 1, draft_file)
            shell = shell or await create_program(bridge, name)
            report = await push_program(bridge, shell, sections)
            if report.valid:
                return summary, report
            self.log.info("robot validation failed:\n%s", "\n".join(report.messages))
            if attempt == self.config.repair_rounds:
                break
            draft.sections = None
            summary = await loop.run(
                SYSTEM, repair_message(name, data.prompt, sections, report), tools, relay
            )
        raise SimpleAgentError(
            f"program '{name}' is on the robot but did not validate after "
            f"{self.config.repair_rounds} repair round(s); last draft in {draft_file}:\n"
            + "\n".join(f"- {m}" for m in report.messages)
        )

    async def _node_types(self, bridge: Bridge) -> dict[str, bool]:
        types = (await bridge.call("program_list_node_types"))["nodeTypes"]
        return {t["type"]: bool(t.get("allowsChildren")) for t in types}

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
        self, bridge: Bridge, draft: ProgramDraft, tool: str, args: dict[str, Any], turn: int
    ) -> tuple[str, bool]:
        """Run one tool call for the model; failures become error results it can see."""
        self.log.info("[%02d] %s", turn + 1, tool)
        if tool == SUBMIT_TOOL:
            text, failed = draft.submit(args)
            self.log.info("[%02d] %s: %s", turn + 1, tool, "rejected" if failed else "accepted")
            return text, failed
        if tool not in CATALOG_TOOLS:
            return f"tool '{tool}' is not available to this agent", True
        try:
            text, failed = await bridge.call_raw(tool, args)
        except BridgeError as exc:
            text, failed = f"bridge call failed: {exc}", True
        return text or "(empty result)", failed


def task_message(name: str, prompt: str) -> str:
    return f"Program name: {name}\n\nTask:\n{prompt}"


def repair_message(name: str, prompt: str, sections: Sections, report: PushResult) -> str:
    problems = "\n".join(f"- {m}" for m in report.messages)
    return (
        f"{task_message(name, prompt)}\n\n"
        "Your previous draft was built on the robot, and its validation reported:\n"
        f"{problems}\n\n"
        "Previous draft (the program_submit sections):\n"
        f"{json.dumps(sections, indent=1)}\n\n"
        "Fix these problems and program_submit the whole corrected program, then summarize."
    )
