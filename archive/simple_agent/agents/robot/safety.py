"""Which bridge tools a model may use.

Motion, live I/O, deletion, installs and connection management are never offered,
so a model can build and validate programs but never play them. A model that plans the
program locally gets only CATALOG_TOOLS: read-only lookups, no program on the robot.
"""

from typing import Any

from agents.llm.base import ToolSpec

BLOCKED_TOOLS = frozenset({
    "program_run", "logic_program_run", "logic_program_resume", "smartskill_run",
    "logic_program_set_autoplay", "signal_set", "signal_toggle", "robot_set_robot_state",
    "program_delete", "application_delete_variable", "application_delete_frame",
    "urcap_install", "urcap_remove", "program_import", "artifact_put", "bridge_set_mode",
    "bridge_set_allow_new", "bridge_delete_connection", "bridge_revoke_connection",
    "bridge_enable_connection", "bridge_pause", "bridge_resume", "change_revert",
})  # fmt: skip

# What a node or move can be and where the arm is now; nothing here touches a program.
CATALOG_TOOLS = frozenset({
    "program_list_node_types", "program_describe_node_type", "mcp_get_guide",
    "robot_get_pose", "robot_get_joint_positions", "robot_get_kinematic_info",
    "robot_forward_kinematics", "robot_inverse_kinematics",
})  # fmt: skip

MAX_DESCRIPTION_CHARS = 4000


def offered_tools(
    bridge_tools: list[dict[str, Any]], only: frozenset[str] | None = None
) -> list[ToolSpec]:
    """The bridge's tools minus BLOCKED_TOOLS (and outside `only`, if given), with schemas
    cleaned for model APIs."""
    return [
        ToolSpec(
            name=t["name"],
            description=t.get("description", "")[:MAX_DESCRIPTION_CHARS],
            input_schema={k: v for k, v in t["inputSchema"].items() if k != "$schema"},
        )
        for t in bridge_tools
        if t["name"] not in BLOCKED_TOOLS and (only is None or t["name"] in only)
    ]


def is_allowed(tool: str) -> bool:
    return tool not in BLOCKED_TOOLS
