"""The program the model plans locally, before anything is written to the robot.

The model gets one local tool, program_submit, that takes the whole node tree per program
section in program_add_node's `children` shape. It is checked here against the robot's node
toolbox and kept; the agent pushes the last accepted draft once the model has finished.
"""

import json
from typing import Any

from agents.llm.base import ToolSpec

# Top-level sections of a PolyScope X program, as program_get lists them.
SECTIONS = (
    "ur-code",
    "ur-modules",
    "ur-functions",
    "ur-before-start",
    "ur-configuration",
    "ur-status",
)

SUBMIT_TOOL = "program_submit"

_NODE_SCHEMA: dict[str, Any] = {
    "type": "object",
    "properties": {
        "type": {"type": "string", "minLength": 1},
        "parameters": {"type": "object"},
        "children": {"type": "array", "items": {"$ref": "#/$defs/node"}},
    },
    "required": ["type"],
}

SUBMIT_SPEC = ToolSpec(
    name=SUBMIT_TOOL,
    description=(
        "Submit the WHOLE program, planned locally. Nothing is written to the robot now: the "
        "draft is checked against the node toolbox and kept, and after you finish it is built "
        "on the robot in one go and validated there. Each section maps to its list of "
        "top-level nodes, each node {type, parameters, children} exactly as program_add_node "
        "takes them (parameters per program_describe_node_type). `ur-code` is the main program "
        "body and is required. Call again to replace the draft; the last accepted one is used."
    ),
    input_schema={
        "type": "object",
        "properties": {
            "sections": {
                "type": "object",
                "properties": {
                    s: {"type": "array", "items": {"$ref": "#/$defs/node"}} for s in SECTIONS
                },
                "required": ["ur-code"],
                "additionalProperties": False,
            },
        },
        "required": ["sections"],
        "$defs": {"node": _NODE_SCHEMA},
    },
)

Sections = dict[str, list[dict[str, Any]]]


class ProgramDraft:
    """The latest accepted program_submit, checked against `node_types` (type -> has children)."""

    def __init__(self, node_types: dict[str, bool]) -> None:
        self.node_types = node_types
        self.sections: Sections | None = None

    def submit(self, args: dict[str, Any]) -> tuple[str, bool]:
        sections = args.get("sections")
        problems = self.check(sections)
        if problems:
            return "draft rejected, nothing kept:\n" + "\n".join(f"- {p}" for p in problems), True
        assert isinstance(sections, dict)
        self.sections = {s: nodes for s, nodes in sections.items() if nodes}
        count = sum(_count(n) for nodes in self.sections.values() for n in nodes)
        return json.dumps({
            "status": "accepted",
            "nodes": count,
            "sections": sorted(self.sections),
            "next": "finish with your summary; the robot validates the program after you stop",
        }), False  # fmt: skip

    def check(self, sections: Any) -> list[str]:
        if not isinstance(sections, dict):
            return ["`sections` must be an object of section -> node list"]
        problems = [
            f"unknown section '{s}'; use one of {', '.join(SECTIONS)}"
            for s in sections
            if s not in SECTIONS
        ]
        if not sections.get("ur-code"):
            problems.append("`ur-code` (the main program body) is empty")
        for section, nodes in sections.items():
            if not isinstance(nodes, list):
                problems.append(f"{section}: must be a list of nodes")
                continue
            for i, node in enumerate(nodes):
                problems += self._check_node(node, f"{section}[{i}]")
        return problems

    def _check_node(self, node: Any, path: str) -> list[str]:
        if not isinstance(node, dict) or not isinstance(node.get("type"), str):
            return [f"{path}: a node needs a string `type`"]
        kind = node["type"]
        path = f"{path} {kind}"
        if kind not in self.node_types:
            return [f"{path}: not a node type on this robot (see program_list_node_types)"]
        problems = []
        if not isinstance(node.get("parameters", {}), dict):
            problems.append(f"{path}: `parameters` must be an object")
        children = node.get("children", [])
        if not isinstance(children, list):
            return [*problems, f"{path}: `children` must be a list"]
        if children and not self.node_types[kind]:
            problems.append(f"{path}: this node type does not allow children")
        for i, child in enumerate(children):
            problems += self._check_node(child, f"{path} > [{i}]")
        return problems


def _count(node: dict[str, Any]) -> int:
    return 1 + sum(_count(c) for c in node.get("children", []))
