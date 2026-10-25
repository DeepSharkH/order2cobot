"""Build a locally planned program on the robot in one pass and validate it there.

Each top-level node goes in with one program_add_node call carrying its whole subtree, so
a program costs a handful of bridge round trips instead of one per edit.
"""

import json
import re
from typing import Any

from agents.core.models import AppModel
from agents.robot.bridge import Bridge, BridgeError

# "  ur-code (loopForever=false) [cf21b2ef-...]": indent, node type, guid
_TREE_LINE = re.compile(r"^( *)(\S+).*\[([0-9a-fA-F-]{36})\]\s*$")


class PushResult(AppModel):
    valid: bool
    host_validated: bool = False
    messages: list[str] = []


class ProgramShell(AppModel):
    app_id: str
    program_id: str
    sections: dict[str, str]  # section type -> guid
    section_children: dict[str, list[str]]  # section type -> guids of its direct children


async def create_program(bridge: Bridge, name: str) -> ProgramShell:
    """Create and load an empty program; return its fresh shell."""
    await bridge.call("program_create", name=name)
    apps = {p["name"]: str(p["id"]) for p in (await bridge.call("program_list"))["programs"]}
    if name not in apps:
        raise BridgeError(f"program_create: '{name}' is not in program_list afterwards")
    await bridge.call("program_load", appId=apps[name])
    return await read_shell(bridge, apps[name])


async def read_shell(bridge: Bridge, app_id: str) -> ProgramShell:
    data = await bridge.call("program_get", appId=app_id)
    sections: dict[str, str] = {}
    children: dict[str, list[str]] = {}
    current: str | None = None
    for line in data.get("tree", []):
        m = _TREE_LINE.match(line)
        if not m:
            continue
        depth = len(m.group(1)) // 2
        if depth == 1:
            current = m.group(2)
            sections[current] = m.group(3)
            children[current] = []
        elif depth == 2 and current:
            children[current].append(m.group(3))
    return ProgramShell(
        app_id=app_id,
        program_id=str(data.get("programId", app_id)),
        sections=sections,
        section_children=children,
    )


async def push_program(
    bridge: Bridge, baseline: ProgramShell, sections: dict[str, list[dict[str, Any]]]
) -> PushResult:
    """Replace whatever was added to the program since `baseline` (its fresh shell) with
    `sections`, then validate.

    Failed adds are collected as messages rather than raised, so they can go back to the model.
    """
    shell = await read_shell(bridge, baseline.app_id)
    ids = {"appId": shell.app_id, "programId": shell.program_id}
    for section, guids in shell.section_children.items():
        keep = set(baseline.section_children.get(section, []))
        for guid in guids:
            if guid not in keep:
                await bridge.call("program_delete_node", **ids, guid=guid)

    messages: list[str] = []
    for section, nodes in sections.items():
        parent = shell.sections.get(section)
        if parent is None:
            messages.append(f"{section}: this program has no such section")
            continue
        for i, node in enumerate(nodes):
            args = {**ids, "parentGuid": parent, "type": node["type"]}
            args |= {k: node[k] for k in ("parameters", "children") if k in node}
            text, failed = await bridge.call_raw("program_add_node", args)
            if failed:
                messages.append(f"{section}[{i}] {node['type']}: add failed: {_error(text)}")

    report = await bridge.call("program_validate", appId=shell.app_id)
    messages += [_message(m) for m in report.get("messages", [])]
    return PushResult(
        valid=bool(report.get("valid")) and not messages,
        host_validated=bool(report.get("hostValidated")),
        messages=messages,
    )


def _error(text: str) -> str:
    try:
        err = json.loads(text).get("error", {})
        return str(err.get("message") or err or text)[:800]
    except (json.JSONDecodeError, AttributeError):
        return text[:800]


def _message(m: Any) -> str:
    return m if isinstance(m, str) else json.dumps(m)
