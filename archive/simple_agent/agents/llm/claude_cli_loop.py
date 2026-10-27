"""Tool loop for Claude through the logged-in `claude` CLI (Claude Agent SDK).

Uses the CLI's own login (e.g. a claude.ai subscription) instead of an API key. The
bridge tools are served to the CLI from an in-process MCP server; the CLI's built-in
tools (Bash, Read, Edit, ...) are switched off, and no user or project settings load.
"""

import logging
import os
import shutil
from typing import Any

from claude_agent_sdk import (
    AssistantMessage,
    ClaudeAgentOptions,
    ClaudeSDKClient,
    ClaudeSDKError,
    ResultMessage,
    SdkMcpTool,
    SystemMessage,
    create_sdk_mcp_server,
)

from agents.llm.base import LLMError, ToolHandler, ToolLoop, ToolSpec
from agents.llm.config import LLMConfig

SERVER = "robot"
PREFIX = f"mcp__{SERVER}__"

log = logging.getLogger(__name__)


class ClaudeCLIToolLoop(ToolLoop):
    def __init__(self, config: LLMConfig, max_turns: int) -> None:
        super().__init__(config, max_turns)
        self.cli_path = shutil.which("claude")
        if not self.cli_path:
            raise LLMError("the `claude` CLI is not on PATH; install it and run `claude` to log in")

    async def run(
        self, system: str, task: str, tools: list[ToolSpec], call_tool: ToolHandler
    ) -> str:
        turn = 0

        def relay(spec: ToolSpec) -> SdkMcpTool[Any]:
            async def handler(args: dict[str, Any]) -> dict[str, Any]:
                text, failed = await call_tool(spec.name, args, turn)
                return {"content": [{"type": "text", "text": text}], "is_error": failed}

            return SdkMcpTool(spec.name, spec.description, spec.input_schema, handler)

        options = ClaudeAgentOptions(
            cli_path=self.cli_path,
            model=self.config.model,
            system_prompt=system,
            tools=[],  # no built-in tools
            mcp_servers={SERVER: create_sdk_mcp_server(SERVER, tools=[relay(t) for t in tools])},
            strict_mcp_config=True,
            allowed_tools=[PREFIX + t.name for t in tools],
            permission_mode="dontAsk",  # anything not allowed above is denied
            setting_sources=[],  # ignore ~/.claude and project settings, hooks, CLAUDE.md
            max_turns=self.max_turns,
            # An API key in the environment would take precedence over the CLI login.
            env={"ANTHROPIC_API_KEY": "", **_env(self.config)},
            **self.config.params,
        )

        result: ResultMessage | None = None
        try:
            async with ClaudeSDKClient(options) as client:
                await client.query(task)
                async for msg in client.receive_response():
                    if isinstance(msg, SystemMessage) and msg.subtype == "init":
                        log.info(
                            "claude cli: model=%s auth=%s",
                            msg.data.get("model"),
                            msg.data.get("apiKeySource"),
                        )
                    elif isinstance(msg, AssistantMessage):
                        turn += 1
                    elif isinstance(msg, ResultMessage):
                        result = msg
        except ClaudeSDKError as exc:
            raise LLMError(f"claude cli: {exc}") from exc

        if result is None:
            raise LLMError("the claude cli exited without a result")
        usage = result.usage or {}
        log.info(
            "claude cli: %s after %d turns, %.1f s, tokens in %d (+%d cached) out %d, "
            "cost $%.2f (API-equivalent; billed to the CLI login)",
            result.subtype,
            result.num_turns,
            result.duration_ms / 1000,
            usage.get("input_tokens", 0) + usage.get("cache_creation_input_tokens", 0),
            usage.get("cache_read_input_tokens", 0),
            usage.get("output_tokens", 0),
            result.total_cost_usd or 0.0,
        )
        if result.subtype == "error_max_turns":
            raise LLMError(f"stopped after {self.max_turns} turns without finishing")
        if result.is_error:
            raise LLMError(f"claude cli failed ({result.subtype}): {result.result}")
        return (result.result or "").strip()


def _env(config: LLMConfig) -> dict[str, str]:
    """Pass the request timeout on to the CLI's MCP tool calls."""
    timeout_ms = str(int(config.request_timeout_s * 1000))
    return {"MCP_TOOL_TIMEOUT": os.environ.get("MCP_TOOL_TIMEOUT", timeout_ms)}
