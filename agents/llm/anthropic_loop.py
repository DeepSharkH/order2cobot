"""Tool loop for Claude through the Anthropic SDK."""

from typing import Any

import anthropic

from agents.llm.base import LLMError, ToolHandler, ToolLoop, ToolSpec
from agents.llm.config import LLMConfig

DEFAULT_MAX_TOKENS = 16000


class AnthropicToolLoop(ToolLoop):
    def __init__(
        self, config: LLMConfig, max_turns: int, client: anthropic.AsyncAnthropic | None = None
    ) -> None:
        super().__init__(config, max_turns)
        self.client = client or anthropic.AsyncAnthropic(
            api_key=config.api_key(),
            base_url=config.base_url,
            timeout=config.request_timeout_s,
        )
        self.request: dict[str, Any] = {"max_tokens": DEFAULT_MAX_TOKENS, **config.params}

    async def run(
        self, system: str, task: str, tools: list[ToolSpec], call_tool: ToolHandler
    ) -> str:
        tool_defs: list[Any] = [t.model_dump() for t in tools]
        messages: list[Any] = [{"role": "user", "content": task}]

        for turn in range(self.max_turns):
            response = await self.client.beta.messages.create(
                model=self.config.model,
                system=system,
                tools=tool_defs,
                messages=messages,
                **self.request,
            )
            # Append the full content so thinking blocks are passed back unchanged.
            messages.append({"role": "assistant", "content": response.content})

            if response.stop_reason == "refusal":
                raise LLMError(f"the model declined the task: {response.stop_details}")
            if response.stop_reason == "max_tokens":
                raise LLMError("response hit max_tokens before finishing a step")

            tool_uses = [b for b in response.content if b.type == "tool_use"]
            if not tool_uses:
                return "\n".join(b.text for b in response.content if b.type == "text").strip()

            results = []
            for block in tool_uses:
                text, failed = await call_tool(block.name, dict(block.input), turn)
                results.append(
                    {
                        "type": "tool_result",
                        "tool_use_id": block.id,
                        "content": text,
                        "is_error": failed,
                    }
                )
            messages.append({"role": "user", "content": results})

        raise LLMError(f"stopped after {self.max_turns} turns without finishing")

    async def aclose(self) -> None:
        await self.client.close()
