"""Tool loop for any OpenAI-compatible chat completions API.

Covers OpenAI, Gemini's compatibility endpoint, vLLM, Ollama, LM Studio, OpenRouter, ...
"""

import json
from typing import Any

import httpx

from agents.llm.base import LLMError, ToolHandler, ToolLoop, ToolSpec
from agents.llm.config import LLMConfig

DEFAULT_BASE_URL = "https://api.openai.com/v1"


class OpenAIToolLoop(ToolLoop):
    def __init__(
        self, config: LLMConfig, max_turns: int, client: httpx.AsyncClient | None = None
    ) -> None:
        super().__init__(config, max_turns)
        self.url = (config.base_url or DEFAULT_BASE_URL).rstrip("/") + "/chat/completions"
        headers = {"Content-Type": "application/json"}
        key = config.api_key()
        if key:
            headers["Authorization"] = f"Bearer {key}"
        self.client = client or httpx.AsyncClient(timeout=config.request_timeout_s)
        self.headers = headers

    async def run(
        self, system: str, task: str, tools: list[ToolSpec], call_tool: ToolHandler
    ) -> str:
        tool_defs = [
            {
                "type": "function",
                "function": {
                    "name": t.name,
                    "description": t.description,
                    "parameters": t.input_schema,
                },
            }
            for t in tools
        ]
        messages: list[dict[str, Any]] = [
            {"role": "system", "content": system},
            {"role": "user", "content": task},
        ]

        for turn in range(self.max_turns):
            body = {
                "model": self.config.model,
                "messages": messages,
                "tools": tool_defs,
                **self.config.params,
            }
            try:
                r = await self.client.post(self.url, headers=self.headers, json=body)
            except httpx.HTTPError as exc:
                raise LLMError(f"{self.url}: {exc}") from exc
            if r.is_error:
                raise LLMError(f"{self.url} returned {r.status_code}: {r.text[:800]}")

            choice = r.json()["choices"][0]
            msg = choice["message"]
            if choice.get("finish_reason") == "length":
                raise LLMError("response hit the token limit before finishing a step")

            calls = msg.get("tool_calls") or []
            # Send reasoning back too, so servers that keep thinking across turns
            # (e.g. Qwen's preserve_thinking on vLLM) still see it.
            assistant: dict[str, Any] = {"role": "assistant", "content": msg.get("content")}
            if calls:
                assistant["tool_calls"] = calls
            for key in ("reasoning_content", "reasoning"):
                if msg.get(key):
                    assistant[key] = msg[key]
            messages.append(assistant)

            if not calls:
                if msg.get("refusal"):
                    raise LLMError(f"the model declined the task: {msg['refusal']}")
                return str(msg.get("content") or "").strip()

            for call in calls:
                fn = call["function"]
                try:
                    args = json.loads(fn["arguments"] or "{}")
                except json.JSONDecodeError as exc:
                    text = f"invalid JSON in tool arguments: {exc}; resend the call with valid JSON"
                else:
                    text, _ = await call_tool(fn["name"], args, turn)
                messages.append({"role": "tool", "tool_call_id": call["id"], "content": text})

        raise LLMError(f"stopped after {self.max_turns} turns without finishing")

    async def aclose(self) -> None:
        await self.client.aclose()
