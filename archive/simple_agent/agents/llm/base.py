"""Provider-neutral interface for a model that works through client-side tools."""

from abc import ABC, abstractmethod
from collections.abc import Awaitable, Callable
from typing import Any

from agents.core.models import AppModel
from agents.llm.config import LLMConfig

# (tool name, arguments, turn index) -> (result text, is_error)
ToolHandler = Callable[[str, dict[str, Any], int], Awaitable[tuple[str, bool]]]


class LLMError(RuntimeError):
    """The model declined, ran out of room, or never finished."""


class ToolSpec(AppModel):
    name: str
    description: str
    input_schema: dict[str, Any]


class ToolLoop(ABC):
    """Runs one task to completion: model turn -> tool calls -> results -> ..."""

    def __init__(self, config: LLMConfig, max_turns: int) -> None:
        self.config = config
        self.max_turns = max_turns

    @abstractmethod
    async def run(
        self, system: str, task: str, tools: list[ToolSpec], call_tool: ToolHandler
    ) -> str:
        """Return the model's final text once it stops calling tools."""

    async def aclose(self) -> None:  # noqa: B027 - optional hook
        """Release network resources held by the loop."""
