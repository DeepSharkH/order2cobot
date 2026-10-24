from agents.llm.base import LLMError, ToolHandler, ToolLoop, ToolSpec
from agents.llm.config import LLMConfig


def create_tool_loop(config: LLMConfig, max_turns: int) -> ToolLoop:
    """Build the tool loop for `config.provider`."""
    if config.provider == "anthropic":
        from agents.llm.anthropic_loop import AnthropicToolLoop

        return AnthropicToolLoop(config, max_turns)
    from agents.llm.openai_loop import OpenAIToolLoop

    return OpenAIToolLoop(config, max_turns)


__all__ = ["LLMConfig", "LLMError", "ToolHandler", "ToolLoop", "ToolSpec", "create_tool_loop"]
