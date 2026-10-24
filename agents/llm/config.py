"""Which model to use and how to reach it."""

import os
from typing import Any, Literal

from pydantic import Field

from agents.core.models import AppModel


class LLMConfig(AppModel):
    provider: Literal["anthropic", "openai"]  # "openai" = any OpenAI-compatible API
    model: str
    base_url: str | None = None
    api_key_env: str | None = None  # name of the env var holding the key; None for local servers
    client_name: str = "order2cobot-agent"  # connection name on the pendant (needs Adjust access)
    request_timeout_s: float = Field(default=600.0, gt=0)
    params: dict[str, Any] = Field(default_factory=dict)  # extra request fields, passed as-is

    def api_key(self) -> str | None:
        if not self.api_key_env:
            return None
        key = os.environ.get(self.api_key_env)
        if not key:
            raise LookupError(f"environment variable {self.api_key_env} is not set")
        return key
