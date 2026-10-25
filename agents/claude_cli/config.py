"""Config for the Claude CLI agent: how the `claude` CLI is started for one run."""

from pathlib import Path
from typing import Literal

from pydantic import Field

from agents.core.config import load_yaml
from agents.core.models import AppModel

PermissionMode = Literal["default", "acceptEdits", "bypassPermissions", "dontAsk", "plan"]


class ClaudeCLIAgentConfig(AppModel):
    model: str | None = None  # None = the CLI's default model
    effort: Literal["low", "medium", "high", "xhigh", "max"] | None = None
    max_turns: int | None = Field(default=None, gt=0)
    timeout_s: float = Field(default=3600.0, gt=0)  # whole run
    # Tools run without a prompt; in print mode anything else that needs permission is denied.
    allowed_tools: list[str] = Field(default_factory=list)
    disallowed_tools: list[str] = Field(default_factory=list)
    permission_mode: PermissionMode = "default"
    append_system_prompt: str | None = None
    cwd: Path | None = None  # working directory of the CLI; None = the current one
    # put in front of PATH for the CLI and its commands (e.g. a venv's bin, so `python` is it)
    path: list[Path] = Field(default_factory=list)
    stream_partial: bool = True  # stream text as it is generated (--include-partial-messages)
    # Keep the partial chunks in the recorded events too; the full messages are always kept.
    record_partial: bool = False
    # An API key in the environment would take precedence over the CLI login.
    use_api_key: bool = False
    extra_args: list[str] = Field(default_factory=list)  # passed to the CLI as-is


def load_claude_cli_config(path: str | Path) -> ClaudeCLIAgentConfig:
    """Load `path`; relative `cwd` and `path` entries are resolved against its folder."""
    path = Path(path)
    config = load_yaml(path, ClaudeCLIAgentConfig)

    def resolve(p: Path) -> Path:
        return p if p.is_absolute() else (path.parent / p).resolve()

    return config.model_copy(
        update={
            "cwd": resolve(config.cwd) if config.cwd else None,
            "path": [resolve(p) for p in config.path],
        }
    )
