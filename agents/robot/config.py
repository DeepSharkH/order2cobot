"""Connection settings for the robot's AI Agent Bridge and Robot API."""

from pathlib import Path

from pydantic import Field, field_validator

from agents.core.models import AppModel


class RobotConfig(AppModel):
    bridge_url: str = "https://universal-robots/universal-robots/ai-agent-bridge/mcp/v1/"
    robot_api_url: str = "https://universal-robots/universal-robots/robot-api"
    cert_path: Path = Field(default=Path("~/.config/ur3/robot-cert.pem"), validate_default=True)
    request_timeout_s: float = Field(default=120.0, gt=0)

    @field_validator("cert_path")
    @classmethod
    def _expand_home(cls, value: Path) -> Path:
        return value.expanduser()

    @field_validator("bridge_url")
    @classmethod
    def _trailing_slash(cls, value: str) -> str:
        # The bridge redirects without it, which drops the POST body.
        return value if value.endswith("/") else value + "/"
