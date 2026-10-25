"""Config for the simple agent: run limits plus references to the robot and LLM configs."""

from pathlib import Path

from pydantic import Field, ValidationError

from agents.core.config import ConfigError, load_yaml, read_yaml
from agents.core.models import AppModel
from agents.llm.config import LLMConfig
from agents.robot.config import RobotConfig


class SimpleAgentConfig(AppModel):
    max_turns: int = Field(default=80, gt=0)
    timeout_s: float = Field(default=3600.0, gt=0)  # whole build, all model and tool turns
    robot: RobotConfig
    llm: LLMConfig


def load_simple_config(path: str | Path, llm: str | Path | None = None) -> SimpleAgentConfig:
    """Load `path`, resolving its `robot` and `llm` file references relative to it.

    `llm` overrides the config's `llm` reference (a path relative to the cwd).
    """
    path = Path(path)
    raw = read_yaml(path)
    base = path.parent
    try:
        robot_path = base / raw.pop("robot")
        llm_ref = raw.pop("llm", None)
        llm_path = Path(llm) if llm else base / llm_ref
    except (KeyError, TypeError) as exc:
        raise ConfigError(f"{path}: needs 'robot' and 'llm' file references") from exc
    try:
        return SimpleAgentConfig(
            robot=load_yaml(robot_path, RobotConfig),
            llm=load_yaml(llm_path, LLMConfig),
            **raw,
        )
    except ValidationError as exc:
        raise ConfigError(f"{path}: {exc}") from exc
