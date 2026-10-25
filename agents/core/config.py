"""Load YAML config files into Pydantic models."""

from pathlib import Path
from typing import Any, TypeVar

import yaml
from pydantic import BaseModel, ValidationError

ModelT = TypeVar("ModelT", bound=BaseModel)


class ConfigError(ValueError):
    """A config file is missing, unreadable or invalid."""


def read_yaml(path: str | Path) -> dict[str, Any]:
    path = Path(path)
    try:
        data = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError) as exc:
        raise ConfigError(f"{path}: {exc}") from exc
    if not isinstance(data, dict):
        raise ConfigError(f"{path}: expected a mapping of settings")
    return data


def load_yaml(path: str | Path, model: type[ModelT]) -> ModelT:
    """Read `path` (YAML or JSON) and validate it as `model`."""
    try:
        return model.model_validate(read_yaml(path))
    except ValidationError as exc:
        raise ConfigError(f"{path}: {exc}") from exc
