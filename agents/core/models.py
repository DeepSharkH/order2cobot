"""Shared Pydantic base models for all agent inputs and outputs."""

from datetime import datetime, timezone
from uuid import UUID, uuid4

from pydantic import BaseModel, ConfigDict, Field


def utc_now() -> datetime:
    return datetime.now(timezone.utc)


class AppModel(BaseModel):
    """Root for all schemas: one place to set validation policy."""

    model_config = ConfigDict(
        extra="forbid",
        frozen=True,
        str_strip_whitespace=True,
        validate_assignment=True,
    )


class Entity(AppModel):
    """A model with identity and creation timestamp."""

    id: UUID = Field(default_factory=uuid4)
    created_at: datetime = Field(default_factory=utc_now)
