"""Base class for all agents, using the Template Method pattern.

Subclasses implement ``execute`` and may override the ``validate``, ``after``
and ``on_error`` hooks. ``run`` defines the fixed pipeline and must not be
overridden.
"""

import asyncio
import logging
import time
from abc import ABC, abstractmethod
from typing import Generic, TypeVar, final

from agents.core.models import AppModel
from agents.core.settings import Settings

InT = TypeVar("InT", bound=AppModel)
OutT = TypeVar("OutT", bound=AppModel)


class AgentError(Exception):
    """Raised when an agent fails; wraps the original exception."""

    def __init__(self, agent: str, cause: BaseException) -> None:
        super().__init__(f"agent '{agent}' failed: {cause}")
        self.agent = agent
        self.cause = cause


class BaseAgent(ABC, Generic[InT, OutT]):
    """Async agent with a fixed validate -> execute -> after pipeline."""

    name: str = "base"

    def __init__(self, settings: Settings | None = None) -> None:
        self.settings = settings or Settings()
        self.log = logging.getLogger(f"agents.{self.name}")

    @property
    def timeout_s(self) -> float:
        """Upper bound for one execute call; override for long-running agents."""
        return self.settings.agent_timeout_s

    @final
    async def run(self, data: InT) -> OutT:
        start = time.perf_counter()
        self.log.info("start", extra={"agent": self.name})
        try:
            await self.validate(data)
            result = await asyncio.wait_for(self.execute(data), timeout=self.timeout_s)
            await self.after(data, result)
        except Exception as exc:
            await self.on_error(data, exc)
            raise AgentError(self.name, exc) from exc
        finally:
            elapsed_ms = (time.perf_counter() - start) * 1000
            self.log.info("end", extra={"agent": self.name, "elapsed_ms": round(elapsed_ms, 1)})
        return result

    @abstractmethod
    async def execute(self, data: InT) -> OutT:
        """Do the agent's work. Required."""

    async def validate(self, data: InT) -> None:
        """Check preconditions beyond schema validation. Raise to abort."""

    async def after(self, data: InT, result: OutT) -> None:
        """Run side effects after a successful execute (persist, notify, ...)."""

    async def on_error(self, data: InT, exc: Exception) -> None:
        """React to a failure before it is re-raised as AgentError."""
        self.log.exception("failed", extra={"agent": self.name})
