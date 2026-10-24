import asyncio

import pytest
from pydantic import ValidationError

from agents.core import AgentError, AppModel, BaseAgent, Settings


class Num(AppModel):
    value: int


class Doubler(BaseAgent[Num, Num]):
    name = "doubler"

    def __init__(self, settings: Settings | None = None) -> None:
        super().__init__(settings)
        self.calls: list[str] = []

    async def validate(self, data: Num) -> None:
        self.calls.append("validate")
        if data.value < 0:
            raise ValueError("negative")

    async def execute(self, data: Num) -> Num:
        self.calls.append("execute")
        return Num(value=data.value * 2)

    async def after(self, data: Num, result: Num) -> None:
        self.calls.append("after")


class Slow(BaseAgent[Num, Num]):
    async def execute(self, data: Num) -> Num:
        await asyncio.sleep(1)
        return data


async def test_run_calls_hooks_in_order() -> None:
    agent = Doubler()
    assert await agent.run(Num(value=3)) == Num(value=6)
    assert agent.calls == ["validate", "execute", "after"]


async def test_validate_failure_skips_execute() -> None:
    agent = Doubler()
    with pytest.raises(AgentError) as info:
        await agent.run(Num(value=-1))
    assert isinstance(info.value.cause, ValueError)
    assert agent.calls == ["validate"]


async def test_timeout_raises_agent_error() -> None:
    agent = Slow(Settings(agent_timeout_s=0.01))
    with pytest.raises(AgentError) as info:
        await agent.run(Num(value=1))
    assert isinstance(info.value.cause, asyncio.TimeoutError)


def test_cannot_instantiate_without_execute() -> None:
    with pytest.raises(TypeError):
        BaseAgent()  # type: ignore[abstract]


def test_models_forbid_extra_fields() -> None:
    with pytest.raises(ValidationError):
        Num(value=1, extra=2)  # type: ignore[call-arg]
