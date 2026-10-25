import asyncio
from pathlib import Path

import pytest

from agents.robot import acquire_client_name, pool_names


def test_pool_keeps_base_name_first() -> None:
    assert pool_names("bot", 3) == ["bot", "bot-2", "bot-3"]


async def test_concurrent_holders_get_distinct_names(tmp_path: Path) -> None:
    async with (
        acquire_client_name("bot", 3, tmp_path) as a,
        acquire_client_name("bot", 3, tmp_path) as b,
        acquire_client_name("bot", 3, tmp_path) as c,
    ):
        assert [a, b, c] == ["bot", "bot-2", "bot-3"]


async def test_name_is_reused_after_release(tmp_path: Path) -> None:
    async with acquire_client_name("bot", 2, tmp_path) as first:
        pass
    async with acquire_client_name("bot", 2, tmp_path) as again:
        assert again == first


async def test_waits_while_pool_is_full(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr("agents.robot.pool.POLL_S", 0.01)

    async def take() -> str:
        async with acquire_client_name("bot", 1, tmp_path) as name:
            return name

    async with acquire_client_name("bot", 1, tmp_path):
        waiter = asyncio.create_task(take())
        await asyncio.sleep(0.05)
        assert not waiter.done()
    assert await asyncio.wait_for(waiter, 1) == "bot"
