"""A pool of bridge connection names, each held by at most one process at a time.

The bridge keeps one live credential per connection name and host: a second session under
the same name revokes the first. So concurrent runs each take a free name from the pool,
locked with flock, which the OS releases even if the process dies.

Each name needs Adjust access once on the pendant (AI Agent Bridge -> Connections);
`python -m agents.robot <llm config>` connects every name once so they show up there.
"""

import asyncio
import fcntl
import logging
import os
from collections.abc import AsyncGenerator
from contextlib import asynccontextmanager
from pathlib import Path

log = logging.getLogger(__name__)

POLL_S = 2.0


def pool_names(base: str, size: int) -> list[str]:
    """`base` itself, then `base-2` ... `base-<size>`, so an already approved name stays in use."""
    return [base] + [f"{base}-{i}" for i in range(2, size + 1)]


def _try_lock(path: Path) -> int | None:
    fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o644)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        os.close(fd)
        return None
    return fd


@asynccontextmanager
async def acquire_client_name(base: str, size: int, lock_dir: Path) -> AsyncGenerator[str, None]:
    """Hold the first free name in the pool for the duration of the block; wait if all are taken."""
    lock_dir.mkdir(parents=True, exist_ok=True)
    names = pool_names(base, size)
    waiting = False
    while True:
        for name in names:
            fd = _try_lock(lock_dir / f"{name}.lock")
            if fd is None:
                continue
            log.info("bridge connection: %s", name)
            try:
                yield name
            finally:
                fcntl.flock(fd, fcntl.LOCK_UN)
                os.close(fd)
            return
        if not waiting:
            log.info("all %d bridge connections for '%s' are in use; waiting", size, base)
            waiting = True
        await asyncio.sleep(POLL_S)
