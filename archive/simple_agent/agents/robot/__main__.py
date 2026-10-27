"""Connect every name in an LLM config's connection pool once, so each shows up on the
pendant (AI Agent Bridge -> Connections) to be given Adjust access.

    python -m agents.robot configs/llm/claude_cli.yaml
"""

import asyncio
import sys
from pathlib import Path

from agents.core.config import load_yaml
from agents.llm.config import LLMConfig
from agents.robot import Bridge, RobotConfig, acquire_client_name, pool_names

ROBOT_CONFIG = Path(__file__).resolve().parents[2] / "configs" / "robot.yaml"


async def register(llm_config: str) -> None:
    robot = load_yaml(ROBOT_CONFIG, RobotConfig)
    base = load_yaml(llm_config, LLMConfig).client_name
    for name in pool_names(base, robot.client_pool_size):
        # Lock the name so registering never revokes a running agent's session.
        async with acquire_client_name(name, 1, robot.lock_dir), Bridge(robot, name) as bridge:
            status = await bridge.call("bridge_my_status")
            print(f"{name}: {status.get('tier')}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("usage: python -m agents.robot configs/llm/<provider>.yaml")
    asyncio.run(register(sys.argv[1]))
