"""Build a robot program from a task description and save it as .urpx.

python -m agents.simple out/pick_place.urpx "Build a basic pick-and-place program ..."
python -m agents.simple --llm configs/llm/qwen.yaml out/pick_place.urpx "..."
"""

import argparse
import asyncio
import logging
import sys
from pathlib import Path

from dotenv import load_dotenv

from agents.core import AgentError, ConfigError, Settings
from agents.simple.config import load_simple_config
from agents.simple.single_agent import SimpleTask, SingleAgent

DEFAULT_CONFIG = Path("configs/agents/simple.yaml")


def main() -> None:
    ap = argparse.ArgumentParser(
        prog="python -m agents.simple",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument(
        "output_file", type=Path, help="where to write the .urpx; its name becomes the program name"
    )
    ap.add_argument("prompt", help="what the program should do, in plain language")
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="simple agent config")
    ap.add_argument("--llm", type=Path, help="LLM config; overrides the one in --config")
    args = ap.parse_args()

    load_dotenv()  # API keys such as ANTHROPIC_API_KEY or MORPHEUS_API_KEY
    settings = Settings()
    logging.basicConfig(level=settings.log_level, format="%(asctime)s %(name)s %(message)s")

    try:
        config = load_simple_config(args.config, llm=args.llm)
        agent = SingleAgent(config, settings)
        result = asyncio.run(
            agent.run(SimpleTask(prompt=args.prompt, output_file=args.output_file))
        )
    except (ConfigError, AgentError) as exc:
        sys.exit(f"error: {exc}")
    print(result.summary)
    print(f"\nwrote {result.output_file}")


if __name__ == "__main__":
    main()
