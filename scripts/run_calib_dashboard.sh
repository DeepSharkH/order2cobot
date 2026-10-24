#!/usr/bin/env bash
# Camera + UR3e calibration dashboard -> http://localhost:8090
set -e
cd "$(dirname "$0")/.."
if [ ! -x .venv/bin/python ]; then
  command -v uv >/dev/null || curl -LsSf https://astral.sh/uv/install.sh | sh
  ~/.local/bin/uv venv .venv
  ~/.local/bin/uv pip install -p .venv numpy scipy "opencv-contrib-python-headless<5" pyrealsense2 pyorbbecsdk2 pyyaml requests aiohttp pytest
fi
exec .venv/bin/python -m dashboard.server "$@"
