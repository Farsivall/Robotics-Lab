#!/usr/bin/env bash
# Run tests with ROS2 paths stripped from PYTHONPATH so its pytest
# plugins don't crash before pytest.ini is read.
# Uses the project venv if present, falls back to system python3.
VENV_PYTHON="$(dirname "$0")/../.venv/bin/python"
PYTHON="${VENV_PYTHON}" ; [ -x "$PYTHON" ] || PYTHON="python3"
PYTHONPATH="" "$PYTHON" -m pytest "$@"
