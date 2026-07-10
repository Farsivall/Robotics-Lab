#!/usr/bin/env bash
# Terminal 1 — camera publishes /block_position
# Usage: ./vision.sh
#        ./vision.sh --camera 1
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PY_DIR="$PKG_DIR/mycobot_robot_control"

source "$HOME/miniforge3/etc/profile.d/conda.sh"
conda activate roboenv2
source "$CONDA_PREFIX/setup.bash" 2>/dev/null || true
source "$HOME/mycobot_client/install/setup.bash"
export PYTHONPATH="$HOME/RoboEnv/simulation_and_control:${PYTHONPATH:-}"
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-10}"

echo "== vision: publishing /block_position | ROS_DOMAIN_ID=$ROS_DOMAIN_ID =="
echo "== then in ANOTHER terminal run: ./pick_vision.sh =="
exec python "$PY_DIR/cam_to_coord.py" "$@"
