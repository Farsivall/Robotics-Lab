#!/usr/bin/env bash
# Terminal 2 — wait for /block_position then move the arm
# Usage: ./pick_vision.sh
# Start ./vision.sh first in the other terminal.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PY_DIR="$PKG_DIR/mycobot_robot_control"

if [ -z "${ROBOT_IP:-}" ]; then
  cands=()
  while read -r lip _; do
    [ -n "$lip" ] && ping -c1 -W1 "$lip" >/dev/null 2>&1 && cands+=("$lip")
  done < <(ip neigh show 2>/dev/null | awk '/192\.168\.123\./ {print $1}')
  for tip in 192.168.123.50 10.10.10.235; do
    ping -c1 -W1 "$tip" >/dev/null 2>&1 && cands+=("$tip")
  done
  mapfile -t cands < <(printf '%s\n' "${cands[@]}" | sort -u | grep -v '^$')
  if [ ${#cands[@]} -eq 1 ]; then export ROBOT_IP="${cands[0]}"; echo "== robot: $ROBOT_IP =="; fi
  if [ ${#cands[@]} -gt 1 ]; then echo "Multiple robots: ${cands[*]}. Use ROBOT_IP=<IP> ./pick_vision.sh"; exit 1; fi
fi

source "$HOME/miniforge3/etc/profile.d/conda.sh"
conda activate roboenv2
source "$CONDA_PREFIX/setup.bash" 2>/dev/null || true
source "$HOME/mycobot_client/install/setup.bash"
export PYTHONPATH="$HOME/RoboEnv/simulation_and_control:${PYTHONPATH:-}"
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-10}"

echo "== pick_vision: waiting for /block_position | ROS_DOMAIN_ID=$ROS_DOMAIN_ID =="
# Tighter grip: lower number = more closed. Keep >= 25.
GRIP="${GRIP_VAL:-28}"
echo "== grip_val=$GRIP (override with GRIP_VAL=30 ./pick_vision.sh) =="
NOISE='^\[|pybullet build|iteration|position target|orientation target|found solution|share directory|Joint info|Link info|null|argv|Warning|inertial|robot_base|g_base|joint._to|^ *[cIm] =|^ *[0-9e.+ -]+ *$|^universe|file path|joint6output|^ *m = '
exec python "$PY_DIR/pick_flow.py" --vision --grip-val "$GRIP" 2>&1 | grep --line-buffered -vE "$NOISE"
