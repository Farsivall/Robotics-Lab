#!/usr/bin/env bash
# One-command pick:
#   home -> open gripper -> approach block -> descend -> grip -> lift ->
#   rotate base -> place -> open gripper -> home
#
#   ./pick.sh                 # default: pick at (0.18, 0), rotate left 90 deg
#   ./pick.sh 0.16 -0.06      # use another pick point
#   ./pick.sh 0.18 0 -90      # rotate right 90 deg
# Full arguments: ./pick.sh PX PY ROT GRASP_Z PLACE_Z GRIP_VAL SPEED
# Specify robot: ROBOT_IP=192.168.123.x ./pick.sh
set -o pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PY_DIR="$PKG_DIR/mycobot_robot_control"
if [ -z "${ROBOT_IP:-}" ]; then
  cands=()
  while read -r _ _ lip _; do
    [ -n "$lip" ] && ping -c1 -W1 "$lip" >/dev/null 2>&1 && cands+=("$lip")
  done < <(cat /var/lib/NetworkManager/dnsmasq-*.leases 2>/dev/null)
  while read -r lip _; do
    [ -n "$lip" ] && ping -c1 -W1 "$lip" >/dev/null 2>&1 && cands+=("$lip")
  done < <(ip neigh show 2>/dev/null | awk '/192\.168\.123\./ {print $1}')
  for tip in 192.168.123.50 10.10.10.235; do
    ping -c1 -W1 "$tip" >/dev/null 2>&1 && cands+=("$tip")
  done
  mapfile -t cands < <(printf '%s\n' "${cands[@]}" | sort -u | grep -v '^$')
  if [ ${#cands[@]} -eq 1 ]; then export ROBOT_IP="${cands[0]}"; echo "== robot: $ROBOT_IP =="; fi
  if [ ${#cands[@]} -gt 1 ]; then echo "Multiple robots found: ${cands[*]}. Please specify ROBOT_IP=<IP> ./pick.sh"; exit 1; fi
fi
source "$HOME/miniforge3/etc/profile.d/conda.sh"; conda activate roboenv2
source "$CONDA_PREFIX/setup.bash" 2>/dev/null; source "$HOME/mycobot_client/install/setup.bash"
export PYTHONPATH="$HOME/RoboEnv/simulation_and_control:${PYTHONPATH:-}"
export ROS_LOCALHOST_ONLY=0 ROS_DOMAIN_ID=10
NOISE='^\[|pybullet build|iteration|position target|orientation target|found solution|share directory|Joint info|Link info|null|argv|Warning|inertial|robot_base|g_base|joint._to|^ *[cIm] =|^ *[0-9e.+ -]+ *$|^universe|file path|joint6output|^ *m = '
python "$PY_DIR/pick_flow.py" "${1:-0.18}" "${2:-0}" "${3:-90}" "${@:4}" 2>&1 | grep --line-buffered -vE "$NOISE"
