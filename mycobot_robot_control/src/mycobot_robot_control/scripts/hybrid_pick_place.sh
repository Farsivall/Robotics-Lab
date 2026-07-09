#!/usr/bin/env bash
# Reliable pick + base-rotate + place on the real myCobot 280.
# This is the older step-by-step debug flow. For normal use, prefer
# pick.sh -> pick_flow.py, which is faster because it uses one ROS node.
# Arm motion via ROS; gripper PARTIAL-close via pymycobot (avoids the gripper-current
# brownout that reboots the Pi). Requires comms container running on the robot.
#
# Usage: ./hybrid_pick_place.sh PICK_X PICK_Y ROTATE_DEG [GRASP_Z PLACE_Z GRIP_VAL SPEED]
#   PICK_X PICK_Y : cube location, base frame meters (x fwd, y left)
#   ROTATE_DEG    : base J1 rotation for the place side (+90 left, -90 right)
#   GRASP_Z       : grasp height (default 0.02)
#   PLACE_Z       : release height over box (default 0.06)
#   GRIP_VAL      : partial-close target 0..100, lower=tighter (default 35; too low -> brownout)
#   SPEED         : arm joint speed (default 12)
set -uo pipefail
PX=${1:?pick_x}; PY=${2:?pick_y}; ROT=${3:-90}
GZ=${4:-0.02}; PZ=${5:-0.06}; GV=${6:-35}; SP=${7:-12}
# APPR=0.04 so default approach z = 0.06: the zero-seed IK is verified accurate
# there; at z=0.08 it converges to a bad solution (38mm err) and refuses.
LIFT=0.10; APPR=0.04; RIP=${ROBOT_IP:-192.168.123.50}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PY_DIR="$PKG_DIR/mycobot_robot_control"
SSHP="sshpass -p Elephant"; export PATH="$HOME/miniforge3/bin:$PATH"

set +u  # conda/ros activation scripts reference unset vars
source "$HOME/miniforge3/etc/profile.d/conda.sh"; conda activate roboenv2
source "$CONDA_PREFIX/setup.bash" 2>/dev/null; source "$HOME/mycobot_client/install/setup.bash"
set -u
export PYTHONPATH="$HOME/RoboEnv/simulation_and_control:${PYTHONPATH:-}"
export ROS_LOCALHOST_ONLY=0 ROS_DOMAIN_ID=10
NOISE='^\[|pybullet build|iteration|position target|orientation target|found solution|share directory|Joint info|Link info|null|argv|Warning|inertial|robot_base|g_base|joint._to|^ *[cIm] =|^ *[0-9e.+-]+ *$|^universe|file path'
grip(){ # value speed
  $SSHP ssh -o StrictHostKeyChecking=no -o PreferredAuthentications=password -o PubkeyAuthentication=no er@$RIP \
  "echo 'grip->'$1' boot='\$(uptime -s); docker stop -t 2 mycobot_comms>/dev/null 2>&1; \
   python3 /home/er/grip_set.py $1 $2; \
   echo 'boot_after='\$(uptime -s); docker start mycobot_comms>/dev/null 2>&1" 2>&1 | grep -vE "Permission denied|Warning:"
  sleep 5; }
arm(){
  local script="$1"
  shift
  python "$PY_DIR/$script" "$@" 2>&1 | grep -vE "$NOISE"
  local st=${PIPESTATUS[0]}
  if [ "$st" -ne 0 ]; then
    echo "!! STEP FAILED: $script (exit $st) -- ABORTING. Arm holds position; gripper state unchanged."
    exit 1
  fi; }

echo "== home =="; arm go_home.py 20
echo "== open =="; grip 100 40
echo "== approach =="; arm move_to_pose.py "$PX" "$PY" "$(python -c "print($GZ+$APPR)")" 180 0 0 "$SP"
echo "== descend =="; arm move_z.py "$GZ" 10
echo "== grip (partial $GV) =="; grip $GV 15
echo "== lift =="; arm move_z.py "$LIFT" "$SP"
echo "== rotate $ROT =="; arm rotate_arm.py "$ROT" "$SP"
echo "== place descend =="; arm move_z.py "$PZ" "$SP"
echo "== release =="; grip 100 40
echo "== retreat =="; arm move_z.py 0.12 "$SP"
echo "== home =="; arm go_home.py 20
echo "== DONE =="
