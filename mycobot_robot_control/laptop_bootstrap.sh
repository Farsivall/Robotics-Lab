#!/usr/bin/env bash
# Bootstrap a new Ubuntu 20.04/22.04 laptop for myCobot 280 control.
#
# On a new laptop, first connect to Wi-Fi/internet, copy this folder to
# ~/mycobot_robot_control, then run:
#   bash ~/mycobot_robot_control/laptop_bootstrap.sh
#
# Steps: apt packages -> Miniforge -> RoboEnv(roboenv2, ros-humble) ->
#        build mycobot_client -> environment entry script -> path adaptation ->
#        build this ROS workspace package -> Ethernet DHCP+NAT.
# Safe to rerun. Full setup usually takes 20-40 minutes, mostly conda downloads.
set -o pipefail
step(){ echo; echo "========== $1 =========="; }
die(){ echo "!! failed: $1"; exit 1; }
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$WORKSPACE_DIR/src/mycobot_robot_control"
SCRIPT_DIR="$PACKAGE_DIR/scripts"
PY_DIR="$PACKAGE_DIR/mycobot_robot_control"

step "1/8 apt base tools"
sudo apt-get update -qq
sudo apt-get install -y -qq git sshpass curl openssh-client network-manager || die "apt"

step "2/8 Miniforge"
if [ ! -d "$HOME/miniforge3" ]; then
  curl -fL -o /tmp/miniforge.sh \
    "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-$(uname -m).sh" || die "download miniforge"
  bash /tmp/miniforge.sh -b -p "$HOME/miniforge3" || die "install miniforge"
  "$HOME/miniforge3/bin/conda" init bash
else echo "already exists, skipping"; fi
source "$HOME/miniforge3/etc/profile.d/conda.sh"

step "3/8 RoboEnv -> conda environment roboenv2 (slowest step, 10-30 minutes)"
[ -d "$HOME/RoboEnv" ] || git clone --recurse-submodules https://github.com/VModugno/RoboEnv "$HOME/RoboEnv" || die "clone RoboEnv"
( cd "$HOME/RoboEnv" && git submodule update --init --recursive ) || die "RoboEnv submodule"
if ! conda env list | grep -q "^roboenv2 "; then
  "$HOME/miniforge3/bin/mamba" env create -f "$HOME/RoboEnv/environment_ros2.yaml" || die "mamba env create"
else echo "roboenv2 already exists, skipping"; fi

step "4/8 build mycobot_client workspace"
[ -d "$HOME/mycobot_client" ] || git clone https://github.com/VModugno/mycobot_client "$HOME/mycobot_client" || die "clone mycobot_client"
if [ ! -f "$HOME/mycobot_client/install/setup.bash" ]; then
  conda activate roboenv2 || die "conda activate"
  source "$CONDA_PREFIX/setup.bash" 2>/dev/null
  ( cd "$HOME/mycobot_client" && colcon build --packages-select mycobot_msgs_2 mycobot_client_2 ) || die "colcon build"
else echo "already built, skipping"; fi

step "5/8 environment entry script"
cat > "$HOME/mycobot_client/source_mycobot_env.sh" <<EOF
source \$HOME/miniforge3/etc/profile.d/conda.sh
conda activate roboenv2
source "\$CONDA_PREFIX/setup.bash"
source \$HOME/mycobot_client/install/setup.bash
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
  source "$WORKSPACE_DIR/install/setup.bash"
fi
export PYTHONPATH=\$HOME/RoboEnv/simulation_and_control:\${PYTHONPATH:-}
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-10}
echo "[mycobot] roboenv2 active | ROS_DOMAIN_ID=\$ROS_DOMAIN_ID"
EOF
echo "wrote ~/mycobot_client/source_mycobot_env.sh"

step "6/8 toolbox path adaptation"
SSHP="$(command -v sshpass)"
sed -i "s|/home/lingfanb/miniforge3/bin/sshpass|$SSHP|g; \
        s|/home/lingfanb/Gitcode/mycobot_client|$HOME/mycobot_client|g; \
        s|/home/lingfanb/Gitcode/mycobot_robot_control|$WORKSPACE_DIR|g; \
        s|/home/lingfanb/miniforge3|$HOME/miniforge3|g; \
        s|/home/roboticsstudent/mycobot_client|$HOME/mycobot_client|g; \
        s|/home/roboticsstudent/miniforge3|$HOME/miniforge3|g; \
        s|/home/roboticsstudent/RoboEnv|$HOME/RoboEnv|g; \
        s|/home/roboticsstudent/mycobot_robot_control_student_setup_20260706/mycobot_robot_control|$WORKSPACE_DIR|g" \
  "$SCRIPT_DIR/pick.sh" "$PY_DIR/pick_flow.py" "$SCRIPT_DIR/hybrid_pick_place.sh" "$SCRIPT_DIR/test_robot.sh" || die "sed path adaptation"
[ -s "$WORKSPACE_DIR/provision/mycobot-ros2-1.1.0.tar" ] || echo "WARNING: provision/mycobot-ros2-1.1.0.tar is missing. Fresh robots without the comms image cannot be provisioned until this 2 GB tarball is copied into provision/."
echo "paths set to: $WORKSPACE_DIR / $HOME/mycobot_client / $SSHP"

step "7/8 build this ROS workspace package"
conda activate roboenv2 || die "conda activate"
source "$CONDA_PREFIX/setup.bash" 2>/dev/null
source "$HOME/mycobot_client/install/setup.bash"
( cd "$WORKSPACE_DIR" && colcon build --symlink-install --packages-select mycobot_robot_control ) || die "student workspace build"

step "8/8 robot Ethernet (DHCP + NAT)"
ETH=$(nmcli -t -f DEVICE,TYPE,STATE device status | awk -F: '$2=="ethernet"{print $1; exit}')
if [ -z "$ETH" ]; then
  echo "WARNING: no Ethernet interface found, skipping. If using a USB Ethernet adapter, plug it in and rerun this script:"
  echo "   sudo nmcli con add type ethernet ifname <ETH> con-name robot ipv4.method shared ipv4.addresses 192.168.123.222/24"
elif nmcli -t con show robot >/dev/null 2>&1; then
  sudo nmcli con mod robot ifname "$ETH" ipv4.method shared \
    ipv4.addresses "192.168.123.222/24,10.10.10.100/24"
  sudo nmcli con up robot || true
  echo "robot connection already exists; confirmed shared mode and compatibility address"
else
  sudo nmcli con add type ethernet ifname "$ETH" con-name robot \
    ipv4.method shared ipv4.addresses 192.168.123.222/24 || die "nmcli"
  sudo nmcli con mod robot +ipv4.addresses 10.10.10.100/24
  sudo nmcli con up robot || true
  echo "Ethernet $ETH configured (192.168.123.222 shared + 10.10.10.100)"
fi

echo
echo "================ setup complete ================"
echo "Validate: connect robot Ethernet, power on, wait 30 seconds, then run:"
echo "  cd $WORKSPACE_DIR && ./test_robot.sh"
echo "Daily one-command pick: $WORKSPACE_DIR/pick.sh"
echo "Student course:         $PACKAGE_DIR/docs/STUDENT_TUTORIAL.md"
