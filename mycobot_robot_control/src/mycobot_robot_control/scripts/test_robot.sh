#!/usr/bin/env bash
# End-to-end test for one robot:
#   discover IP -> SSH -> serial joint read -> gripper travel -> Docker/image ->
#   start comms -> ROS link -> optional full pick flow
#
# Usage: ./test_robot.sh [IP]        # auto-discover 192.168.123.x if IP is omitted
#        SKIP_PICK=1 ./test_robot.sh # health check only, no pick motion
set -o pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WS_DIR="$(cd "$PKG_DIR/../.." && pwd)"
PROV_SRC="$PKG_DIR/provision"
REPORT_DIR="$WS_DIR/reports"; mkdir -p "$REPORT_DIR"
SSHPASS=/usr/bin/sshpass
SSHO=(-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=5 \
      -o PreferredAuthentications=password -o PubkeyAuthentication=no)
IMG=mzandtheraspberrypi/mycobot-ros2:1.1.0
TARBALL="$WS_DIR/provision/mycobot-ros2-1.1.0.tar"
PASS=(); FAIL=()
ok(){ echo "  PASS $1"; PASS+=("$1"); }
bad(){ echo "  FAIL $1"; FAIL+=("$1"); }
rssh(){ $SSHPASS -p Elephant ssh "${SSHO[@]}" er@"$IP" "$@" 2>/dev/null; }

# ---------- 1. discover ----------
# Discovery uses DHCP leases, ARP/neigh cache, 192.168.123.50, and
# 10.10.10.235. Some NetworkManager versions do not write shared DHCP leases
# where this script can read them, so neigh lookup is required.
IP=${1:-}
if [ -z "$IP" ]; then
  echo "== discovering robot =="
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
  if [ ${#cands[@]} -eq 0 ]; then echo "FAIL no robot found. Check Ethernet/power, wait 30 seconds, or run ./test_robot.sh <IP>."; exit 1; fi
  if [ ${#cands[@]} -gt 1 ]; then echo "WARNING multiple robots found: ${cands[*]}. Please specify: ./test_robot.sh <IP>"; exit 1; fi
  IP=${cands[0]}
fi
echo "== target robot: $IP =="
LOG="$REPORT_DIR/robot-${IP##*.}-$(date +%m%d-%H%M).log"
exec > >(tee "$LOG") 2>&1

# ---------- 2. ping + ssh ----------
ping -c2 -W2 "$IP" >/dev/null && ok "ping" || { bad "ping"; exit 1; }
HN=$(rssh "hostname; uptime -s" | tr '\n' ' ')
[ -n "$HN" ] && ok "ssh login (er/Elephant) host=$HN" || { bad "ssh login failed (is the password Elephant?)"; exit 1; }

# ---------- 3. disable Bluetooth serial bridge ----------
if rssh "pgrep -f uart_peripheral_serial >/dev/null" ; then
  rssh "echo Elephant | sudo -S bash -c 'sed -i \"s|^cd /home/er/mycobot_pi_bluetooth|#&|; s|^\./bt_auto_start.sh|#&|\" /etc/rc.local; pkill -f uart_peripheral_serial; pkill -f bt_auto_start'" >/dev/null 2>&1
  ok "Bluetooth serial bridge disabled in rc.local"
else
  ok "no Bluetooth serial bridge"
fi

# ---------- 3b. stop comms if running, then test serial ----------
rssh "docker stop -t 2 mycobot_comms" >/dev/null 2>&1
rssh "python3 -c 'import pymycobot'" && ok "host pymycobot" || bad "host missing pymycobot"
ANG=$(rssh "python3 -c \"
from pymycobot.mycobot import MyCobot; import time
mc=MyCobot('/dev/ttyAMA0',1000000); time.sleep(0.3)
print(mc.get_angles())\"")
if echo "$ANG" | grep -qE '^\[.*[0-9].*\]'; then ok "serial joint read: $ANG"; else bad "cannot read joint angles over serial: $ANG"; fi

# ---------- 4. gripper travel ----------
$SSHPASS -p Elephant scp "${SSHO[@]}" "$PROV_SRC/grip_set.py" er@"$IP":/home/er/grip_set.py >/dev/null 2>&1
gval(){ # target speed -> final reading; get_gripper_value can return None, so retry up to 3 times
  local v _t
  for _t in 1 2 3; do
    v=$(rssh "python3 /home/er/grip_set.py $1 $2" | tail -1 | grep -oE '[0-9]+$')
    [ -n "$v" ] && { echo "$v"; return; }
  done
  echo ""
}
G1=$(gval 100 40)
G2=$(gval 20 30)
rssh "python3 /home/er/grip_set.py 100 40" >/dev/null
if [ -n "$G1" ] && [ -n "$G2" ] && [ $((G1 - G2)) -gt 30 ]; then
  ok "gripper travel (open=$G1 closed=$G2)"
else
  bad "abnormal gripper travel (open=$G1 closed=$G2). Check that the gripper cable is fully seated."
fi

# ---------- 5. Docker + image + comms ----------
if ! rssh "command -v docker" >/dev/null; then
  echo "  no Docker found; installing through laptop NAT, about 3-5 minutes"
  rssh "echo Elephant | sudo -S bash -c 'apt-get update -qq >/dev/null 2>&1; DEBIAN_FRONTEND=noninteractive apt-get install -y -qq docker.io >/dev/null 2>&1; usermod -aG docker er; systemctl enable --now docker >/dev/null 2>&1'" >/dev/null 2>&1
fi
if ! rssh "command -v docker" >/dev/null; then
  bad "Docker install failed; check robot apt sources/network"; SKIP_PICK=1
else
  ok "docker"
  if ! rssh "docker image inspect $IMG" >/dev/null 2>&1; then
    if [ -s "$TARBALL" ]; then
      echo "  image missing; loading local tarball, about 2 GB"
      $SSHPASS -p Elephant scp "${SSHO[@]}" "$TARBALL" er@"$IP":/home/er/ >/dev/null &&
      rssh "docker load < /home/er/$(basename "$TARBALL") && rm /home/er/$(basename "$TARBALL")" &&
      ok "image loaded" || { bad "image load failed"; SKIP_PICK=1; }
    else
      bad "image missing and local tarball not found ($TARBALL)"; SKIP_PICK=1
    fi
  else ok "comms image exists"; fi
  if rssh "docker inspect mycobot_comms" >/dev/null 2>&1; then
    rssh "docker start mycobot_comms" >/dev/null && ok "comms container started"
  else
    rssh "docker run -d --name mycobot_comms --restart unless-stopped --network host \
      --device /dev/ttyAMA0 -v /dev:/dev --volume /home/er/:/mnt_folder \
      --device-cgroup-rule 'c 81:* rmw' --device-cgroup-rule 'c 189:* rmw' \
      -e ROS_DOMAIN_ID=10 $IMG bash -lc 'source install/setup.bash && export ROS_DOMAIN_ID=10 && \
      ros2 launch mycobot_interface_2 mycobot_comms_launch.py use_realsense:=False'" >/dev/null &&
    ok "comms container created and started" || { bad "comms container creation failed"; SKIP_PICK=1; }
  fi
fi

# ---------- 6. ROS link ----------
if ! rssh "docker ps --format '{{.Names}}'" | grep -q mycobot_comms; then
  SKIP_ROS=1
fi
if [ "${SKIP_ROS:-0}" != "1" ]; then
  source "$HOME/miniforge3/etc/profile.d/conda.sh"; conda activate roboenv2
  source "$CONDA_PREFIX/setup.bash" 2>/dev/null; source "$HOME/mycobot_client/install/setup.bash"
  export PYTHONPATH="$HOME/RoboEnv/simulation_and_control:${PYTHONPATH:-}"
  export ROS_LOCALHOST_ONLY=0 ROS_DOMAIN_ID=10
  echo "  waiting for /mycobot/angles_real; cold comms start can take 15-30 seconds, max 75 seconds"
  ROS_OK=0
  for _try in 1 2 3; do
    # Explicit type avoids early ros2 topic echo failure while daemon discovery is incomplete.
    if timeout 25 ros2 topic echo /mycobot/angles_real mycobot_msgs_2/msg/MycobotAngles --once >/dev/null 2>&1; then ROS_OK=1; break; fi
    sleep 5
  done
  if [ "$ROS_OK" = "1" ]; then
    ok "ROS link up (/mycobot/angles_real is streaming)"
  else
    bad "cannot receive /mycobot/angles_real"; SKIP_PICK=1
  fi
fi

# ---------- 7. full pick ----------
if [ "${SKIP_PICK:-0}" != "1" ]; then
  if [ -t 0 ]; then read -rp ">> Place the block about 18 cm in front of the robot, then press Enter to start pick test "; fi
  PICKLOG=$(mktemp)
  if ROBOT_IP=$IP "$SCRIPT_DIR/pick.sh" 2>&1 | tee "$PICKLOG"; then
    ok "full pick flow"
  elif grep -qE "FAILED at (home|open|approach|descend)" "$PICKLOG"; then
    # Pre-grasp failures can be retried safely; do not auto-retry after gripping.
    echo "  failure happened before grasp; retrying once"
    ROBOT_IP=$IP "$SCRIPT_DIR/pick.sh" && ok "full pick flow after retry" || bad "pick flow failed after retry; check log"
  else
    bad "pick flow failed; check log above"
  fi
  rm -f "$PICKLOG"
fi

# ---------- summary ----------
echo; echo "======== $IP test summary ========"
for p in "${PASS[@]}"; do echo "  PASS $p"; done
for f in "${FAIL[@]}"; do echo "  FAIL $f"; done
echo "report: $LOG"
[ ${#FAIL[@]} -eq 0 ] && echo "all checks passed" || echo "${#FAIL[@]} checks failed"
