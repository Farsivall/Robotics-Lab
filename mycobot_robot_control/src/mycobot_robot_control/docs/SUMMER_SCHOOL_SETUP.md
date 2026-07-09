# myCobot 280 Summer School Setup and Teaching Guide

This guide has three parts:

- Part A: laptop setup for staff
- Part B: student-facing ROS 2 control notes
- Part C: on-site troubleshooting notes

## Architecture

```text
[Laptop: ROS 2 client + student scripts]  <Ethernet/DDS>  [Robot Pi: comms in Docker]  <Serial>  [Arm controller + servos]
```

- `comms` runs in a Docker container on the robot Pi and bridges serial control
  to ROS 2 topics.
- Students only need to use three ROS 2 topics from the laptop.
- The gripper is controlled separately over SSH with host-side `pymycobot` and
  partial close. This avoids gripper current spikes that can reboot the Pi.

## Part A: New Laptop Setup

Target: Ubuntu 20.04 or 22.04 with Wi-Fi internet, sudo access, and one Ethernet
port or USB Ethernet adapter for the robot.

### One-command Setup

Copy this toolbox to the laptop, then run:

```bash
cd ~/mycobot_robot_control
bash laptop_bootstrap.sh
```

The setup takes about 20 to 40 minutes, mostly due to conda downloads. It is
repeatable. If a network download fails, fix Wi-Fi/DNS and run it again.

The script performs:

- apt package install
- Miniforge install
- RoboStack ROS 2 conda environment (`roboenv2`)
- `RoboEnv` checkout with submodules, including `simulation_and_control`
- `mycobot_client` checkout and build
- `~/mycobot_client/source_mycobot_env.sh`
- toolbox path adaptation for the current user
- this ROS workspace package build
- Ethernet shared-mode connection for the robot

### Manual Reference

Install system tools:

```bash
sudo apt update
sudo apt install -y git sshpass openssh-client curl network-manager
```

Install Miniforge:

```bash
curl -L -O https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh
bash Miniforge3-Linux-x86_64.sh -b -p ~/miniforge3
~/miniforge3/bin/conda init bash
```

Install RoboEnv:

```bash
git clone --recurse-submodules https://github.com/VModugno/RoboEnv ~/RoboEnv
cd ~/RoboEnv
git submodule update --init --recursive
~/miniforge3/bin/mamba env create -f environment_ros2.yaml
```

Build `mycobot_client`:

```bash
git clone https://github.com/VModugno/mycobot_client ~/mycobot_client
cd ~/mycobot_client
source ~/miniforge3/etc/profile.d/conda.sh
conda activate roboenv2
source "$CONDA_PREFIX/setup.bash"
colcon build --packages-select mycobot_msgs_2 mycobot_client_2
```

Create `~/mycobot_client/source_mycobot_env.sh`:

```bash
source $HOME/miniforge3/etc/profile.d/conda.sh
conda activate roboenv2
source "$CONDA_PREFIX/setup.bash"
source $HOME/mycobot_client/install/setup.bash
if [ -f $HOME/mycobot_robot_control/install/setup.bash ]; then
  source $HOME/mycobot_robot_control/install/setup.bash
fi
export PYTHONPATH=$HOME/RoboEnv/simulation_and_control:${PYTHONPATH:-}
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-10}
```

Build this teaching workspace:

```bash
cd ~/mycobot_robot_control
source ~/mycobot_client/source_mycobot_env.sh
colcon build --symlink-install --packages-select mycobot_robot_control
source install/setup.bash
```

Configure the Ethernet port in NetworkManager shared mode:

```bash
sudo nmcli con add type ethernet ifname <ETH> con-name robot \
  ipv4.method shared ipv4.addresses 192.168.123.222/24
sudo nmcli con mod robot +ipv4.addresses 10.10.10.100/24
sudo nmcli con up robot
```

Replace `<ETH>` with the wired interface name from `ip link`, such as `enp3s0`
or `eth0`. Shared mode provides DHCP and NAT so the robot can receive an IP and
use the laptop as an internet gateway.

### Validation

Connect the robot by Ethernet, power it on, wait about 30 seconds, then run:

```bash
cd ~/mycobot_robot_control
SKIP_PICK=1 ./test_robot.sh
```

If the health check passes and the workspace is clear, run:

```bash
./pick.sh
```

## Part B: Student Guide

For the complete student course, use [STUDENT_TUTORIAL.md](STUDENT_TUTORIAL.md).

### ROS 2 Topics

| Topic | Type | Direction | Meaning |
|---|---|---|---|
| `/mycobot/angles_real` | `mycobot_msgs_2/msg/MycobotAngles` | Read | Actual six joint angles in degrees, published at tens of Hz. |
| `/mycobot/angles_goal` | `mycobot_msgs_2/msg/MycobotSetAngles` | Write | Target joint angles in degrees plus `speed` from 1 to 100. Use 30 or below. |
| `/mycobot/gripper_status` | `mycobot_msgs_2/msg/MycobotGripperStatus` | Write | `state` (`True` close, `False` open) plus `speed`. Avoid full-speed full-close. |

Coordinate frame: base origin, `x` forward, `y` left, `z` up, with `z=0` near
the tabletop.

Joint limits: J1 to J5 are approximately `-165..165` degrees, J6 is
approximately `-175..175` degrees. If any joint is outside limits, comms may
silently reject the whole command.

### Minimal Script

```python
import rclpy
from rclpy.node import Node
from mycobot_msgs_2.msg import MycobotAngles, MycobotSetAngles

class Hello(Node):
    def __init__(self):
        super().__init__('hello_cobot')
        self.create_subscription(MycobotAngles, '/mycobot/angles_real', self.cb, 10)
        self.pub = self.create_publisher(MycobotSetAngles, '/mycobot/angles_goal', 5)

    def cb(self, m):
        print([round(x, 1) for x in
               (m.joint_1, m.joint_2, m.joint_3, m.joint_4, m.joint_5, m.joint_6)])

rclpy.init()
node = Hello()
goal = MycobotSetAngles()
goal.joint_1 = 20.0
goal.speed = 20
node.pub.publish(goal)
rclpy.spin(node)
```

Before running:

```bash
source ~/mycobot_client/source_mycobot_env.sh
```

### FK and IK

`mycobot_client_2.ik.CobotIK` is a ROS node with a Pinocchio model:

```python
from mycobot_client_2.ik import CobotIK
import numpy as np

node = CobotIK(visualize=False)

# Forward kinematics: joint angles in radians -> end-effector pose
pos, euler = node.get_pose(q_radians, 'gripper')

# Inverse kinematics: damped least-squares numerical IK
sol = node.calculate_ik(
    np.array([0.18, 0.0, 0.06]),
    np.array([180.0, 0.0, 0.0]),
    'gripper', 1e-5, 0.3, 0.02,
    False,
    4000, False)
```

Required validation before sending a solution:

1. Convert radians to degrees and wrap to `-180..180`.
2. Check every joint against the joint limits.
3. Run FK on the solution and require position error below 2 cm.
4. Publish the goal and monitor `/mycobot/angles_real` until the robot arrives.

### Suggested Exercises

1. Read `/mycobot/angles_real` and print current joint angles.
2. Send one small single-joint command and observe the real robot.
3. Use IK to move to a given `(x, y, z)` point.
4. Descend vertically in 1 cm steps using current-pose IK as the seed.
5. Implement home, approach, descend, grip, lift, rotate, place, open, home.
6. Replace `calculate_ik` with your own damped least-squares Jacobian IK.

### Safety Rules

- Send gentle commands. One goal at a time and about 1 Hz resend while waiting
  is enough. High-frequency publishing can overload the serial bridge.
- Keep target `z >= 0.01`. Descend in small steps and confirm arrival at each step.
- Do not use full-speed full-close gripper commands through ROS. Use the
  partial-close gripper path in `pick_flow.py`.
- Start from home before running IK. IK from an unusual pose can find a bad branch.
- If the arm behaves unexpectedly, switch off power at the base.

## Part C: On-Site Notes

### Robot Bring-Up

Connect Ethernet, power on, wait about 30 seconds, then run:

```bash
./test_robot.sh
```

The script discovers the IP, logs in over SSH, tests serial, tests gripper
travel, disables the Bluetooth serial bridge if present, checks Docker/comms,
checks ROS, and optionally runs the full pick flow.

### Common Failures

| Symptom | Cause | Fix |
|---|---|---|
| Gripper reading is stuck in a narrow range | Loose gripper cable | Reseat the gripper cable at the arm top. Normal travel is roughly `20..99`. |
| Some joints ignore multi-joint commands | Bluetooth serial bridge is using `/dev/ttyAMA0` | Run `test_robot.sh`; it disables the bridge. |
| No IPv4 found but link light is on | Robot has an old static IP | Use IPv6 link-local discovery with `ping6 -c2 -I <ETH> ff02::1`, then SSH and change `eth0` to DHCP. |
| SSH password fails | Password was changed | Use a monitor/keyboard and reset to `er` / `Elephant` if allowed. |
| Pi reboots during pick | Gripper current spike | Use partial close around `GRIP_VAL=35`; do not go below about `25`. |
| `failed to open device sdcard` | SD card is loose or bad | Reseat or replace the SD card. |
| Comms keeps crashing after bridge is disabled | Old Atom firmware | Reflash with myStudio, then retest. |

### Login Details

Default robot login is `er` / `Elephant`. SSH accepts password auth. The comms
container is named `mycobot_comms` and uses `ROS_DOMAIN_ID=10`.

### Multiple Robots

The simplest setup is one laptop, one robot, one direct Ethernet cable.

If multiple robots share a network, give each laptop/robot pair a different
`ROS_DOMAIN_ID`. Set the robot container `-e ROS_DOMAIN_ID=<n>` and export the
same value on the laptop.
