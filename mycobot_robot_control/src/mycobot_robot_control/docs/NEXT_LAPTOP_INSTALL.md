# Next laptop install

Target: Ubuntu 20.04/22.04 laptop with Wi-Fi internet, sudo access, and one Ethernet port/adapter for the robot.

## 1. Copy and unpack

```bash
cd ~
tar xzf mycobot_robot_control_student.tar.gz
cd mycobot_robot_control
```

## 2. Bootstrap the laptop

```bash
bash laptop_bootstrap.sh
```

Enter the laptop user's sudo password when prompted. The script is repeatable; if a network download fails, fix Wi-Fi/DNS and run it again.

The bootstrap now handles:

- Miniforge + `roboenv2`
- `RoboEnv` submodules, including `simulation_and_control`
- `mycobot_client` build
- this ROS workspace package build
- `source_mycobot_env.sh` with the correct `PYTHONPATH`
- toolbox path adaptation for the current user
- Ethernet `robot` connection at `192.168.123.222/24`

## 3. Test a robot

Connect robot Ethernet, power on the robot, wait about 30 seconds:

```bash
SKIP_PICK=1 ./test_robot.sh
```

`test_robot.sh` now discovers robots from DHCP lease files, ARP/neighbor cache, and default addresses. If discovery still fails, specify the IP manually:

```bash
SKIP_PICK=1 ./test_robot.sh 192.168.123.117
```

## 4. Run pick

Only after the health check passes and the workspace is clear:

```bash
./pick.sh
```

If more than one robot is visible, specify one:

```bash
ROBOT_IP=192.168.123.117 ./pick.sh
```

Note: this package does not include `provision/mycobot-ros2-1.1.0.tar`. If a robot does not already have the Docker comms image, copy that tarball into `provision/` before running `test_robot.sh`.
