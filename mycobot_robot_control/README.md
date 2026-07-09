# myCobot 280 Student ROS Workspace

This folder is a ROS 2 workspace for teaching students how to control a real
myCobot 280 robot arm. It includes setup scripts, robot health checks, simple
student examples, and a step-by-step course that leads toward writing a
pick-and-place algorithm.

Start here:

- Full course: [src/mycobot_robot_control/docs/STUDENT_TUTORIAL.md](src/mycobot_robot_control/docs/STUDENT_TUTORIAL.md)
- First-time walkthrough: [src/mycobot_robot_control/docs/BEGINNER_TUTORIAL.md](src/mycobot_robot_control/docs/BEGINNER_TUTORIAL.md)
- Short student guide: [src/mycobot_robot_control/docs/BASIC.md](src/mycobot_robot_control/docs/BASIC.md)
- Student distribution guide: [src/mycobot_robot_control/docs/STUDENT_DISTRIBUTION.md](src/mycobot_robot_control/docs/STUDENT_DISTRIBUTION.md)
- Staff setup notes: [src/mycobot_robot_control/docs/SUMMER_SCHOOL_SETUP.md](src/mycobot_robot_control/docs/SUMMER_SCHOOL_SETUP.md)

## Workspace Layout

```text
mycobot_robot_control/
  README.md
  laptop_bootstrap.sh
  make_student_bundle.sh
  pick.sh
  test_robot.sh
  hybrid_pick_place.sh
  provision/
  reports/
  src/
    mycobot_robot_control/
      package.xml
      setup.py
      setup.cfg
      resource/
      mycobot_robot_control/
      examples/
      docs/
      scripts/
      provision/
```

Important folders:

- `src/mycobot_robot_control/` is the ROS 2 package.
- `src/mycobot_robot_control/mycobot_robot_control/` contains Python robot
  programs.
- `src/mycobot_robot_control/examples/` contains small student learning scripts.
- `src/mycobot_robot_control/docs/` contains tutorials.
- `src/mycobot_robot_control/scripts/` contains shell tools used by the top-level
  wrappers.
- `provision/` is for large local files such as `mycobot-ros2-1.1.0.tar`.
- `reports/` stores generated robot test logs.

## Install on a New Laptop

Target laptop:

- Ubuntu 20.04 or 22.04
- Wi-Fi internet
- sudo access
- Ethernet port or USB Ethernet adapter

Copy this workspace to the laptop as `~/mycobot_robot_control`, then run:

```bash
cd ~/mycobot_robot_control
bash laptop_bootstrap.sh
```

The bootstrap installs system tools, Miniforge, the RoboStack ROS 2 environment,
`RoboEnv`, `mycobot_client`, this ROS package, and the laptop Ethernet sharing
configuration.

It is safe to rerun if a network download fails.

## Make a Clean Student Bundle

On the test laptop, create a clean tarball before giving files to students:

```bash
cd ~/mycobot_robot_control
bash make_student_bundle.sh
```

The bundle excludes local `build/`, `install/`, `log/`, `reports/`, and Python
cache files. Students should build the workspace on their own laptops.

See
[src/mycobot_robot_control/docs/STUDENT_DISTRIBUTION.md](src/mycobot_robot_control/docs/STUDENT_DISTRIBUTION.md)
for the full distribution workflow.

## Build This Workspace Manually

If the laptop is already set up:

```bash
cd ~/mycobot_robot_control
source ~/mycobot_client/source_mycobot_env.sh
colcon build --symlink-install --packages-select mycobot_robot_control
source install/setup.bash
```

After building, installed Python tools can be run with:

```bash
ros2 run mycobot_robot_control go_home.py
ros2 run mycobot_robot_control rotate_arm.py 20 12
```

The top-level wrappers still work:

```bash
./test_robot.sh
./pick.sh
```

## Connect and Find the Robot

Connect one laptop directly to one robot by Ethernet, power on the robot, and
wait about 30 seconds.

Try the common robot IP:

```bash
ping -c 2 192.168.123.50
ssh er@192.168.123.50
```

Default login:

```text
username: er
password: Elephant
```

If that IP does not respond, let the test script search:

```bash
SKIP_PICK=1 ./test_robot.sh
```

Look for:

```text
== target robot: 192.168.123.xxx ==
```

Helpers can also run:

```bash
ip neigh show
cat /var/lib/NetworkManager/dnsmasq-*.leases
```

Look for an address in the `192.168.123.x` range.

## Health Check

Before moving the arm:

```bash
SKIP_PICK=1 ./test_robot.sh
```

The health check tests ping, SSH, serial joint reading, gripper travel, Docker,
the comms container, and ROS 2 topic data.

Only continue when the final summary says:

```text
all checks passed
```

## Run Pick-and-Place

Clear the table and place the block about 18 cm in front of the robot base.

```bash
./pick.sh
```

Common variants:

```bash
./pick.sh 0.16 -0.06
./pick.sh 0.18 0 -90
ROBOT_IP=192.168.123.117 ./pick.sh
```

Full arguments:

```bash
./pick.sh PICK_X PICK_Y ROTATE_DEG [GRASP_Z PLACE_Z GRIP_VAL SPEED]
```

Defaults:

```text
GRASP_Z=0.02
PLACE_Z=0.06
GRIP_VAL=35
SPEED=25
```

Do not set `GRIP_VAL` below `25`.

## Student Learning Path

Use the course in
[src/mycobot_robot_control/docs/STUDENT_TUTORIAL.md](src/mycobot_robot_control/docs/STUDENT_TUTORIAL.md).

The student examples are:

```bash
python src/mycobot_robot_control/examples/01_read_joint_angles.py
python src/mycobot_robot_control/examples/02_move_one_joint.py 1 10 12
python src/mycobot_robot_control/examples/03_check_pick_algorithm_ik.py
```

The course teaches:

- how to find and test the robot
- how ROS topics connect programs to the arm
- how to read joint angles
- how to send one small joint command
- how to check joint limits
- how inverse kinematics fits into robot algorithms
- how the pick-and-place state machine is built

## Safety Rules

- Use one laptop with one robot when teaching beginners.
- One student sends commands at a time.
- Keep hands, hair, sleeves, and loose objects away from the arm.
- Start from home before running IK experiments.
- Use slow speeds for student work.
- If the robot moves unexpectedly, switch off power at the base.

## Troubleshooting

- No ping reply: check Ethernet, robot power, and wait 30 seconds.
- Ping works but SSH fails: check user `er` and password `Elephant`.
- More than one robot is found: specify the IP with `./test_robot.sh <IP>` or
  `ROBOT_IP=<IP> ./pick.sh`.
- Gripper reading is stuck: reseat the gripper cable at the top of the arm.
- Fresh robot has no comms image: copy `mycobot-ros2-1.1.0.tar` into
  top-level `provision/`, then rerun `./test_robot.sh`.
