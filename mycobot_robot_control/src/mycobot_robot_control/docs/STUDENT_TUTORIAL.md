# Student Tutorial: Learn to Control the myCobot 280

This course teaches students how to use the robot step by step. The goal is not
only to run the demo. The goal is to understand the robot, write small ROS 2
programs, and build a simple pick-and-place algorithm.

## Course Map

1. Safety and workspace rules.
2. Find the robot and test the connection.
3. Learn the ROS 2 workspace layout.
4. Read robot joint angles.
5. Move one joint safely.
6. Understand coordinates and joint limits.
7. Check inverse kinematics targets.
8. Design a pick-and-place algorithm.
9. Turn the algorithm into code.
10. Improve and test the algorithm.

## Lesson 0: Safety

Rules:

- One student sends commands at a time.
- Keep hands away while the robot has power.
- Use small movements first.
- Use low speed, usually `12` to `25`.
- Do not pull the robot arm by hand.
- Do not set `GRIP_VAL` below `25`.
- If the robot moves unexpectedly, switch off power at the base.

## Lesson 1: Find the Robot

Open a terminal:

```bash
cd ~/mycobot_robot_control
```

Try the common robot IP:

```bash
ping -c 2 192.168.123.50
```

If the ping works, test SSH:

```bash
ssh er@192.168.123.50
```

Default login:

```text
username: er
password: Elephant
```

Exit SSH:

```bash
exit
```

If the ping does not work, ask the test script to search:

```bash
SKIP_PICK=1 ./test_robot.sh
```

If ping prints:

```text
From 192.168.123.222 icmp_seq=1 Destination Host Unreachable
```

the laptop has the expected shared Ethernet IP, but it cannot reach the robot at
`192.168.123.50`. Check robot power, Ethernet, and boot time, then let
`test_robot.sh` search for the correct robot IP.

Look for:

```text
== target robot: 192.168.123.xxx ==
```

Helpers can also check:

```bash
ip neigh show
cat /var/lib/NetworkManager/dnsmasq-*.leases
```

Look for an address in the `192.168.123.x` range.

## Lesson 2: Understand the Workspace

This folder is a ROS 2 workspace:

```text
mycobot_robot_control/
  README.md
  laptop_bootstrap.sh
  pick.sh
  test_robot.sh
  src/
    mycobot_robot_control/
      package.xml
      setup.py
      mycobot_robot_control/
      examples/
      docs/
      scripts/
      provision/
```

The important idea:

- `src/mycobot_robot_control/` is the ROS package.
- `mycobot_robot_control/` inside the package contains Python robot programs.
- `examples/` contains small student programs.
- `docs/` contains tutorials.
- Top-level `pick.sh` and `test_robot.sh` are simple wrappers for students.

Build the workspace:

```bash
source ~/mycobot_client/source_mycobot_env.sh
colcon build --symlink-install --packages-select mycobot_robot_control
source install/setup.bash
```

## Lesson 3: Run the Health Check

Before moving the arm:

```bash
SKIP_PICK=1 ./test_robot.sh
```

The check tests:

- network connection
- SSH login
- serial joint reading
- gripper travel
- Docker and the comms container
- ROS 2 topic data

Only continue if the final summary says:

```text
all checks passed
```

## Lesson 4: Read Joint Angles

The robot publishes real joint angles on:

```text
/mycobot/angles_real
```

Command-line test:

```bash
source ~/mycobot_client/source_mycobot_env.sh
ros2 topic echo /mycobot/angles_real mycobot_msgs_2/msg/MycobotAngles --once
```

Run the student example:

```bash
python src/mycobot_robot_control/examples/01_read_joint_angles.py
```

What the program does:

1. Starts a ROS 2 node.
2. Subscribes to `/mycobot/angles_real`.
3. Prints ten joint-angle samples.

Programming idea:

```python
self.create_subscription(MycobotAngles, "/mycobot/angles_real", self.cb, 10)
```

A subscription lets your program listen to robot data.

## Lesson 5: Move One Joint

The robot accepts joint goals on:

```text
/mycobot/angles_goal
```

Run a small move:

```bash
python src/mycobot_robot_control/examples/02_move_one_joint.py 1 10 12
```

Move back:

```bash
python src/mycobot_robot_control/examples/02_move_one_joint.py 1 -10 12
```

Arguments:

```text
joint_number delta_degrees speed
```

Example:

```text
1 10 12
```

means:

- move joint 1
- add 10 degrees
- use speed 12

Programming idea:

```python
self.pub = self.create_publisher(MycobotSetAngles, "/mycobot/angles_goal", 5)
self.pub.publish(goal_msg)
```

A publisher lets your program send commands to the robot.

## Lesson 6: Coordinates and Limits

The robot uses this simple coordinate frame:

- `x` is forward from the base.
- `y` is left from the base.
- `z` is up from the table.

Good beginner pick range:

```text
x = 0.12 to 0.20 meters
y = -0.12 to 0.06 meters
z = 0.02 to 0.12 meters
```

Joint limits are approximately:

```text
J1 to J5: -165 to 165 degrees
J6:       -175 to 175 degrees
```

Every student algorithm must check limits before sending motion.

## Lesson 7: Check IK Before Moving

Inverse kinematics means:

```text
wanted gripper position -> needed joint angles
```

Forward kinematics means:

```text
joint angles -> actual gripper position
```

A safe algorithm should:

1. Compute IK.
2. Convert joint angles to degrees.
3. Check joint limits.
4. Run FK on the result.
5. Refuse motion if the FK error is too large.

Run the IK checking example:

```bash
python src/mycobot_robot_control/examples/03_check_pick_algorithm_ik.py
```

Try another pick point:

```bash
python src/mycobot_robot_control/examples/03_check_pick_algorithm_ik.py 0.16 -0.06
```

This example plans three positions:

```text
approach -> descend -> lift
```

It does not move the robot. It only checks if the targets look reachable.

## Lesson 8: Design the Algorithm

A robot algorithm is a list of states with safety checks between them.

Pick-and-place algorithm:

```text
home
open gripper
move above block
move down
close gripper partly
move up
rotate base
move down to place
open gripper
move up
home
```

In code, write it as data first:

```python
steps = [
    ("home", home),
    ("open", open_gripper),
    ("approach", approach_block),
    ("descend", descend_to_block),
    ("grip", close_gripper),
    ("lift", lift_block),
    ("rotate", rotate_base),
    ("place", descend_to_place),
    ("release", open_gripper),
    ("retreat", lift_away),
    ("home_end", home),
]
```

Then execute it:

```python
for name, step in steps:
    print("running", name)
    ok = step()
    if not ok:
        print("failed at", name)
        break
```

This pattern makes the algorithm easier to debug.

## Lesson 9: Read the Real Pick Algorithm

The full implementation is:

```text
src/mycobot_robot_control/mycobot_robot_control/pick_flow.py
```

Start by reading these functions:

- `fresh()` receives recent joint angles.
- `goto()` sends a joint goal and waits for arrival.
- `home()` returns the robot to a known pose.
- `approach()` computes IK and validates FK error.
- `move_z()` moves vertically in small steps.
- `rotate()` rotates the base joint.
- `grip()` controls the gripper through the robot Pi.

The key lesson is that every movement is checked. The program does not blindly
send a target and hope it works.

## Lesson 10: Run the Demo

Place the block about 18 cm in front of the base.

Run:

```bash
./pick.sh
```

Try a different pick point:

```bash
./pick.sh 0.16 -0.06
```

Rotate the other direction:

```bash
./pick.sh 0.18 0 -90
```

If your robot has a custom IP:

```bash
ROBOT_IP=192.168.123.117 ./pick.sh
```

## Lesson 11: Student Challenges

Challenge 1:

Read joint angles and print only joint 1.

Challenge 2:

Move joint 1 left 10 degrees and back right 10 degrees.

Challenge 3:

Write a function called `safe_joint_goal(goal)` that returns `False` if any
joint is outside its limit.

Challenge 4:

Change the IK checking example so it tests five pick points.

Challenge 5:

Write your own step list for a different task, such as:

```text
home -> rotate left -> rotate right -> home
```

Challenge 6:

Read `pick_flow.py` and draw the state machine on paper before editing code.

## Lesson 12: Debugging Checklist

Connection:

```bash
ping -c 2 192.168.123.50
ssh er@192.168.123.50
```

ROS data:

```bash
ros2 topic echo /mycobot/angles_real mycobot_msgs_2/msg/MycobotAngles --once
```

Health check:

```bash
SKIP_PICK=1 ./test_robot.sh
```

Workspace rebuild:

```bash
colcon build --symlink-install --packages-select mycobot_robot_control
source install/setup.bash
```

If the robot behaves unexpectedly, stop the robot before changing code.
