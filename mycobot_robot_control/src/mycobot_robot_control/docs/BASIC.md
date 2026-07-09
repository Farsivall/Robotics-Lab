# Basic Student Guide

This guide is for the first robot activity. It keeps the steps short and uses
safe, slow movements.

For a more detailed first-time walkthrough, including how to find the robot IP
address, see [BEGINNER_TUTORIAL.md](BEGINNER_TUTORIAL.md).
For the full course on writing robot programs and algorithms, see
[STUDENT_TUTORIAL.md](STUDENT_TUTORIAL.md).

## Before You Start

Ask a teacher or helper to check:

- The robot is on a clear table.
- The robot is connected to the laptop with Ethernet.
- The robot has power.
- The small block is about 18 cm in front of the robot base.
- Nobody has hands or loose clothing near the robot arm.

## Start the Robot Check

Try this robot IP first:

```text
192.168.123.50
```

Many robots use this address. If it does not work, stop and ask a teacher or
helper to find the correct robot IP.

Open a terminal and first check that the laptop can see the robot:

```bash
ping -c 2 192.168.123.50
```

If the ping works, test SSH login:

```bash
ssh er@192.168.123.50
```

Use this password when asked:

```text
Elephant
```

Then leave SSH:

```bash
exit
```

If ping or SSH fails, stop and ask a teacher or helper.

Next, run the robot health check:

```bash
cd ~/mycobot_robot_control
SKIP_PICK=1 ./test_robot.sh
```

Wait for the test to finish. If it says `all checks passed`, the robot is ready.

If the test fails, stop and ask a teacher or helper.

## Run the Pick-and-Place Demo

Make sure the table is clear, then run:

```bash
./pick.sh
```

The robot will:

1. Go home.
2. Open the gripper.
3. Move to the block.
4. Pick it up.
5. Turn left.
6. Put it down.
7. Go home again.

Wait until the robot is fully stopped before touching anything.

## Try Small Changes

Move the block a little and try a new pick point:

```bash
./pick.sh 0.16 -0.06
```

Turn the other way:

```bash
./pick.sh 0.18 0 -90
```

Use small changes only. If the robot cannot reach the position, the program
will stop instead of moving.

## Learn the Coordinates

The robot uses meters:

- `x` means forward from the robot base.
- `y` means left from the robot base.
- `z` means up from the table.

Good beginner pick points:

```text
x = 0.12 to 0.20
y = -0.12 to 0.06
```

## Move One Step at a Time

Load the robot environment:

```bash
source ~/mycobot_client/source_mycobot_env.sh
```

Send the robot home:

```bash
ros2 run mycobot_robot_control go_home.py
```

Rotate the base slowly:

```bash
ros2 run mycobot_robot_control rotate_arm.py 20 12
ros2 run mycobot_robot_control rotate_arm.py -20 12
```

Move down or up carefully:

```bash
ros2 run mycobot_robot_control move_z.py 0.08 12
ros2 run mycobot_robot_control move_z.py 0.12 12
```

## Rules for Students

- One person gives commands at a time.
- Keep fingers away from the robot while it is powered.
- Keep speeds low.
- Do not pull the arm by hand.
- Do not change gripper values below `25`.
- If anything looks wrong, stop and ask for help.

## What to Explore Next

Try these challenges:

1. Print the current joint angles.
2. Move one joint by a small amount.
3. Pick from a new safe point.
4. Place the block on the other side.
5. Write down which coordinates worked best.
