# Student Distribution Guide

Use this when preparing files on a test laptop and giving them to students for
installation, robot testing, and exercises.

## Why Use a Clean Bundle

The test laptop may contain local generated files:

- `build/`
- `install/`
- `log/`
- `reports/`
- `__pycache__/`
- `*.pyc`

Do not give those files to students as the source of truth. They are specific to
the test laptop and can confuse new installs. Students should receive the source
workspace and build it on their own laptops.

## Make the Student Bundle

On the test laptop:

```bash
cd ~/mycobot_robot_control
bash make_student_bundle.sh
```

This creates:

```text
/tmp/mycobot_robot_control_student.tar.gz
```

To choose another output path:

```bash
bash make_student_bundle.sh ~/Desktop/mycobot_robot_control_student.tar.gz
```

The bundle includes the ROS workspace source, tutorials, examples, setup
scripts, and robot test scripts. It excludes local build products, logs, reports,
and Python cache files.

## Optional Docker Image Tarball

If a fresh robot does not already have the comms Docker image, place this file
before making the bundle:

```text
provision/mycobot-ros2-1.1.0.tar
```

The file is large. If it is not included, students can still install the laptop
workspace, but a fresh robot may need the tarball before `./test_robot.sh` can
provision the comms image.

## Student Install Steps

On each student laptop:

```bash
cd ~
tar xzf mycobot_robot_control_student.tar.gz
cd mycobot_robot_control
bash laptop_bootstrap.sh
```

After bootstrap completes:

```bash
source ~/mycobot_client/source_mycobot_env.sh
SKIP_PICK=1 ./test_robot.sh
```

If the health check passes and the table is clear:

```bash
./pick.sh
```

## Student Learning Order

Students should read and follow:

1. [BASIC.md](BASIC.md)
2. [BEGINNER_TUTORIAL.md](BEGINNER_TUTORIAL.md)
3. [STUDENT_TUTORIAL.md](STUDENT_TUTORIAL.md)

The full course teaches connection testing, ROS 2 topics, small joint commands,
IK checking, and how to write a pick-and-place algorithm.
