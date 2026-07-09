# Beginner Tutorial

This tutorial shows the first complete workflow:

1. Connect the robot.
2. Find the robot IP address.
3. Test SSH.
4. Run the robot health check.
5. Run the pick-and-place demo.

After this walkthrough, use [STUDENT_TUTORIAL.md](STUDENT_TUTORIAL.md) to learn
how to write ROS 2 programs and robot algorithms.

## 1. Connect the Robot

Use one laptop with one robot.

1. Connect the laptop to Wi-Fi if you still need internet.
2. Connect the robot to the laptop with an Ethernet cable.
3. Power on the robot.
4. Wait about 30 seconds.
5. Keep the table clear.

## 2. Open a Terminal

Go to the project folder:

```bash
cd ~/mycobot_robot_control
```

If this folder does not exist, the project has not been copied or installed on
this laptop yet.

## 3. Find the Robot IP Address

Try the default robot IP first:

```bash
ping -c 2 192.168.123.50
```

If the output includes replies, the robot IP is probably:

```text
192.168.123.50
```

If there are no replies, use the automatic search:

```bash
SKIP_PICK=1 ./test_robot.sh
```

If you see this output:

```text
From 192.168.123.222 icmp_seq=1 Destination Host Unreachable
```

that means the laptop Ethernet connection is probably configured, but the robot
did not answer at `192.168.123.50`. This is not a successful connection. It may
mean the robot is still booting, the cable is not connected, the robot has a
different IP, or the robot Ethernet setting is not ready.

Wait 30 seconds, check the cable and robot power, then run:

```bash
SKIP_PICK=1 ./test_robot.sh
```

Look for a line like this:

```text
== target robot: 192.168.123.117 ==
```

The number after `target robot:` is the robot IP.

## 4. Manual IP Search for Helpers

If automatic search does not find the robot, check the neighbor table:

```bash
ip neigh show
```

Look for an address that starts with:

```text
192.168.123.
```

Example:

```text
192.168.123.117 dev enx... lladdr ... REACHABLE
```

In that example, the robot IP is:

```text
192.168.123.117
```

If the neighbor table is empty, try to refresh it:

```bash
ping -c 1 192.168.123.50
ip neigh show
```

You can also check NetworkManager DHCP leases:

```bash
cat /var/lib/NetworkManager/dnsmasq-*.leases
```

The lease file may show an address in the `192.168.123.x` range.

## 5. Test SSH

Use the IP you found. For the default IP:

```bash
ssh er@192.168.123.50
```

Default login:

```text
username: er
password: Elephant
```

If your robot IP is different, replace the IP:

```bash
ssh er@192.168.123.117
```

When SSH works, leave the robot shell:

```bash
exit
```

## 6. Run the Health Check

If the robot uses the default IP or only one robot is connected:

```bash
SKIP_PICK=1 ./test_robot.sh
```

If you know the robot IP:

```bash
SKIP_PICK=1 ./test_robot.sh 192.168.123.117
```

The health check tests:

- network ping
- SSH login
- serial joint reading
- gripper travel
- Docker
- robot comms container
- ROS 2 topic connection

If the final summary says `all checks passed`, the robot is ready.

## 7. Place the Block

Put the block about 18 cm in front of the robot base.

Good beginner range:

```text
x = 0.12 to 0.20 meters
y = -0.12 to 0.06 meters
```

Keep hands away from the robot before starting motion.

## 8. Run Pick-and-Place

Run:

```bash
./pick.sh
```

If your robot has a custom IP:

```bash
ROBOT_IP=192.168.123.117 ./pick.sh
```

The robot will go home, open the gripper, pick the block, rotate, place the
block, open the gripper, and return home.

## 9. Try Small Changes

Pick from a different safe point:

```bash
./pick.sh 0.16 -0.06
```

Rotate the other direction:

```bash
./pick.sh 0.18 0 -90
```

Use small changes only. If the target is unsafe or unreachable, the program
should stop instead of moving.

## 10. Common Problems

No ping reply:

- Check the Ethernet cable.
- Check robot power.
- Wait 30 seconds after power-on.
- Try `SKIP_PICK=1 ./test_robot.sh`.

Ping works but SSH fails:

- Check the username is `er`.
- Check the password is `Elephant`.
- Ask a helper to confirm the robot image.

Robot IP is not `192.168.123.50`:

- This can happen.
- Use the IP printed by `test_robot.sh`.
- Use `ROBOT_IP=<IP> ./pick.sh` when running the demo.

More than one robot is found:

- Do not guess.
- Use one laptop per robot if possible.
- Ask a helper to choose the correct IP.

The gripper does not move correctly:

- Stop the activity.
- Ask a helper to check the gripper cable.
- Do not lower `GRIP_VAL` below `25`.

## 11. Stop Safely

If the robot moves unexpectedly, switch off power at the base.

After the demo, wait until the robot is fully stopped before touching the block
or the robot.
