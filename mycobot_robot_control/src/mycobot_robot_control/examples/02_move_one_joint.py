#!/usr/bin/env python3
"""Move one joint by a small amount.

Examples:
    python src/mycobot_robot_control/examples/02_move_one_joint.py
    python src/mycobot_robot_control/examples/02_move_one_joint.py 1 10 12
    python src/mycobot_robot_control/examples/02_move_one_joint.py 1 -10 12
"""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from mycobot_msgs_2.msg import MycobotAngles, MycobotSetAngles


LIMITS = np.array([165.0, 165.0, 165.0, 165.0, 165.0, 175.0])


class OneJointMove(Node):
    def __init__(self):
        super().__init__("student_move_one_joint")
        self.real_angles = None
        self.create_subscription(MycobotAngles, "/mycobot/angles_real", self.cb, 10)
        self.pub = self.create_publisher(MycobotSetAngles, "/mycobot/angles_goal", 5)

    def cb(self, msg):
        self.real_angles = np.array(
            [
                msg.joint_1,
                msg.joint_2,
                msg.joint_3,
                msg.joint_4,
                msg.joint_5,
                msg.joint_6,
            ],
            dtype=float,
        )


def wait_for_angles(node, timeout=6.0):
    start = time.time()
    while time.time() - start < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.real_angles is not None:
            return node.real_angles.copy()
    return None


def publish_goal(node, goal, speed):
    msg = MycobotSetAngles()
    msg.joint_1, msg.joint_2, msg.joint_3 = [float(v) for v in goal[:3]]
    msg.joint_4, msg.joint_5, msg.joint_6 = [float(v) for v in goal[3:]]
    msg.speed = int(speed)
    for _ in range(20):
        node.pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.15)


def main():
    joint = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    delta = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
    speed = int(sys.argv[3]) if len(sys.argv) > 3 else 12

    if joint < 1 or joint > 6:
        raise SystemExit("joint must be 1..6")
    if abs(delta) > 20:
        raise SystemExit("use a small delta, no more than 20 degrees")
    if speed > 25:
        raise SystemExit("use speed 25 or lower for student experiments")

    rclpy.init()
    node = OneJointMove()
    current = wait_for_angles(node)
    if current is None:
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit("no /mycobot/angles_real data received")

    goal = current.copy()
    goal[joint - 1] += delta
    if abs(goal[joint - 1]) > LIMITS[joint - 1]:
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit("refused: target joint would exceed the joint limit")

    print("current:", [round(a, 1) for a in current])
    print("goal:   ", [round(a, 1) for a in goal])
    publish_goal(node, goal, speed)
    print("sent small joint move")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
