#!/usr/bin/env python3
"""Read real joint angles from the robot.

Run after sourcing the robot environment:
    python src/mycobot_robot_control/examples/01_read_joint_angles.py
"""
import rclpy
from rclpy.node import Node
from mycobot_msgs_2.msg import MycobotAngles


class AnglePrinter(Node):
    def __init__(self):
        super().__init__("student_read_joint_angles")
        self.count = 0
        self.done = False
        self.create_subscription(MycobotAngles, "/mycobot/angles_real", self.cb, 10)

    def cb(self, msg):
        angles = [
            msg.joint_1,
            msg.joint_2,
            msg.joint_3,
            msg.joint_4,
            msg.joint_5,
            msg.joint_6,
        ]
        print("joints:", [round(a, 1) for a in angles])
        self.count += 1
        self.done = self.count >= 10


def main():
    rclpy.init()
    node = AnglePrinter()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
