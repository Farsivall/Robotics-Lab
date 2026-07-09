#!/usr/bin/env python3
"""Check IK targets for a simple pick algorithm without moving the robot.

This is a planning exercise. It computes joint goals and verifies FK error, but
it does not publish movement commands.
"""
import sys

import numpy as np
import rclpy
from mycobot_client_2.ik import CobotIK


DEG = np.pi / 180.0
LIMITS = np.array([165.0, 165.0, 165.0, 165.0, 165.0, 175.0])
DOWN = np.array([180.0, 0.0, 0.0])


def check_pose(node, name, xyz):
    ik = node.calculate_ik(xyz, DOWN, "gripper", 1e-5, 0.3, 0.02, False, 4000, False)
    if ik is None:
        print(f"{name}: IK failed")
        return False

    joints = np.array(node.adjust_angles(np.array(ik)), dtype=float)
    pos, eul = node.get_pose(joints * DEG, "gripper")
    err = float(np.linalg.norm(pos - xyz))
    within_limits = bool(np.all(np.abs(joints) <= LIMITS))
    ok = err < 0.02 and within_limits

    print(f"{name}: {'OK' if ok else 'REFUSED'}")
    print("  xyz target:", np.round(xyz, 3).tolist())
    print("  joints deg:", np.round(joints, 1).tolist())
    print(f"  FK error: {err * 1000:.1f} mm")
    print(f"  within limits: {within_limits}")
    return ok


def main():
    pick_x = float(sys.argv[1]) if len(sys.argv) > 1 else 0.18
    pick_y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    grasp_z = 0.02
    approach_z = 0.06
    lift_z = 0.10

    rclpy.init()
    node = CobotIK(visualize=False)

    steps = [
        ("approach", np.array([pick_x, pick_y, approach_z])),
        ("descend", np.array([pick_x, pick_y, grasp_z])),
        ("lift", np.array([pick_x, pick_y, lift_z])),
    ]

    all_ok = True
    for name, xyz in steps:
        all_ok = check_pose(node, name, xyz) and all_ok

    node.destroy_node()
    rclpy.shutdown()

    if not all_ok:
        raise SystemExit("one or more planned poses are unsafe or unreachable")
    print("algorithm path looks reachable")


if __name__ == "__main__":
    main()
