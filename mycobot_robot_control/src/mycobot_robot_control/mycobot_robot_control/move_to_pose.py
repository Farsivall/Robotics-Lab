#!/usr/bin/env python3
"""Move end-effector to a Cartesian pose with orientation, via accurate 6D IK.
Usage: python move_to_pose.py X Y Z RX RY RZ [SPEED]   (meters, degrees)
Only commands the arm if IK converges, FK error < THRESH, and within joint limits.
"""
import sys, time
import numpy as np
import rclpy
from mycobot_client_2.ik import CobotIK
from mycobot_msgs_2.msg import MycobotSetAngles

DEG = np.pi / 180.0
LIMS = np.array([165, 165, 165, 165, 165, 179.0])
THRESH = 0.02  # 20 mm

x, y, z, rx, ry, rz = [float(a) for a in sys.argv[1:7]]
speed = int(sys.argv[7]) if len(sys.argv) > 7 else 30

rclpy.init()
node = CobotIK(visualize=False)
t = time.time()
while time.time() - t < 6:
    rclpy.spin_once(node, timeout_sec=0.1)
before = np.array(node.get_real_angles(), dtype=float)
print('before joints (deg):', np.round(before, 1))

tgt = np.array([x, y, z])
eul = np.array([rx, ry, rz])
ik = node.calculate_ik(tgt, eul, 'gripper', 1e-5, 0.3, 0.02, False, 4000, False)
if ik is None:
    print('IK=None (unreachable). NOT moving.'); node.destroy_node(); rclpy.shutdown(); raise SystemExit(1)
adj = np.array(node.adjust_angles(np.array(ik)), dtype=float)
pos, e = node.get_pose(adj * DEG, 'gripper')
err = float(np.linalg.norm(pos - tgt))
within = bool(np.all(np.abs(adj) <= LIMS))
print(f'IK joints (deg): {np.round(adj,1)}')
print(f'FK -> pos {np.round(pos,3)}  eul {np.round(e,0)}  |  target {tgt.tolist()} {eul.tolist()}')
print(f'pos err = {err*1000:.1f} mm | within limits = {within}')
if err > THRESH or not within:
    print(f'REFUSING (err>{THRESH*1000:.0f}mm or out of limits). NOT moving.')
    node.destroy_node(); rclpy.shutdown(); raise SystemExit(1)

m = MycobotSetAngles()
m.joint_1, m.joint_2, m.joint_3, m.joint_4, m.joint_5, m.joint_6 = [float(v) for v in adj]
m.speed = speed
for _ in range(6):
    node.cmd_angle_pub.publish(m); rclpy.spin_once(node, timeout_sec=0.05); time.sleep(0.1)
print(f'SENT, speed={speed}')
t = time.time()
while time.time() - t < 9:
    rclpy.spin_once(node, timeout_sec=0.1); time.sleep(0.05)
after = np.array(node.get_real_angles(), dtype=float)
pa, ea = node.get_pose(after * DEG, 'gripper')
print('after joints (deg):', np.round(after, 1))
print('after gripper pos/eul:', np.round(pa, 3), np.round(ea, 0))
node.destroy_node(); rclpy.shutdown(); print('DONE')
