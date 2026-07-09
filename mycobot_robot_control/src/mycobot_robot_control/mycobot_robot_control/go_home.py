#!/usr/bin/env python3
"""Return the arm to the home pose (all joints 0 = pointing straight up).
Run BEFORE a pick sequence so every run starts from the same known pose
(the zero-seed approach IK was validated from here). Gripper untouched.
Usage: python go_home.py [SPEED]
"""
import sys, time, numpy as np, rclpy
from mycobot_client_2.ik import CobotIK
from mycobot_msgs_2.msg import MycobotAngles, MycobotSetAngles
SPEED = int(sys.argv[1]) if len(sys.argv) > 1 else 20
TOL = 4.0        # deg, per-joint arrival tolerance
TIMEOUT = 25.0   # s, full swing at low speed can be slow
rclpy.init(); node = CobotIK(visualize=False)
real = {'a': None}
node.create_subscription(MycobotAngles, '/mycobot/angles_real',
    lambda m: real.__setitem__('a', np.array([m.joint_1, m.joint_2, m.joint_3, m.joint_4, m.joint_5, m.joint_6], float)), 10)
def fresh(t=4):
    real['a'] = None; t0 = time.time()
    while time.time() - t0 < t:
        rclpy.spin_once(node, timeout_sec=0.1)
        if real['a'] is not None: return real['a'].copy()
    return None
a = fresh(6)
if a is None: print('NO_ANGLES'); rclpy.shutdown(); raise SystemExit(1)
print('home: from joints', np.round(a, 1))
if np.max(np.abs(a)) < TOL:
    print('home: already there'); node.destroy_node(); rclpy.shutdown(); raise SystemExit(0)
m = MycobotSetAngles(); m.joint_1 = m.joint_2 = m.joint_3 = m.joint_4 = m.joint_5 = m.joint_6 = 0.0; m.speed = SPEED
node.cmd_angle_pub.publish(m)
t0 = time.time(); ok = False
while time.time() - t0 < TIMEOUT:
    b = fresh(1.5)
    if b is not None and np.max(np.abs(b)) < TOL:
        ok = True; break
b = fresh(2)
print('home: now at', np.round(b, 1) if b is not None else 'UNKNOWN')
node.destroy_node(); rclpy.shutdown()
if not ok: print('home: NOT reached (timeout)'); raise SystemExit(1)
print('home: OK')
