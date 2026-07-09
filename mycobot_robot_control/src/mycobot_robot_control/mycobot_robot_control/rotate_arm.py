#!/usr/bin/env python3
import sys, time, numpy as np, rclpy
from mycobot_client_2.ik import CobotIK
from mycobot_msgs_2.msg import MycobotAngles, MycobotSetAngles
DEG=np.pi/180; DELTA=float(sys.argv[1]); SPEED=int(sys.argv[2]) if len(sys.argv)>2 else 12
rclpy.init(); node=CobotIK(visualize=False)
real={'a':None}
node.create_subscription(MycobotAngles,'/mycobot/angles_real',lambda m:real.__setitem__('a',np.array([m.joint_1,m.joint_2,m.joint_3,m.joint_4,m.joint_5,m.joint_6],float)),10)
def fresh(t=4):
    real['a']=None; t0=time.time()
    while time.time()-t0<t:
        rclpy.spin_once(node,timeout_sec=0.1)
        if real['a'] is not None: return real['a'].copy()
    return None
cur=fresh(6)
if cur is None: print('NO_ANGLES'); rclpy.shutdown(); raise SystemExit(1)
newJ1=cur[0]+DELTA
if abs(newJ1)>165: print(f'newJ1 {newJ1:.0f} limit STOP'); rclpy.shutdown(); raise SystemExit(1)
steps=max(2,int(np.ceil(abs(DELTA)/30)))
for i in range(1,steps+1):
    j1=cur[0]+DELTA*(i/steps)
    m=MycobotSetAngles(); m.joint_1=float(j1); m.joint_2=float(cur[1]); m.joint_3=float(cur[2]); m.joint_4=float(cur[3]); m.joint_5=float(cur[4]); m.joint_6=float(cur[5]); m.speed=SPEED
    node.cmd_angle_pub.publish(m)
    t0=time.time()
    while time.time()-t0<6:
        b=fresh(1)
        if b is not None and abs(b[0]-j1)<3: break
    print(f'J1 -> {j1:.0f}')
node.destroy_node(); rclpy.shutdown(); print('ROTATE_DONE')
