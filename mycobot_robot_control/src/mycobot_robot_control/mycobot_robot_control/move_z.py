#!/usr/bin/env python3
import sys, time, numpy as np, rclpy
from mycobot_client_2.ik import CobotIK
from mycobot_msgs_2.msg import MycobotAngles, MycobotSetAngles
DEG=np.pi/180; RAD=180/np.pi; LIMS=np.array([165,165,165,165,165,179.]); DOWN=np.array([180.,0,0])
TZ=float(sys.argv[1]); STEP=0.01; SPEED=int(sys.argv[2]) if len(sys.argv)>2 else 12
rclpy.init(); node=CobotIK(visualize=False)
real={'a':None}
node.create_subscription(MycobotAngles,'/mycobot/angles_real',lambda m:real.__setitem__('a',np.array([m.joint_1,m.joint_2,m.joint_3,m.joint_4,m.joint_5,m.joint_6],float)),10)
def fresh(t=4):
    real['a']=None; t0=time.time()
    while time.time()-t0<t:
        rclpy.spin_once(node,timeout_sec=0.1)
        if real['a'] is not None: return real['a'].copy()
    return None
a=fresh(6)
if a is None: print('NO_ANGLES'); rclpy.shutdown(); raise SystemExit(1)
p,_=node.get_pose(a*DEG,'gripper'); X,Y,Z0=float(p[0]),float(p[1]),float(p[2])
print(f'from z={Z0:.3f} -> {TZ:.3f} at ({X:.3f},{Y:.3f})')
seed=a*DEG; step=STEP if TZ>Z0 else -STEP
for z in np.arange(Z0+step, TZ+step*0.5, step):
    node.real_angles=seed
    ik=node.calculate_ik(np.array([X,Y,z]),DOWN,'gripper',1e-5,0.3,0.02,True,4000,False)
    if ik is None: print(f'z={z:.3f} IK None STOP'); break
    sol=np.array(ik); sd=(sol*RAD+180)%360-180
    if not np.all(np.abs(sd)<=LIMS): print(f'z={z:.3f} limits STOP'); break
    pos,eul=node.get_pose(sol,'gripper'); err=np.linalg.norm(pos-[X,Y,z])
    if err>0.01 or abs(abs(eul[0])-180)>20: print(f'z={z:.3f} unsafe STOP'); break
    m=MycobotSetAngles(); m.joint_1,m.joint_2,m.joint_3,m.joint_4,m.joint_5,m.joint_6=[float(v) for v in sd]; m.speed=SPEED
    node.cmd_angle_pub.publish(m)
    t0=time.time()
    while time.time()-t0<5:
        b=fresh(1)
        if b is not None:
            pp,_=node.get_pose(b*DEG,'gripper')
            if abs(pp[2]-z)<0.012: break
    seed=sol
bb=fresh(2); pf,_=node.get_pose(bb*DEG,'gripper'); print(f'DONE at z={pf[2]:.3f}')
node.destroy_node(); rclpy.shutdown()
