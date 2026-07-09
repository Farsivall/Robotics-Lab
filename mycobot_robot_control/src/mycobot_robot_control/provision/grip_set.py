#!/usr/bin/env python3
import sys, time
from pymycobot.mycobot import MyCobot
v=int(sys.argv[1]); sp=int(sys.argv[2]) if len(sys.argv)>2 else 20
mc=MyCobot('/dev/ttyAMA0',1000000); time.sleep(0.3)
try: mc.set_gripper_mode(0)
except Exception: pass
print('  grip before:', mc.get_gripper_value())
mc.set_gripper_value(v,sp); time.sleep(3)
print(f'  grip set->{v} after:', mc.get_gripper_value())
