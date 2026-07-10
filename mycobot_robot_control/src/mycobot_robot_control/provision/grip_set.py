#!/usr/bin/env python3
"""Set myCobot gripper value on the Pi (used by hybrid_pick_place / pick_flow).

  100 = open
  lower = more closed (keep >= 25 to avoid Pi brownout)

Usage on Pi:  python3 /home/er/grip_set.py VALUE [SPEED]
"""
import sys
import time

from pymycobot.mycobot import MyCobot

v = int(sys.argv[1])
sp = int(sys.argv[2]) if len(sys.argv) > 2 else 20

mc = MyCobot("/dev/ttyAMA0", 1000000)
time.sleep(0.5)
try:
    mc.set_gripper_mode(0)
except Exception:
    pass

before = mc.get_gripper_value()
print("  grip before:", before, flush=True)

# Send twice — some firmware drops the first command after docker stop
mc.set_gripper_value(v, sp)
time.sleep(1.5)
mc.set_gripper_value(v, sp)
time.sleep(2.5)

after = mc.get_gripper_value()
print(f"  grip set->{v} after:", after, flush=True)
