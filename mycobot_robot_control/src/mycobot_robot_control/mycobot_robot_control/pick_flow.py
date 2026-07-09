#!/usr/bin/env python3
"""One-shot pick + base-rotate + place orchestrator. Single ROS node (init paid
once), gripper via SSH -> Pi-host pymycobot partial close (no docker spin-up,
avoids the gripper-current brownout). Fail-fast: any step failure aborts, arm
holds position.

Sequence: home -> open -> approach -> descend -> partial close -> lift ->
          rotate J1 -> place descend -> open -> retreat -> home.

Usage: python pick_flow.py [PX PY ROT [GRASP_Z PLACE_Z GRIP_VAL SPEED]]
       defaults: 0.18 0 90 0.02 0.06 35 25
Env:   ROBOT_IP overrides target robot (default 192.168.123.50)
"""
import os, subprocess, sys, time
import numpy as np
import rclpy
from mycobot_client_2.ik import CobotIK
from mycobot_msgs_2.msg import MycobotAngles, MycobotSetAngles

a = sys.argv[1:]
PX = float(a[0]) if len(a) > 0 else 0.18
PY = float(a[1]) if len(a) > 1 else 0.0
ROT = float(a[2]) if len(a) > 2 else 90.0
GZ = float(a[3]) if len(a) > 3 else 0.02
PZ = float(a[4]) if len(a) > 4 else 0.06
GV = int(a[5]) if len(a) > 5 else 35
SP = int(a[6]) if len(a) > 6 else 25

DEG = np.pi / 180.0; RAD = 180.0 / np.pi
LIMS = np.array([165, 165, 165, 165, 165, 179.0])
DOWN = np.array([180.0, 0.0, 0.0])
LIFT = 0.10; APPR = 0.04; STEP = 0.01
RIP = os.environ.get('ROBOT_IP', '192.168.123.50')
SSH = ['/usr/bin/sshpass', '-p', 'Elephant', 'ssh', '-o', 'StrictHostKeyChecking=no',
       '-o', 'PreferredAuthentications=password', '-o', 'PubkeyAuthentication=no', 'er@' + RIP]
if GV < 25: print('GRIP_VAL < 25 risks stall-current brownout'); raise SystemExit(2)

rclpy.init(); node = CobotIK(visualize=False)
real = {'a': None}
node.create_subscription(MycobotAngles, '/mycobot/angles_real',
    lambda m: real.__setitem__('a', np.array([m.joint_1, m.joint_2, m.joint_3, m.joint_4, m.joint_5, m.joint_6], float)), 10)

def fresh(t=4.0):
    real['a'] = None; t0 = time.time()
    while time.time() - t0 < t:
        rclpy.spin_once(node, timeout_sec=0.1)
        if real['a'] is not None: return real['a'].copy()
    return None

def cmd(deg6, speed):
    m = MycobotSetAngles()
    m.joint_1, m.joint_2, m.joint_3, m.joint_4, m.joint_5, m.joint_6 = [float(v) for v in deg6]
    m.speed = speed; node.cmd_angle_pub.publish(m)

def goto(deg6, speed, tol=3.5, timeout=14.0):
    # Resend the goal each poll: after a comms restart the first publish can be
    # lost while DDS pub/sub discovery is still completing. ~0.7 Hz is gentle.
    t0 = time.time()
    while time.time() - t0 < timeout:
        cmd(deg6, speed)
        b = fresh(1.5)
        if b is not None and np.max(np.abs(b - np.asarray(deg6))) < tol: return True
    return False

def home(speed=30):
    b = fresh(12)
    if b is None: print('home: NO_ANGLES'); return False
    if np.max(np.abs(b)) < 4: print('home: already there'); return True
    ok = goto(np.zeros(6), speed, 4.0, 25.0)
    print('home:', 'OK' if ok else 'TIMEOUT'); return ok

def grip(v, sp):
    sh = (f"echo '  grip boot='$(uptime -s); docker stop -t 2 mycobot_comms>/dev/null 2>&1; "
          f"python3 /home/er/grip_set.py {v} {sp}; docker start mycobot_comms>/dev/null 2>&1; "
          f"echo '  boot_after='$(uptime -s)")
    r = subprocess.run(SSH + [sh], capture_output=True, text=True, timeout=90)
    out = r.stdout.strip()
    print(out)
    boots = [l.split('=', 1)[1] for l in out.splitlines() if 'boot' in l and '=' in l]
    if len(boots) == 2 and boots[0] != boots[1]: print('grip: PI REBOOTED'); return False
    if fresh(20) is None: print('grip: comms did not come back'); return False
    return True

def approach(x, y, z):
    ik = node.calculate_ik(np.array([x, y, z]), DOWN, 'gripper', 1e-5, 0.3, 0.02, False, 4000, False)
    if ik is None: print('approach: IK None'); return False
    adj = np.array(node.adjust_angles(np.array(ik)), float)
    pos, eul = node.get_pose(adj * DEG, 'gripper')
    err = float(np.linalg.norm(pos - [x, y, z]))
    if err > 0.02 or not np.all(np.abs(adj) <= LIMS):
        print(f'approach: REFUSED (err {err*1000:.0f}mm)'); return False
    ok = goto(adj, SP)
    print(f'approach: {"OK" if ok else "TIMEOUT"} err={err*1000:.1f}mm'); return ok

def move_z(tz, speed):
    b = fresh(6)
    if b is None: print('move_z: NO_ANGLES'); return False
    p, _ = node.get_pose(b * DEG, 'gripper'); X, Y, Z0 = float(p[0]), float(p[1]), float(p[2])
    seed = b * DEG; step = STEP if tz > Z0 else -STEP
    for z in np.arange(Z0 + step, tz + step * 0.5, step):
        node.real_angles = seed
        ik = node.calculate_ik(np.array([X, Y, z]), DOWN, 'gripper', 1e-5, 0.3, 0.02, True, 4000, False)
        if ik is None: print(f'move_z: IK None at {z:.3f}'); return False
        sol = np.array(ik); sd = (sol * RAD + 180) % 360 - 180
        if not np.all(np.abs(sd) <= LIMS): print(f'move_z: limits at {z:.3f}'); return False
        pos, eul = node.get_pose(sol, 'gripper')
        if np.linalg.norm(pos - [X, Y, z]) > 0.01 or abs(abs(eul[0]) - 180) > 20:
            print(f'move_z: unsafe at {z:.3f}'); return False
        t0 = time.time()
        while time.time() - t0 < 5:
            cmd(sd, speed)
            bb = fresh(1)
            if bb is not None:
                pp, _ = node.get_pose(bb * DEG, 'gripper')
                if abs(pp[2] - z) < 0.012: break
        seed = sol
    bb = fresh(2)
    if bb is not None:
        pf, _ = node.get_pose(bb * DEG, 'gripper'); print(f'move_z: at z={pf[2]:.3f}')
    return True

def rotate(delta):
    cur = fresh(6)
    if cur is None: print('rotate: NO_ANGLES'); return False
    if abs(cur[0] + delta) > 165: print(f'rotate: newJ1 {cur[0]+delta:.0f} limit'); return False
    for i in range(1, max(2, int(np.ceil(abs(delta) / 30))) + 1):
        j1 = cur[0] + delta * (i / max(2, int(np.ceil(abs(delta) / 30))))
        t0 = time.time()
        while time.time() - t0 < 6:
            cmd([j1, cur[1], cur[2], cur[3], cur[4], cur[5]], SP)
            b = fresh(1)
            if b is not None and abs(b[0] - j1) < 3: break
    print('rotate: OK'); return True

steps = [
    ('home',          lambda: home()),
    ('open',          lambda: grip(100, 40)),
    ('approach',      lambda: approach(PX, PY, GZ + APPR)),
    ('descend',       lambda: move_z(GZ, 15)),
    ('grip',          lambda: grip(GV, 15)),
    ('lift',          lambda: move_z(LIFT, SP)),
    ('rotate',        lambda: rotate(ROT)),
    ('place-descend', lambda: move_z(PZ, SP)),
    ('release',       lambda: grip(100, 40)),
    ('retreat',       lambda: move_z(0.12, SP)),
    ('home-end',      lambda: home()),
]
t_start = time.time()
for name, fn in steps:
    print(f'== {name} ==')
    if not fn():
        print(f'!! FAILED at {name} -- ABORT, arm holds position')
        node.destroy_node(); rclpy.shutdown(); raise SystemExit(1)
print(f'== DONE in {time.time()-t_start:.0f}s ==')
node.destroy_node(); rclpy.shutdown()
