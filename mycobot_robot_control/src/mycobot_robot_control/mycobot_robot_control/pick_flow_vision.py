#!/usr/bin/env python3
"""One-shot pick + base-rotate + place orchestrator, WITH in-process vision.

Combines cam_to_coord.py (blue-block detection) directly into pick_flow.py:
the webcam is opened and read inside vision_step(), a smoothed pixel centroid
is converted to table coords via homography, and that's fed straight into
approach_pick(). No /block_position ROS topic, no second terminal, no DDS
discovery race.

Sequence: vision -> home -> open -> approach -> descend -> partial close ->
          lift -> rotate J1 -> place descend -> open -> retreat -> home.

Usage: python pick_flow_vision.py [PX PY ROT [GRASP_Z PLACE_Z GRIP_VAL SPEED]]
       defaults: 0.18 0 90 0.02 0.06 28 25
       omit PX/PY to use the webcam instead (default behavior)
Env:   ROBOT_IP     overrides target robot (default 192.168.123.50)
       CAMERA_INDEX overrides cv2.VideoCapture index (default 0)
"""
import os, shutil, subprocess, sys, time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
from mycobot_client_2.ik import CobotIK
from mycobot_msgs_2.msg import MycobotAngles, MycobotSetAngles

sys.path.insert(0, str(Path(__file__).resolve().parent))

# --- unit note: pick_flow's arm math is in METERS (PX=0.18 etc). If your
# homography_transform only exposes pixel_to_cm, we convert cm -> m below.
# CONFIRM pixel_to_cm actually returns centimeters, or targets will be 100x off.
from homography_transform import pixel_to_meters as _pixel_to_table

a = sys.argv[1:]
USE_VISION = len(a) < 2
PX = float(a[0]) if len(a) > 0 else 0.18
PY = float(a[1]) if len(a) > 1 else 0.0
ROT = float(a[2]) if len(a) > 2 else 90.0
GZ = float(a[3]) if len(a) > 3 else 0.02
PZ = float(a[4]) if len(a) > 4 else 0.06
GV = int(a[5]) if len(a) > 5 else 28
SP = int(a[6]) if len(a) > 6 else 25

DEG = np.pi / 180.0; RAD = 180.0 / np.pi
LIMS = np.array([165, 165, 165, 165, 165, 179.0])
DOWN = np.array([180.0, 0.0, 0.0])
LIFT = 0.10; APPR = 0.04; STEP = 0.01
RIP = os.environ.get('ROBOT_IP', '192.168.123.50')
_mf = os.path.expanduser('~/miniforge3/bin')
if os.path.isdir(_mf):
    os.environ['PATH'] = _mf + os.pathsep + os.environ.get('PATH', '')
SSHPASS = shutil.which('sshpass') or '/usr/bin/sshpass'
SSH_OPTS = ['-o', 'StrictHostKeyChecking=no',
            '-o', 'PreferredAuthentications=password',
            '-o', 'PubkeyAuthentication=no']
SSH = [SSHPASS, '-p', 'Elephant', 'ssh'] + SSH_OPTS + ['er@' + RIP]
SCP = [SSHPASS, '-p', 'Elephant', 'scp'] + SSH_OPTS
_GRIP_SET_LOCAL = Path(__file__).resolve().parents[1] / 'provision' / 'grip_set.py'
if not _GRIP_SET_LOCAL.is_file():
    _GRIP_SET_LOCAL = Path(__file__).resolve().parents[2] / 'provision' / 'grip_set.py'
_grip_ready = False
if GV < 25: print('GRIP_VAL < 25 risks stall-current brownout'); raise SystemExit(2)
print(f'grip_val={GV} (100=open, lower=tighter)')

# ---------------------------------------------------------------------------
# Vision (formerly cam_to_coord.py) -- pure OpenCV, no ROS involved.
# ---------------------------------------------------------------------------
CAM_WIDTH = 1280
CAM_HEIGHT = 720
CAMERA_INDEX = int(os.environ.get('CAMERA_INDEX', 0))

MID_RGB = (33, 52, 100)  # blue calibration midpoint
_mid_bgr = np.uint8([[[MID_RGB[2], MID_RGB[1], MID_RGB[0]]]])
_mid_hsv = cv2.cvtColor(_mid_bgr, cv2.COLOR_BGR2HSV)[0, 0]
H, S, V = int(_mid_hsv[0]), int(_mid_hsv[1]), int(_mid_hsv[2])
H_TOL, S_TOL, V_TOL = 12, 70, 70
HSV_LO = np.array([max(0, H - H_TOL), max(40, S - S_TOL), max(30, V - V_TOL)])
HSV_HI = np.array([min(179, H + H_TOL), 255, min(255, V + V_TOL)])
DRAW_COLOR = (int(_mid_bgr[0, 0, 0]), int(_mid_bgr[0, 0, 1]), int(_mid_bgr[0, 0, 2]))

MIN_AREA_PX = 200
SMOOTH_N = 3          # frames to average
STABLE_N = 5          # consecutive good detections required before accepting
VISION_TIMEOUT = 60.0


def _color_mask(frame_bgr):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    hsv = cv2.GaussianBlur(hsv, (5, 5), 0)
    mask = cv2.inRange(hsv, HSV_LO, HSV_HI)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask


def _detect_block(frame, min_area=MIN_AREA_PX):
    mask = _color_mask(frame)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    if area < min_area:
        return None
    M = cv2.moments(contour)
    if M["m00"] == 0:
        return None
    cx = M["m10"] / M["m00"]
    cy = M["m01"] / M["m00"]
    return cx, cy, area, contour, mask


def capture_block_xy(timeout=VISION_TIMEOUT, show=True):
    """Blocking webcam capture -> smoothed pixel centroid -> table coords.

    Requires STABLE_N consecutive good detections (after SMOOTH_N-frame
    smoothing) before accepting, to avoid firing on a single noisy frame.
    Returns (x, y) in meters, or (None, None) on timeout/camera failure.
    """
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"vision: failed to open camera {CAMERA_INDEX}")
        return None, None

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"vision: camera {CAMERA_INDEX} {actual_w}x{actual_h}, waiting for blue block "
          f"(need {STABLE_N} stable frames, timeout {timeout:.0f}s, 'q' to abort)")

    recent = deque(maxlen=SMOOTH_N)
    stable_count = 0
    t0 = time.time()
    result_xy = (None, None)

    try:
        while time.time() - t0 < timeout:
            ret, frame = cap.read()
            if not ret:
                print("vision: failed to capture frame")
                break

            det = _detect_block(frame)
            display = frame.copy()

            if det is None:
                stable_count = 0
                recent.clear()
                if show:
                    dbg = cv2.cvtColor(_color_mask(frame), cv2.COLOR_GRAY2BGR)
                    scale = 200 / max(dbg.shape[1], 1)
                    dbg = cv2.resize(dbg, (int(dbg.shape[1] * scale), int(dbg.shape[0] * scale)))
                    display[0:dbg.shape[0], 0:dbg.shape[1]] = dbg
                    cv2.putText(display, "no blue block", (10, dbg.shape[0] + 24),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            else:
                cx, cy, area, contour, mask = det
                recent.append((cx, cy))
                stable_count += 1
                sx = sum(p[0] for p in recent) / len(recent)
                sy = sum(p[1] for p in recent) / len(recent)
                x, y = _pixel_to_table(sx, sy)

                if show:
                    cv2.drawContours(display, [contour], -1, DRAW_COLOR, 2)
                    cv2.circle(display, (int(sx), int(sy)), 10, (0, 255, 0), -1)
                    cv2.putText(display, f"({x:.3f},{y:.3f}) m  n={stable_count}/{STABLE_N}",
                                (int(sx) + 14, int(sy) - 14),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                if stable_count >= STABLE_N:
                    result_xy = (x, y)
                    print(f"vision: locked block at x={x:.3f} y={y:.3f} (m)")
                    break

            if show:
                cv2.imshow("pick_flow vision", display)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    print("vision: aborted by user")
                    break
    finally:
        cap.release()
        if show:
            cv2.destroyAllWindows()

    if result_xy == (None, None):
        print("vision: TIMEOUT / no stable detection")
    return result_xy


# ---------------------------------------------------------------------------
# Arm control (unchanged from pick_flow.py, minus the /block_position topic)
# ---------------------------------------------------------------------------
rclpy.init(); node = CobotIK(visualize=False)
real = {'a': None}
target = {'x': None, 'y': None}

node.create_subscription(MycobotAngles, '/mycobot/angles_real',
    lambda m: real.__setitem__('a', np.array([m.joint_1, m.joint_2, m.joint_3, m.joint_4, m.joint_5, m.joint_6], float)), 10)
print(f"pick_flow ready | vision={USE_VISION} | ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}")

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

def ensure_grip_script():
    global _grip_ready
    if _grip_ready:
        return True
    if not _GRIP_SET_LOCAL.is_file():
        print(f'grip: WARNING local grip_set.py missing at {_GRIP_SET_LOCAL}')
        return True
    print(f'grip: copying {_GRIP_SET_LOCAL.name} -> er@{RIP}:/home/er/grip_set.py')
    try:
        r = subprocess.run(
            SCP + [str(_GRIP_SET_LOCAL), f'er@{RIP}:/home/er/grip_set.py'],
            capture_output=True, text=True, timeout=30,
        )
    except FileNotFoundError:
        print(f'grip: FAILED — sshpass not found ({SSHPASS})')
        return False
    if r.returncode != 0:
        print('grip: FAILED scp:', (r.stderr or r.stdout or '').strip())
        return False
    _grip_ready = True
    return True

def grip(v, sp):
    """Same open/close path as hybrid_pick_place.sh (SSH + grip_set.py + sleep 5)."""
    if not ensure_grip_script():
        return False
    action = 'OPEN' if v >= 90 else 'CLOSE'
    print(f'grip: {action}  value={v} speed={sp}  via er@{RIP} grip_set.py')
    sh = (
        f"echo 'grip->{v} boot='$(uptime -s); "
        f"docker stop -t 2 mycobot_comms>/dev/null 2>&1; "
        f"python3 /home/er/grip_set.py {v} {sp}; rc=$?; "
        f"echo 'boot_after='$(uptime -s); "
        f"docker start mycobot_comms>/dev/null 2>&1; "
        f"exit $rc"
    )
    try:
        r = subprocess.run(SSH + [sh], capture_output=True, text=True, timeout=90)
    except FileNotFoundError:
        print(f'grip: FAILED — sshpass not found ({SSHPASS})')
        return False
    except subprocess.TimeoutExpired:
        print('grip: FAILED — SSH timed out')
        return False
    out = ((r.stdout or '') + '\n' + (r.stderr or '')).strip()
    for line in out.splitlines():
        if 'Permission denied' in line or 'Warning:' in line:
            continue
        if line.strip():
            print(' ', line)
    if 'grip before:' not in out and 'grip set->' not in out:
        print('grip: FAILED — grip_set.py did not run (claw will not move)')
        print(f'  tip: export ROBOT_IP=<pi-ip>  (current={RIP})')
        return False
    if r.returncode != 0:
        print(f'grip: FAILED exit={r.returncode}')
        return False
    boots = [l.split('=', 1)[1] for l in out.splitlines() if 'boot' in l and '=' in l]
    if len(boots) >= 2 and boots[0] != boots[-1]:
        print('grip: PI REBOOTED during grip')
        return False
    time.sleep(5)  # hybrid always waits here
    if fresh(25) is None:
        print('grip: WARNING angles not back yet — continuing')
    print(f'grip: OK {action} -> {v}')
    return True

def open_gripper():
    return grip(100, 40)

def close_gripper():
    return grip(GV, 15)

def approach_pick():
    px, py = (PX, PY) if not USE_VISION else (target['x'], target['y'])
    if px is None or py is None:
        print('approach: no target coords')
        return False
    return approach(px, py, GZ + APPR)

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

def vision_step():
    """Grab block pose from the webcam BEFORE moving the arm."""
    if not USE_VISION:
        print(f'vision: using CLI PX={PX} PY={PY}')
        return True
    px, py = capture_block_xy()
    if px is None:
        return False
    target['x'], target['y'] = px, py
    return True

steps = [
    ('vision',        vision_step),
    ('home',          lambda: home()),
    ('open',          open_gripper),       # hybrid: grip 100 40
    ('approach',      approach_pick),
    ('descend',       lambda: move_z(GZ, 15)),
    ('grip',          close_gripper),      # hybrid: grip $GV 15
    ('lift',          lambda: move_z(LIFT, SP)),
    ('rotate',        lambda: rotate(ROT)),
    ('place-descend', lambda: move_z(PZ, SP)),
    ('release',       open_gripper),       # hybrid: grip 100 40
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