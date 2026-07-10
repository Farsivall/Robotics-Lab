#!/usr/bin/env python3
"""Color-driven pick + base-rotate + place, WITH in-process vision.

Mimics the original pick_flow.py motion exactly (home -> open -> approach ->
descend -> partial close -> lift -> rotate J1 -> place descend -> open ->
retreat -> home) but you tell it WHICH color to grab. The webcam detects that
color (tracking visuals on), then the arm goes to the hardcoded pick position
for that color and runs the pick.

Sequence: vision(detect chosen color) -> home -> open -> approach -> descend ->
          partial close -> lift -> rotate J1 -> place descend -> open ->
          retreat -> home.

Usage: python pick_flow_vision.py [COLOR] [ROT]
       COLOR = blue | yellow | purple   (which block to find + pick; default blue)
       ROT   = base rotate degrees for the place side (default 90)
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

_args = sys.argv[1:]
TARGET_COLOR = _args[0].lower() if len(_args) > 0 else 'blue'
ROT = float(_args[1]) if len(_args) > 1 else 90.0
GZ = 0.02
PZ = 0.06
GV = 28
SP = 25

DEG = np.pi / 180.0; RAD = 180.0 / np.pi
LIMS = np.array([165, 165, 165, 165, 165, 179.0])
DOWN = np.array([180.0, 0.0, 0.0])
LIFT = 0.10; APPR = 0.04; STEP = 0.01
RIP = os.environ.get('ROBOT_IP', '192.168.123.50')
_mf = os.path.expanduser('~/miniforge3/bin')
if os.path.isdir(_mf):
    os.environ['PATH'] = _mf + os.pathsep + os.environ.get('PATH', '')
SSHPASS = shutil.which('sshpass') or '/usr/bin/sshpass'
_OPTS = ['-o', 'StrictHostKeyChecking=no',
         '-o', 'PreferredAuthentications=password',
         '-o', 'PubkeyAuthentication=no']
SSH = [SSHPASS, '-p', 'Elephant', 'ssh'] + _OPTS + ['er@' + RIP]
SCP = [SSHPASS, '-p', 'Elephant', 'scp'] + _OPTS
_GRIP_SET_LOCAL = Path(__file__).resolve().parents[1] / 'provision' / 'grip_set.py'
if not _GRIP_SET_LOCAL.is_file():
    _GRIP_SET_LOCAL = Path(__file__).resolve().parents[2] / 'provision' / 'grip_set.py'
_grip_ready = {'ok': False}
GRIP_OPEN = 100
GRIP_CLOSE = 28
print(f'grip hardcoded: open={GRIP_OPEN} close={GRIP_CLOSE}')

# ---------------------------------------------------------------------------
# Vision (formerly cam_to_coord.py) -- pure OpenCV, no ROS involved.
# ---------------------------------------------------------------------------
CAM_WIDTH = 1280
CAM_HEIGHT = 720
CAMERA_INDEX = int(os.environ.get('CAMERA_INDEX', 0))

# Color midpoints (RGB) — same values as cam_to_coord.py
COLOR_RGB = {
    'blue':   (33, 52, 100),
    'purple': (67, 41, 65),
    'yellow': (249, 222, 0),
}

def _hsv_range(r, g, b):
    mid_bgr = np.uint8([[[b, g, r]]])
    mid_hsv = cv2.cvtColor(mid_bgr, cv2.COLOR_BGR2HSV)[0, 0]
    h, s, v = int(mid_hsv[0]), int(mid_hsv[1]), int(mid_hsv[2])
    h_tol, s_tol, v_tol = 12, 70, 70
    lo = np.array([max(0, h - h_tol), max(40, s - s_tol), max(30, v - v_tol)])
    hi = np.array([min(179, h + h_tol), 255, min(255, v + v_tol)])
    draw = (int(mid_bgr[0, 0, 0]), int(mid_bgr[0, 0, 1]), int(mid_bgr[0, 0, 2]))
    return lo, hi, draw

COLOR_HSV = {}
DRAW_BY_COLOR = {}
for _name, _rgb in COLOR_RGB.items():
    _lo, _hi, _draw = _hsv_range(*_rgb)
    COLOR_HSV[_name] = (_lo, _hi)
    DRAW_BY_COLOR[_name] = _draw

# HARDCODED pick position per color, base-frame METERS (x forward, y left).
# The camera confirms/tracks the chosen color; the arm goes to these coords.
# EDIT to match where each block actually sits.
PICK_POSITIONS = {
    'blue':   (0.18,  0.00),
    'yellow': (0.18, -0.06),
    'purple': (0.18,  0.06),
}

if TARGET_COLOR not in COLOR_HSV:
    print(f"unknown color '{TARGET_COLOR}'. Use one of: {', '.join(COLOR_HSV)}")
    raise SystemExit(2)
HSV_LO, HSV_HI = COLOR_HSV[TARGET_COLOR]
DRAW_COLOR = DRAW_BY_COLOR[TARGET_COLOR]
PX, PY = PICK_POSITIONS[TARGET_COLOR]
print(f"target color: {TARGET_COLOR} -> hardcoded pick ({PX:.3f}, {PY:.3f}) m")

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
    print(f"vision: camera {CAMERA_INDEX} {actual_w}x{actual_h}, waiting for {TARGET_COLOR} block "
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
                    cv2.putText(display, f"no {TARGET_COLOR} block", (10, dbg.shape[0] + 24),
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
                    cv2.putText(display,
                                f"{TARGET_COLOR} -> pick ({PX:.3f},{PY:.3f})m  n={stable_count}/{STABLE_N}",
                                (int(sx) + 14, int(sy) - 14),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                if stable_count >= STABLE_N:
                    result_xy = (x, y)
                    print(f"vision: {TARGET_COLOR} locked (mapped x={x:.3f} y={y:.3f} m)")
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
print(f"pick_flow ready | color={TARGET_COLOR} | ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}")

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
    """Copy provision/grip_set.py to the Pi so the claw script is present."""
    if _grip_ready['ok']:
        return True
    if not _GRIP_SET_LOCAL.is_file():
        print(f'grip: local grip_set.py missing at {_GRIP_SET_LOCAL} — hoping Pi has it')
        _grip_ready['ok'] = True
        return True
    try:
        r = subprocess.run(
            SCP + [str(_GRIP_SET_LOCAL), f'er@{RIP}:/home/er/grip_set.py'],
            capture_output=True, text=True, timeout=30,
        )
    except FileNotFoundError:
        print(f'grip: FAILED — sshpass not found ({SSHPASS}). sudo apt install sshpass')
        return False
    except subprocess.TimeoutExpired:
        print(f'grip: FAILED — scp timed out (check ROBOT_IP={RIP} / network)')
        return False
    if r.returncode != 0:
        print('grip: FAILED to copy grip_set.py:', (r.stderr or r.stdout or '').strip())
        return False
    _grip_ready['ok'] = True
    return True

def grip(v, sp):
    """Hardcoded open/close via SSH + grip_set.py (hybrid_pick_place style)."""
    action = 'OPEN' if v >= 90 else 'CLOSE'
    print(f'grip: {action} value={v} speed={sp} via er@{RIP}')
    if not ensure_grip_script():
        return False
    sh = (
        f"docker stop -t 2 mycobot_comms>/dev/null 2>&1; "
        f"python3 /home/er/grip_set.py {v} {sp}; "
        f"docker start mycobot_comms>/dev/null 2>&1"
    )
    try:
        r = subprocess.run(SSH + [sh], capture_output=True, text=True, timeout=90)
    except FileNotFoundError:
        print(f'grip: FAILED — sshpass not found ({SSHPASS}). sudo apt install sshpass')
        return False
    except subprocess.TimeoutExpired:
        print(f'grip: FAILED — SSH timed out (check ROBOT_IP={RIP} / network)')
        return False
    out = ((r.stdout or '') + '\n' + (r.stderr or '')).strip()
    for line in out.splitlines():
        if line.strip() and 'Permission denied' not in line and 'Warning:' not in line:
            print(' ', line)
    if 'grip before:' not in out and 'grip set->' not in out:
        print('grip: FAILED — grip_set.py produced no output (claw did NOT move)')
        print(f'  check: ROBOT_IP={RIP}, Pi reachable, /home/er/grip_set.py exists')
        print('  from laptop:  ./test_robot.sh')
        return False
    time.sleep(5)
    fresh(20)
    print(f'grip: OK {action} -> {v}')
    return True

def approach_pick():
    if target['x'] is None or target['y'] is None:
        print('approach: no target coords')
        return False
    return approach(target['x'], target['y'], GZ + APPR)

def solve_ik(x, y, z):
    """Best IK for (x,y,z): try current-pose seed (lets off-center targets
    solve) then zero seed (accurate near the front-center default)."""
    cur = fresh(4)
    attempts = []
    if cur is not None:
        attempts.append(('current-seed', cur * DEG, True))
    attempts.append(('zero-seed', np.zeros(6), False))
    best = None
    for label, seed, use_seed in attempts:
        node.real_angles = seed
        ik = node.calculate_ik(np.array([x, y, z]), DOWN, 'gripper', 1e-5, 0.3, 0.02, use_seed, 4000, False)
        if ik is None:
            continue
        adj = np.array(node.adjust_angles(np.array(ik)), float)
        pos, _eul = node.get_pose(adj * DEG, 'gripper')
        err = float(np.linalg.norm(pos - [x, y, z]))
        if not np.all(np.abs(adj) <= LIMS):
            continue
        if best is None or err < best[1]:
            best = (adj, err, label)
    return best

def approach(x, y, z):
    best = solve_ik(x, y, z)
    if best is None:
        print('approach: IK None (no in-limit solution from any seed)'); return False
    adj, err, label = best
    if err > 0.02:
        print(f'approach: REFUSED (err {err*1000:.0f}mm, {label})'); return False
    ok = goto(adj, SP)
    print(f'approach: {"OK" if ok else "TIMEOUT"} err={err*1000:.1f}mm ({label})'); return ok

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
    """Detect the chosen color with the webcam BEFORE moving the arm, then
    pick at that color's HARDCODED position."""
    mx, my = capture_block_xy()
    if mx is None:
        return False
    target['x'], target['y'] = PX, PY
    print(f"vision: {TARGET_COLOR} detected (mapped {mx:.3f},{my:.3f}) "
          f"-> using hardcoded pick ({PX:.3f},{PY:.3f})")
    return True

steps = [
    ('vision',        vision_step),
    ('home',          lambda: home()),
    ('open',          lambda: grip(GRIP_OPEN, 40)),
    ('approach',      approach_pick),
    ('descend',       lambda: move_z(GZ, 15)),
    ('grip',          lambda: grip(GRIP_CLOSE, 15)),
    ('lift',          lambda: move_z(LIFT, SP)),
    ('rotate',        lambda: rotate(ROT)),
    ('place-descend', lambda: move_z(PZ, SP)),
    ('release',       lambda: grip(GRIP_OPEN, 40)),
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