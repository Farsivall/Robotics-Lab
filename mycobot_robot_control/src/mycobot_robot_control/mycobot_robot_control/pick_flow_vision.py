#!/usr/bin/env python3
"""Color-select pick + place with detection UI overlay.

Pass blue / yellow / purple: camera shows that color briefly, then the arm
picks at a hardcoded offset for that color (blue=left, yellow=center,
purple=right) and runs the original place motion.

Sequence: vision(show detection) -> home -> open -> approach -> descend ->
          grip -> lift -> rotate -> place -> open -> retreat -> home.

Usage: python pick_flow_vision.py [COLOR] [ROT]
       COLOR = blue | yellow | purple  (default yellow = center)
       ROT   = place-side base rotate degrees (default 90)
Env:   ROBOT_IP, CAMERA_INDEX
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

_args = sys.argv[1:]
ROT = 90.0
TARGET_COLOR = 'yellow'  # center default
if len(_args) >= 1:
    if _args[0].lower() in ('blue', 'yellow', 'purple'):
        TARGET_COLOR = _args[0].lower()
        if len(_args) >= 2:
            ROT = float(_args[1])
    else:
        ROT = float(_args[0])
UI_COLOR_PREF = TARGET_COLOR

GZ = 0.02
PZ = 0.06
SP = 25

DEG = np.pi / 180.0; RAD = 180.0 / np.pi
LIMS = np.array([165, 165, 165, 165, 165, 179.0])
DOWN = np.array([180.0, 0.0, 0.0])
LIFT = 0.10; APPR = 0.04; STEP = 0.01


def discover_robot_ip():
    """Find a reachable Pi IP (same idea as pick_vision.sh / test_robot.sh)."""
    env = os.environ.get('ROBOT_IP', '').strip()
    if env:
        return env
    cands = []
    try:
        neigh = subprocess.run(['ip', 'neigh', 'show'], capture_output=True, text=True, timeout=5)
        for line in (neigh.stdout or '').splitlines():
            parts = line.split()
            if parts and parts[0].startswith('192.168.123.'):
                cands.append(parts[0])
    except Exception:
        pass
    for tip in ('192.168.123.50', '10.10.10.235'):
        cands.append(tip)
    # unique, keep order
    seen, uniq = set(), []
    for ip in cands:
        if ip not in seen:
            seen.add(ip); uniq.append(ip)
    reachable = []
    for ip in uniq:
        try:
            r = subprocess.run(['ping', '-c1', '-W1', ip], capture_output=True, timeout=3)
            if r.returncode == 0:
                reachable.append(ip)
        except Exception:
            pass
    if len(reachable) == 1:
        print(f'== robot auto-discovered: {reachable[0]} ==')
        return reachable[0]
    if len(reachable) > 1:
        print(f'Multiple robots reachable: {reachable}. Set ROBOT_IP=<one> and re-run.')
        raise SystemExit(2)
    print('No robot found on 192.168.123.x / 10.10.10.235.')
    print('  Check Ethernet to the Pi, wait ~30s after power-on, then:')
    print('  export ROBOT_IP=<pi-ip>   # find with: ip neigh | grep 192.168.123')
    raise SystemExit(2)


RIP = discover_robot_ip()
_mf = os.path.expanduser('~/miniforge3/bin')
if os.path.isdir(_mf):
    os.environ['PATH'] = _mf + os.pathsep + os.environ.get('PATH', '')
SSHPASS = shutil.which('sshpass') or '/usr/bin/sshpass'
_OPTS = ['-o', 'StrictHostKeyChecking=no',
         '-o', 'ConnectTimeout=5',
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
print(f'grip hardcoded: open={GRIP_OPEN} close={GRIP_CLOSE} | ROBOT_IP={RIP}')

# ---------------------------------------------------------------------------
# Hardcoded pick per color (meters: x forward, y left).
# blue = left (+y), yellow = center, purple = right (-y)
# ---------------------------------------------------------------------------
PICK_POSITIONS = {
    'blue':   (0.18,  0.06),  # left
    'yellow': (0.18,  0.00),  # center
    'purple': (0.18, -0.06),  # right
}
PRESET_PICK = PICK_POSITIONS[TARGET_COLOR]
# place is: lift -> rotate ROT -> descend to PZ (same as original pick_flow)

CAM_WIDTH = 1280
CAM_HEIGHT = 720
CAMERA_INDEX = int(os.environ.get('CAMERA_INDEX', 0))
PREVIEW_SEC = 4.0  # show detection UI this long, then always start the pick

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

MIN_AREA_PX = 200
SMOOTH_N = 3

print(f"color={TARGET_COLOR} -> pick ({PRESET_PICK[0]:.3f}, {PRESET_PICK[1]:.3f}) m | rotate={ROT}")


def _color_mask(frame_bgr, color):
    lo, hi = COLOR_HSV[color]
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    hsv = cv2.GaussianBlur(hsv, (5, 5), 0)
    mask = cv2.inRange(hsv, lo, hi)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask


def _detect_any(frame, prefer=None, min_area=MIN_AREA_PX):
    """Return best blob among colors (prefer optional). (cx,cy,area,contour,mask,name) or None."""
    order = list(COLOR_HSV.keys())
    if prefer in COLOR_HSV:
        order = [prefer] + [c for c in order if c != prefer]
    best = None
    for name in order:
        mask = _color_mask(frame, name)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue
        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        if area < min_area:
            continue
        M = cv2.moments(contour)
        if M["m00"] == 0:
            continue
        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]
        if best is None or area > best[2]:
            best = (cx, cy, area, contour, mask, name)
    return best


def show_detection_preview(seconds=PREVIEW_SEC):
    """Show color tracking in the UI, then return. Never blocks the pick."""
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"vision: camera {CAMERA_INDEX} not available — skipping UI, starting pick anyway")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
    print(f"vision: showing color detection for {seconds:.0f}s (then preset pick starts). Press q to skip.")
    recent = deque(maxlen=SMOOTH_N)
    t0 = time.time()
    last_seen = None
    try:
        while time.time() - t0 < seconds:
            ret, frame = cap.read()
            if not ret:
                break
            display = frame.copy()
            prefer = UI_COLOR_PREF if UI_COLOR_PREF != "auto" else None
            det = _detect_any(frame, prefer=prefer)
            if det is None:
                recent.clear()
                dbg = _color_mask(frame, prefer or "blue")
                preview = cv2.cvtColor(dbg, cv2.COLOR_GRAY2BGR)
                scale = 200 / max(preview.shape[1], 1)
                preview = cv2.resize(preview, (int(preview.shape[1] * scale), int(preview.shape[0] * scale)))
                display[0:preview.shape[0], 0:preview.shape[1]] = preview
                cv2.putText(display, "detecting... (pick will run anyway)",
                            (10, preview.shape[0] + 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
            else:
                cx, cy, area, contour, mask, name = det
                recent.append((cx, cy))
                sx = sum(p[0] for p in recent) / len(recent)
                sy = sum(p[1] for p in recent) / len(recent)
                last_seen = name
                draw = DRAW_BY_COLOR[name]
                cv2.drawContours(display, [contour], -1, draw, 2)
                cv2.circle(display, (int(sx), int(sy)), 10, (0, 255, 0), -1)
                cv2.putText(display,
                            f"{name} | pick {TARGET_COLOR} ({PRESET_PICK[0]:.2f},{PRESET_PICK[1]:.2f})",
                            (int(sx) + 14, int(sy) - 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
                preview = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                scale = 200 / max(preview.shape[1], 1)
                preview = cv2.resize(preview, (int(preview.shape[1] * scale), int(preview.shape[0] * scale)))
                display[0:preview.shape[0], 0:preview.shape[1]] = preview

            left = max(0.0, seconds - (time.time() - t0))
            cv2.putText(display, f"starting pick in {left:.1f}s",
                        (10, display.shape[0] - 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            cv2.imshow("pick_flow vision", display)
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                print("vision: preview skipped")
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
    if last_seen:
        print(f"vision: last saw {last_seen} (UI only — motion uses preset pick)")
    else:
        print("vision: no color locked (UI only — motion uses preset pick anyway)")


# ---------------------------------------------------------------------------
# Arm control — same as original Summer School pick_flow.py
# ---------------------------------------------------------------------------
rclpy.init(); node = CobotIK(visualize=False)
real = {'a': None}
target = {'x': PRESET_PICK[0], 'y': PRESET_PICK[1]}

node.create_subscription(MycobotAngles, '/mycobot/angles_real',
    lambda m: real.__setitem__('a', np.array([m.joint_1, m.joint_2, m.joint_3, m.joint_4, m.joint_5, m.joint_6], float)), 10)
print(f"pick_flow ready | color={TARGET_COLOR} pick={PRESET_PICK} | ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}")

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
        err = (r.stderr or r.stdout or '').strip()
        print('grip: FAILED to copy grip_set.py:', err)
        if 'No route to host' in err or 'Connection timed out' in err or 'Connection refused' in err:
            print(f'  Pi not reachable at ROBOT_IP={RIP}.')
            print('  Fix:  export ROBOT_IP=<real-pi-ip>')
            print('  Find: ip neigh | grep 192.168.123   OR   ./test_robot.sh')
        return False
    _grip_ready['ok'] = True
    return True

def grip(v, sp):
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
        return False
    time.sleep(5)
    fresh(20)
    print(f'grip: OK {action} -> {v}')
    return True

def approach(x, y, z):
    # Try current-pose seed first so left/right offsets converge; fall back to zero seed.
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
    if best is None:
        print('approach: IK None'); return False
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
    """Show color detection in the UI, then pick at the color's hardcoded spot."""
    show_detection_preview(PREVIEW_SEC)
    target['x'], target['y'] = PRESET_PICK
    print(f"vision: {TARGET_COLOR} -> pick ({PRESET_PICK[0]:.3f}, {PRESET_PICK[1]:.3f})")
    return True

def approach_pick():
    return approach(float(target['x']), float(target['y']), GZ + APPR)

# Same step list as original Summer School pick_flow.py (+ brief vision UI)
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
