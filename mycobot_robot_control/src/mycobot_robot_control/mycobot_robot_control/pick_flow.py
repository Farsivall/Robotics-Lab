#!/usr/bin/env python3
"""One-shot pick + base-rotate + place orchestrator.

Usage:
  python pick_flow.py 0.18 -0.06          # coords as args (no ROS vision wait)
  python pick_flow.py --vision             # wait for /block_position then pick
  python pick_flow.py                      # same as --vision

  # from another script:
  #   from pick_flow import run_pick
  #   run_pick(0.18, -0.06)

Env: ROBOT_IP overrides target robot (default 192.168.123.50)
"""
import argparse
import os
import shutil
import subprocess
import sys
import time

# RoboEnv IK dependency (needed even if shell forgot to export PYTHONPATH)
_ROBO = os.path.expanduser("~/RoboEnv/simulation_and_control")
if os.path.isdir(_ROBO) and _ROBO not in sys.path:
    sys.path.insert(0, _ROBO)
_env_pp = os.environ.get("PYTHONPATH", "")
if _ROBO not in _env_pp.split(os.pathsep):
    os.environ["PYTHONPATH"] = _ROBO + (os.pathsep + _env_pp if _env_pp else "")

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from mycobot_client_2.ik import CobotIK
from mycobot_msgs_2.msg import MycobotAngles, MycobotSetAngles
from rclpy.qos import QoSProfile, ReliabilityPolicy

DEG = np.pi / 180.0
RAD = 180.0 / np.pi
LIMS = np.array([165, 165, 165, 165, 165, 179.0])
DOWN = np.array([180.0, 0.0, 0.0])
LIFT = 0.10
APPR = 0.04
STEP = 0.01
BLOCK_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
# Hardcoded claw values (hybrid_pick_place.sh style)
GRIP_OPEN = 100
GRIP_CLOSE = 28


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="Pick-and-place; pass PX PY or --vision")
    p.add_argument("px", nargs="?", type=float, default=None, help="pick X meters")
    p.add_argument("py", nargs="?", type=float, default=None, help="pick Y meters")
    p.add_argument("rot", nargs="?", type=float, default=90.0, help="base rotate deg")
    p.add_argument("grasp_z", nargs="?", type=float, default=0.02)
    p.add_argument("place_z", nargs="?", type=float, default=0.06)
    p.add_argument("grip_val", nargs="?", type=int, default=None,
                   help="gripper close value (100=open, lower=tighter; min safe 25)")
    p.add_argument("speed", nargs="?", type=int, default=25)
    p.add_argument(
        "--vision",
        action="store_true",
        help="wait for /block_position instead of using PX PY args",
    )
    p.add_argument("--grip-val", type=int, default=None, dest="grip_val_opt",
                   help="override gripper close (e.g. 28). 100=open, lower=tighter. Min 25.")
    p.add_argument("--grasp-z", type=float, default=None, dest="grasp_z_opt",
                   help="override grasp height meters (default 0.02)")
    return p.parse_args(argv)


def run_pick(px, py, rot=90.0, grasp_z=0.02, place_z=0.06, grip_val=None, speed=25,
             use_vision=False, robot_ip=None):
    """Run one full pick-place cycle. Pass px/py OR use_vision=True.

    Grip is hardcoded: open=100, close=28 (grip_val ignored).
    """
    print(f"grip hardcoded: open={GRIP_OPEN} close={GRIP_CLOSE}")

    rip = robot_ip or os.environ.get("ROBOT_IP", "192.168.123.50")
    _mf = os.path.expanduser("~/miniforge3/bin")
    if os.path.isdir(_mf):
        os.environ["PATH"] = _mf + os.pathsep + os.environ.get("PATH", "")
    sshpass = shutil.which("sshpass") or "/usr/bin/sshpass"
    ssh = [
        sshpass, "-p", "Elephant", "ssh",
        "-o", "StrictHostKeyChecking=no",
        "-o", "PreferredAuthentications=password",
        "-o", "PubkeyAuthentication=no",
        "er@" + rip,
    ]

    if not rclpy.ok():
        rclpy.init()
    node = CobotIK(visualize=False)
    real = {"a": None}
    target = {"x": None, "y": None, "locked": False}
    vision_got = {"n": 0}

    def block_callback(msg):
        if target.get("locked"):
            return
        target["x"] = msg.point.x
        target["y"] = msg.point.y
        vision_got["n"] += 1
        if vision_got["n"] == 1:
            print(f"  got /block_position x={msg.point.x:.3f} y={msg.point.y:.3f}", flush=True)

    node.create_subscription(
        MycobotAngles,
        "/mycobot/angles_real",
        lambda m: real.__setitem__(
            "a",
            np.array([m.joint_1, m.joint_2, m.joint_3, m.joint_4, m.joint_5, m.joint_6], float),
        ),
        10,
    )
    node.create_subscription(PointStamped, "/block_position", block_callback, BLOCK_QOS)
    print(
        f"pick_flow ready | vision={use_vision} | "
        f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')} | ROBOT_IP={rip} | sshpass={sshpass}"
    )

    def fresh(t=4.0):
        real["a"] = None
        t0 = time.time()
        while time.time() - t0 < t:
            rclpy.spin_once(node, timeout_sec=0.1)
            if real["a"] is not None:
                return real["a"].copy()
        return None

    def cmd(deg6, spd):
        m = MycobotSetAngles()
        m.joint_1, m.joint_2, m.joint_3, m.joint_4, m.joint_5, m.joint_6 = [
            float(v) for v in deg6
        ]
        m.speed = spd
        node.cmd_angle_pub.publish(m)

    def goto(deg6, spd, tol=3.5, timeout=14.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            cmd(deg6, spd)
            b = fresh(1.5)
            if b is not None and np.max(np.abs(b - np.asarray(deg6))) < tol:
                return True
        return False

    def home(spd=30):
        b = fresh(12)
        if b is None:
            print("home: NO_ANGLES")
            return False
        if np.max(np.abs(b)) < 4:
            print("home: already there")
            return True
        ok = goto(np.zeros(6), spd, 4.0, 25.0)
        print("home:", "OK" if ok else "TIMEOUT")
        return ok

    # Hardcoded open/close — same remote command as hybrid_pick_place.sh
    def grip(v, sp):
        print(f"grip: set {v} speed {sp}")
        sh = (
            f"docker stop -t 2 mycobot_comms>/dev/null 2>&1; "
            f"python3 /home/er/grip_set.py {v} {sp}; "
            f"docker start mycobot_comms>/dev/null 2>&1"
        )
        try:
            r = subprocess.run(ssh + [sh], capture_output=True, text=True, timeout=90)
            if r.stdout:
                print(r.stdout.strip())
            if r.stderr:
                print(r.stderr.strip())
        except Exception as e:
            print(f"grip: SSH error: {e}")
            return False
        time.sleep(5)
        fresh(20)
        return True

    def wait_for_vision(timeout=60.0):
        print("waiting for /block_position from vision...")
        t0 = time.time()
        last_hb = t0
        while target["x"] is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.time()
            if now - last_hb >= 3.0:
                print(f"  still waiting... {now - t0:.0f}s", flush=True)
                last_hb = now
            if now - t0 > timeout:
                print("vision: TIMEOUT")
                return None, None
        settle_end = time.time() + 0.5
        while time.time() < settle_end:
            rclpy.spin_once(node, timeout_sec=0.05)
        return float(target["x"]), float(target["y"])

    def approach(x, y, z):
        print(f"approach: target x={x:.3f} y={y:.3f} z={z:.3f}")
        ik = node.calculate_ik(
            np.array([x, y, z]), DOWN, "gripper", 1e-5, 0.3, 0.02, False, 4000, False
        )
        if ik is None:
            print("approach: IK None")
            return False
        adj = np.array(node.adjust_angles(np.array(ik)), float)
        pos, _eul = node.get_pose(adj * DEG, "gripper")
        err = float(np.linalg.norm(pos - [x, y, z]))
        if err > 0.02 or not np.all(np.abs(adj) <= LIMS):
            print(f"approach: REFUSED (err {err*1000:.0f}mm) — coords may be out of reach")
            return False
        ok = goto(adj, speed)
        print(f'approach: {"OK" if ok else "TIMEOUT"} err={err*1000:.1f}mm')
        return ok

    def move_z(tz, spd):
        b = fresh(6)
        if b is None:
            print("move_z: NO_ANGLES")
            return False
        p, _ = node.get_pose(b * DEG, "gripper")
        X, Y, Z0 = float(p[0]), float(p[1]), float(p[2])
        seed = b * DEG
        step = STEP if tz > Z0 else -STEP
        for z in np.arange(Z0 + step, tz + step * 0.5, step):
            node.real_angles = seed
            ik = node.calculate_ik(
                np.array([X, Y, z]), DOWN, "gripper", 1e-5, 0.3, 0.02, True, 4000, False
            )
            if ik is None:
                print(f"move_z: IK None at {z:.3f}")
                return False
            sol = np.array(ik)
            sd = (sol * RAD + 180) % 360 - 180
            if not np.all(np.abs(sd) <= LIMS):
                print(f"move_z: limits at {z:.3f}")
                return False
            pos, eul = node.get_pose(sol, "gripper")
            if np.linalg.norm(pos - [X, Y, z]) > 0.01 or abs(abs(eul[0]) - 180) > 20:
                print(f"move_z: unsafe at {z:.3f}")
                return False
            t0 = time.time()
            while time.time() - t0 < 5:
                cmd(sd, spd)
                bb = fresh(1)
                if bb is not None:
                    pp, _ = node.get_pose(bb * DEG, "gripper")
                    if abs(pp[2] - z) < 0.012:
                        break
            seed = sol
        bb = fresh(2)
        if bb is not None:
            pf, _ = node.get_pose(bb * DEG, "gripper")
            print(f"move_z: at z={pf[2]:.3f}")
        return True

    def rotate(delta):
        cur = fresh(6)
        if cur is None:
            print("rotate: NO_ANGLES")
            return False
        if abs(cur[0] + delta) > 165:
            print(f"rotate: newJ1 {cur[0]+delta:.0f} limit")
            return False
        n = max(2, int(np.ceil(abs(delta) / 30)))
        for i in range(1, n + 1):
            j1 = cur[0] + delta * (i / n)
            t0 = time.time()
            while time.time() - t0 < 6:
                cmd([j1, cur[1], cur[2], cur[3], cur[4], cur[5]], speed)
                b = fresh(1)
                if b is not None and abs(b[0] - j1) < 3:
                    break
        print("rotate: OK")
        return True

    def vision_step():
        if not use_vision:
            print(f"vision: using args PX={px:.3f} PY={py:.3f}")
            target["x"], target["y"] = float(px), float(py)
            target["locked"] = True
            return True
        vx, vy = wait_for_vision()
        if vx is None:
            return False
        target["x"], target["y"] = vx, vy
        target["locked"] = True
        print(f"vision: LOCKED PX={vx:.3f} PY={vy:.3f} — starting arm motion")
        return True

    def approach_pick():
        return approach(float(target["x"]), float(target["y"]), grasp_z + APPR)

    # Hardcoded grip: open=100, close=28 (same as hybrid_pick_place.sh)
    steps = [
        ("vision", vision_step),
        ("home", lambda: home()),
        ("open", lambda: grip(GRIP_OPEN, 40)),
        ("approach", approach_pick),
        ("descend", lambda: move_z(grasp_z, 15)),
        ("grip", lambda: grip(GRIP_CLOSE, 15)),
        ("lift", lambda: move_z(LIFT, speed)),
        ("rotate", lambda: rotate(rot)),
        ("place-descend", lambda: move_z(place_z, speed)),
        ("release", lambda: grip(GRIP_OPEN, 40)),
        ("retreat", lambda: move_z(0.12, speed)),
        ("home-end", lambda: home()),
    ]

    t_start = time.time()
    try:
        for name, fn in steps:
            print(f"== {name} ==")
            if not fn():
                print(f"!! FAILED at {name} -- ABORT, arm holds position")
                raise SystemExit(1)
        print(f"== DONE in {time.time()-t_start:.0f}s ==")
        return True
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None):
    args = parse_args(argv)
    if args.px is not None and args.py is not None and not args.vision:
        use_vision = False
        px, py = args.px, args.py
    else:
        use_vision = True
        px = args.px if args.px is not None else 0.18
        py = args.py if args.py is not None else 0.0

    gz = args.grasp_z_opt if args.grasp_z_opt is not None else args.grasp_z

    run_pick(
        px,
        py,
        rot=args.rot,
        grasp_z=gz,
        place_z=args.place_z,
        speed=args.speed,
        use_vision=use_vision,
    )


if __name__ == "__main__":
    main()
