#!/usr/bin/env python3
"""Detect blue block and publish /block_position for pick_flow (two terminals).

Easy two-terminal flow (no --pick needed):
  Terminal 1:  ./vision.sh          # or: python cam_to_coord.py
  Terminal 2:  ./pick_vision.sh     # or: python pick_flow.py --vision

Optional:
  python cam_to_coord.py --camera 1
  python cam_to_coord.py --pick     # one-terminal: detect then launch pick_flow

Env: ROS_DOMAIN_ID should be 10 on both terminals.
"""
import argparse
import os
import subprocess
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

sys.path.insert(0, str(Path(__file__).resolve().parent))
from homography_transform import pixel_to_meters

CAM_WIDTH = 1280
CAM_HEIGHT = 720
PICK_FLOW = str(Path(__file__).resolve().parent / "pick_flow.py")

def convert_to_hsv(Rvalue, Gvalue, Bvalue):
    _mid_bgr = np.uint8([[[Bvalue, Gvalue, Rvalue]]])
    _mid_hsv = cv2.cvtColor(_mid_bgr, cv2.COLOR_BGR2HSV)[0, 0]
    H, S, V = int(_mid_hsv[0]), int(_mid_hsv[1]), int(_mid_hsv[2])

    H_TOL = 12
    S_TOL = 70
    V_TOL = 70

    lo = np.array([max(0, H - H_TOL), max(40, S - S_TOL), max(30, V - V_TOL)])
    hi = np.array([min(179, H + H_TOL), 255, min(255, V + V_TOL)])
    draw_color = (int(_mid_bgr[0, 0, 0]), int(_mid_bgr[0, 0, 1]), int(_mid_bgr[0, 0, 2]))
    return lo, hi, draw_color


blue_rgb = (33, 52, 100)
purple_rgb = (67, 41, 65)
yellow_rgb = (249, 222, 0)

COLOR_HSV = {}
DRAW_COLOR = {}
for name, rgb in [("blue", blue_rgb), ("purple", purple_rgb), ("yellow", yellow_rgb)]:
    lo, hi, draw = convert_to_hsv(*rgb)
    COLOR_HSV[name] = [(lo, hi)]
    DRAW_COLOR[name] = draw

MIN_AREA_PX = 200
SMOOTH_N = 3
BLOCK_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)


def color_mask(frame_bgr, color="blue"):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    hsv = cv2.GaussianBlur(hsv, (5, 5), 0)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo_b, hi_b in COLOR_HSV[color]:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo_b, hi_b))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask

def detect_block(frame, color="auto", min_area=MIN_AREA_PX):
    colors_to_check = COLOR_HSV.keys() if color == "auto" else [color]

    best = None
    for name in colors_to_check:
        mask = color_mask(frame, color=name)
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


def run_pick_flow(px, py, rot=90.0, grip_val=28, dry_run=False):
    cmd = [
        sys.executable, PICK_FLOW,
        f"{px:.4f}", f"{py:.4f}", f"{rot:.1f}",
        "0.02", "0.06", str(int(grip_val)), "25",
    ]
    print("->", " ".join(cmd), flush=True)
    if dry_run:
        return True
    return subprocess.run(cmd).returncode == 0


class BlockPublisher(Node):
    def __init__(self):
        super().__init__("cam_to_coord")
        self.pub = self.create_publisher(PointStamped, "/block_position", BLOCK_QOS)
        self.get_logger().info(
            f"publishing /block_position | ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}"
        )

    def publish_xy(self, x, y):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "table"
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = 0.0
        self.pub.publish(msg)
        print(f"PUB /block_position  x={x:.3f} y={y:.3f}", flush=True)


def main():
    parser = argparse.ArgumentParser(
        description="Blue block vision. Default: publish /block_position (use 2 terminals)."
    )
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--color", choices=["auto"] + sorted(COLOR_HSV.keys()), default="auto")
    parser.add_argument("--min-area", type=int, default=MIN_AREA_PX)
    parser.add_argument(
        "--pick",
        action="store_true",
        help="optional one-terminal mode: after lock, run pick_flow.py PX PY",
    )
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--rot", type=float, default=90.0)
    parser.add_argument("--grip-val", type=int, default=28,
                        help="gripper close value for --pick (100=open, lower=tighter, min 25)")
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--width", type=int, default=CAM_WIDTH)
    parser.add_argument("--height", type=int, default=CAM_HEIGHT)
    # accept old flag names so older habit / docs don't error
    parser.add_argument("--stream", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--once", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"Failed to open camera {args.camera}")
        raise SystemExit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    print(
        f"Camera {args.camera} | blue | "
        f"mode={'PICK' if args.pick else 'TWO-TERMINAL publish'} | "
        f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}"
    )
    if args.pick:
        print("Will lock then launch pick_flow with PX PY args.")
    else:
        print("Publishing /block_position continuously.")
        print("In OTHER terminal run:  ./pick_vision.sh   OR   python pick_flow.py --vision")
    print("Press q to quit.")

    node = None
    if not args.pick:
        rclpy.init()
        node = BlockPublisher()
        time.sleep(1.0)

    period = 1.0 / max(args.rate, 0.1)
    recent = deque(maxlen=SMOOTH_N)
    pick_started = False

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            result = detect_block(frame, color=args.color, min_area=args.min_area)
            if result is None:
                display = frame.copy()
                debug_color = args.color if args.color != "auto" else "blue"
                dbg = color_mask(frame, debug_color)
                preview = cv2.cvtColor(dbg, cv2.COLOR_GRAY2BGR)
                scale = 200 / max(preview.shape[1], 1)
                preview = cv2.resize(
                    preview, (int(preview.shape[1] * scale), int(preview.shape[0] * scale))
                )
                display[0:preview.shape[0], 0:preview.shape[1]] = preview
                cv2.putText(
                    display, "no blue block — check mask (top-left)",
                    (10, preview.shape[0] + 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2,
                )
            else:
                cx, cy, area, contour, mask , found_colour = result
                recent.append((cx, cy))
                sx = sum(p[0] for p in recent) / len(recent)
                sy = sum(p[1] for p in recent) / len(recent)
                x, y = pixel_to_meters(sx, sy)

                cv2.drawContours(display, [contour], -1, DRAW_COLOR, 2)
                cv2.circle(display, (int(sx), int(sy)), 10, (0, 255, 0), -1)
                cv2.putText(
                    display, f"blue ({x:.3f},{y:.3f})m",
                    (int(sx) + 12, int(sy) - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
                )
                preview = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                scale = 200 / max(preview.shape[1], 1)
                preview = cv2.resize(
                    preview, (int(preview.shape[1] * scale), int(preview.shape[0] * scale))
                )
                display[0:preview.shape[0], 0:preview.shape[1]] = preview

                stable = len(recent) >= SMOOTH_N
                if args.pick and stable and not pick_started:
                    pick_started = True
                    print(f"LOCKED x={x:.3f} y={y:.3f} — launching pick_flow", flush=True)
                    cap.release()
                    cv2.destroyAllWindows()
                    ok = run_pick_flow(x, y, rot=args.rot, grip_val=args.grip_val, dry_run=args.dry_run)
                    print("pick_flow:", "OK" if ok else "FAILED")
                    raise SystemExit(0 if ok else 1)

                if not args.pick and stable and node is not None:
                    node.publish_xy(x, y)
                    status = "PUBLISHING"
                else:
                    status = f"locking... {len(recent)}/{SMOOTH_N}"
            if result is None:
                recent.clear()

            cv2.putText(
                display, status, (10, display.shape[0] - 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2,
            )
            cv2.imshow("cam_to_coord", display)
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                break
            if node is not None:
                rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)
    finally:
        if cap.isOpened():
            cap.release()
        cv2.destroyAllWindows()
        if node is not None:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
