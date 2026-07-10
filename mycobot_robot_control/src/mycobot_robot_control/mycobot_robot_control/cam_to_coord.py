#!/usr/bin/env python3
"""Detect blue block, then either publish ROS topic OR launch pick_flow with args.

Recommended (no hang): detect → call pick_flow.py PX PY as subprocess.

  python cam_to_coord.py --pick          # detect then move arm (best)
  python cam_to_coord.py                 # publish /block_position only
  python cam_to_coord.py --stream        # keep publishing (debug)
  python cam_to_coord.py --camera 1

Env: ROS_DOMAIN_ID=10 when using topic mode; --pick does not need a second terminal.
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

MID_RGB = (33, 52, 100)
_mid_bgr = np.uint8([[[MID_RGB[2], MID_RGB[1], MID_RGB[0]]]])
_mid_hsv = cv2.cvtColor(_mid_bgr, cv2.COLOR_BGR2HSV)[0, 0]
H, S, V = int(_mid_hsv[0]), int(_mid_hsv[1]), int(_mid_hsv[2])
H_TOL, S_TOL, V_TOL = 12, 70, 70
lo = np.array([max(0, H - H_TOL), max(40, S - S_TOL), max(30, V - V_TOL)])
hi = np.array([min(179, H + H_TOL), 255, min(255, V + V_TOL)])
COLOR_HSV = {"blue": [(lo, hi)]}
DRAW_COLOR = (int(_mid_bgr[0, 0, 0]), int(_mid_bgr[0, 0, 1]), int(_mid_bgr[0, 0, 2]))

MIN_AREA_PX = 200
SMOOTH_N = 5
HANDOFF_BURSTS = 8
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


def detect_block(frame, color="blue", min_area=MIN_AREA_PX):
    mask = color_mask(frame, color=color)
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
    return M["m10"] / M["m00"], M["m01"] / M["m00"], area, contour, mask


def run_pick_flow(px, py, rot=90.0, dry_run=False):
    """Call pick_flow.py with PX PY args — no ROS topic wait."""
    cmd = [
        sys.executable, PICK_FLOW,
        f"{px:.4f}", f"{py:.4f}", f"{rot:.1f}",
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
            f"ready | ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}"
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
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--color", choices=["blue"], default="blue")
    parser.add_argument("--min-area", type=int, default=MIN_AREA_PX)
    parser.add_argument("--stream", action="store_true",
                        help="keep publishing forever (topic debug)")
    parser.add_argument("--pick", action="store_true",
                        help="after lock, run pick_flow.py PX PY (recommended)")
    parser.add_argument("--dry-run", action="store_true",
                        help="with --pick, only print the pick_flow command")
    parser.add_argument("--rot", type=float, default=90.0, help="ROT passed to pick_flow")
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--width", type=int, default=CAM_WIDTH)
    parser.add_argument("--height", type=int, default=CAM_HEIGHT)
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"Failed to open camera {args.camera}")
        raise SystemExit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    mode = "PICK (args)" if args.pick else ("STREAM" if args.stream else "TOPIC handoff")
    print(
        f"Camera {args.camera} | color=blue | mode={mode} | "
        f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}"
    )
    if args.pick:
        print("Will lock blue block then: python pick_flow.py PX PY ...")
    else:
        print("Publishing /block_position — start pick_flow --vision in other terminal.")
    print("Press q to quit.")

    need_ros = not args.pick or args.stream
    node = None
    if need_ros or not args.pick:
        rclpy.init()
        node = BlockPublisher()
        time.sleep(1.0)

    period = 1.0 / max(args.rate, 0.1)
    recent = deque(maxlen=SMOOTH_N)
    bursts_left = HANDOFF_BURSTS
    handed_off = False
    last_xy = None
    last_pub_slow = 0.0
    pick_started = False

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            result = detect_block(frame, color=args.color, min_area=args.min_area)
            display = frame.copy()
            status = "no blue block"

            if result is None:
                recent.clear()
                if handed_off and last_xy is not None and node is not None and not args.pick:
                    if time.time() - last_pub_slow >= 1.0:
                        node.publish_xy(last_xy[0], last_xy[1])
                        last_pub_slow = time.time()
                    status = "HANDOFF DONE — re-PUB 1Hz"
            else:
                cx, cy, area, contour, mask = result
                recent.append((cx, cy))
                sx = sum(p[0] for p in recent) / len(recent)
                sy = sum(p[1] for p in recent) / len(recent)
                x, y = pixel_to_meters(sx, sy)
                last_xy = (x, y)

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
                    handed_off = True
                    print(f"LOCKED x={x:.3f} y={y:.3f} — launching pick_flow with args", flush=True)
                    cv2.putText(
                        display, "LAUNCHING PICK...", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
                    )
                    cv2.imshow("cam_to_coord", display)
                    cv2.waitKey(1)
                    # release camera before pick (USB cam often exclusive)
                    cap.release()
                    cv2.destroyAllWindows()
                    if node is not None:
                        node.destroy_node()
                        rclpy.shutdown()
                        node = None
                    ok = run_pick_flow(x, y, rot=args.rot, dry_run=args.dry_run)
                    print("pick_flow:", "OK" if ok else "FAILED")
                    raise SystemExit(0 if ok else 1)

                if args.stream and node is not None:
                    node.publish_xy(x, y)
                    status = "STREAM publishing"
                elif not args.pick and not handed_off and stable and node is not None:
                    node.publish_xy(x, y)
                    bursts_left -= 1
                    status = f"handoff {HANDOFF_BURSTS - bursts_left}/{HANDOFF_BURSTS}"
                    if bursts_left <= 0:
                        handed_off = True
                        last_pub_slow = time.time()
                        print(f"HANDOFF DONE x={x:.3f} y={y:.3f}", flush=True)
                elif handed_off and not args.pick and node is not None:
                    if time.time() - last_pub_slow >= 1.0:
                        node.publish_xy(last_xy[0], last_xy[1])
                        last_pub_slow = time.time()
                    status = "HANDOFF DONE — re-PUB 1Hz"
                else:
                    status = f"locking... {len(recent)}/{SMOOTH_N}"

            if last_xy and handed_off:
                cv2.putText(
                    display, "LOCKED", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
                )
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
