#!/usr/bin/env python3
"""Person 2+3: detect blue block, apply homography, publish for pick_flow.

Publishes geometry_msgs/PointStamped on /block_position with latched QoS so
pick_flow can join late and still get the last detection.

Blue HSV is tuned from measured midpoint RGB=(33, 52, 100).

Usage:
  python cam_to_coord.py              # default: blue
  python cam_to_coord.py --once
  python cam_to_coord.py --camera 1

Env: ROS_DOMAIN_ID must match pick_flow (usually 10).
"""
import argparse
import os
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

sys.path.insert(0, str(Path(__file__).resolve().parent))
from homography_transform import pixel_to_meters

CAM_WIDTH = 1280
CAM_HEIGHT = 720

# Blue HSV from measured midpoint RGB=(33, 52, 100) — friend's calibration
MID_RGB = (33, 52, 100)
_mid_bgr = np.uint8([[[MID_RGB[2], MID_RGB[1], MID_RGB[0]]]])
_mid_hsv = cv2.cvtColor(_mid_bgr, cv2.COLOR_BGR2HSV)[0, 0]
H, S, V = int(_mid_hsv[0]), int(_mid_hsv[1]), int(_mid_hsv[2])

H_TOL = 12
S_TOL = 70
V_TOL = 70

lo = np.array([max(0, H - H_TOL), max(40, S - S_TOL), max(30, V - V_TOL)])
hi = np.array([min(179, H + H_TOL), 255, min(255, V + V_TOL)])
COLOR_HSV = {
    "blue": [(lo, hi)],
}
DRAW_COLOR = (int(_mid_bgr[0, 0, 0]), int(_mid_bgr[0, 0, 1]), int(_mid_bgr[0, 0, 2]))

MIN_AREA_PX = 200
SMOOTH_N = 3

# Late-joining pick_flow still receives last pose (fixes hang)
BLOCK_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


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
    cx = M["m10"] / M["m00"]
    cy = M["m01"] / M["m00"]
    return cx, cy, area, contour, mask


def pixel_to_table(cx, cy):
    return pixel_to_meters(cx, cy)


class BlockPublisher(Node):
    def __init__(self):
        super().__init__("cam_to_coord")
        self.pub = self.create_publisher(PointStamped, "/block_position", BLOCK_QOS)
        domain = os.environ.get("ROS_DOMAIN_ID", "0")
        self.get_logger().info(
            f"publishing /block_position (latched) | ROS_DOMAIN_ID={domain}"
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
    parser = argparse.ArgumentParser(description="Detect blue block and publish /block_position")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--color", choices=sorted(COLOR_HSV.keys()), default="blue")
    parser.add_argument("--min-area", type=int, default=MIN_AREA_PX)
    parser.add_argument("--once", action="store_true")
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
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(
        f"Camera {args.camera}: {actual_w}x{actual_h}, color={args.color}, "
        f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}, "
        f"HSV blue lo={lo.tolist()} hi={hi.tolist()}"
    )
    if abs(actual_w - args.width) > 40 or abs(actual_h - args.height) > 40:
        print(
            f"WARNING: requested {args.width}x{args.height} but got {actual_w}x{actual_h}; "
            f"homography may be off"
        )
    print("Place the blue block in view. Expect 'PUB /block_position' lines. Press q to quit.")

    rclpy.init()
    node = BlockPublisher()
    time.sleep(1.0)  # DDS advertise before first publish
    period = 1.0 / max(args.rate, 0.1)
    published = False
    recent = deque(maxlen=SMOOTH_N)
    last_pub = 0.0

    try:
        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                print("Failed to capture frame")
                break

            result = detect_block(frame, color=args.color, min_area=args.min_area)
            if result is None:
                display = frame.copy()
                dbg = color_mask(frame, "blue")
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
                cx, cy, area, contour, mask = result
                recent.append((cx, cy))
                sx = sum(p[0] for p in recent) / len(recent)
                sy = sum(p[1] for p in recent) / len(recent)
                x, y = pixel_to_table(sx, sy)

                now = time.time()
                if now - last_pub >= 0.2:
                    node.publish_xy(x, y)
                    last_pub = now
                    published = True

                display = frame.copy()
                cv2.drawContours(display, [contour], -1, DRAW_COLOR, 2)
                cv2.circle(display, (int(sx), int(sy)), 10, (0, 255, 0), -1)
                cv2.circle(display, (int(sx), int(sy)), 14, (0, 255, 0), 2)
                cv2.putText(
                    display,
                    f"blue ({x:.3f}, {y:.3f}) m  area={int(area)}",
                    (int(sx) + 14, int(sy) - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
                )
                preview = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                scale = 200 / max(preview.shape[1], 1)
                preview = cv2.resize(
                    preview, (int(preview.shape[1] * scale), int(preview.shape[0] * scale))
                )
                display[0:preview.shape[0], 0:preview.shape[1]] = preview

            cv2.imshow("cam_to_coord", display)
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                break

            rclpy.spin_once(node, timeout_sec=0.0)
            if args.once and published and len(recent) >= 2:
                break
            time.sleep(period)
    finally:
        cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
