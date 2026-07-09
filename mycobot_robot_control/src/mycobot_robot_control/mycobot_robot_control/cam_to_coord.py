#!/usr/bin/env python3
"""Person 2+3: detect colored block, apply homography, publish table (x, y) for pick_flow.

Publishes geometry_msgs/PointStamped on /block_position — the topic
pick_flow.py waits for when run without PX/PY args.

Pipeline:
  webcam -> HSV color mask -> largest blob centroid (cx, cy)
        -> homography_transform.pixel_to_meters -> (x, y) meters
        -> /block_position -> pick_flow.py

Usage:
  python cam_to_coord.py                 # auto: red/green/blue (largest blob)
  python cam_to_coord.py --color blue    # force blue block
  python cam_to_coord.py --camera 1
  python cam_to_coord.py --once

Env:
  ROS_DOMAIN_ID should match pick_flow (usually 10).
"""
import argparse
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node

# Same-folder import when run as a script from Downloads / source tree
sys.path.insert(0, str(Path(__file__).resolve().parent))
from homography_transform import pixel_to_meters

# Match Person 3 calibration image size (px points go up to ~1174 x 647)
CAM_WIDTH = 1280
CAM_HEIGHT = 720

# Wide HSV ranges for plastic blocks under lab lighting (OpenCV H: 0-179).
# Blue plastic often reads as cyan/teal — keep a broad band.
COLOR_HSV = {
    "red": [
        (np.array([0, 50, 40]), np.array([12, 255, 255])),
        (np.array([160, 50, 40]), np.array([179, 255, 255])),
    ],
    "green": [
        (np.array([35, 40, 40]), np.array([90, 255, 255])),
    ],
    "blue": [
        # covers deep blue through cyan (common for "blue" blocks on webcams)
        (np.array([85, 40, 40]), np.array([140, 255, 255])),
    ],
}

MIN_AREA_PX = 200          # smaller blocks / farther from camera
SMOOTH_N = 3               # light smoothing so pointer still tracks motion


def color_mask(frame_bgr, color):
    """Return binary mask for the chosen block color."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    # mild blur helps solid plastic under noisy webcam
    hsv = cv2.GaussianBlur(hsv, (5, 5), 0)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in COLOR_HSV[color]:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask


def detect_one_color(frame, color, min_area):
    """Detect one color; return (cx, cy, area, contour, mask, color) or None."""
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
    return cx, cy, area, contour, mask, color


def detect_block(frame, color="auto", min_area=MIN_AREA_PX):
    """Detect block. color='auto' tries red/green/blue and keeps largest blob."""
    if color == "auto":
        best = None
        for name in ("blue", "red", "green"):
            hit = detect_one_color(frame, name, min_area)
            if hit is None:
                continue
            if best is None or hit[2] > best[2]:
                best = hit
        return best
    return detect_one_color(frame, color, min_area)


def pixel_to_table(cx, cy):
    """Pixel centroid -> table meters (x forward, y lateral) via Person 3 homography."""
    return pixel_to_meters(cx, cy)


class BlockPublisher(Node):
    def __init__(self):
        super().__init__("cam_to_coord")
        self.pub = self.create_publisher(PointStamped, "/block_position", 10)
        self.get_logger().info("publishing detections on /block_position (HSV + homography)")

    def publish_xy(self, x, y):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "table"
        msg.point.x = x
        msg.point.y = y
        msg.point.z = 0.0
        self.pub.publish(msg)
        self.get_logger().info(f"block PX={x:.3f} PY={y:.3f}")


def main():
    parser = argparse.ArgumentParser(description="Detect colored block and publish /block_position")
    parser.add_argument("--camera", type=int, default=0, help="webcam index")
    parser.add_argument(
        "--color",
        choices=["auto", "red", "green", "blue"],
        default="auto",
        help="block color (default: auto = largest of red/green/blue)",
    )
    parser.add_argument("--min-area", type=int, default=MIN_AREA_PX, help="min contour area px")
    parser.add_argument("--once", action="store_true", help="publish one detection then exit")
    parser.add_argument("--rate", type=float, default=10.0, help="loop rate Hz")
    parser.add_argument("--width", type=int, default=CAM_WIDTH, help="capture width (match calibration)")
    parser.add_argument("--height", type=int, default=CAM_HEIGHT, help="capture height")
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"Failed to open camera {args.camera}")
        raise SystemExit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    # Prefer auto exposure off if supported — more stable HSV
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera {args.camera}: {actual_w}x{actual_h}, detecting color={args.color}")
    if abs(actual_w - args.width) > 40 or abs(actual_h - args.height) > 40:
        print(
            f"WARNING: requested {args.width}x{args.height} but got {actual_w}x{actual_h}; "
            f"homography may be off — recalibrate or pass --width/--height"
        )

    print("Place the block in view. Press q to quit. Mask preview is top-left (white = detected).")

    rclpy.init()
    node = BlockPublisher()
    period = 1.0 / max(args.rate, 0.1)
    published = False
    recent = deque(maxlen=SMOOTH_N)

    try:
        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                print("Failed to capture frame")
                break

            result = detect_block(frame, color=args.color, min_area=args.min_area)
            if result is None:
                # keep last pointer briefly so it doesn't vanish on one bad frame
                node.get_logger().info(
                    f"no {args.color} block detected — check mask preview / lighting",
                    throttle_duration_sec=2.0,
                )
                display = frame.copy()
                # still show blue-only mask so user can tune lighting
                dbg = color_mask(frame, "blue")
                preview = cv2.cvtColor(dbg, cv2.COLOR_GRAY2BGR)
                ph, pw = preview.shape[:2]
                scale = 200 / max(pw, 1)
                preview = cv2.resize(preview, (int(pw * scale), int(ph * scale)))
                display[0:preview.shape[0], 0:preview.shape[1]] = preview
                cv2.putText(
                    display, "no block (showing blue mask)", (10, preview.shape[0] + 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2,
                )
            else:
                cx, cy, area, contour, mask, found = result
                recent.append((cx, cy))
                sx = sum(p[0] for p in recent) / len(recent)
                sy = sum(p[1] for p in recent) / len(recent)
                x, y = pixel_to_table(sx, sy)
                node.publish_xy(x, y)
                published = True

                display = frame.copy()
                cv2.drawContours(display, [contour], -1, (0, 255, 0), 2)
                cv2.circle(display, (int(sx), int(sy)), 10, (0, 255, 0), -1)
                cv2.circle(display, (int(sx), int(sy)), 14, (0, 255, 0), 2)
                cv2.putText(
                    display,
                    f"{found} ({x:.3f}, {y:.3f}) m  area={int(area)}",
                    (int(sx) + 14, int(sy) - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
                )
                preview = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                ph, pw = preview.shape[:2]
                scale = 200 / max(pw, 1)
                preview = cv2.resize(preview, (int(pw * scale), int(ph * scale)))
                display[0:preview.shape[0], 0:preview.shape[1]] = preview

            cv2.imshow("cam_to_coord", display)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

            rclpy.spin_once(node, timeout_sec=0.0)
            if args.once and published and len(recent) >= min(2, SMOOTH_N):
                break
            time.sleep(period)
    finally:
        cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
