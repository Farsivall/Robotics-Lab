#!/usr/bin/env python3
"""Person 2+3: detect red block, apply homography, publish table (x, y) for pick_flow.

Publishes geometry_msgs/PointStamped on /block_position — the topic
pick_flow.py waits for when run without PX/PY args.

Pipeline:
  webcam -> HSV red mask -> largest blob centroid (cx, cy)
        -> homography_transform.pixel_to_meters -> (x, y) meters
        -> /block_position -> pick_flow.py

Usage:
  python cam_to_coord.py              # live detect + publish
  python cam_to_coord.py --once       # one detection then exit
  python cam_to_coord.py --camera 1   # non-default webcam index
  python cam_to_coord.py --color red  # red (default) | green | blue

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

# HSV ranges (OpenCV H: 0-179). Red wraps around 0.
COLOR_HSV = {
    '''
    "red": [
        (np.array([0, 80, 60]), np.array([10, 255, 255])),
        (np.array([160, 80, 60]), np.array([179, 255, 255])),
    ],
    "green": [
        (np.array([40, 60, 40]), np.array([85, 255, 255])),
    ],
    '''
    
    "blue": [
        (np.array([95, 60, 40]), np.array([130, 255, 255])),
    ],
}

MIN_AREA_PX = 400          # ignore tiny noise blobs
SMOOTH_N = 5               # average last N centroids for stable publish


def color_mask(frame_bgr, color="red"):
    """Return binary mask for the chosen block color."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    ranges = COLOR_HSV[color]
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in ranges:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))

    # Clean speckles, fill small holes
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask


def detect_block(frame, color="red", min_area=MIN_AREA_PX):
    """Detect colored block; return (cx, cy, area, contour) or None."""
    mask = color_mask(frame, color=color)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    # Largest contour by area = the block
    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)
    if area < min_area:
        return None

    M = cv2.moments(contour)
    if M["m00"] == 0:
        return None
    cx = M["m10"] / M["m00"]
    cy = M["m01"] / M["m00"]
    return cx, cy, area, contour, mask


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
    parser.add_argument("--color", choices=sorted(COLOR_HSV.keys()), default="red",
                        help="block color to detect (default: red)")
    parser.add_argument("--min-area", type=int, default=MIN_AREA_PX, help="min contour area px")
    parser.add_argument("--once", action="store_true", help="publish one detection then exit")
    parser.add_argument("--rate", type=float, default=5.0, help="loop rate Hz")
    parser.add_argument("--width", type=int, default=CAM_WIDTH, help="capture width (match calibration)")
    parser.add_argument("--height", type=int, default=CAM_HEIGHT, help="capture height")
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"Failed to open camera {args.camera}")
        raise SystemExit(1)

    # Force resolution to match homography calibration pixels
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera {args.camera}: {actual_w}x{actual_h}, detecting color={args.color}")
    if abs(actual_w - args.width) > 40 or abs(actual_h - args.height) > 40:
        print(f"WARNING: requested {args.width}x{args.height} but got {actual_w}x{actual_h}; "
              f"homography may be off — recalibrate or pass --width/--height")

    print("Place the red block in view. Press q to quit.")

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
                recent.clear()
                node.get_logger().info(f"no {args.color} block detected", throttle_duration_sec=2.0)
                display = frame
            else:
                cx, cy, area, contour, mask = result
                recent.append((cx, cy))
                # Smooth centroid so published pose is less jumpy
                sx = sum(p[0] for p in recent) / len(recent)
                sy = sum(p[1] for p in recent) / len(recent)
                x, y = pixel_to_table(sx, sy)
                node.publish_xy(x, y)
                published = True

                display = frame.copy()
                cv2.drawContours(display, [contour], -1, (0, 255, 0), 2)
                cv2.circle(display, (int(sx), int(sy)), 8, (0, 255, 0), 2)
                cv2.putText(
                    display,
                    f"{args.color} ({x:.3f}, {y:.3f}) m  area={int(area)}",
                    (int(sx) + 12, int(sy) - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2,
                )
                # Small mask preview (top-left)
                preview = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                ph, pw = preview.shape[:2]
                scale = 160 / max(pw, 1)
                preview = cv2.resize(preview, (int(pw * scale), int(ph * scale)))
                display[0:preview.shape[0], 0:preview.shape[1]] = preview

            cv2.imshow("cam_to_coord", display)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

            rclpy.spin_once(node, timeout_sec=0.0)
            if args.once and published and len(recent) >= min(3, SMOOTH_N):
                break
            time.sleep(period)
    finally:
        cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
