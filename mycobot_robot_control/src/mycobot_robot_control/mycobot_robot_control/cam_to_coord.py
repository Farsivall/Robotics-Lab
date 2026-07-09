#!/usr/bin/env python3
"""Person 2: detect block in webcam and publish table (x, y) for pick_flow.

Publishes geometry_msgs/PointStamped on /block_position — the topic
pick_flow.py waits for when run without PX/PY args.

Person 3 later replaces pixel_to_table() with real homography/calibration.
Until then, a simple linear map is used (tune CAMERA_* / TABLE_* below).

Usage:
  python cam_to_coord.py              # live detect + publish
  python cam_to_coord.py --once       # one detection then exit
  python cam_to_coord.py --camera 1   # non-default webcam index

Env:
  ROS_DOMAIN_ID should match pick_flow (usually 10).
"""
import argparse
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node

# --- placeholder calibration (Person 3 replaces pixel_to_table) ---
# Map image pixel (u,v) into approximate table meters (x forward, y left).
# Defaults assume camera looks down at a region in front of the robot.
CAMERA_WIDTH = 640.0
CAMERA_HEIGHT = 480.0
TABLE_X_MIN, TABLE_X_MAX = 0.10, 0.26   # meters forward from base
TABLE_Y_MIN, TABLE_Y_MAX = -0.10, 0.10  # meters left(+)/right(-)


def get_background(cap, num_frames=30):
    frames = []
    for _ in range(num_frames):
        ret, frame = cap.read()
        if ret:
            frames.append(frame)
        time.sleep(0.03)
    if not frames:
        return None
    return np.median(frames, axis=0).astype(np.uint8)


def detect_block(frame, background, threshold=30):
    diff = cv2.absdiff(frame, background)
    gray_diff = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray_diff, threshold, 255, cv2.THRESH_BINARY)

    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    M = cv2.moments(mask, binaryImage=True)
    if M["m00"] == 0:
        return None
    cx = M["m10"] / M["m00"]
    cy = M["m01"] / M["m00"]
    return cx, cy


def pixel_to_table(cx, cy, frame_w, frame_h):
    """Placeholder pixel -> table meters. Person 3: replace with homography."""
    u = cx / max(frame_w, 1.0)
    v = cy / max(frame_h, 1.0)
    # image top ~ farther from robot (larger x), image left ~ +y
    x = TABLE_X_MAX - v * (TABLE_X_MAX - TABLE_X_MIN)
    y = TABLE_Y_MAX - u * (TABLE_Y_MAX - TABLE_Y_MIN)
    return float(x), float(y)


class BlockPublisher(Node):
    def __init__(self):
        super().__init__("cam_to_coord")
        self.pub = self.create_publisher(PointStamped, "/block_position", 10)
        self.get_logger().info("publishing detections on /block_position")

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
    parser = argparse.ArgumentParser(description="Detect block and publish /block_position")
    parser.add_argument("--camera", type=int, default=0, help="webcam index")
    parser.add_argument("--threshold", type=int, default=30, help="bg-sub threshold")
    parser.add_argument("--once", action="store_true", help="publish one detection then exit")
    parser.add_argument("--rate", type=float, default=2.0, help="publish rate Hz when looping")
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"Failed to open camera {args.camera}")
        raise SystemExit(1)

    print("Capturing background (clear the table)...")
    background = get_background(cap, num_frames=30)
    if background is None:
        print("Failed to capture background")
        cap.release()
        raise SystemExit(1)
    print("Background ready. Place the block, detecting...")

    rclpy.init()
    node = BlockPublisher()
    period = 1.0 / max(args.rate, 0.1)
    published = False

    try:
        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                print("Failed to capture frame")
                break

            h, w = frame.shape[:2]
            result = detect_block(frame, background, threshold=args.threshold)
            if result is None:
                node.get_logger().info("no block detected", throttle_duration_sec=2.0)
            else:
                cx, cy = result
                x, y = pixel_to_table(cx, cy, w, h)
                node.publish_xy(x, y)
                published = True
                # overlay for local debug
                cv2.circle(frame, (int(cx), int(cy)), 8, (0, 255, 0), 2)
                cv2.putText(
                    frame, f"({x:.3f}, {y:.3f}) m",
                    (int(cx) + 10, int(cy) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
                )

            cv2.imshow("cam_to_coord", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

            rclpy.spin_once(node, timeout_sec=0.0)
            if args.once and published:
                break
            time.sleep(period)
    finally:
        cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
