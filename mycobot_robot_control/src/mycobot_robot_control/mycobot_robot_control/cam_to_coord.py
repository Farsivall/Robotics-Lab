#!/usr/bin/env python3
"""Person 2+3: detect block, apply homography, publish table (x, y) for pick_flow.

Publishes geometry_msgs/PointStamped on /block_position — the topic
pick_flow.py waits for when run without PX/PY args.

Pipeline:
  webcam -> background subtract -> pixel (cx, cy)
        -> homography_transform.pixel_to_meters -> (x, y) meters
        -> /block_position -> pick_flow.py

Usage:
  python cam_to_coord.py              # live detect + publish
  python cam_to_coord.py --once       # one detection then exit
  python cam_to_coord.py --camera 1   # non-default webcam index

Env:
  ROS_DOMAIN_ID should match pick_flow (usually 10).
"""
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node

# Same-folder import when run as a script from Downloads / source tree
sys.path.insert(0, str(Path(__file__).resolve().parent))
from homography_transform import pixel_to_meters


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


def pixel_to_table(cx, cy, frame_w=None, frame_h=None):
    """Pixel centroid -> table meters (x forward, y lateral) via Person 3 homography."""
    del frame_w, frame_h
    return pixel_to_meters(cx, cy)


class BlockPublisher(Node):
    def __init__(self):
        super().__init__("cam_to_coord")
        self.pub = self.create_publisher(PointStamped, "/block_position", 10)
        self.get_logger().info("publishing detections on /block_position (homography)")

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

            result = detect_block(frame, background, threshold=args.threshold)
            if result is None:
                node.get_logger().info("no block detected", throttle_duration_sec=2.0)
            else:
                cx, cy = result
                x, y = pixel_to_table(cx, cy)
                node.publish_xy(x, y)
                published = True
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
