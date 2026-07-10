#!/usr/bin/env python3
"""Sort yellow and purple blocks side by side (no stacking).

Detects each colored block via HSV mask + homography (same approach as
cam_to_coord.py), then invokes pick_flow.py once per block as a
subprocess, passing the detected PX PY plus a per-color ROT so the two
blocks are placed at different base-rotation angles instead of on top
of each other.

IMPORTANT — how "place" actually works here:
  pick_flow.py picks at (PX, PY), lifts, rotates the base joint (J1) by
  ROT degrees, then descends to PLACE_Z and releases. It does NOT accept
  an arbitrary place (x, y) -- the place spot is the pick radius, rotated
  ROT degrees around the base. So "sort side by side" = pick each block
  from wherever it is, and give each color its own ROT so they end up at
  different angles. SORT_ROT below are placeholder guesses -- verify on
  your table that both resulting spots are reachable, on the table, and
  don't collide before running for real (use --dry-run first).

Usage:
  python sort_blocks.py                 # detect + sort yellow then purple
  python sort_blocks.py --dry-run       # detect + print pick_flow commands only
  python sort_blocks.py --order purple yellow
  python sort_blocks.py --camera 1

Env:
  ROBOT_IP / ROS_DOMAIN_ID are handled inside pick_flow.py itself.
"""
import argparse
import subprocess
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from homography_transform import pixel_to_meters

CAM_WIDTH = 1280
CAM_HEIGHT = 720

# HSV ranges (OpenCV H: 0-179). Estimates -- tune to your actual blocks/lighting.
COLOR_HSV = {
    "yellow": [(np.array([20, 100, 100]), np.array([34, 255, 255]))],
    "purple": [(np.array([130, 60, 40]), np.array([155, 255, 255]))],
}
DRAW_COLOR = {"yellow": (0, 220, 220), "purple": (150, 40, 130)}

# Per-color rotation (degrees) passed as ROT to pick_flow.py after lifting.
# Distinct values so the two blocks land at different base angles.
# TODO: calibrate against your table/reach -- these are placeholders.
SORT_ROT = {"yellow": 60.0, "purple": 120.0}

# Grasp/place height + grip/speed, passed straight through to pick_flow.py.
GRASP_Z = 0.02
PLACE_Z = 0.06
GRIP_VAL = 35
SPEED = 25

MIN_AREA_PX = 400
SMOOTH_N = 5
PICK_FLOW = str(Path(__file__).resolve().parent / "pick_flow.py")


def color_mask(frame_bgr, color):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in COLOR_HSV[color]:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask


def detect_block(frame, color, min_area=MIN_AREA_PX):
    mask = color_mask(frame, color)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)
    if area < min_area:
        return None
    M = cv2.moments(contour)
    if M["m00"] == 0:
        return None
    return M["m10"] / M["m00"], M["m01"] / M["m00"], area, contour, mask


def find_stable_position(cap, color, min_area, settle_frames=SMOOTH_N, timeout=15.0):
    """Watch the feed until `color` gives a stable centroid; return table (x, y) meters."""
    recent = deque(maxlen=settle_frames)
    t0 = time.time()
    while time.time() - t0 < timeout:
        ret, frame = cap.read()
        if not ret:
            return None, None
        result = detect_block(frame, color, min_area)
        if result is None:
            recent.clear()
            display = frame
        else:
            cx, cy, area, contour, mask = result
            recent.append((cx, cy))
            display = frame.copy()
            cv2.drawContours(display, [contour], -1, DRAW_COLOR[color], 2)
            cv2.circle(display, (int(cx), int(cy)), 8, DRAW_COLOR[color], 2)
        cv2.putText(display, f"locating {color}...", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, DRAW_COLOR[color], 2)
        cv2.imshow("sort_blocks", display)
        cv2.waitKey(1)
        if len(recent) >= settle_frames:
            sx = sum(p[0] for p in recent) / len(recent)
            sy = sum(p[1] for p in recent) / len(recent)
            return pixel_to_meters(sx, sy)
    return None, None


def run_pick_flow(px, py, rot, dry_run=False):
    args = [sys.executable, PICK_FLOW,
            f"{px:.4f}", f"{py:.4f}", f"{rot:.1f}",
            f"{GRASP_Z}", f"{PLACE_Z}", str(GRIP_VAL), str(SPEED)]
    print("->", " ".join(args))
    if dry_run:
        return True
    result = subprocess.run(args)
    return result.returncode == 0


def main():
    parser = argparse.ArgumentParser(description="Sort yellow/purple blocks side by side")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--min-area", type=int, default=MIN_AREA_PX)
    parser.add_argument("--width", type=int, default=CAM_WIDTH)
    parser.add_argument("--height", type=int, default=CAM_HEIGHT)
    parser.add_argument("--dry-run", action="store_true",
                         help="detect + print pick_flow commands only, don't move the arm")
    parser.add_argument("--order", nargs="+", default=["yellow", "purple"],
                         choices=["yellow", "purple"],
                         help="which color to sort first (space-separated)")
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"Failed to open camera {args.camera}")
        raise SystemExit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera {args.camera}: {actual_w}x{actual_h}")
    if abs(actual_w - args.width) > 40 or abs(actual_h - args.height) > 40:
        print(f"WARNING: requested {args.width}x{args.height} but got {actual_w}x{actual_h}; "
              f"homography may be off -- recalibrate or pass --width/--height")

    try:
        for color in args.order:
            print(f"\n=== locating {color} block ===")
            x, y = find_stable_position(cap, color, args.min_area)
            if x is None:
                print(f"{color}: not found in time, skipping")
                continue
            print(f"{color}: pick at PX={x:.3f} PY={y:.3f}, rotate {SORT_ROT[color]:.0f} deg to place")
            ok = run_pick_flow(x, y, SORT_ROT[color], dry_run=args.dry_run)
            print(f"{color}: {'OK' if ok else 'FAILED'}")
            if not ok and not args.dry_run:
                print(f"!! {color} pick_flow failed -- stopping (arm holds position, check before retrying)")
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
