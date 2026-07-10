#!/usr/bin/env python3
"""
homography_transform.py

Person 3: pixel -> table coordinates via calibrated homography.

Takes a pixel coordinate in, returns the corresponding cm coordinate out,
using a homography computed from known calibration point pairs.

CM_POINTS convention (from calibration):
  column 0 ≈ lateral  (robot Y, cm; left positive / right negative — match your sheet)
  column 1 ≈ forward  (robot X, cm from base)

pick_flow /cam_to_coord convert cm -> meters and publish (x_forward, y_lateral).
"""

import numpy as np
import cv2

# ---------------------------------------------------------------------------
# Calibration data (edit here if you re-calibrate)
# ---------------------------------------------------------------------------
# CM: (lateral_cm, forward_cm)  — robot Y, robot X
# PX: (u, v) camera pixels
CM_POINTS = np.array([
    (-16.0, 0.0), (0.0, 0.0), (16.0, 0.0),
    (-6.0, 7.5), (8.0, 8.0),
    (-16.0, 13.5), (0.0, 15.0), (16.0, 15.0),
    (-9.0, 20.0), (9.0, 20.0),
], dtype=np.float64)

PX_POINTS = np.array([
    (945, 310), (647, 321), (352, 314),
    (791, 401), (467, 408),
    (1051, 471), (647, 511), (223, 513),
    (384, 633), (933, 603),
], dtype=np.float64)

# Homography: pixel -> cm (swap src/dst if you need cm -> px instead)
_H, _mask = cv2.findHomography(PX_POINTS, CM_POINTS, method=cv2.RANSAC)
if _H is None:
    raise RuntimeError('Homography computation failed — check calibration points.')


def pixel_to_cm(x_px, y_px):
    """Transform a single pixel coordinate to a cm coordinate (lateral, forward)."""
    pt = np.array([[[x_px, y_px]]], dtype=np.float64)
    out = cv2.perspectiveTransform(pt, _H)
    return float(out[0, 0, 0]), float(out[0, 0, 1])


def pixel_to_meters(x_px, y_px):
    """Pixel -> (x_forward_m, y_lateral_m) for pick_flow /block_position."""
    lateral_cm, forward_cm = pixel_to_cm(x_px, y_px)
    return forward_cm / 100.0, lateral_cm / 100.0


if __name__ == '__main__':
    # Example — calibration pair (647, 321) <-> (0, 0) cm
    x_in, y_in = 647, 321
    x_out, y_out = pixel_to_cm(x_in, y_in)
    xf, yf = pixel_to_meters(x_in, y_in)
    print(f'px=({x_in}, {y_in}) -> cm=({x_out:.2f}, {y_out:.2f})')
    print(f'px=({x_in}, {y_in}) -> m  x={xf:.3f} y={yf:.3f} (for pick_flow)')
