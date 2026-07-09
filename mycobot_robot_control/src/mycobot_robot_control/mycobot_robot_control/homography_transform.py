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
# Original coordinates in cm (robot / world frame)
CM_POINTS = np.array([
    (-15.5, 24), (4, 25), (-24, 16), (-5, 17), (12, 19),
    (-15.5, 9), (4.5, 9), (19.5, 9), (-5, 5), (-24, -1),
    (0, 0), (12.5, 0), (27.5, 0)
], dtype=np.float64)

# Corresponding transformed coordinates in px (camera / image frame)
PX_POINTS = np.array([
    (1039, 618), (450, 647), (1174, 484), (701, 531), (274, 527),
    (900, 407), (496, 409), (178, 406), (679, 374), (1006, 332),
    (600, 332), (371, 339), (115, 334)
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
    # Example usage — calibration pair (600, 332) <-> (0, 0) cm
    x_in, y_in = 600, 332
    x_out, y_out = pixel_to_cm(x_in, y_in)
    xf, yf = pixel_to_meters(x_in, y_in)
    print(f'px=({x_in}, {y_in}) -> cm=({x_out:.2f}, {y_out:.2f})')
    print(f'px=({x_in}, {y_in}) -> m  x={xf:.3f} y={yf:.3f} (for pick_flow)')
