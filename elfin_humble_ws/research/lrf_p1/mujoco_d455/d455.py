"""Intel RealSense D455 (IMU) camera model used by the MuJoCo spike.

This is a pinhole RGB-D approximation, not stereo matching and not librealsense.
Depth is the algorithm input. RGB is for human overlay only (aligned, same pose).
IMU is present as a site and is not consumed by the box estimator.
"""

from __future__ import division

import math

# Datasheet: https://www.intelrealsense.com/depth-camera-d455/
NAME = "realsense_d455"
# Intel ships D455 with an IMU (the informal "D455i" name). D435i is the
# IMU SKU in the 400-series naming; D455 already includes IMU.
HAS_IMU = True
STEREO_BASELINE_M = 0.095

# Depth stream used as estimator input.
DEPTH_WIDTH = 848
DEPTH_HEIGHT = 480
DEPTH_FPS = 30
DEPTH_VFOV_DEG = 58.0  # official vertical FOV
# Horizontal FOV with square pixels at 848x480 is ~88.8 deg vs datasheet 87 deg.
DEPTH_RANGE_MIN_M = 0.40
DEPTH_RANGE_MAX_M = 6.0

# Aligned RGB overlay (not a second imager).
RGB_WIDTH = DEPTH_WIDTH
RGB_HEIGHT = DEPTH_HEIGHT

# Housing, millimetres -> metres, half-sizes for MuJoCo box geoms.
HOUSING_SIZE = (0.124 * 0.5, 0.026 * 0.5, 0.029 * 0.5)
MASS_KG = 0.124

# Depth noise used after render. Not a stereo model.
RANGE_NOISE_SIGMA_M = 0.004
DROPOUT_RATE = 0.01


def depth_intrinsics(width=DEPTH_WIDTH, height=DEPTH_HEIGHT, vfov_deg=DEPTH_VFOV_DEG):
    """Pinhole K matching MuJoCo's vertical-FOV camera with square pixels."""
    fy = (0.5 * float(height)) / math.tan(math.radians(float(vfov_deg)) * 0.5)
    fx = fy
    cx = (float(width) - 1.0) * 0.5
    cy = (float(height) - 1.0) * 0.5
    hfov_deg = math.degrees(2.0 * math.atan((0.5 * float(width)) / fx))
    return {
        "fx": float(fx),
        "fy": float(fy),
        "cx": float(cx),
        "cy": float(cy),
        "width": int(width),
        "height": int(height),
        "vfov_deg": float(vfov_deg),
        "hfov_deg": float(hfov_deg),
        "distortion_model": "plumb_bob",
        "distortion_coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
    }


def hfov_note():
    k = depth_intrinsics()
    return (
        "D455 depth K uses official VFOV 58 deg at %dx%d (fx=fy=%.3f). "
        "Resulting HFOV is %.2f deg vs datasheet 87 deg."
        % (k["width"], k["height"], k["fx"], k["hfov_deg"])
    )
