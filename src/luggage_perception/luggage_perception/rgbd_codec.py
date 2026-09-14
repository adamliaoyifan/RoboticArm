"""D555 transport codec: JPEG colour and lossless 16UC1 PNG depth."""

from __future__ import annotations

import numpy as np
from sensor_msgs.msg import CompressedImage, Image

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None


COLOR_JPEG_FORMAT = "jpeg"
DEPTH_PNG_FORMAT = "16UC1; png"


def _require_cv2():
    if cv2 is None:
        raise RuntimeError("OpenCV (cv2) is required for D555 compression")


def _rows(msg, bytes_per_pixel):
    height, width = int(msg.height), int(msg.width)
    step = int(msg.step or width * bytes_per_pixel)
    required = height * step
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if height <= 0 or width <= 0 or step < width * bytes_per_pixel:
        raise ValueError("invalid image dimensions/step")
    if raw.nbytes < required:
        raise ValueError("truncated image payload")
    return raw[:required].reshape(height, step)[:, :width * bytes_per_pixel]


def color_to_jpeg(msg, quality=80):
    _require_cv2()
    if str(msg.encoding).replace("-", "").lower() not in ("rgb8", "rgb"):
        raise ValueError("color_to_jpeg expects rgb8, got %s" % msg.encoding)
    rgb = _rows(msg, 3).reshape(int(msg.height), int(msg.width), 3)
    q = max(1, min(100, int(quality)))
    ok, encoded = cv2.imencode(
        ".jpg", rgb[:, :, ::-1], [int(cv2.IMWRITE_JPEG_QUALITY), q])
    if not ok:
        raise RuntimeError("cv2.imencode jpeg failed")
    out = CompressedImage()
    out.header = msg.header
    out.format = COLOR_JPEG_FORMAT
    out.data = encoded.tobytes()
    return out


def depth_to_png(msg):
    _require_cv2()
    if str(msg.encoding).replace("-", "").lower() not in ("16uc1", "mono16"):
        raise ValueError("depth_to_png expects 16UC1, got %s" % msg.encoding)
    if bool(msg.is_bigendian):
        raise ValueError("big-endian D555 depth is unsupported")
    packed = np.ascontiguousarray(_rows(msg, 2))
    depth = packed.view("<u2").reshape(int(msg.height), int(msg.width))
    ok, encoded = cv2.imencode(".png", depth)
    if not ok:
        raise RuntimeError("cv2.imencode png failed")
    out = CompressedImage()
    out.header = msg.header
    out.format = DEPTH_PNG_FORMAT
    out.data = encoded.tobytes()
    return out
