"""Encode / decode D555 RGB-D for site bags.

Color is JPEG (lossy). Aligned depth is 16-bit PNG (lossless). Stamps and
frame_ids are copied so host-clock sync still holds after compression.
"""

from __future__ import annotations

import numpy as np
from sensor_msgs.msg import CompressedImage, Image

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None

COLOR_JPEG_FORMAT = "jpeg"
DEPTH_PNG_FORMAT = "16UC1; png"


def is_jpeg_format(fmt: str) -> bool:
    f = (fmt or "").lower()
    return "jpeg" in f or "jpg" in f


def is_png_format(fmt: str) -> bool:
    return "png" in (fmt or "").lower()


def canonicalize_color_compressed(msg: CompressedImage) -> CompressedImage:
    """Keep JPEG bytes; normalize format for our bag / decompress contract."""
    if not is_jpeg_format(msg.format):
        raise ValueError("expected jpeg compressed color, got %s" % msg.format)
    msg.format = COLOR_JPEG_FORMAT
    return msg


def canonicalize_depth_compressed(msg: CompressedImage) -> CompressedImage:
    """Keep PNG bytes; 16-bit depth must not be JPEG."""
    if not is_png_format(msg.format):
        raise ValueError("expected png compressed depth, got %s" % msg.format)
    msg.format = DEPTH_PNG_FORMAT
    return msg


def _require_cv2() -> None:
    if cv2 is None:
        raise RuntimeError("OpenCV (cv2) is required to compress D555 frames")


def _color_array(msg: Image) -> np.ndarray:
    h, w = int(msg.height), int(msg.width)
    step = int(msg.step) if msg.step else w * 3
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    row = buf.reshape((h, step))[:, : w * 3]
    return row.reshape((h, w, 3))


def _depth_array(msg: Image) -> np.ndarray:
    h, w = int(msg.height), int(msg.width)
    step = int(msg.step) if msg.step else w * 2
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    row = buf.reshape((h, step))[:, : w * 2]
    packed = np.ascontiguousarray(row)
    return packed.view(np.uint16).reshape((h, w))


def color_to_jpeg(msg: Image, quality: int = 80) -> CompressedImage:
    """RGB8 Image → CompressedImage jpeg. Quality 1–100."""
    _require_cv2()
    if msg.encoding.replace("-", "").lower() not in ("rgb8", "rgb"):
        raise ValueError("color_to_jpeg expects rgb8, got %s" % msg.encoding)
    rgb = _color_array(msg)
    bgr = rgb[:, :, ::-1]
    q = max(1, min(100, int(quality)))
    ok, enc = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), q])
    if not ok:
        raise RuntimeError("cv2.imencode jpeg failed")
    out = CompressedImage()
    out.header = msg.header
    out.format = COLOR_JPEG_FORMAT
    out.data = enc.tobytes()
    return out


def depth_to_png(msg: Image) -> CompressedImage:
    """16UC1 depth Image → lossless PNG CompressedImage."""
    _require_cv2()
    enc_name = msg.encoding.replace("-", "").lower()
    if enc_name not in ("16uc1", "mono16"):
        raise ValueError("depth_to_png expects 16UC1, got %s" % msg.encoding)
    depth = _depth_array(msg)
    ok, enc = cv2.imencode(".png", depth)
    if not ok:
        raise RuntimeError("cv2.imencode png failed")
    out = CompressedImage()
    out.header = msg.header
    out.format = DEPTH_PNG_FORMAT
    out.data = enc.tobytes()
    return out


def jpeg_to_color(msg: CompressedImage, frame_id: str = "") -> Image:
    _require_cv2()
    arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if bgr is None:
        raise RuntimeError("cv2.imdecode jpeg failed")
    rgb = bgr[:, :, ::-1]
    out = Image()
    out.header = msg.header
    if frame_id:
        out.header.frame_id = frame_id
    out.height = rgb.shape[0]
    out.width = rgb.shape[1]
    out.encoding = "rgb8"
    out.is_bigendian = 0
    out.step = rgb.shape[1] * 3
    out.data = rgb.tobytes()
    return out


def png_to_depth(msg: CompressedImage, frame_id: str = "") -> Image:
    _require_cv2()
    arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    depth = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
    if depth is None:
        raise RuntimeError("cv2.imdecode png failed")
    if depth.dtype != np.uint16:
        depth = depth.astype(np.uint16)
    if depth.ndim != 2:
        raise RuntimeError("decoded depth is not 2-D")
    out = Image()
    out.header = msg.header
    if frame_id:
        out.header.frame_id = frame_id
    out.height = depth.shape[0]
    out.width = depth.shape[1]
    out.encoding = "16UC1"
    out.is_bigendian = 0
    out.step = depth.shape[1] * 2
    out.data = depth.tobytes()
    return out
