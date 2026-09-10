#!/usr/bin/env python3
"""ROS message <-> sensor_types adapters.

Node-layer code: it imports `sensor_msgs`, so it is not an algorithm module
and `SensorPreprocessor` must never import it. The split exists so the
byte-level decoding (which is where layout bugs hide) can be unit tested
without starting a ROS graph.

Decoders return ``None`` when a message layout is not supported, so the
caller drops the frame and logs. A structurally valid message carrying zero
points is not an error: it decodes to an empty array.
"""

from __future__ import division

import numpy as np

from builtin_interfaces.msg import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header

from luggage_perception.sensor_types import (
    CameraCloud,
    CameraInfoFrame,
    DepthFrame,
    JointSample,
    OpaquePayload,
    RgbFrame,
)


COLOR_ENCODINGS = ("rgb8", "bgr8")
MONO_ENCODINGS = ("mono8",)
DEPTH_ENCODINGS = ("16UC1", "mono16")

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None


def _compressed_format(msg):
    return str(getattr(msg, "format", "") or "").lower()


def _compressed_bytes(msg):
    return np.frombuffer(bytes(msg.data), dtype=np.uint8)


def rgb_frame_from_compressed_msg(msg):
    """JPEG (or PNG) CompressedImage -> RgbFrame rgb8, or None."""
    if cv2 is None or msg is None or not msg.data:
        return None
    fmt = _compressed_format(msg)
    if fmt and ("jpeg" not in fmt and "jpg" not in fmt and "png" not in fmt):
        return None
    bgr = cv2.imdecode(_compressed_bytes(msg), cv2.IMREAD_COLOR)
    if bgr is None or bgr.ndim != 3 or bgr.shape[2] != 3:
        return None
    rgb = np.ascontiguousarray(bgr[:, :, ::-1])
    return RgbFrame(
        stamp=stamp_to_sec(msg.header.stamp),
        frame_id=msg.header.frame_id,
        image=rgb,
        encoding="rgb8",
    )


def depth_frame_from_compressed_msg(msg):
    """16-bit PNG CompressedImage -> DepthFrame millimetres, or None.

    JPEG depth is rejected: it is not metric-preserving.
    """
    if cv2 is None or msg is None or not msg.data:
        return None
    fmt = _compressed_format(msg)
    if "jpeg" in fmt or "jpg" in fmt:
        return None
    if fmt and "png" not in fmt:
        return None
    depth = cv2.imdecode(_compressed_bytes(msg), cv2.IMREAD_UNCHANGED)
    if depth is None or depth.ndim != 2 or depth.dtype != np.uint16:
        return None
    return DepthFrame(
        stamp=stamp_to_sec(msg.header.stamp),
        frame_id=msg.header.frame_id,
        depth=np.ascontiguousarray(depth),
        units="millimetres",
        encoding="16UC1",
    )


def stamp_to_sec(stamp):
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


def stamp_key(stamp):
    """Exact integer payload identity: the (sec, nanosec) header pair."""
    return (int(stamp.sec), int(stamp.nanosec))


def stamp_key_ns(stamp):
    return int(stamp.sec) * 1000000000 + int(stamp.nanosec)


def sec_to_stamp(seconds):
    stamp = Time()
    if seconds <= 0.0:
        return stamp
    whole = int(seconds)
    stamp.sec = whole
    stamp.nanosec = int(round((seconds - whole) * 1e9))
    if stamp.nanosec >= 1000000000:
        stamp.sec += 1
        stamp.nanosec -= 1000000000
    return stamp


def transform_matrix(tf_msg):
    """4x4 homogeneous matrix from a geometry_msgs/TransformStamped."""
    t = tf_msg.transform.translation
    r = tf_msg.transform.rotation
    qx, qy, qz, qw = r.x, r.y, r.z, r.w
    rot = np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float64)
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rot
    matrix[:3, 3] = (t.x, t.y, t.z)
    return matrix


# --------------------------------------------------------------------------
# Decode: ROS message -> sensor_types
# --------------------------------------------------------------------------

def image_array_from_msg(msg):
    """(H,W,3) or (H,W) uint8 READ-ONLY view, or None for a bad layout.

    PF-R9 g2: no eager copy. Contiguous and row-padded (step > width*ch)
    layouts are both valid when the declared buffer is long enough.
    """
    if msg.width <= 0 or msg.height <= 0:
        return None
    if msg.encoding in COLOR_ENCODINGS:
        channels = 3
    elif msg.encoding in MONO_ENCODINGS:
        channels = 1
    else:
        return None
    step = int(msg.step or (msg.width * channels))
    declared = int(msg.height) * step
    base = np.frombuffer(msg.data, dtype=np.uint8)
    base.setflags(write=False)
    if base.size * base.itemsize < declared:
        return None  # truncated buffer: caller counts it by name
    shape = ((msg.height, msg.width, channels) if channels > 1
             else (msg.height, msg.width))
    strides = ((step, channels, 1) if channels > 1 else (step, 1))
    view = np.lib.stride_tricks.as_strided(
        base[:declared], shape=shape, strides=strides)
    view.setflags(write=False)
    return view


def rgb_frame_from_msg(msg):
    """Payload-owning frame: shares the source array.array, copies nothing."""
    if msg.width <= 0 or msg.height <= 0:
        return None
    if msg.encoding not in COLOR_ENCODINGS + MONO_ENCODINGS:
        return None
    return RgbFrame(
        stamp=stamp_to_sec(msg.header.stamp),
        frame_id=msg.header.frame_id,
        encoding=msg.encoding,
        payload=OpaquePayload(msg.data, origin="ros"),
        height=int(msg.height),
        width=int(msg.width),
        step=int(msg.step or 0),
        is_bigendian=int(bool(msg.is_bigendian)),
        stamp_key=stamp_key(msg.header.stamp),
    )


def depth_array_from_msg(msg):
    """(H,W) uint16-family READ-ONLY view (endian-preserving), or None.

    PF-R9 g2: no astype to native byte order — a big-endian depth stays a
    big-endian view; conversion by copying is forbidden on this path.
    """
    if msg.width <= 0 or msg.height <= 0:
        return None
    if msg.encoding not in DEPTH_ENCODINGS:
        return None
    dtype = np.dtype(">u2" if msg.is_bigendian else "<u2")
    step = int(msg.step or (msg.width * 2))
    declared = int(msg.height) * step
    base = np.frombuffer(msg.data, dtype=dtype)
    base.setflags(write=False)
    if base.size * base.itemsize < declared:
        return None
    view = np.lib.stride_tricks.as_strided(
        base[:declared // dtype.itemsize],
        shape=(msg.height, msg.width), strides=(step, 2))
    view.setflags(write=False)
    return view


def depth_frame_from_msg(msg):
    """Payload-owning frame: shares the source array.array, copies nothing."""
    if msg.width <= 0 or msg.height <= 0:
        return None
    if msg.encoding not in DEPTH_ENCODINGS:
        return None
    return DepthFrame(
        stamp=stamp_to_sec(msg.header.stamp),
        frame_id=msg.header.frame_id,
        units="millimetres",
        encoding=msg.encoding,
        payload=OpaquePayload(msg.data, origin="ros"),
        height=int(msg.height),
        width=int(msg.width),
        step=int(msg.step or 0),
        is_bigendian=int(bool(msg.is_bigendian)),
        stamp_key=stamp_key(msg.header.stamp),
    )


def camera_info_frame_from_msg(msg):
    k = msg.k
    return CameraInfoFrame(
        stamp=stamp_to_sec(msg.header.stamp),
        frame_id=msg.header.frame_id,
        width=int(msg.width),
        height=int(msg.height),
        fx=float(k[0]),
        fy=float(k[4]),
        cx=float(k[2]),
        cy=float(k[5]),
        distortion_model=msg.distortion_model,
        distortion_coeffs=tuple(float(v) for v in msg.d),
        rectification=tuple(float(v) for v in msg.r),
        projection=tuple(float(v) for v in msg.p),
        binning_x=int(msg.binning_x),
        binning_y=int(msg.binning_y),
    )


def cloud_points_from_msg(msg):
    """(N,3) float64 XYZ, or None when the point layout is unsupported.

    Reads x/y/z at their declared offsets inside `point_step`, so padded or
    reordered layouts (XYZRGB, XYZI, ...) decode correctly. A cloud whose
    xyz are not FLOAT32 is rejected rather than silently misread.
    """
    if msg.point_step <= 0:
        return None
    fields = {field.name: field for field in msg.fields}
    if not {"x", "y", "z"} <= set(fields):
        return None
    for name in ("x", "y", "z"):
        field = fields[name]
        if field.datatype != PointField.FLOAT32 or field.count != 1:
            return None
        if field.offset + 4 > msg.point_step:
            return None

    npoints = int(msg.width) * int(msg.height)
    if npoints <= 0:
        return np.zeros((0, 3), dtype=np.float64)
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    npoints = min(npoints, raw.size // int(msg.point_step))
    if npoints <= 0:
        return np.zeros((0, 3), dtype=np.float64)
    raw = raw[: npoints * int(msg.point_step)]

    fmt = ">f4" if msg.is_bigendian else "<f4"
    structured = np.ndarray(
        shape=(npoints,),
        dtype=np.dtype({
            "names": ["x", "y", "z"],
            "formats": [fmt, fmt, fmt],
            "offsets": [fields[n].offset for n in ("x", "y", "z")],
            "itemsize": int(msg.point_step),
        }),
        buffer=raw,
        order="C",
    )
    return np.column_stack(
        (structured["x"], structured["y"], structured["z"])
    ).astype(np.float64)


def camera_cloud_from_msg(msg, data_frame):
    points = cloud_points_from_msg(msg)
    if points is None:
        return None
    return CameraCloud(
        stamp=stamp_to_sec(msg.header.stamp),
        frame_id=msg.header.frame_id,
        data_frame=data_frame,
        points=points,
    )


def joint_sample_from_msg(msg, fallback_stamp_sec=None):
    """JointSample from a JointState.

    `joint_state_broadcaster` always stamps, but a bag or a hand-written
    publisher may not; `fallback_stamp_sec` keeps the motion gate fed
    instead of rejecting the sample outright.
    """
    stamp_sec = stamp_to_sec(msg.header.stamp)
    if stamp_sec <= 0.0:
        if fallback_stamp_sec is None:
            return None
        stamp_sec = float(fallback_stamp_sec)
    return JointSample(
        stamp=stamp_sec,
        joint_names=tuple(msg.name),
        positions=np.asarray(msg.position, dtype=np.float64),
        velocities=(
            np.asarray(msg.velocity, dtype=np.float64)
            if len(msg.velocity) else None
        ),
    )


# --------------------------------------------------------------------------
# Encode: sensor_types -> ROS message
# --------------------------------------------------------------------------

def image_msg_from_frame(frame, stamp):
    """Republish by payload identity when the frame owns one (PF-R9 g2).

    The output Image rewrites only the paired header and metadata and
    assigns the ORIGINAL source ``array.array('B')`` to ``out.data`` — no
    copy, astype, ascontiguousarray, or tobytes. Legacy frames constructed
    directly from NumPy arrays (tests) fall back to a materialising path.
    CDR serialisation below the publisher still copies; that is stated and
    is not zero-copy end-to-end.
    """
    out = Image()
    out.header = Header(stamp=stamp, frame_id=frame.frame_id)
    out.encoding = frame.encoding or "rgb8"
    payload = getattr(frame, "payload", None)
    if payload is not None and payload.origin == "ros":
        out.height = int(frame.height)
        out.width = int(frame.width)
        out.is_bigendian = int(frame.is_bigendian)
        out.step = int(frame.step or (frame.width * (
            3 if out.encoding in COLOR_ENCODINGS else 1)))
        out.data = payload.ros_data()
        return out
    image = np.asarray(frame.view() if hasattr(frame, "view") else frame.image)
    out.height = int(image.shape[0])
    out.width = int(image.shape[1])
    out.is_bigendian = 0
    out.step = out.width if image.ndim == 2 else out.width * int(image.shape[2])
    out.data = np.ascontiguousarray(image, dtype=np.uint8).tobytes()
    return out


def depth_msg_from_frame(frame, stamp):
    """Republish by payload identity (see image_msg_from_frame)."""
    out = Image()
    out.header = Header(stamp=stamp, frame_id=frame.frame_id)
    payload = getattr(frame, "payload", None)
    if payload is not None and payload.origin == "ros":
        out.height = int(frame.height)
        out.width = int(frame.width)
        out.encoding = frame.encoding or "16UC1"
        out.is_bigendian = int(frame.is_bigendian)
        out.step = int(frame.step or (frame.width * 2))
        out.data = payload.ros_data()
        return out
    depth = np.asarray(frame.view() if hasattr(frame, "view") else frame.depth)
    out.height = int(depth.shape[0])
    out.width = int(depth.shape[1])
    out.encoding = "16UC1"
    out.is_bigendian = 0
    out.step = out.width * 2
    out.data = np.ascontiguousarray(depth, dtype="<u2").tobytes()
    return out


def mask_msg_from_array(label_map, stamp, frame_id):
    """mono8 Image from an HxW uint8 label map (pixel value = label id)."""
    arr = np.ascontiguousarray(np.asarray(label_map), dtype=np.uint8)
    if arr.ndim != 2:
        raise ValueError("label_map must be HxW, got shape %r" % (arr.shape,))
    out = Image()
    out.header = Header(stamp=stamp, frame_id=frame_id)
    out.height = int(arr.shape[0])
    out.width = int(arr.shape[1])
    out.encoding = "mono8"
    out.is_bigendian = 0
    out.step = out.width
    out.data = arr.tobytes()
    return out


def instance_mask_msg_from_array(instance_map, stamp, frame_id):
    """mono16 Image from an HxW uint16 instance id map (0 = background)."""
    arr = np.ascontiguousarray(np.asarray(instance_map), dtype="<u2")
    if arr.ndim != 2:
        raise ValueError("instance_map must be HxW, got shape %r" % (arr.shape,))
    out = Image()
    out.header = Header(stamp=stamp, frame_id=frame_id)
    out.height = int(arr.shape[0])
    out.width = int(arr.shape[1])
    out.encoding = "mono16"
    out.is_bigendian = 0
    out.step = out.width * 2
    out.data = arr.tobytes()
    return out


def mono16_array_from_msg(msg):
    """(H,W) uint16 array from a mono16 Image, or None for another layout."""
    if msg.width <= 0 or msg.height <= 0:
        return None
    if msg.encoding != "mono16":
        return None
    dtype = np.dtype(">u2" if msg.is_bigendian else "<u2")
    need = int(msg.height) * int(msg.width)
    arr = np.frombuffer(msg.data, dtype=dtype)
    if arr.size < need:
        return None
    return arr[:need].reshape(msg.height, msg.width).astype(np.uint16)


def camera_info_msg_from_frame(frame, stamp):
    out = CameraInfo()
    out.header = Header(stamp=stamp, frame_id=frame.frame_id)
    out.width = int(frame.width)
    out.height = int(frame.height)
    out.distortion_model = frame.distortion_model
    out.d = [float(v) for v in frame.distortion_coeffs]
    out.k = [
        frame.fx, 0.0, frame.cx,
        0.0, frame.fy, frame.cy,
        0.0, 0.0, 1.0,
    ]
    if len(frame.rectification) == 9:
        out.r = [float(v) for v in frame.rectification]
    else:
        out.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    if len(frame.projection) == 12:
        out.p = [float(v) for v in frame.projection]
    else:
        out.p = [
            frame.fx, 0.0, frame.cx, 0.0,
            0.0, frame.fy, frame.cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
    out.binning_x = int(frame.binning_x)
    out.binning_y = int(frame.binning_y)
    return out


def cloud_msg_from_points(points, stamp, frame_id):
    pts = np.ascontiguousarray(
        np.asarray(points, dtype=np.float32).reshape(-1, 3))
    msg = PointCloud2()
    msg.header = Header(stamp=stamp, frame_id=frame_id)
    msg.height = 1
    msg.width = int(pts.shape[0])
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = pts.tobytes()
    return msg
