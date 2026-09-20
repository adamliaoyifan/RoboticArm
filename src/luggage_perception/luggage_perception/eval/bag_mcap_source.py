"""Eval-only mcap bag source. Not imported by online nodes.

The teach-pendant bags under robotarm_bags were recorded by a newer
rosbag2 whose embedded metadata record (``offered_qos_profiles`` with
``history: unknown``) Humble's rosbag2 cannot parse — ``ros2 bag play``,
``ros2 bag info`` and ``rosbag2_py.SequentialReader`` all fail on them,
and patching the on-disk metadata.yaml is not enough because the embedded
mcap metadata record is parsed too. This module bypasses rosbag2
entirely: ``mcap.reader`` streams the records chunk by chunk and
``rclpy.serialization`` deserializes the CDR payloads. Bags are read-only
and never modified.
"""
from __future__ import division

import importlib
import os
from dataclasses import dataclass, field

import numpy as np
from mcap.reader import make_reader
from rclpy.serialization import deserialize_message

from luggage_perception.ros_message_adapters import (
    cloud_points_from_msg,
    depth_array_from_msg,
    image_array_from_msg,
)

# Explicit topic registry: only these are deserialized. Everything else
# (/livox/imu at 200 Hz, /elfin/cps_*, ...) is skipped and only counted.
# /livox/lidar is PointCloud2 (xfer_format 0) or Livox CustomMsg
# (xfer_format 1); CustomMsg is decoded from CDR without livox_ros_driver2.
COLOR_TOPIC = "/camera/d555/color/image_raw"
DEPTH_TOPIC = "/camera/d555/aligned_depth_to_color/image_raw"
COLOR_COMPRESSED_TOPIC = "/camera/d555/color/image_raw/compressed"
DEPTH_COMPRESSED_TOPIC = "/camera/d555/aligned_depth_to_color/image_raw/compressed"
COLOR_INFO_TOPIC = "/camera/d555/color/camera_info"
DEPTH_INFO_TOPIC = "/camera/d555/aligned_depth_to_color/camera_info"
JOINT_TOPIC = "/joint_states"
TCP_TOPIC = "/elfin/tcp_pose"
LIDAR_TOPIC = "/livox/lidar"
TF_TOPIC = "/tf"
TF_STATIC_TOPIC = "/tf_static"
LIVOX_CUSTOM_SCHEMA = "livox_ros_driver2/msg/CustomMsg"

TOPIC_TYPES = {
    COLOR_TOPIC: "sensor_msgs/msg/Image",
    DEPTH_TOPIC: "sensor_msgs/msg/Image",
    COLOR_COMPRESSED_TOPIC: "sensor_msgs/msg/CompressedImage",
    DEPTH_COMPRESSED_TOPIC: "sensor_msgs/msg/CompressedImage",
    COLOR_INFO_TOPIC: "sensor_msgs/msg/CameraInfo",
    DEPTH_INFO_TOPIC: "sensor_msgs/msg/CameraInfo",
    JOINT_TOPIC: "sensor_msgs/msg/JointState",
    TCP_TOPIC: "geometry_msgs/msg/PoseStamped",
    LIDAR_TOPIC: "sensor_msgs/msg/PointCloud2",
    TF_TOPIC: "tf2_msgs/msg/TFMessage",
    TF_STATIC_TOPIC: "tf2_msgs/msg/TFMessage",
}

_MSG_CLASS_CACHE = {}

# Schemas whose first CDR field is std_msgs/Header (stamp first: int32 sec
# + uint32 nanosec right after the 4-byte encapsulation header), so the
# stamp can be parsed without deserializing the payload. Livox CustomMsg
# has the same header layout. tf2_msgs/TFMessage has NO top-level header
# and never qualifies.
HEADER_FIRST_SCHEMAS = {
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/CompressedImage",
    "sensor_msgs/msg/CameraInfo",
    "sensor_msgs/msg/JointState",
    "geometry_msgs/msg/PoseStamped",
}


def _msg_class(schema_name):
    """Resolve 'pkg/msg/Type' to the ROS message class (cached), or None."""
    if schema_name in _MSG_CLASS_CACHE:
        return _MSG_CLASS_CACHE[schema_name]
    parts = str(schema_name).split("/")
    cls = None
    if len(parts) == 3:
        try:
            module = importlib.import_module("%s.%s" % (parts[0], parts[1]))
            cls = getattr(module, parts[2])
        except (ImportError, AttributeError):
            cls = None
    _MSG_CLASS_CACHE[schema_name] = cls
    return cls


def parse_cdr_header_stamp(data):
    """header.stamp (ns) straight from a little-endian CDR payload whose
    first field is std_msgs/Header, or None when it is not that shape.

    The 4-byte encapsulation header is followed by the stamp (int32 sec,
    uint32 nanosec) — the same offsets decode_livox_custom_cdr reads.
    Anything else (big-endian encapsulation, truncated payload) returns
    None and the caller falls back to full deserialization.
    """
    head = bytes(data[:12])
    if len(head) < 12 or head[0] != 0x00 or head[1] != 0x01:
        return None
    import struct
    sec, nsec = struct.unpack_from("<iI", head, 4)
    return int(sec) * 1_000_000_000 + int(nsec)


@dataclass
class LivoxCustomScan(object):
    """Decoded Livox CustomMsg. ``points`` uses ``LIDAR_SCAN_FIELDS``."""

    header_stamp_ns: int
    frame_id: str
    timebase: int
    lidar_id: int
    point_num: int
    points: object


def _cdr_align(off, align, origin=4):
    """Pad ``off`` so (off - origin) is a multiple of ``align``."""
    payload = int(off) - int(origin)
    return int(off) + ((int(align) - (payload % int(align))) % int(align))


def decode_livox_custom_cdr(data):
    """Parse a ROS 2 CDR ``livox_ros_driver2/msg/CustomMsg`` payload.

    Alignment is relative to the byte after the 4-byte encapsulation
    header. CustomPoint is 19 payload bytes plus 1 padding byte; a
    truncated final pad byte is tolerated.
    """
    import struct

    data = bytes(data)
    if len(data) < 24 or data[0] != 0x00 or data[1] != 0x01:
        return None
    off = 4
    sec, nsec = struct.unpack_from("<iI", data, off)
    off += 8
    slen = struct.unpack_from("<I", data, off)[0]
    off += 4
    if slen < 1 or off + slen > len(data):
        return None
    frame = data[off:off + slen - 1].decode("utf-8", errors="replace")
    off += slen
    off = _cdr_align(off, 8)
    if off + 16 > len(data):
        return None
    timebase = struct.unpack_from("<Q", data, off)[0]
    off += 8
    point_num = struct.unpack_from("<I", data, off)[0]
    off += 4
    lidar_id = struct.unpack_from("<B", data, off)[0]
    off += 1 + 3  # lidar_id + rsvd[3]
    off = _cdr_align(off, 4)
    if off + 4 > len(data):
        return None
    nseq = struct.unpack_from("<I", data, off)[0]
    off += 4
    n = int(nseq)
    if n < 0 or n > 2_000_000:
        return None
    need = n * 20
    remain = len(data) - off
    if remain < max(0, n * 19):
        return None
    padded = data[off:off + need]
    if len(padded) < need:
        padded = padded + (b"\x00" * (need - len(padded)))
    raw_dtype = np.dtype({
        "names": ["offset_time", "x", "y", "z", "reflectivity",
                  "tag", "line", "_pad"],
        "formats": ["<u4", "<f4", "<f4", "<f4", "u1", "u1", "u1", "u1"],
        "offsets": [0, 4, 8, 12, 16, 17, 18, 19],
        "itemsize": 20,
    })
    raw = np.frombuffer(padded, dtype=raw_dtype, count=n)
    out = np.empty(n, dtype=np.dtype([
        ("x", "f4"), ("y", "f4"), ("z", "f4"), ("intensity", "f4"),
        ("tag", "u1"), ("line", "u1"), ("timestamp", "f8"),
    ]))
    out["x"] = raw["x"]
    out["y"] = raw["y"]
    out["z"] = raw["z"]
    out["intensity"] = raw["reflectivity"].astype(np.float32)
    out["tag"] = raw["tag"]
    out["line"] = raw["line"]
    out["timestamp"] = np.float64(timebase) + raw["offset_time"].astype(
        np.float64)
    return LivoxCustomScan(
        header_stamp_ns=int(sec) * 1_000_000_000 + int(nsec),
        frame_id=frame,
        timebase=int(timebase),
        lidar_id=int(lidar_id),
        point_num=int(point_num),
        points=out,
    )


@dataclass
class BagMessage(object):
    """One deserialized bag record. ``header_stamp_ns`` is the sensor stamp
    used for joins; ``log_time_ns`` is the recorder receive time
    (typically 10-15 ms later)."""

    topic: str
    header_stamp_ns: int
    log_time_ns: int
    message: object


@dataclass
class BagScan(object):
    """Per-topic bag summary from the mcap summary section (no payload
    scan). ``skipped_topics`` lists everything outside TOPIC_TYPES."""

    bag_path: str
    mcap_path: str
    topics: dict = field(default_factory=dict)
    skipped_topics: dict = field(default_factory=dict)


def find_mcap_file(bag_path):
    """Accept a bag directory or a direct .mcap path; error when missing
    or ambiguous."""
    bag_path = os.path.abspath(os.path.expanduser(str(bag_path)))
    if os.path.isfile(bag_path):
        return bag_path
    if not os.path.isdir(bag_path):
        raise FileNotFoundError("bag path not found: %s" % bag_path)
    mcaps = sorted(
        name for name in os.listdir(bag_path) if name.endswith(".mcap"))
    if not mcaps:
        raise FileNotFoundError("no .mcap file under %s" % bag_path)
    if len(mcaps) > 1:
        raise ValueError(
            "multiple .mcap files under %s (%s); pass the file directly"
            % (bag_path, ", ".join(mcaps)))
    return os.path.join(bag_path, mcaps[0])


def scan_bag(bag_path):
    """Summarize topics/counts from the mcap summary section."""
    mcap_path = find_mcap_file(bag_path)
    with open(mcap_path, "rb") as handle:
        summary = make_reader(handle).get_summary()
    if summary is None:
        raise ValueError("mcap has no summary section: %s" % mcap_path)
    counts = dict(summary.statistics.channel_message_counts or {})
    scan = BagScan(bag_path=bag_path, mcap_path=mcap_path)
    for channel in summary.channels.values():
        schema = summary.schemas.get(channel.schema_id)
        type_name = schema.name if schema else ""
        count = int(counts.get(channel.id, 0))
        entry = {"msg_type": type_name, "message_count": count}
        if channel.topic in TOPIC_TYPES:
            scan.topics[channel.topic] = entry
        else:
            scan.skipped_topics[channel.topic] = entry
    return scan


_CHANNEL_TYPES_CACHE = {}


def _channel_types(mcap_path, reader):
    """{channel_id: (topic, schema_name)} from the summary, memoized.

    A replay opens the mcap several times (scan_bag plus one stream per
    pass); the summary section parse is pure overhead after the first.
    Keyed on (realpath, size, mtime) — a bag is immutable, so identity
    holds for the process lifetime.
    """
    try:
        stat = os.stat(mcap_path)
        key = (os.path.realpath(mcap_path), stat.st_size, stat.st_mtime_ns)
    except OSError:
        key = None
    if key is not None and key in _CHANNEL_TYPES_CACHE:
        return _CHANNEL_TYPES_CACHE[key]
    summary = reader.get_summary()
    if summary is None:
        raise ValueError("mcap has no summary section: %s" % mcap_path)
    channel_types = {}
    for channel in summary.channels.values():
        schema = summary.schemas.get(channel.schema_id)
        channel_types[channel.id] = (
            channel.topic, schema.name if schema else "")
    if key is not None:
        if len(_CHANNEL_TYPES_CACHE) >= 32:
            _CHANNEL_TYPES_CACHE.clear()
        _CHANNEL_TYPES_CACHE[key] = channel_types
    return channel_types


def _is_livox_custom(schema_name, topic):
    return schema_name == LIVOX_CUSTOM_SCHEMA or (
        topic == LIDAR_TOPIC and str(schema_name).endswith("CustomMsg"))


def iter_bag_messages(bag_path, topics=None, header_only_topics=None):
    """Stream (BagMessage) records for the requested topics.

    ``topics`` defaults to every TOPIC_TYPES topic present in the bag.
    Topics whose type cannot be resolved are skipped (counted by scan_bag,
    not here). The mcap reader iterates chunk-by-chunk, so a 1 GB bag never
    materializes in memory.

    ``header_only_topics`` suppresses full CDR deserialization for those
    topics: the stamp is parsed straight from the encapsulated bytes and
    ``message`` is None. Only header-first schemas (and Livox CustomMsg,
    same header layout) qualify; anything else falls back to the full
    path. Pass A reads stamps only — deserializing every image just to
    read 8 stamp bytes is the largest avoidable cost of the first pass.
    """
    mcap_path = find_mcap_file(bag_path)
    requested = list(topics) if topics else [
        topic for topic in TOPIC_TYPES]
    header_only = set(header_only_topics or ())
    with open(mcap_path, "rb") as handle:
        reader = make_reader(handle)
        channel_types = _channel_types(mcap_path, reader)
        for _schema, channel, message in reader.iter_messages(
                topics=requested):
            _topic, schema_name = channel_types[channel.id]
            fast_stamp = None
            if channel.topic in header_only:
                if (schema_name in HEADER_FIRST_SCHEMAS
                        or _is_livox_custom(schema_name, channel.topic)):
                    fast_stamp = parse_cdr_header_stamp(message.data)
            if fast_stamp is not None:
                yield BagMessage(
                    topic=channel.topic,
                    header_stamp_ns=fast_stamp,
                    log_time_ns=int(message.log_time),
                    message=None,
                )
                continue
            if _is_livox_custom(schema_name, channel.topic):
                scan = decode_livox_custom_cdr(message.data)
                if scan is None:
                    continue
                yield BagMessage(
                    topic=channel.topic,
                    header_stamp_ns=int(scan.header_stamp_ns),
                    log_time_ns=int(message.log_time),
                    message=scan,
                )
                continue
            cls = _msg_class(schema_name or TOPIC_TYPES.get(channel.topic, ""))
            if cls is None:
                continue
            msg = deserialize_message(message.data, cls)
            stamp = getattr(msg, "header", None)
            stamp = getattr(stamp, "stamp", None) if stamp is not None else None
            header_ns = (int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
                         if stamp is not None else int(message.log_time))
            yield BagMessage(
                topic=channel.topic,
                header_stamp_ns=header_ns,
                log_time_ns=int(message.log_time),
                message=msg,
            )


def decode_color_rgb(msg):
    """Contiguous HxWx3 RGB uint8 copy (bgr8 converted), or None for an
    unsupported layout. Copies because the adapter view is read-only and
    backed by the CDR buffer."""
    view = image_array_from_msg(msg)
    if view is None or view.ndim != 3:
        return None
    rgb = np.ascontiguousarray(view)
    if str(msg.encoding) == "bgr8":
        rgb = rgb[:, :, ::-1]
        rgb = np.ascontiguousarray(rgb)
    return rgb


def decode_compressed_color_rgb(msg):
    """Decode ``sensor_msgs/CompressedImage`` JPEG/PNG to HxWx3 RGB."""
    import cv2
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if buf.size == 0:
        return None
    bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    if bgr is None or bgr.ndim != 3:
        return None
    return np.ascontiguousarray(bgr[:, :, ::-1])


def decode_depth_mm(msg):
    """HxW uint16-family millimetre array (endian-preserving view), or
    None for an unsupported layout."""
    return depth_array_from_msg(msg)


def decode_compressed_depth_mm(msg):
    """Decode compressed aligned depth (16-bit PNG) to HxW millimetres."""
    import cv2
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if buf.size == 0:
        return None
    img = cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)
    if img is None or img.ndim != 2:
        return None
    return np.ascontiguousarray(img)


def decode_color_message(msg):
    """Image or CompressedImage -> RGB uint8, or None."""
    if hasattr(msg, "encoding"):
        return decode_color_rgb(msg)
    if hasattr(msg, "format"):
        return decode_compressed_color_rgb(msg)
    return None


def decode_depth_message(msg):
    """Image or CompressedImage -> millimetre depth, or None."""
    if hasattr(msg, "encoding"):
        return decode_depth_mm(msg)
    if hasattr(msg, "format"):
        return decode_compressed_depth_mm(msg)
    return None


def select_image_topics(scan):
    """Prefer raw Image topics; fall back to the site compressed pair.

    Returns ``(color_topic, depth_topic)``. Raises if neither layout is
    present. Raw wins when a bag has both so Humble Image decoders stay
    on the uncompressed path.
    """
    topics = set(scan.topics)
    if COLOR_TOPIC in topics:
        color = COLOR_TOPIC
    elif COLOR_COMPRESSED_TOPIC in topics:
        color = COLOR_COMPRESSED_TOPIC
    else:
        color = None
    if DEPTH_TOPIC in topics:
        depth = DEPTH_TOPIC
    elif DEPTH_COMPRESSED_TOPIC in topics:
        depth = DEPTH_COMPRESSED_TOPIC
    else:
        depth = None
    if color is None or depth is None:
        raise ValueError(
            "bag has no colour/depth pair (raw or compressed): %s"
            % sorted(topics))
    return color, depth


def decode_cloud_xyz(msg):
    """(N, 3) float64 XYZ from a PointCloud2, or None."""
    return cloud_points_from_msg(msg)


# Livox Mid360 PointXYZRTLT (PointCloud2, xfer_format 0), point_step 26.
# ``timestamp`` is an absolute unix-epoch nanosecond count carried as
# FLOAT64 (resolution ~0.25 ns at 1.8e18) — the per-point time a deskew
# pass needs. tag/line are usually 0 with this driver config but are
# archived faithfully.
LIDAR_SCAN_FIELDS = (
    ("x", "f4"), ("y", "f4"), ("z", "f4"), ("intensity", "f4"),
    ("tag", "u1"), ("line", "u1"), ("timestamp", "f8"),
)


def decode_lidar_scan(msg):
    """Structured (N,) array with all seven livox scan fields, or None.

    Accepts PointCloud2 PointXYZRTLT (xfer_format 0) or a decoded
    ``LivoxCustomScan`` (xfer_format 1). Access columns by name:
    ``pts['x']``, ``pts['timestamp']``, ...
    """
    if isinstance(msg, LivoxCustomScan):
        pts = getattr(msg, "points", None)
        if pts is None or getattr(pts, "size", 0) == 0:
            return None
        return pts

    import numpy as _np
    from sensor_msgs.msg import PointField

    if msg.width <= 0 or msg.height <= 0 or msg.point_step <= 0:
        return None
    fields = {field.name: field for field in msg.fields}
    type_map = {
        ("u1",): PointField.UINT8, ("f4",): PointField.FLOAT32,
        ("f8",): PointField.FLOAT64,
    }
    wanted = {}
    for name, fmt in LIDAR_SCAN_FIELDS:
        field = fields.get(name)
        if field is None or field.count != 1:
            return None
        if field.datatype != type_map[(fmt,)]:
            return None
        wanted[name] = (fmt, int(field.offset))
    step = int(msg.point_step)
    n = int(msg.width) * int(msg.height)
    if len(msg.data) < n * step:
        return None
    dtype = _np.dtype({
        "names": [name for name, _fmt in LIDAR_SCAN_FIELDS],
        "formats": [fmt for _name, fmt in LIDAR_SCAN_FIELDS],
        "offsets": [wanted[name][1] for name, _fmt in LIDAR_SCAN_FIELDS],
        "itemsize": step,
    })
    return _np.frombuffer(msg.data, dtype=dtype, count=n)
