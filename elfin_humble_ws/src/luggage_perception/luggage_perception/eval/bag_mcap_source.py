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
# (/livox/imu at 200 Hz, /elfin/cps_*, livox CustomMsg, ...) is skipped and
# only counted, so a 60k-message bag decodes just the ~8k payloads needed.
TOPIC_TYPES = {
    "/camera/d555/color/image_raw": "sensor_msgs/msg/Image",
    "/camera/d555/aligned_depth_to_color/image_raw": "sensor_msgs/msg/Image",
    "/camera/d555/color/camera_info": "sensor_msgs/msg/CameraInfo",
    "/camera/d555/aligned_depth_to_color/camera_info": "sensor_msgs/msg/CameraInfo",
    "/joint_states": "sensor_msgs/msg/JointState",
    "/elfin/tcp_pose": "geometry_msgs/msg/PoseStamped",
    "/livox/lidar": "sensor_msgs/msg/PointCloud2",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/tf_static": "tf2_msgs/msg/TFMessage",
}

_MSG_CLASS_CACHE = {}


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


def iter_bag_messages(bag_path, topics=None):
    """Stream (BagMessage) records for the requested topics.

    ``topics`` defaults to every TOPIC_TYPES topic present in the bag.
    Topics whose type cannot be resolved are skipped (counted by scan_bag,
    not here). The mcap reader iterates chunk-by-chunk, so a 1 GB bag never
    materializes in memory.
    """
    mcap_path = find_mcap_file(bag_path)
    requested = list(topics) if topics else [
        topic for topic in TOPIC_TYPES]
    with open(mcap_path, "rb") as handle:
        reader = make_reader(handle)
        summary = reader.get_summary()
        if summary is None:
            raise ValueError("mcap has no summary section: %s" % mcap_path)
        channel_types = {}
        for channel in summary.channels.values():
            schema = summary.schemas.get(channel.schema_id)
            channel_types[channel.id] = (
                channel.topic, schema.name if schema else "")
        for _schema, channel, message in reader.iter_messages(
                topics=requested):
            _topic, schema_name = channel_types[channel.id]
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


def decode_depth_mm(msg):
    """HxW uint16-family millimetre array (endian-preserving view), or
    None for an unsupported layout."""
    return depth_array_from_msg(msg)


def decode_cloud_xyz(msg):
    """(N, 3) float64 XYZ from a PointCloud2, or None."""
    return cloud_points_from_msg(msg)
