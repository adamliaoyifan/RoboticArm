#!/usr/bin/env python3
"""Generate a tiny synthetic replay fixture bag (mcap, ros2 profile).

The teach-pendant bags cannot be read by Humble's rosbag2 (see
luggage_perception/eval/bag_mcap_source.py), so the reader is exercised
against a synthetic bag built with the low-level mcap writer plus
rclpy CDR serialization — the same record layout the real bags use.

The fixture is regenerated on demand by the tests (nothing binary is
committed) and can also be run standalone to eyeball a bag:

    python3 make_tiny_replay_bag.py [out.mcap]

Frame plan (base +33 ms steps): f0 exact pair, f1 exact pair with a
row-padded color step, a duplicate color at f1's stamp (newer log_time),
f2 color orphan (nearest depth is 5 ms away), f3 depth orphan.
"""
import os
import re
import sys

from mcap.writer import Writer
from rclpy.serialization import serialize_message

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

BASE_NS = 1_788_943_637_000_000_000
FRAME_DT_NS = 33_000_000
WIDTH, HEIGHT = 16, 12

_FIELD_TYPE_RE = re.compile(r"^\s*([a-zA-Z0-9_/]+)\[[^\]]*\]\s+\w+", re.M)
_BUILTIN = {
    "bool", "byte", "char", "int8", "uint8", "int16", "uint16", "int32",
    "uint32", "int64", "uint64", "float32", "float64", "string",
    "wstring", "builtin_interfaces/Time", "builtin_interfaces/Duration",
}


def _msg_path(pkg, type_name):
    prefixes = os.environ.get("AMENT_PREFIX_PATH", "/opt/ros/humble").split(":")
    for prefix in prefixes:
        candidate = os.path.join(prefix, "share", pkg, "msg",
                                 "%s.msg" % type_name)
        if os.path.isfile(candidate):
            return candidate
    raise FileNotFoundError("no share msg for %s/%s" % (pkg, type_name))


def load_msgdef(pkg, type_name, _seen=None):
    """Concatenated ROS2 message definition (msg text + '==== MSG:' deps)."""
    _seen = _seen if _seen is not None else set()
    key = "%s/%s" % (pkg, type_name)
    if key in _seen:
        return ""
    _seen.add(key)
    text = open(_msg_path(pkg, type_name), encoding="utf-8").read()
    parts = [text]
    for match in _FIELD_TYPE_RE.finditer(text):
        token = match.group(1)
        if token in _BUILTIN or "/" not in token:
            continue
        dep_pkg, dep_type = token.split("/", 1)
        dep_text = load_msgdef(dep_pkg, dep_type, _seen)
        if dep_text:
            parts.append("=" * 80 + "\nMSG: %s\n%s" % (token, dep_text))
    return "".join(parts)


def _stamp(msg, ns):
    msg.header.stamp.sec = ns // 1_000_000_000
    msg.header.stamp.nanosec = ns % 1_000_000_000
    return msg


def _color_msg(ns, padding=0, encoding="rgb8"):
    msg = Image()
    _stamp(msg, ns)
    msg.header.frame_id = "d555_color_optical_frame"
    msg.width, msg.height = WIDTH, HEIGHT
    msg.encoding = encoding
    msg.step = WIDTH * 3 + padding
    row = bytes(range(WIDTH * 3))
    msg.data = bytearray((row + b"\x00" * padding) * HEIGHT)
    return msg


def _depth_msg(ns, fill_mm=1500):
    msg = Image()
    _stamp(msg, ns)
    msg.header.frame_id = "d555_color_optical_frame"
    msg.width, msg.height = WIDTH, HEIGHT
    msg.encoding = "16UC1"
    msg.step = WIDTH * 2
    msg.data = bytearray((fill_mm).to_bytes(2, "little") * (WIDTH * HEIGHT))
    return msg


def _camera_info(topic, ns):
    msg = CameraInfo()
    _stamp(msg, ns)
    msg.header.frame_id = "d555_color_optical_frame"
    msg.width, msg.height = WIDTH, HEIGHT
    msg.distortion_model = "plumb_bob"
    msg.k = [200.0, 0.0, 8.0, 0.0, 200.0, 6.0, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [200.0, 0.0, 8.0, 0.0, 0.0, 200.0, 6.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    msg.d = [0.0] * 5
    return msg


def build_fixture(path):
    """Write the tiny fixture bag; returns the output path."""
    if os.path.exists(path):
        os.remove(path)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    schemas, channels, seq = {}, {}, [0]

    def _channel(topic, pkg, type_name):
        if topic not in channels:
            schema_id = None
            datatype = "%s/msg/%s" % (pkg, type_name)
            if datatype not in schemas:
                schema_id = schemas[datatype] = writer.register_schema(
                    name=datatype, encoding="ros2msg",
                    data=load_msgdef(pkg, type_name).encode("utf-8"))
            else:
                schema_id = schemas[datatype]
            channels[topic] = writer.register_channel(
                topic=topic, message_encoding="cdr", schema_id=schema_id)
        return channels[topic]

    def _write(topic, pkg, type_name, msg, hdr_ns, log_ns):
        seq[0] += 1
        writer.add_message(
            channel_id=_channel(topic, pkg, type_name),
            log_time=log_ns, publish_time=hdr_ns,
            data=serialize_message(msg), sequence=seq[0])

    t0, t1, t2, t3 = (BASE_NS + i * FRAME_DT_NS for i in range(4))
    with open(path, "wb") as handle:
        writer = Writer(handle)
        writer.start(profile="ros2", library="tiny_replay_fixture")

        _write("/camera/d555/color/camera_info", "sensor_msgs",
               "CameraInfo", _camera_info("color", t0 - FRAME_DT_NS),
               t0 - FRAME_DT_NS, t0 - FRAME_DT_NS)
        _write("/camera/d555/aligned_depth_to_color/camera_info", "sensor_msgs",
               "CameraInfo", _camera_info("depth", t0 - FRAME_DT_NS),
               t0 - FRAME_DT_NS, t0 - FRAME_DT_NS)

        # f0: exact pair.
        _write("/camera/d555/color/image_raw", "sensor_msgs", "Image",
               _color_msg(t0), t0, t0 + 12_000_000)
        _write("/camera/d555/aligned_depth_to_color/image_raw", "sensor_msgs",
               "Image", _depth_msg(t0), t0, t0 + 13_000_000)
        # f1: exact pair with a row-padded color step (4 pad bytes).
        _write("/camera/d555/color/image_raw", "sensor_msgs", "Image",
               _color_msg(t1, padding=4), t1, t1 + 12_000_000)
        _write("/camera/d555/aligned_depth_to_color/image_raw", "sensor_msgs",
               "Image", _depth_msg(t1), t1, t1 + 13_000_000)
        # duplicate color at f1's stamp with a newer log_time (dedupe case).
        _write("/camera/d555/color/image_raw", "sensor_msgs", "Image",
               _color_msg(t1), t1, t1 + 20_000_000)
        # f2: color orphan (nearest depth is 5 ms away, over a 1 ms join
        # tolerance).
        _write("/camera/d555/color/image_raw", "sensor_msgs", "Image",
               _color_msg(t2), t2, t2 + 12_000_000)
        _write("/camera/d555/aligned_depth_to_color/image_raw", "sensor_msgs",
               "Image", _depth_msg(t2 + 5_000_000), t2 + 5_000_000,
               t2 + 18_000_000)
        # f3: depth orphan (no color at all nearby).
        _write("/camera/d555/aligned_depth_to_color/image_raw", "sensor_msgs",
               "Image", _depth_msg(t3), t3, t3 + 13_000_000)

        joints = JointState()
        _stamp(joints, t0 - 20_000_000)
        joints.name = ["elfin_joint%d" % i for i in range(1, 7)]
        joints.position = [0.1 * i for i in range(6)]
        _write("/joint_states", "sensor_msgs", "JointState", joints,
               t0 - 20_000_000, t0 - 18_000_000)

        tcp = PoseStamped()
        _stamp(tcp, t0 - 20_000_000)
        tcp.header.frame_id = "elfin_base_link"
        tcp.pose.position.x, tcp.pose.position.y = 0.5, 0.1
        _write("/elfin/tcp_pose", "geometry_msgs", "PoseStamped", tcp,
               t0 - 20_000_000, t0 - 18_000_000)

        tf_static = TFMessage()
        tf = TransformStamped()
        _stamp(tf, 0)
        tf.header.frame_id, tf.child_frame_id = "world", "elfin_base_link"
        tf_static.transforms = [tf]
        _write("/tf_static", "tf2_msgs", "TFMessage", tf_static, 0, 0)

        # Unregistered high-rate topic: only counted, never deserialized.
        imu = Imu()
        _stamp(imu, t0)
        _write("/livox/imu", "sensor_msgs", "Imu", imu, t0, t0 + 1_000_000)

        writer.finish()
    return path


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "tiny_replay_bag.mcap"
    print("wrote", build_fixture(out))
