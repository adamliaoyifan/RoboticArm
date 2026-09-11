#!/usr/bin/env python3
"""ChArUco eye-in-hand capture (live or replay). Joints from CPS, never executor /joint_states."""

from __future__ import division

import argparse
import json
import math
import os
import shutil
import sys
import time

_PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)

from luggage_perception.charuco_board import (  # noqa: E402
    apply_overrides,
    default_config_path,
    detect_charuco,
    draw_detection,
    load_spec,
)


def read_cps_joints_deg(host="192.168.0.10", port=10003):
    sdk = os.environ.get(
        "HUAYAN_SDK_PATH",
        "/home/adamliao/work/RoboticArm/third_party/huayan_python_sdk",
    )
    if sdk not in sys.path:
        sys.path.insert(0, sdk)
    from CPS import CPSClient
    client = CPSClient()
    nret = client.HRIF_Connect(0, host, int(port))
    if nret != 0:
        raise RuntimeError("HRIF_Connect failed nRet=%s" % nret)
    try:
        result = []
        nret = client.HRIF_ReadActACS(0, 0, result)
        if nret != 0 or not result:
            result = []
            nret = client.HRIF_ReadActJointPos(0, 0, result)
        if nret != 0:
            raise RuntimeError("HRIF_ReadActACS/JointPos failed nRet=%s" % nret)
        return [float(v) for v in result[:6]]
    finally:
        client.HRIF_DisConnect(0)


def read_joint_states_rad(timeout_sec=2.0):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    owned = not rclpy.ok()
    if owned:
        rclpy.init()
    node = Node("handeye_joint_states")
    box = {"msg": None}

    def _cb(msg):
        box["msg"] = msg

    node.create_subscription(JointState, "/joint_states", _cb, 10)
    deadline = time.time() + timeout_sec
    try:
        while rclpy.ok() and box["msg"] is None and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if owned and rclpy.ok():
            rclpy.shutdown()
    msg = box["msg"]
    if msg is None:
        raise RuntimeError("no /joint_states (start cps_telemetry)")
    by_name = dict(zip(list(msg.name), list(msg.position)))
    names = [
        "elfin_joint1", "elfin_joint2", "elfin_joint3",
        "elfin_joint4", "elfin_joint5", "elfin_joint6",
    ]
    if all(n in by_name for n in names):
        return [float(by_name[n]) for n in names]
    if len(msg.position) >= 6:
        return [float(v) for v in msg.position[:6]]
    raise RuntimeError("joint_states missing elfin_joint1..6")


def lookup_T_base_flange(base_frame, flange_frame, timeout_sec=2.0):
    import numpy as np
    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener

    owned = not rclpy.ok()
    if owned:
        rclpy.init()
    node = Node("handeye_tf_lookup")
    buf = Buffer()
    TransformListener(buf, node)
    deadline = time.time() + timeout_sec
    try:
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if buf.can_transform(base_frame, flange_frame, rclpy.time.Time()):
                break
        tf = buf.lookup_transform(
            base_frame, flange_frame, rclpy.time.Time(),
            timeout=Duration(seconds=0.5))
    finally:
        node.destroy_node()
        if owned and rclpy.ok():
            rclpy.shutdown()
    t = tf.transform.translation
    q = tf.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = [t.x, t.y, t.z]
    return T.tolist()


def _image_msg_to_bgr(msg):
    import cv2
    import numpy as np
    h, w = int(msg.height), int(msg.width)
    if msg.encoding in ("bgr8", "rgb8"):
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(h, w, 3)
        if msg.encoding == "rgb8":
            return arr[:, :, ::-1].copy()
        return arr.copy()
    if msg.encoding in ("mono8",):
        gray = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(h, w)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    raise RuntimeError("unsupported encoding %s" % msg.encoding)


def _camera_info_to_k_d(msg):
    k = list(msg.k)
    dist = [float(v) for v in msg.d] if msg.d else [0.0, 0.0, 0.0, 0.0, 0.0]
    matrix = [
        [float(k[0]), float(k[1]), float(k[2])],
        [float(k[3]), float(k[4]), float(k[5])],
        [float(k[6]), float(k[7]), float(k[8])],
    ]
    return matrix, dist


def _qos_profiles():
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    # d555_calib publishes DEFAULT = RELIABLE + VOLATILE. TRANSIENT_LOCAL
    # does not match and only produces the DURABILITY warning.
    default = QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=20,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    sensor = QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=20,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    image_hw = QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=20,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    return default, sensor, image_hw


def grab_color_and_info(color_topic, info_topic, compressed_topic, timeout_sec):
    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, CompressedImage, Image

    owned = not rclpy.ok()
    if owned:
        rclpy.init()
    node = Node("handeye_grab")
    box = {"image": None, "info": None, "compressed": None, "jpeg": False,
           "topic": None}
    default, sensor, image_hw = _qos_profiles()

    def on_raw(msg, topic=None):
        if box["image"] is None:
            box["image"] = msg
            box["topic"] = topic
            box["jpeg"] = False

    def on_info(msg):
        box["info"] = msg

    def on_compressed(msg, topic=None):
        if box["compressed"] is None:
            box["compressed"] = msg
            box["topic"] = topic

    topics = [color_topic]
    alt = None
    if color_topic.endswith("/image_raw"):
        alt = color_topic[:-len("/image_raw")] + "/image_hw"
    elif color_topic.endswith("/image_hw"):
        alt = color_topic[:-len("/image_hw")] + "/image_raw"
    if alt and alt not in topics:
        topics.append(alt)
    info_topics = [info_topic]
    if "/camera_info" in info_topic:
        hw_info = info_topic.replace("/camera_info", "/camera_info_hw")
        if hw_info not in info_topics:
            info_topics.append(hw_info)
    for topic in topics:
        for qos in (default, sensor, image_hw):
            node.create_subscription(
                Image, topic, lambda msg, t=topic: on_raw(msg, t), qos)
    for topic in info_topics:
        for qos in (default, sensor):
            node.create_subscription(CameraInfo, topic, on_info, qos)
    if compressed_topic:
        for qos in (default, sensor, image_hw):
            node.create_subscription(
                CompressedImage, compressed_topic,
                lambda msg, t=compressed_topic: on_compressed(msg, t), qos)
        hw_jpg = compressed_topic.replace("/image_raw/compressed",
                                          "/image_hw/compressed")
        if hw_jpg != compressed_topic:
            for qos in (default, sensor, image_hw):
                node.create_subscription(
                    CompressedImage, hw_jpg,
                    lambda msg, t=hw_jpg: on_compressed(msg, t), qos)
    deadline = time.time() + timeout_sec
    bgr = None
    try:
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if box["image"] is not None:
                bgr = _image_msg_to_bgr(box["image"])
                box["jpeg"] = False
                break
            if box["compressed"] is not None:
                decoded = cv2.imdecode(
                    np.frombuffer(bytes(box["compressed"].data), dtype=np.uint8),
                    cv2.IMREAD_COLOR)
                if decoded is not None:
                    bgr = decoded
                    box["jpeg"] = True
                    break
        while rclpy.ok() and box["info"] is None and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if owned and rclpy.ok():
            rclpy.shutdown()
    if bgr is None:
        raise RuntimeError(
            "no image on %s / image_hw (tried RELIABLE+VOLATILE then "
            "BEST_EFFORT then TRANSIENT_LOCAL)."
            % color_topic)
    cam = dist = None
    if box["info"] is not None:
        cam, dist = _camera_info_to_k_d(box["info"])
    return bgr, cam, dist, bool(box["jpeg"])


def _read_joints(args):
    if args.joints_from_topic:
        joints_rad = read_joint_states_rad()
        return {
            "joints_deg": [math.degrees(v) for v in joints_rad],
            "joints_rad": joints_rad,
            "joint_source": "/joint_states (require cps_telemetry, not executor zeros)",
        }
    joints_deg = read_cps_joints_deg(args.cps_host, args.cps_port)
    return {
        "joints_deg": joints_deg,
        "joints_rad": [math.radians(v) for v in joints_deg],
        "joint_source": "CPS HRIF_ReadActACS (deg)",
    }


def _maybe_tf(args):
    if not args.lookup_tf:
        return None, None
    try:
        return lookup_T_base_flange(args.base_frame, args.flange_frame), None
    except Exception as exc:
        return None, str(exc)


def write_pose_record(out_dir, pose_id, image_bgr, spec, det, extra):
    import cv2
    os.makedirs(out_dir, exist_ok=True)
    dest_img = os.path.join(out_dir, "pose_%s.png" % pose_id)
    cv2.imwrite(dest_img, image_bgr)
    overlay = draw_detection(image_bgr, det)
    dest_overlay = os.path.join(out_dir, "pose_%s_overlay.png" % pose_id)
    cv2.imwrite(dest_overlay, overlay)
    rec = {
        "pose_id": pose_id,
        "image": os.path.basename(dest_img),
        "overlay": os.path.basename(dest_overlay),
        "detection": det,
        "board": spec,
        "square_length_m": spec["square_length_m"],
        "marker_length_m": spec["marker_length_m"],
        "note": "Do not use executor /joint_states; HB-3 recorded all-zero states",
    }
    rec.update(extra)
    dest_json = os.path.join(out_dir, "pose_%s.json" % pose_id)
    with open(dest_json, "w", encoding="utf-8") as handle:
        json.dump(rec, handle, indent=2)
        handle.write("\n")
    return dest_json, rec


def replay_dir(path, spec):
    import cv2
    poses = []
    names = sorted(
        n for n in os.listdir(path)
        if n.startswith("pose_") and n.endswith(".json") and "_overlay" not in n
    )
    for name in names:
        meta_path = os.path.join(path, name)
        with open(meta_path, "r", encoding="utf-8") as handle:
            meta = json.load(handle)
        image_path = meta.get("image") or name.replace(".json", ".png")
        if not os.path.isabs(image_path):
            image_path = os.path.join(path, image_path)
        image = cv2.imread(image_path, cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError("missing image %s" % image_path)
        det = detect_charuco(image, spec)
        rec = dict(meta)
        rec["detection"] = det
        rec["board"] = spec
        rec["square_length_m"] = spec["square_length_m"]
        rec["marker_length_m"] = spec["marker_length_m"]
        rec["source"] = "replay"
        poses.append(rec)
    out = os.path.join(path, "capture_index.json")
    with open(out, "w", encoding="utf-8") as handle:
        json.dump({"poses": poses, "board": spec,
                   "square_length_m": spec["square_length_m"],
                   "marker_length_m": spec["marker_length_m"]}, handle, indent=2)
        handle.write("\n")
    return out


def live_once(args, spec):
    time.sleep(max(0.0, args.settle_sec))
    image, cam, dist, jpeg = grab_color_and_info(
        args.color_topic, args.camera_info_topic, args.compressed_topic,
        args.grab_timeout_sec)
    det = detect_charuco(image, spec)
    extra = {"jpeg_source": jpeg, "camera_matrix": cam, "dist": dist}
    if jpeg:
        extra["jpeg_warning"] = (
            "JPEG source; prefer publish_raw:=true for subpixel corners")
    try:
        extra.update(_read_joints(args))
        extra["joints_error"] = None
    except Exception as exc:
        extra["joints_deg"] = None
        extra["joints_rad"] = None
        extra["joint_source"] = "unavailable: %s" % exc
    T, tf_err = _maybe_tf(args)
    if T is not None:
        extra["T_base_flange"] = T
        extra["flange_frame"] = args.flange_frame
        extra["base_frame"] = args.base_frame
    elif args.lookup_tf:
        extra["tf_error"] = tf_err
    path, rec = write_pose_record(args.out_dir, args.pose_id, image, spec, det, extra)
    print(path)
    count = 0 if det is None else det["count"]
    print("corners=%s min=%s jpeg=%s" % (count, spec["min_corners"], jpeg))
    return 0 if det and count >= spec["min_corners"] else 1


def probe_image(image_path, spec, overlay_path=None):
    import cv2
    image = cv2.imread(image_path, cv2.IMREAD_COLOR)
    if image is None:
        raise SystemExit("cannot read %s" % image_path)
    det = detect_charuco(image, spec)
    if overlay_path:
        cv2.imwrite(overlay_path, draw_detection(image, det))
    print(json.dumps({
        "image": image_path,
        "board": spec,
        "detection": None if det is None else {
            "count": det["count"],
            "marker_count": det["marker_count"],
            "corner_ids": det["corner_ids"][:16],
            "marker_ids": det["marker_ids"][:16],
        },
        "interior_corners_expected": (
            (spec["squares_x"] - 1) * (spec["squares_y"] - 1)),
    }, indent=2))
    return 0 if det and det["count"] >= spec["min_corners"] else 1


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--board-config", default=default_config_path())
    parser.add_argument("--square-length-m", type=float, default=None,
                        help="Caliper 5-square pitch / 5, metres. Default: YAML 0.040")
    parser.add_argument("--marker-length-m", type=float, default=None)
    parser.add_argument("--replay")
    parser.add_argument("--out-dir")
    parser.add_argument("--pose-id")
    parser.add_argument("--image")
    parser.add_argument("--probe", action="store_true",
                        help="Detect one image and print counts; do not write a pose")
    parser.add_argument("--overlay")
    parser.add_argument("--live", action="store_true")
    parser.add_argument("--lookup-tf", action="store_true",
                        help="Record T_base_flange from TF (scene_hardware + cps_telemetry)")
    parser.add_argument("--joints-from-topic", action="store_true",
                        help="Read /joint_states from cps_telemetry (CPS socket is exclusive)")
    parser.add_argument("--cps-host", default="192.168.0.10")
    parser.add_argument("--cps-port", type=int, default=10003)
    parser.add_argument("--base-frame", default="elfin_base_link")
    parser.add_argument("--flange-frame", default="elfin_end_link")
    parser.add_argument("--settle-sec", type=float, default=1.0)
    parser.add_argument("--grab-timeout-sec", type=float, default=8.0)
    parser.add_argument("--color-topic", default="/camera/d555/color/image_raw")
    parser.add_argument("--camera-info-topic",
                        default="/camera/d555/color/camera_info")
    parser.add_argument("--compressed-topic",
                        default="/camera/d555/color/image_raw/compressed")
    args = parser.parse_args(argv)
    spec = apply_overrides(
        load_spec(args.board_config), args.square_length_m, args.marker_length_m)
    if args.replay:
        print(replay_dir(args.replay, spec))
        return 0
    if args.probe and args.image:
        return probe_image(args.image, spec, args.overlay)
    if args.live:
        if not args.out_dir or not args.pose_id:
            parser.error("--live needs --out-dir and --pose-id")
        return live_once(args, spec)
    if args.image and args.out_dir and args.pose_id:
        import cv2
        image = cv2.imread(args.image, cv2.IMREAD_COLOR)
        if image is None:
            raise SystemExit("cannot read %s" % args.image)
        det = detect_charuco(image, spec)
        extra = {}
        try:
            extra.update(_read_joints(args))
        except Exception as exc:
            extra["joint_source"] = "unavailable: %s" % exc
        T, tf_err = _maybe_tf(args)
        if T is not None:
            extra["T_base_flange"] = T
        elif args.lookup_tf:
            extra["tf_error"] = tf_err
        if args.image and os.path.abspath(args.image) != os.path.abspath(
                os.path.join(args.out_dir, "pose_%s.png" % args.pose_id)):
            os.makedirs(args.out_dir, exist_ok=True)
            shutil.copy2(
                args.image, os.path.join(args.out_dir, "pose_%s_src.png" % args.pose_id))
        path, rec = write_pose_record(
            args.out_dir, args.pose_id, image, spec, det, extra)
        print(path)
        count = 0 if rec["detection"] is None else rec["detection"]["count"]
        return 0 if rec["detection"] and count >= spec["min_corners"] else 1
    parser.error("use --probe --image, --live, --replay DIR, or --image/--out-dir/--pose-id")
    return 2


if __name__ == "__main__":
    sys.exit(main())
