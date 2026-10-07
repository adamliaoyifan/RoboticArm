#!/usr/bin/env python3
"""Compare the live world->container_link static TF with a scene file.

Exit 0 when the live transform equals the scene file's container_link entry
(translation within 1 mm, yaw within 0.001 rad); exit 1 when it differs or no
transform arrives; exit 2 on usage errors. Prints one JSON line.

This guards against a stack that runs the deployment layout while a driver
plans on another scene file, or the reverse.

  check_live_layout.py --scene scene_tf_deploy.yaml [--timeout 20]
"""
import argparse
import json
import math
import sys
import time

import yaml

TRANSLATION_TOL_M = 1e-3
YAW_TOL_RAD = 1e-3


def expected_container_pose(scene_path):
    with open(scene_path, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    for entry in config.get("static_transforms", []):
        if entry.get("child") == "container_link":
            return (list(entry["translation"]),
                    float(entry["rotation_rpy"][2]))
    raise ValueError("no container_link static transform in %s" % scene_path)


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def angle_difference(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def compare(expected, live):
    """Return (ok, detail) for expected and live (translation, yaw)."""
    translation_error = max(abs(a - b) for a, b in zip(expected[0], live[0]))
    yaw_error = angle_difference(expected[1], live[1])
    ok = translation_error <= TRANSLATION_TOL_M and yaw_error <= YAW_TOL_RAD
    return ok, {"translation_error_m": round(translation_error, 6),
                "yaw_error_rad": round(yaw_error, 6)}


def read_live_pose(timeout_s):
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    node = rclpy.create_node("check_live_layout")
    found = {}

    def on_tf(message):
        for transform in message.transforms:
            if (transform.header.frame_id == "world"
                    and transform.child_frame_id == "container_link"):
                t, q = transform.transform.translation, transform.transform.rotation
                found["pose"] = ([t.x, t.y, t.z],
                                 yaw_from_quaternion(q.x, q.y, q.z, q.w))

    qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(TFMessage, "/tf_static", on_tf, qos)
    deadline = time.monotonic() + timeout_s
    while "pose" not in found and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    return found.get("pose")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", required=True)
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args(argv)
    try:
        expected = expected_container_pose(args.scene)
    except (OSError, ValueError, KeyError) as error:
        print(json.dumps({"ok": False, "reason": "SCENE_UNREADABLE",
                          "detail": str(error)}))
        return 2
    live = read_live_pose(args.timeout)
    if live is None:
        print(json.dumps({"ok": False, "reason": "NO_LIVE_TRANSFORM",
                          "expected": expected, "scene": args.scene}))
        return 1
    ok, detail = compare(expected, live)
    print(json.dumps({"ok": ok, "reason": "" if ok else "LAYOUT_MISMATCH",
                      "expected": expected, "live": live, "scene": args.scene,
                      **detail}))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
