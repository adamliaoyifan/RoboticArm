#!/usr/bin/env python3
"""Record the tool side of the release window on the simulation clock.

/joint_states (header stamps = sim time) at full rate; /tf sampled.
One JSONL per message kind, written until killed. Run next to the
release observation: the box pose stream (gz transport) and these
stamps share the same simulation clock.
"""
import json
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage


def main():
    out_path = sys.argv[1]
    rclpy.init()
    node = Node("release_tool_recorder")
    handle = open(out_path, "w", encoding="utf-8")

    def stamp_ns(header):
        return int(header.stamp.sec) * 1_000_000_000 + int(
            header.stamp.nanosec)

    def on_joints(msg):
        handle.write(json.dumps({
            "kind": "joint_states",
            "t_sim_ns": stamp_ns(msg.header),
            "name": list(msg.name),
            "position": list(msg.position),
            "velocity": list(msg.velocity),
        }) + "\n")

    def on_tf(msg):
        rows = []
        for tf in msg.transforms:
            rows.append({
                "parent": tf.header.frame_id,
                "child": tf.child_frame_id,
                "t_sim_ns": stamp_ns(tf.header),
                "translation": [tf.transform.translation.x,
                                tf.transform.translation.y,
                                tf.transform.translation.z],
                "rotation_xyzw": [tf.transform.rotation.x,
                                  tf.transform.rotation.y,
                                  tf.transform.rotation.z,
                                  tf.transform.rotation.w],
            })
        if rows:
            handle.write(json.dumps({"kind": "tf", "rows": rows}) + "\n")

    best = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST)
    node.create_subscription(JointState, "/joint_states", on_joints, best)
    node.create_subscription(TFMessage, "/tf", on_tf, best)

    import time
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.2)
            handle.flush() if int(time.time() * 5) % 5 == 0 else None
    except KeyboardInterrupt:
        pass
    finally:
        handle.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
