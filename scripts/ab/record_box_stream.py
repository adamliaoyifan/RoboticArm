#!/usr/bin/env python3
"""In-process filtered capture for the A/B runs (harness fix 2026-10-19).

The gz `topic -e` file echo reordered messages under the 4.8 MB/s load
of unfiltered streams (3 backward stamp jumps in run 01's 65 s window —
the validity gate correctly invalidated it). This recorder bridges the
two gz topics INTO the process and writes only the experiment's samples
(box pose + box contacts), cutting written volume ~50x; the bridge is
the same production path the camera stream uses.
"""
import json
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from ros_gz_interfaces.msg import Contacts
from ros_gz_interfaces.msg import EntityFactory  # noqa: F401 (type probe)


def main():
    out_pose, out_contact, model = sys.argv[1:4]
    rclpy.init()
    node = Node("ab_box_recorder")
    pose_h = open(out_pose, "w", encoding="utf-8")
    contact_h = open(out_contact, "w", encoding="utf-8")

    best = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST)

    def stamp_ns(header):
        return int(header.stamp.sec) * 1_000_000_000 + int(
            header.stamp.nanosec)

    def on_contacts(msg):
        rows = []
        for contact in msg.contacts:
            names = [c.name for c in
                     (contact.collision1, contact.collision2) if c is not None and c.name]
            if not any(model in n for n in names):
                continue
            rows.append({
                "t_sim_ns": stamp_ns(contact.header),
                "partners": [n for n in names if model not in n],
                "positions": [[p.x, p.y, p.z] for p in contact.positions],
            })
        for row in rows:
            contact_h.write(json.dumps(row) + "\n")
        contact_h.flush()

    node.create_subscription(Contacts, "/obs/box_contact",
                            on_contacts, best)
    import time
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        pose_h.close()
        contact_h.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
