#!/usr/bin/env python3
"""In-process box-contact recorder for the A/B runs (harness fix).

Run as: record_box_stream.py <out.jsonl> <model-substring>

The recorder bridges nothing itself — spawn_contact_bridge.py provides
the /obs/box_contact gz->ROS bridge; this node subscribes ROS-side and
writes only contacts involving `model`. The unfiltered gz echo that
previously captured contacts produced 4.8 MB/s of pipe traffic whose
delivery contention reordered the pose echo (run-01 invalidation).

Provenance note (2026-10-19 defect): the first deployed version
unpacked THREE argv slots while the runner passed TWO — it crashed at
startup and every run's box_contacts.jsonl was silently absent. The
file is now created eagerly and a startup banner is printed so its
absence is diagnosable in replay/launch logs.
"""
import json
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from ros_gz_interfaces.msg import Contacts


def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: record_box_stream.py <out.jsonl> <model>")
    out_contact, model = sys.argv[1:3]
    rclpy.init()
    node = Node("ab_box_recorder")
    contact_h = open(out_contact, "w", encoding="utf-8")
    contact_h.write(json.dumps({"recorder": "start", "model": model}) + "\n")
    contact_h.flush()
    print("[ab_box_recorder] start model=%s -> %s" % (model, out_contact),
          flush=True)

    best = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST)

    def stamp_ns(header):
        return int(header.stamp.sec) * 1_000_000_000 + int(
            header.stamp.nanosec)

    def on_contacts(msg):
        for contact in msg.contacts:
            names = [c.name for c in
                     (contact.collision1, contact.collision2)
                     if c is not None and c.name]
            if not any(model in n for n in names):
                continue
            contact_h.write(json.dumps({
                "t_sim_ns": stamp_ns(contact.header),
                "partners": [n for n in names if model not in n],
                "positions": [[p.x, p.y, p.z]
                              for p in contact.positions],
            }) + "\n")
        contact_h.flush()

    node.create_subscription(Contacts, "/obs/box_contact",
                            on_contacts, best)
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        contact_h.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
