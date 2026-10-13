#!/usr/bin/env python3
"""Republish the fat sensor topics at the capture-budget period.

The continuous bag records the throttled names from
``luggage_gazebo.capture_budget``, not the full-rate sources.
"""
from __future__ import annotations

import os
import sys
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "src", "luggage_gazebo"))

from luggage_gazebo.capture_budget import FAT_TOPICS, THROTTLE_PERIOD_S  # noqa: E402


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.utilities import get_message

    rclpy.init()
    node = Node("st09_capture_throttle")
    last = {}
    for source, output, type_name in FAT_TOPICS:
        msg_type = get_message(type_name)
        publisher = node.create_publisher(msg_type, output, qos_profile_sensor_data)

        def callback(message, source=source, publisher=publisher):
            now = time.monotonic()
            if now - last.get(source, 0.0) < THROTTLE_PERIOD_S:
                return
            last[source] = now
            publisher.publish(message)

        node.create_subscription(
            msg_type, source, callback, qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
