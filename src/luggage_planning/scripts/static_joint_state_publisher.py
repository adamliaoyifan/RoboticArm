#!/usr/bin/env python3
"""Static /joint_states publisher for offline stacks without a simulator.

Publishes every non-fixed joint from the expanded robot_description at
zero position so MoveIt's current-state monitor stays green while the
deploy assets are sampled (no Gazebo, no hardware). Read-only tooling:
never part of a live cell.
"""

from __future__ import division

import argparse
import os
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def nonfixed_joint_names(urdf_xml):
    import xml.etree.ElementTree as ElementTree
    root = ElementTree.fromstring(urdf_xml)
    names = []
    for element in root.iter():
        tag = element.tag.split("}", 1)[-1]
        if tag != "joint":
            continue
        if (element.get("type") or "fixed").lower() in ("fixed", "unknown"):
            continue
        name = element.get("name")
        if name:
            names.append(name)
    return names


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--xacro", required=True)
    parser.add_argument("--scene-tf", required=True)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    # launch injects --ros-args remapping after our own arguments.
    args, _unknown = parser.parse_known_args()

    # src tree: .. is src/; installed: fall back to ament share.
    here = os.path.dirname(os.path.abspath(__file__))
    src_root = os.path.normpath(os.path.join(here, ".."))
    if src_root not in sys.path:
        sys.path.insert(0, src_root)
    from luggage_description.xacro_robot_with_scene_base import expand_and_patch
    urdf_xml, _pose = expand_and_patch(
        args.xacro, config_path=args.scene_tf, xacro_args=[])
    names = nonfixed_joint_names(urdf_xml)

    rclpy.init()
    node = Node("static_joint_state_publisher")
    publisher = node.create_publisher(JointState, "/joint_states", 10)
    period = 1.0 / max(float(args.rate_hz), 0.1)

    def publish():
        message = JointState()
        message.header.stamp = node.get_clock().now().to_msg()
        message.name = list(names)
        message.position = [0.0] * len(names)
        publisher.publish(message)

    node.create_timer(period, publish)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
