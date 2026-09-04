#!/usr/bin/env python3
"""Publish zero JointState for every non-fixed URDF joint.

Humble robot_state_publisher only emits revolute/prismatic TF after
/joint_states. This is not ros2_control; it holds the arm at URDF zeros so
suction/camera frames exist on the TF tree.
"""

import xml.etree.ElementTree as ET

from luggage_description.xacro_robot_with_scene_base import nonfixed_joint_names


class ZeroJointStatePublisher:
    def __init__(self, node):
        self._node = node
        node.declare_parameter("robot_description", "")
        urdf = node.get_parameter("robot_description").value
        if not urdf:
            raise RuntimeError("robot_description is empty")
        self._names = nonfixed_joint_names(ET.fromstring(urdf))
        from sensor_msgs.msg import JointState

        self._pub = node.create_publisher(JointState, "joint_states", 10)
        self._msg = JointState()
        self._msg.name = list(self._names)
        self._msg.position = [0.0] * len(self._names)
        self._msg.velocity = [0.0] * len(self._names)
        self._msg.effort = [0.0] * len(self._names)
        node.create_timer(0.05, self._tick)
        node.get_logger().info(
            "Publishing zero joint_states for %d joints" % len(self._names)
        )

    def _tick(self):
        self._msg.header.stamp = self._node.get_clock().now().to_msg()
        self._pub.publish(self._msg)


def main(args=None):
    import rclpy
    from rclpy.node import Node

    rclpy.init(args=args)
    node = Node("zero_joint_state_publisher")
    ZeroJointStatePublisher(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
