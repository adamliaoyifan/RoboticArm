#!/usr/bin/env python3
"""Thin ROS 2 adapter for the ROS-free reachability atlas builder."""

from __future__ import annotations

import math
import os

from luggage_description.scene_tf_config_utils import (
    container_in_base_link,
    container_inner_geometry_descriptor,
    load_scene_tf_config,
    resolve_scene_tf_config_path,
)

from .atlas_builder import AtlasBuilderKernel, AtlasGrid, PayloadProfile
from .reachability_atlas import REACHABLE, ReachabilityAtlas


def _rpy_matrix(rpy):
    roll, pitch, yaw = [float(value) for value in rpy]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _transform_point(origin, rotation, point):
    return tuple(
        float(origin[row]) + sum(rotation[row][col] * float(point[col])
                                 for col in range(3))
        for row in range(3))


def main(args=None):
    import rclpy
    from moveit_msgs.srv import GetPositionIK
    from rclpy.node import Node

    class AtlasBuilderNode(Node):
        def __init__(self):
            super().__init__("reachability_atlas_builder")
            self.declare_parameter("scene_tf_config", resolve_scene_tf_config_path())
            self.declare_parameter("output_prefix", "")
            self.declare_parameter("resolution_xyz", 0.15)
            self.declare_parameter("yaw_bins", [0.0, math.pi / 2.0])
            self.declare_parameter("payload_size", [])
            self.declare_parameter("ik_service", "/compute_ik")
            self.declare_parameter("ik_group", "elfin_arm")
            self.declare_parameter("ik_link", "suction_contact_frame")
            self.declare_parameter("base_frame", "elfin_base_link")
            self.declare_parameter("avoid_collisions", True)
            self.declare_parameter("ik_timeout_sec", 0.05)
            self.declare_parameter(
                "seed_joints", [0.0, -1.57, 1.57, 0.0, 1.57, 0.0])

            scene_path = str(self.get_parameter("scene_tf_config").value)
            scene = load_scene_tf_config(scene_path)
            self._descriptor = container_inner_geometry_descriptor(scene)
            self._container_origin, container_rpy = container_in_base_link(scene)
            self._container_rotation = _rpy_matrix(container_rpy)
            self._ik_group = str(self.get_parameter("ik_group").value)
            self._ik_link = str(self.get_parameter("ik_link").value)
            self._base_frame = str(self.get_parameter("base_frame").value)
            self._avoid_collisions = bool(
                self.get_parameter("avoid_collisions").value)
            self._ik_timeout = float(
                self.get_parameter("ik_timeout_sec").value)
            self._seed = tuple(float(value) for value in
                               self.get_parameter("seed_joints").value)
            if len(self._seed) != 6:
                raise ValueError("seed_joints must contain six values")
            service = str(self.get_parameter("ik_service").value)
            self._client = self.create_client(GetPositionIK, service)
            if not self._client.wait_for_service(timeout_sec=60.0):
                raise RuntimeError("compute_ik service unavailable")

            resolution = float(self.get_parameter("resolution_xyz").value)
            yaw_bins = tuple(float(value) for value in
                             self.get_parameter("yaw_bins").value)
            payload_size = tuple(float(value) for value in
                                 self.get_parameter("payload_size").value)
            payload = (
                PayloadProfile(enabled=True, size=payload_size).validated()
                if payload_size else PayloadProfile())
            grid = AtlasGrid.covering_geometry(
                self._descriptor, resolution, yaw_bins)
            data, meta = AtlasBuilderKernel(
                self._descriptor, grid, self._solve_ik,
                payload=payload).build()
            prefix = str(self.get_parameter("output_prefix").value).strip()
            if not prefix:
                prefix = os.path.join(os.getcwd(), "reachability_atlas_v3")
            atlas = ReachabilityAtlas.from_builder(meta=meta, **data)
            atlas.save(prefix + ".npz", prefix + ".yaml")
            self.get_logger().info("saved hull-aware atlas to %s" % prefix)

        def _solve_ik(self, x, y, z, yaw):
            from builtin_interfaces.msg import Duration
            from geometry_msgs.msg import PoseStamped
            from moveit_msgs.msg import RobotState
            from sensor_msgs.msg import JointState

            request = GetPositionIK.Request()
            ik = request.ik_request
            ik.group_name = self._ik_group
            ik.ik_link_name = self._ik_link
            ik.pose_stamped = PoseStamped()
            ik.pose_stamped.header.frame_id = self._base_frame
            position = _transform_point(
                self._container_origin, self._container_rotation, (x, y, z))
            ik.pose_stamped.pose.position.x = position[0]
            ik.pose_stamped.pose.position.y = position[1]
            ik.pose_stamped.pose.position.z = position[2]
            # Tool-down with a container-Z yaw convention.
            ik.pose_stamped.pose.orientation.x = math.cos(0.5 * yaw)
            ik.pose_stamped.pose.orientation.y = math.sin(0.5 * yaw)
            ik.pose_stamped.pose.orientation.z = 0.0
            ik.pose_stamped.pose.orientation.w = 0.0
            ik.avoid_collisions = self._avoid_collisions
            seconds = int(self._ik_timeout)
            ik.timeout = Duration(
                sec=seconds,
                nanosec=int((self._ik_timeout - seconds) * 1e9))
            ik.robot_state = RobotState()
            ik.robot_state.joint_state = JointState(
                name=["elfin_joint%d" % index for index in range(1, 7)],
                position=list(self._seed))
            future = self._client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            if response is None:
                return {"status": 0}
            if response.error_code.val != response.error_code.SUCCESS:
                return {"status": 1}
            positions = dict(zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position))
            seed = [positions.get("elfin_joint%d" % index, self._seed[index - 1])
                    for index in range(1, 7)]
            return {
                "status": REACHABLE,
                "contact_seeds": [seed],
                "transit_seeds": [seed],
                "opening_connected": True,
                "neighbor_confidence": 1.0,
            }

    rclpy.init(args=args)
    node = None
    try:
        node = AtlasBuilderNode()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    return 0


__all__ = ["main"]
