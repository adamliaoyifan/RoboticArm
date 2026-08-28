#!/usr/bin/env python3
"""ROS adapter for the pure container opening estimator.

Only standard ROS messages are required.  A fiducial detector can either
publish ``geometry_msgs/PoseStamped`` or provide a small Python adapter via the
``~prior_adapter`` parameter (``package.module:function``).
"""

import importlib
import json
import math
import os
import sys

import numpy as np
import rospkg

SOURCE_SCRIPTS = os.path.join(
    rospkg.RosPack().get_path("luggage_perception"), "scripts")
if not sys.path or sys.path[0] != SOURCE_SCRIPTS:
    sys.path.insert(0, SOURCE_SCRIPTS)

from container_opening_estimator import (
    ContainerOpeningEstimator,
    OpeningPrior,
)


def _quaternion_matrix(quaternion):
    """Return a 3x3 rotation matrix for ROS quaternion [x, y, z, w]."""
    x, y, z, w = [float(value) for value in quaternion]
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError("pose quaternion is zero")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ])


def _matrix_quaternion(matrix):
    """Return ROS quaternion [x, y, z, w] from a 3x3 rotation matrix."""
    matrix = np.asarray(matrix, dtype=float)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return [
            (matrix[2, 1] - matrix[1, 2]) / s,
            (matrix[0, 2] - matrix[2, 0]) / s,
            (matrix[1, 0] - matrix[0, 1]) / s,
            0.25 * s,
        ]
    index = int(np.argmax(np.diag(matrix)))
    if index == 0:
        s = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        return [
            0.25 * s,
            (matrix[0, 1] + matrix[1, 0]) / s,
            (matrix[0, 2] + matrix[2, 0]) / s,
            (matrix[2, 1] - matrix[1, 2]) / s,
        ]
    if index == 1:
        s = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        return [
            (matrix[0, 1] + matrix[1, 0]) / s,
            0.25 * s,
            (matrix[1, 2] + matrix[2, 1]) / s,
            (matrix[0, 2] - matrix[2, 0]) / s,
        ]
    s = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
    return [
        (matrix[0, 2] + matrix[2, 0]) / s,
        (matrix[1, 2] + matrix[2, 1]) / s,
        0.25 * s,
        (matrix[1, 0] - matrix[0, 1]) / s,
    ]


def pose_stamped_to_prior(message, node):
    """Default adapter: pose X=width axis, Z=outward opening normal."""
    pose = message.pose
    quaternion = [
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w,
    ]
    rotation = _quaternion_matrix(quaternion)
    return OpeningPrior(
        center=[pose.position.x, pose.position.y, pose.position.z],
        width_axis=rotation[:, 0],
        normal=rotation[:, 2],
        width=node.tag_width,
        height=node.tag_height,
        stamp=message.header.stamp.to_sec(),
        confidence=node.tag_confidence,
        source="tag",
    )


def load_prior_adapter(specification):
    """Load ``module:function`` without coupling this package to tag messages."""
    if not specification:
        return pose_stamped_to_prior
    module_name, function_name = specification.rsplit(":", 1)
    return getattr(importlib.import_module(module_name), function_name)


class ContainerOpeningEstimatorNode:
    def __init__(self):
        import rospy
        from geometry_msgs.msg import PolygonStamped, PoseWithCovarianceStamped
        from luggage_msgs.msg import ContainerOpeningEstimate
        from roslib.message import get_message_class
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import String

        self.rospy = rospy
        self.PolygonStamped = PolygonStamped
        self.PoseWithCovarianceStamped = PoseWithCovarianceStamped
        self.String = String
        self.ContainerOpeningEstimate = ContainerOpeningEstimate
        self.hardware_strict = bool(rospy.get_param("~hardware_strict", False))
        self.tag_width = float(rospy.get_param("~tag_aperture_width", 1.0))
        self.tag_height = float(rospy.get_param("~tag_aperture_height", 1.0))
        self.tag_confidence = float(rospy.get_param("~tag_confidence", 0.8))
        self.sensor_origin = rospy.get_param("~sensor_origin", None)
        self.max_cloud_points = int(rospy.get_param("~max_cloud_points", 12000))
        self.inner_size = [
            float(value) for value in rospy.get_param(
                "~inner_size", [1.49, 1.97, 2.01])
        ]
        if len(self.inner_size) != 3:
            raise ValueError("~inner_size must contain length, width, height")
        self.geometry_version = 0
        self._last_geometry_signature = None
        self.estimator = ContainerOpeningEstimator(rospy.get_param("~estimator", {}))
        self.prior_adapter = load_prior_adapter(rospy.get_param("~prior_adapter", ""))
        prior_message_type = rospy.get_param(
            "~prior_message_type", "geometry_msgs/PoseStamped"
        )
        prior_message_class = get_message_class(prior_message_type)
        if prior_message_class is None:
            raise ValueError(
                "ROS message type is unavailable: %s" % prior_message_type
            )
        self.latest_prior = self._configured_prior()
        self.latest_prior_frame = str(rospy.get_param("~output_frame", ""))

        self.pose_pub = rospy.Publisher(
            "~opening_pose", PoseWithCovarianceStamped, queue_size=1
        )
        self.aperture_pub = rospy.Publisher(
            "~opening_aperture", PolygonStamped, queue_size=1
        )
        self.status_pub = rospy.Publisher(
            "~status", String, queue_size=1, latch=True
        )
        self.estimate_pub = rospy.Publisher(
            "~opening_estimate",
            ContainerOpeningEstimate,
            queue_size=1,
            latch=True,
        )
        self.pose_sub = rospy.Subscriber(
            rospy.get_param("~tag_pose_topic", "tag_opening_pose"),
            prior_message_class, self._pose_callback, queue_size=1,
        )
        self.cloud_sub = rospy.Subscriber(
            rospy.get_param("~depth_points_topic", "depth/points"),
            PointCloud2, self._cloud_callback, queue_size=1,
            buff_size=16 * 1024 * 1024,
        )

    def _configured_prior(self):
        values = self.rospy.get_param("~fallback_prior", {})
        if not values or not bool(values.get("enabled", False)):
            return None
        return OpeningPrior(
            center=values.get("center", [0.0, 0.0, 0.0]),
            normal=values.get("normal", [0.0, 0.0, 1.0]),
            width_axis=values.get("width_axis", [1.0, 0.0, 0.0]),
            width=values.get("width", 1.0),
            height=values.get("height", 1.0),
            confidence=values.get("confidence", 0.35),
            source="prior",
        )

    def _pose_callback(self, message):
        try:
            self.latest_prior = self.prior_adapter(message, self)
            header = getattr(message, "header", None)
            self.latest_prior_frame = getattr(header, "frame_id", "")
            estimate = self.estimator.estimate(
                prior=self.latest_prior, now=self.rospy.Time.now().to_sec(),
                hardware_strict=self.hardware_strict,
            )
            self._publish(estimate, self.latest_prior_frame)
        except Exception as exc:
            self.rospy.logwarn_throttle(2.0, "opening tag adapter failed: %s", exc)

    def _cloud_callback(self, message):
        from sensor_msgs import point_cloud2

        if (
            self.latest_prior is not None
            and self.latest_prior_frame
            and message.header.frame_id != self.latest_prior_frame
        ):
            self.rospy.logwarn_throttle(
                2.0, "opening inputs have different frames (%s, %s); "
                "configure an upstream transform",
                self.latest_prior_frame, message.header.frame_id,
            )
            return
        points = []
        step = max(1, int(message.width * message.height) // self.max_cloud_points)
        for index, point in enumerate(point_cloud2.read_points(
                message, field_names=("x", "y", "z"), skip_nans=True)):
            if index % step == 0:
                points.append(point[:3])
        estimate = self.estimator.estimate(
            prior=self.latest_prior,
            depth_points=points,
            depth_stamp=message.header.stamp.to_sec(),
            now=self.rospy.Time.now().to_sec(),
            hardware_strict=self.hardware_strict,
            sensor_origin=self.sensor_origin,
        )
        self._publish(estimate, message.header.frame_id)

    def _publish(self, estimate, frame_id):
        if estimate is None:
            self.status_pub.publish(self.String(data=json.dumps({
                "accepted": False, "rejection_reason": "no_estimate",
            })))
            return
        self.status_pub.publish(self.String(data=json.dumps(estimate.as_dict())))
        if estimate.accepted:
            signature = np.concatenate((
                estimate.center,
                estimate.normal,
                [estimate.width, estimate.height],
            ))
            changed = self._last_geometry_signature is None
            if not changed:
                previous = self._last_geometry_signature
                position_delta = np.linalg.norm(signature[:3] - previous[:3])
                normal_dot = float(np.clip(
                    np.dot(signature[3:6], previous[3:6]), -1.0, 1.0))
                normal_delta_deg = math.degrees(math.acos(normal_dot))
                extent_delta = float(np.max(np.abs(
                    signature[6:8] - previous[6:8])))
                changed = (
                    position_delta > float(self.rospy.get_param(
                        "~geometry_change_position", 0.01))
                    or normal_delta_deg > float(self.rospy.get_param(
                        "~geometry_change_angle_deg", 2.0))
                    or extent_delta > float(self.rospy.get_param(
                        "~geometry_change_extent", 0.01))
                )
            if changed:
                self.geometry_version += 1
                self._last_geometry_signature = signature
        contract = self.ContainerOpeningEstimate()
        contract.header.stamp = self.rospy.Time.from_sec(estimate.stamp)
        contract.header.frame_id = frame_id
        contract.opening_pose.pose.position.x = estimate.center[0]
        contract.opening_pose.pose.position.y = estimate.center[1]
        contract.opening_pose.pose.position.z = estimate.center[2]
        rotation = np.column_stack((
            estimate.width_axis, estimate.height_axis, estimate.normal
        ))
        quaternion = _matrix_quaternion(rotation)
        contract.opening_pose.pose.orientation.x = quaternion[0]
        contract.opening_pose.pose.orientation.y = quaternion[1]
        contract.opening_pose.pose.orientation.z = quaternion[2]
        contract.opening_pose.pose.orientation.w = quaternion[3]
        contract.opening_pose.covariance = (
            estimate.pose_covariance.reshape(-1).tolist())
        from geometry_msgs.msg import Point32
        half_width = estimate.width * 0.5
        half_height = estimate.height * 0.5
        contract.aperture.points = [
            Point32(x=-half_width, y=-half_height, z=0.0),
            Point32(x=half_width, y=-half_height, z=0.0),
            Point32(x=half_width, y=half_height, z=0.0),
            Point32(x=-half_width, y=half_height, z=0.0),
        ]
        contract.inner_size = self.inner_size
        contract.confidence = estimate.confidence
        contract.source = estimate.source_status
        contract.valid = estimate.accepted
        contract.geometry_version = self.geometry_version
        contract.rejection_reason = estimate.rejection_reason
        self.estimate_pub.publish(contract)
        # In strict mode a rejected fallback/stale result remains observable
        # through status, but must not appear on actionable geometry topics.
        if not estimate.accepted:
            return
        stamp = self.rospy.Time.from_sec(estimate.stamp)
        pose_message = self.PoseWithCovarianceStamped()
        pose_message.header.stamp = stamp
        pose_message.header.frame_id = frame_id
        pose_message.pose.pose.position.x = estimate.center[0]
        pose_message.pose.pose.position.y = estimate.center[1]
        pose_message.pose.pose.position.z = estimate.center[2]
        pose_message.pose.pose.orientation.x = quaternion[0]
        pose_message.pose.pose.orientation.y = quaternion[1]
        pose_message.pose.pose.orientation.z = quaternion[2]
        pose_message.pose.pose.orientation.w = quaternion[3]
        pose_message.pose.covariance = estimate.pose_covariance.reshape(-1).tolist()
        self.pose_pub.publish(pose_message)

        polygon = self.PolygonStamped()
        polygon.header = pose_message.header
        half_width, half_height = estimate.width * 0.5, estimate.height * 0.5
        for width_sign, height_sign in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            xyz = (
                estimate.center
                + width_sign * half_width * estimate.width_axis
                + height_sign * half_height * estimate.height_axis
            )
            polygon.polygon.points.append(Point32(*xyz))
        self.aperture_pub.publish(polygon)


def main():
    import rospy

    rospy.init_node("container_opening_estimator")
    ContainerOpeningEstimatorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
