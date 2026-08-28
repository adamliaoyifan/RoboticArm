#!/usr/bin/env python3
"""Thin ROS 2 node around SensorPreprocessor.

Subscribes to canonical D435 topics plus /joint_states, pairs them, and
publishes a synchronised set of standard messages. SyncedObservation stays
inside the algorithm class; message conversion lives in
luggage_perception.ros_message_adapters.
"""

from __future__ import division

import json

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
from std_msgs.msg import String
import tf2_ros

from luggage_perception import ros_message_adapters as adapters
from luggage_perception.motion_stability_filter import MotionStabilityGate
from luggage_perception.sensor_preprocessor import SensorPreprocessor


class SensorPreprocessorNode(Node):

    def __init__(self):
        super().__init__("sensor_preprocessor")
        self._declare_params()
        if bool(self.get_parameter("enable_lidar_output").value):
            raise RuntimeError(
                "enable_lidar_output is true, but per-point lidar deskew is "
                "not implemented; refusing to publish uncompensated geometry"
            )

        joint_names = list(
            self.get_parameter("motion_gate.joint_names").value or []
        )
        self._core = SensorPreprocessor(
            camera_maxlen=int(self.get_parameter("camera_maxlen").value),
            camera_horizon_sec=float(
                self.get_parameter("camera_horizon_sec").value),
            camera_slop_sec=float(self.get_parameter("camera_slop_sec").value),
            camera_info_max_age_sec=float(
                self.get_parameter("camera_info_max_age_sec").value),
            joint_horizon_sec=float(
                self.get_parameter("joint_horizon_sec").value),
            output_cloud_frame=str(
                self.get_parameter("output_cloud_frame").value),
            stale_sec=float(self.get_parameter("stale_sec").value),
            motion_gate=MotionStabilityGate(
                joint_names=joint_names,
                velocity_threshold=float(
                    self.get_parameter("motion_gate.velocity_threshold").value),
                settle_time_sec=float(
                    self.get_parameter("motion_gate.settle_time_sec").value),
                joint_state_timeout_sec=float(
                    self.get_parameter(
                        "motion_gate.joint_state_timeout_sec").value),
                enabled=bool(self.get_parameter("motion_gate.enabled").value),
            ),
            joint_names=joint_names,
        )
        self._input_cloud_data_frame = str(
            self.get_parameter("input_cloud_data_frame").value).strip()
        self._output_cloud_frame = str(
            self.get_parameter("output_cloud_frame").value)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._tf_warn_stamp = 0.0
        self._cb_counts = {
            "rgb": 0, "depth": 0, "cloud": 0, "info": 0, "joints": 0,
        }

        # gz ros_gz_bridge currently offers RELIABLE on these topics.
        # BEST_EFFORT subscriptions connect but barely deliver on this RMW.
        input_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        joint_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._pub_color = self.create_publisher(
            Image, self.get_parameter("output.color_image").value, sensor_qos)
        self._pub_color_info = self.create_publisher(
            CameraInfo, self.get_parameter("output.color_info").value, sensor_qos)
        self._pub_depth = self.create_publisher(
            Image, self.get_parameter("output.depth_image").value, sensor_qos)
        self._pub_depth_info = self.create_publisher(
            CameraInfo, self.get_parameter("output.depth_info").value, sensor_qos)
        self._pub_cloud = self.create_publisher(
            PointCloud2, self.get_parameter("output.camera_points").value, sensor_qos)
        self._pub_status = self.create_publisher(
            String, self.get_parameter("output.status").value, status_qos)

        self.create_subscription(
            Image, self.get_parameter("input.color_image").value,
            self._on_color, input_qos)
        self.create_subscription(
            Image, self.get_parameter("input.depth_image").value,
            self._on_depth, input_qos)
        self.create_subscription(
            CameraInfo, self.get_parameter("input.camera_info").value,
            self._on_camera_info, input_qos)
        color_info_topic = str(
            self.get_parameter("input.color_camera_info").value).strip()
        if color_info_topic:
            self.create_subscription(
                CameraInfo, color_info_topic, self._on_color_info, input_qos)
        self.create_subscription(
            PointCloud2, self.get_parameter("input.camera_points").value,
            self._on_cloud, input_qos)
        self.create_subscription(
            JointState, self.get_parameter("input.joint_states").value,
            self._on_joints, joint_qos)

        self.get_logger().info(
            "sensor_preprocessor ready: data_frame=%r output_frame=%s"
            % (self._input_cloud_data_frame or "<header>",
               self._output_cloud_frame)
        )
        self.create_timer(1.0, self._on_status_timer)

    def _declare_params(self):
        defaults = {
            "input.color_image": "/camera/color/image_raw",
            "input.depth_image": "/camera/depth/image_raw",
            "input.camera_info": "/camera/depth/camera_info",
            "input.color_camera_info": "",
            "input.camera_points": "/camera/depth/points",
            "input.joint_states": "/joint_states",
            "input.lidar": "/livox/lidar",
            "input.imu": "/livox/imu",
            "output.color_image": "/luggage/preprocessed/camera/color/image",
            "output.color_info": "/luggage/preprocessed/camera/color/camera_info",
            "output.depth_image": "/luggage/preprocessed/camera/depth/image",
            "output.depth_info": "/luggage/preprocessed/camera/depth/camera_info",
            "output.camera_points": "/luggage/preprocessed/camera/depth/points",
            "output.status": "/luggage/preprocessed/status",
            "input_cloud_data_frame": "camera_link",
            "output_cloud_frame": "camera_depth_optical_frame",
            "enable_lidar_output": False,
            "enable_imu": False,
            "camera_slop_sec": 0.020,
            "camera_info_max_age_sec": 1.0,
            "camera_maxlen": 10,
            "camera_horizon_sec": 0.35,
            "joint_horizon_sec": 1.0,
            "stale_sec": 0.15,
            "motion_gate.joint_names": [
                "elfin_joint1", "elfin_joint2", "elfin_joint3",
                "elfin_joint4", "elfin_joint5", "elfin_joint6",
            ],
            "motion_gate.velocity_threshold": 0.02,
            "motion_gate.settle_time_sec": 0.5,
            "motion_gate.joint_state_timeout_sec": 1.0,
            "motion_gate.enabled": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _on_color(self, msg):
        self._cb_counts["rgb"] += 1
        frame = adapters.rgb_frame_from_msg(msg)
        if frame is None:
            self._warn_throttled(
                "dropping color image with encoding %s" % msg.encoding)
            return
        self._handle(self._core.update_rgb(frame))

    def _on_depth(self, msg):
        self._cb_counts["depth"] += 1
        frame = adapters.depth_frame_from_msg(msg)
        if frame is None:
            self._warn_throttled(
                "dropping depth image with encoding %s" % msg.encoding)
            return
        self._handle(self._core.update_depth(frame))

    def _on_camera_info(self, msg):
        self._cb_counts["info"] += 1
        frame = adapters.camera_info_frame_from_msg(msg)
        self._handle(self._core.update_camera_info(frame, slots=("depth", "color")))

    def _on_color_info(self, msg):
        frame = adapters.camera_info_frame_from_msg(msg)
        self._handle(self._core.update_camera_info(frame, slots=("color",)))

    def _on_cloud(self, msg):
        self._cb_counts["cloud"] += 1
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        data_frame = self._input_cloud_data_frame or msg.header.frame_id
        matrix = self._lookup_cloud_transform(data_frame, stamp)
        if matrix is None:
            return
        cloud = adapters.camera_cloud_from_msg(msg, data_frame)
        if cloud is None:
            self._warn_throttled("dropping cloud with unsupported point layout")
            return
        self._handle(self._core.update_camera_cloud(cloud, point_transform=matrix))

    def _on_joints(self, msg):
        self._cb_counts["joints"] += 1
        sample = adapters.joint_sample_from_msg(
            msg, fallback_stamp_sec=self.get_clock().now().nanoseconds * 1e-9)
        if sample is None:
            return
        self._handle(self._core.update_joint_state(sample))

    def _lookup_cloud_transform(self, data_frame, stamp):
        if data_frame == self._output_cloud_frame:
            return np.eye(4, dtype=np.float64)
        timeout = rclpy.duration.Duration(seconds=0.0)
        try:
            if not self._tf_buffer.can_transform(
                    self._output_cloud_frame, data_frame, stamp, timeout):
                self._warn_throttled(
                    "TF %s <- %s not available at stamp %s"
                    % (self._output_cloud_frame, data_frame, stamp))
                return None
            tf_msg = self._tf_buffer.lookup_transform(
                self._output_cloud_frame, data_frame, stamp, timeout,
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            self._warn_throttled(
                "TF %s <- %s failed: %s" % (
                    self._output_cloud_frame, data_frame, exc))
            return None
        return adapters.transform_matrix(tf_msg)

    def _warn_throttled(self, message, period_sec=2.0):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._tf_warn_stamp <= period_sec:
            return
        self._tf_warn_stamp = now
        self.get_logger().warning(message)

    def _handle(self, observation):
        if observation is None:
            return
        self._publish_observation(observation)
        self._publish_status(observation)

    def _publish_observation(self, obs):
        stamp = adapters.sec_to_stamp(obs.primary_stamp)
        if obs.rgb is not None and obs.flags.rgb_ok:
            self._pub_color.publish(
                adapters.image_msg_from_frame(obs.rgb, stamp))
        if obs.depth is not None and obs.flags.depth_ok:
            self._pub_depth.publish(
                adapters.depth_msg_from_frame(obs.depth, stamp))
        if obs.color_info is not None and obs.flags.color_info_ok:
            self._pub_color_info.publish(
                adapters.camera_info_msg_from_frame(obs.color_info, stamp))
        if obs.depth_info is not None and obs.flags.depth_info_ok:
            self._pub_depth_info.publish(
                adapters.camera_info_msg_from_frame(obs.depth_info, stamp))
        if obs.camera_points is not None and obs.flags.cloud_ok:
            self._pub_cloud.publish(adapters.cloud_msg_from_points(
                obs.camera_points, stamp, obs.frame_id))

    def _on_status_timer(self):
        payload = self._core.diagnostics()
        payload["callbacks"] = dict(self._cb_counts)
        self._publish_status_payload(payload)

    def _publish_status(self, obs):
        payload = self._core.diagnostics(now=obs.primary_stamp)
        payload["callbacks"] = dict(self._cb_counts)
        payload["flags"] = obs.flags.as_dict()
        payload["data_frame"] = obs.data_frame
        payload["frame_id"] = obs.frame_id
        payload["dropped_nonfinite"] = obs.dropped_nonfinite
        payload["depth_dt"] = obs.depth_dt
        payload["cloud_dt"] = obs.cloud_dt
        self._publish_status_payload(payload)

    def _publish_status_payload(self, payload):
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self._pub_status.publish(msg)


def main():
    rclpy.init()
    node = SensorPreprocessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
