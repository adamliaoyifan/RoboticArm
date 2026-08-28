#!/usr/bin/env python3
"""Publish only post-settle depth frames for the MoveIt OctoMap updater."""

from __future__ import division

import math
import os
import sys

import rospy
import rospkg
import tf2_ros
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import JointState, PointCloud2

DESC_SCRIPTS = os.path.join(
    rospkg.RosPack().get_path("luggage_description"), "scripts"
)
PERC_SCRIPTS = os.path.join(
    rospkg.RosPack().get_path("luggage_perception"), "scripts"
)
for path in (DESC_SCRIPTS, PERC_SCRIPTS):
    if path not in sys.path:
        sys.path.insert(0, path)

from known_scene_point_filter import KnownScenePointFilter  # noqa: E402
from motion_stability_filter import MotionStabilityGate  # noqa: E402
from robot_self_point_filter import RobotSelfPointFilter  # noqa: E402
from scene_tf_config_utils import (  # noqa: E402
    load_scene_tf_config,
    resolve_scene_tf_config_path,
)


def _quat_rotate(qx, qy, qz, qw, vx, vy, vz):
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


class MotionGatedPointCloudRelay:
    def __init__(self):
        self._input_topic = rospy.get_param(
            "~input_cloud", "/camera/depth/points_filtered"
        )
        self._output_topic = rospy.get_param(
            "~output_cloud", "/camera/depth/points_settled"
        )
        self._base_frame = rospy.get_param("~base_frame", "elfin_base_link")
        self._gate = MotionStabilityGate(
            joint_names=rospy.get_param("~motion_gate/joint_names", []),
            velocity_threshold=float(
                rospy.get_param("~motion_gate/velocity_threshold", 0.02)
            ),
            settle_time_sec=float(
                rospy.get_param("~motion_gate/settle_time_sec", 0.5)
            ),
            joint_state_timeout_sec=float(
                rospy.get_param("~motion_gate/joint_state_timeout_sec", 1.0)
            ),
            enabled=bool(rospy.get_param("~motion_gate/enabled", True)),
        )
        scene_path = rospy.get_param(
            "~scene_tf_config",
            rospy.get_param(
                "/luggage/scene_tf_config", resolve_scene_tf_config_path()
            ),
        )
        self._static_filter = KnownScenePointFilter.from_scene_config(
            load_scene_tf_config(scene_path),
            padding=float(rospy.get_param("~known_scene_filter/padding", 0.03)),
            enabled=bool(
                rospy.get_param("~known_scene_filter/enabled", True)
            ),
            filter_ground=bool(
                rospy.get_param("~known_scene_filter/filter_ground", True)
            ),
        )
        self._self_filter = RobotSelfPointFilter.load_yaml(rospy.get_param(
            "~self_filter_config",
            os.path.join(
                rospkg.RosPack().get_path("luggage_description"),
                "config",
                "robot_self_filter.yaml.example",
            ),
        ))
        self._self_filter.enabled = bool(
            rospy.get_param("~self_filter/enabled", True))
        # Strict exact-stamp TF (no latest fallback) to match the
        # world_scene_mapper standard - prevents self-points from leaking
        # into the octomap on TF jitter.
        self._self_filter.allow_latest_tf_fallback = bool(
            rospy.get_param("~self_filter/allow_latest_tf_fallback", False))
        self._last_stamp = None
        self._stats = {
            "received": 0,
            "published": 0,
            "motion_dropped": 0,
            "duplicate_dropped": 0,
            "tf_dropped": 0,
            "self_filter_tf_dropped": 0,
        }
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._publisher = rospy.Publisher(
            self._output_topic, PointCloud2, queue_size=1
        )
        rospy.Subscriber(
            rospy.get_param("~motion_gate/joint_states_topic", "/joint_states"),
            JointState,
            self._on_joint_state,
            queue_size=1,
        )
        rospy.Subscriber(
            self._input_topic, PointCloud2, self._on_cloud, queue_size=1
        )

    def _publish_stats(self):
        stats = dict(self._stats)
        stats.update(self._gate.diagnostics(rospy.Time.now()))
        stats["static_filter"] = self._static_filter.last_stats
        stats["self_filter"] = self._self_filter.last_stats
        rospy.set_param("/luggage/perception/cloud_gate_stats", stats)

    def _on_joint_state(self, msg):
        stamp = msg.header.stamp
        if stamp == rospy.Time():
            stamp = rospy.Time.now()
        self._gate.update(
            msg.name, msg.position, msg.velocity,
            stamp=stamp, now=rospy.Time.now(),
        )

    def _transform_to_base(self, raw_points, msg):
        if msg.header.frame_id == self._base_frame:
            return list(raw_points)
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame,
                msg.header.frame_id,
                msg.header.stamp,
                rospy.Duration(0.2),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return None
        tr = transform.transform.translation
        qr = transform.transform.rotation
        result = []
        for x, y, z in raw_points:
            rx, ry, rz = _quat_rotate(
                qr.x, qr.y, qr.z, qr.w, x, y, z
            )
            result.append((rx + tr.x, ry + tr.y, rz + tr.z))
        return result

    def _filtered_raw_indices(self, base_points, stamp):
        """Compose robot-self and known-scene filters in raw-cloud order."""
        self_kept_base, self_kept_indices = (
            self._self_filter.filter_points_with_indices(
                base_points, self._tf_buffer, stamp=stamp)
        )
        if self._self_filter.last_stats.get("tf_missing_links"):
            return None
        _static_kept, static_kept_indices = self._static_filter.filter_points(
            self_kept_base)
        return [self_kept_indices[i] for i in static_kept_indices]

    def _on_cloud(self, msg):
        self._stats["received"] += 1
        now = rospy.Time.now()
        if not self._gate.accepts_cloud(msg.header.stamp, now=now):
            self._stats["motion_dropped"] += 1
            self._publish_stats()
            return
        stamp_key = (msg.header.stamp.secs, msg.header.stamp.nsecs)
        if stamp_key == self._last_stamp:
            self._stats["duplicate_dropped"] += 1
            self._publish_stats()
            return

        raw_points = list(
            pc2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True
            )
        )
        raw_points = [
            (float(p[0]), float(p[1]), float(p[2]))
            for p in raw_points
            if all(math.isfinite(float(v)) for v in p[:3])
        ]
        base_points = self._transform_to_base(raw_points, msg)
        if base_points is None:
            self._stats["tf_dropped"] += 1
            self._publish_stats()
            return
        kept_indices = self._filtered_raw_indices(
            base_points, msg.header.stamp)
        if kept_indices is None:
            self._stats["self_filter_tf_dropped"] += 1
            self._publish_stats()
            return
        filtered = [raw_points[i] for i in kept_indices]
        output = pc2.create_cloud_xyz32(msg.header, filtered)
        self._publisher.publish(output)
        self._last_stamp = stamp_key
        self._stats["published"] += 1
        self._publish_stats()



# Log level must be chosen before init_node, so it cannot come from a private
# param; log_level_utils reads the LUGGAGE_LOG_LEVEL environment variable.
import os as _os
import sys as _sys
import rospkg as _rospkg
_DESC = _os.path.join(
    _rospkg.RosPack().get_path("luggage_description"), "scripts")
if _DESC not in _sys.path:
    _sys.path.insert(0, _DESC)
from log_level_utils import resolve_log_level  # noqa: E402

def main():
    rospy.init_node("motion_gated_pointcloud_relay", log_level=resolve_log_level())
    MotionGatedPointCloudRelay()
    rospy.loginfo("motion_gated_pointcloud_relay ready")
    rospy.spin()


if __name__ == "__main__":
    main()
