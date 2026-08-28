#!/usr/bin/env python3
"""ROS node: semantic point cloud filter.

Subscribes to the raw depth point cloud and the semantic label mask, projects
each point into the color image to look up its label, and publishes two
filtered point clouds:

  /luggage/semantic/cargo_points      — labels in cargo_labels (default [2])
  /luggage/semantic/obstacle_points   — labels in obstacle_labels (default [2, 4])

Both clouds keep the original depth frame_id and stamp so downstream consumers
(cargo_volume_mapper, world_scene_mapper) can drop-in replace their input
topic.

If no mask has arrived within ``mask_timeout_sec`` and ``fallback_to_raw`` is
true, the node republishes the latest raw cloud on both outputs so the
pipeline stays alive when the segmenter is slow or down. A timer at
``fallback_rate_hz`` handles the fallback path; the synced callback handles
the normal filtered path.
"""

from __future__ import division

import os
import sys
import threading
import time

import rospy
import rospkg
import yaml
import message_filters
from cv_bridge import CvBridge
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField, Image


DESC_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_description"), "scripts")
PERC_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_perception"), "scripts")
for path in (DESC_SCRIPTS, PERC_SCRIPTS):
    if path not in sys.path:
        sys.path.insert(0, path)

from semantic_point_filter import (  # noqa: E402
    CameraIntrinsics,
    DepthToColorExtrinsics,
    SemanticPointFilter,
)


def _load_yaml(path):
    with open(path, "r") as handle:
        return yaml.safe_load(handle) or {}


class SemanticPointFilterNode:
    def __init__(self):
        self._camera_config = rospy.get_param(
            "~camera_config",
            os.path.join(
                rospkg.RosPack().get_path("luggage_description"),
                "config", "realsense_d435.yaml",
            ),
        )
        self._semantic_config = rospy.get_param(
            "~semantic_config",
            os.path.join(
                rospkg.RosPack().get_path("luggage_perception"),
                "config", "semantic_segmenter.yaml",
            ),
        )
        self._bridge = CvBridge()

        color_intr, depth_intr, extrinsics = self._load_intrinsics(self._camera_config)
        self._color_intrinsics = color_intr
        self._depth_intrinsics = depth_intr
        self._extrinsics = extrinsics

        pf_cfg = self._load_point_filter_config(self._semantic_config)
        self._cargo_labels = list(rospy.get_param(
            "~cargo_labels", pf_cfg.get("cargo_labels", [2])
        ))
        self._obstacle_labels = list(rospy.get_param(
            "~obstacle_labels", pf_cfg.get("obstacle_labels", [2, 4])
        ))
        self._mask_timeout_sec = float(rospy.get_param(
            "~mask_timeout_sec", pf_cfg.get("mask_timeout_sec", 0.5)
        ))
        self._fallback_to_raw = bool(rospy.get_param(
            "~fallback_to_raw", pf_cfg.get("fallback_to_raw", True)
        ))
        self._fallback_rate_hz = float(rospy.get_param("~fallback_rate_hz", 5.0))
        self._sync_slop = float(rospy.get_param(
            "~sync_slop", pf_cfg.get("sync_slop", 0.05)
        ))

        self._filter = SemanticPointFilter(
            color_intrinsics=color_intr,
            depth_intrinsics=depth_intr,
            depth_to_color=extrinsics,
            cargo_labels=self._cargo_labels,
            obstacle_labels=self._obstacle_labels,
        )

        self._cargo_pub = rospy.Publisher(
            "/luggage/semantic/cargo_points", PointCloud2, queue_size=1
        )
        self._obstacle_pub = rospy.Publisher(
            "/luggage/semantic/obstacle_points", PointCloud2, queue_size=1
        )

        self._depth_topic = rospy.get_param("~depth_topic", "/camera/depth/points")
        self._mask_topic = rospy.get_param("~mask_topic", "/luggage/semantic/mask")
        self._instance_mask_topic = rospy.get_param(
            "~instance_mask_topic", "/luggage/semantic/instance_mask"
        )

        self._latest_depth = None
        self._latest_mask_time = None
        self._latest_instance_mask = None
        self._lock = threading.Lock()
        self._fallback_active = False
        self._has_instance_mask = False

        # Synced path: depth + mask (+ optional instance_mask).
        self._depth_sub = message_filters.Subscriber(self._depth_topic, PointCloud2)
        self._mask_sync_sub = message_filters.Subscriber(self._mask_topic, Image)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._depth_sub, self._mask_sync_sub],
            queue_size=5, slop=self._sync_slop,
        )
        self._sync.registerCallback(self._on_synced)

        # Instance mask arrives on a separate subscriber; we cache the latest
        # and pair it with the synced depth+mask callback by timestamp proximity.
        self._instance_mask_sub = rospy.Subscriber(
            self._instance_mask_topic, Image,
            self._on_instance_mask, queue_size=1,
        )

        # Fallback path: track the latest depth + mask arrival time.
        self._raw_depth_sub = rospy.Subscriber(
            self._depth_topic, PointCloud2, self._on_raw_depth, queue_size=1
        )
        self._raw_mask_sub = rospy.Subscriber(
            self._mask_topic, Image, self._on_raw_mask, queue_size=1
        )

        if self._fallback_to_raw and self._fallback_rate_hz > 0:
            period = rospy.Duration(1.0 / self._fallback_rate_hz)
            rospy.Timer(period, self._on_fallback_timer)

        rospy.loginfo(
            "semantic_point_filter ready: depth=%s mask=%s instance_mask=%s "
            "cargo_labels=%s obstacle_labels=%s sync_slop=%.2fs "
            "mask_timeout=%.2fs fallback=%s",
            self._depth_topic, self._mask_topic, self._instance_mask_topic,
            self._cargo_labels, self._obstacle_labels,
            self._sync_slop, self._mask_timeout_sec, self._fallback_to_raw,
        )

    def _load_intrinsics(self, path):
        data = _load_yaml(path)
        cam = data.get("camera", {})
        color = CameraIntrinsics.from_dict(cam.get("color", {}))
        depth = CameraIntrinsics.from_dict(cam.get("depth", {}))
        extr = DepthToColorExtrinsics.from_dict(
            cam.get("extrinsics", {}).get("depth_to_color", {})
        )
        return color, depth, extr

    def _load_point_filter_config(self, path):
        try:
            data = _load_yaml(path)
        except (IOError, OSError) as exc:
            rospy.logwarn("semantic_point_filter: cannot load config %s: %s", path, exc)
            return {}
        return data.get("point_filter", {})

    def _on_raw_depth(self, msg):
        with self._lock:
            self._latest_depth = msg

    def _on_raw_mask(self, _msg):
        with self._lock:
            self._latest_mask_time = rospy.Time.now()

    def _on_instance_mask(self, msg):
        with self._lock:
            self._latest_instance_mask = msg
            self._has_instance_mask = True

    def _get_instance_map(self, ref_stamp):
        """Return the cached instance map if it is close in time to ref_stamp."""
        with self._lock:
            inst_msg = self._latest_instance_mask
        if inst_msg is None:
            return None
        dt = abs((inst_msg.header.stamp - ref_stamp).to_sec())
        if dt > self._sync_slop * 2:
            return None
        try:
            return self._bridge.imgmsg_to_cv2(inst_msg, desired_encoding="mono16")
        except Exception:
            return None

    def _on_synced(self, depth_msg, mask_msg):
        try:
            label_map = self._bridge.imgmsg_to_cv2(mask_msg, desired_encoding="mono8")
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "semantic_point_filter: mask decode failed: %s", exc)
            return

        instance_map = self._get_instance_map(mask_msg.header.stamp)
        has_instance = instance_map is not None

        points_depth = self._read_points(depth_msg)
        if not points_depth:
            return

        cargo_pts, obstacle_pts = self._filter.filter_points(
            points_depth, label_map, instance_map=instance_map,
        )
        stats = self._filter.last_stats

        self._publish(self._cargo_pub, depth_msg, cargo_pts, with_semantic=has_instance)
        self._publish(self._obstacle_pub, depth_msg, obstacle_pts, with_semantic=has_instance)
        self._fallback_active = False

        rospy.set_param("/luggage/semantic/point_filter_stats", stats)
        rospy.logdebug(
            "semantic_point_filter: raw=%d cargo=%d obstacle=%d excluded=%d oob=%d instance=%s",
            stats["raw_count"], stats["cargo_count"], stats["obstacle_count"],
            stats["excluded_count"], stats["out_of_frame_count"], has_instance,
        )

    def _on_fallback_timer(self, _event):
        if not self._fallback_to_raw:
            return
        with self._lock:
            depth_msg = self._latest_depth
            last_mask = self._latest_mask_time
        if depth_msg is None:
            return
        # Only fall back if no mask has arrived within the timeout window.
        now = rospy.Time.now()
        if last_mask is not None and (now - last_mask).to_sec() < self._mask_timeout_sec:
            return

        if not self._fallback_active:
            rospy.logwarn_throttle(
                10.0,
                "semantic_point_filter: no mask within %.2fs — republishing raw depth",
                self._mask_timeout_sec,
            )
            self._fallback_active = True

        # Republish the raw cloud on both outputs so downstream mappers keep
        # getting data. This is the safe degradation path.
        self._cargo_pub.publish(depth_msg)
        self._obstacle_pub.publish(depth_msg)
        rospy.set_param(
            "/luggage/semantic/point_filter_stats",
            {"fallback": True, "raw_count": -1},
        )

    def _publish(self, pub, ref_msg, points, with_semantic=False):
        """Publish a PointCloud2 in ref_msg's frame_id and stamp.

        When ``with_semantic`` is True, points are (x, y, z, label, instance_id)
        tuples and the cloud includes ``label`` (uint8) and ``instance_id``
        (uint16) fields. Otherwise the classic (x, y, z) layout is used.
        """
        if with_semantic:
            import struct
            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="label", offset=12, datatype=PointField.UINT8, count=1),
                PointField(name="instance_id", offset=14, datatype=PointField.UINT16, count=1),
            ]
            point_step = 16
            buf = bytearray()
            for pt in points:
                x, y, z = float(pt[0]), float(pt[1]), float(pt[2])
                label = int(pt[3]) if len(pt) > 3 else 0
                inst = int(pt[4]) if len(pt) > 4 else 0
                buf += struct.pack("<fffBxH", x, y, z, label, inst)
            cloud = PointCloud2()
            cloud.header = ref_msg.header
            cloud.height = 1
            cloud.width = len(points)
            cloud.fields = fields
            cloud.is_bigendian = False
            cloud.point_step = point_step
            cloud.row_step = point_step * len(points)
            cloud.data = bytes(buf)
            cloud.is_dense = True
        else:
            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            cloud = pc2.create_cloud(ref_msg.header, fields, points)
        pub.publish(cloud)

    @staticmethod
    def _read_points(msg):
        return [
            (float(p[0]), float(p[1]), float(p[2]))
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ]


def main():
    rospy.init_node("semantic_point_filter")
    SemanticPointFilterNode()
    rospy.spin()


if __name__ == "__main__":
    main()
