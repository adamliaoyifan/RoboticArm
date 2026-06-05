#!/usr/bin/env python3
"""ROS node: Cargo interior voxel occupancy from depth point clouds."""

from __future__ import division

import os
import sys

import rospy
import rospkg
import tf2_ros
from geometry_msgs.msg import PointCloud2
from octomap_msgs.msg import Octomap
from sensor_msgs import point_cloud2 as pc2
from visualization_msgs.msg import MarkerArray

from luggage_msgs.srv import (
    GetCargoMapStats,
    GetCargoMapStatsResponse,
    IntegrateCargoView,
    IntegrateCargoViewResponse,
    ResetCargoMap,
    ResetCargoMapResponse,
)

DESC_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_description"), "scripts")
PERC_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_perception"), "scripts")
for path in (DESC_SCRIPTS, PERC_SCRIPTS):
    if path not in sys.path:
        sys.path.insert(0, path)

from container_config_utils import (  # noqa: E402
    container_in_base_link,
    default_config_path,
    inner_dimensions,
    load_container_config,
)
from cargo_volume_mapper import CargoVolumeMapper, octomap as octomap_module  # noqa: E402


class CargoVolumeMapperNode:
    def __init__(self):
        self._config = load_container_config(
            rospy.get_param("~container_config", default_config_path())
        )
        self._resolution = float(rospy.get_param("~voxel_resolution", 0.10))
        self._base_frame = rospy.get_param("~base_frame", "elfin_base_link")
        self._viz_frame = rospy.get_param("~viz_frame_id", self._base_frame)
        self._optical_frame = rospy.get_param(
            "~optical_frame", "camera_depth_optical_frame"
        )
        self._depth_topic = rospy.get_param(
            "~depth_points_topic", "/camera/depth/points"
        )
        self._publish_viz = bool(rospy.get_param("~publish_viz", True))
        self._viz_show_free = bool(rospy.get_param("~viz_show_free", False))
        self._viz_show_unknown = bool(rospy.get_param("~viz_show_unknown", True))
        self._viz_include_free_octomap = bool(
            rospy.get_param("~viz_include_free_octomap", True)
        )

        self._mapper = self._build_mapper()
        self._latest_cloud = None
        self._octomap_warned = False

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._octomap_pub = None
        self._markers_pub = None
        if self._publish_viz:
            self._octomap_pub = rospy.Publisher(
                "/luggage/cargo_map/octomap", Octomap, queue_size=1, latch=True
            )
            self._markers_pub = rospy.Publisher(
                "/luggage/cargo_map/markers",
                MarkerArray,
                queue_size=1,
                latch=True,
            )

        rospy.Subscriber(self._depth_topic, PointCloud2, self._on_cloud, queue_size=1)

        rospy.Service("~reset_cargo_map", ResetCargoMap, self.handle_reset)
        rospy.Service("~integrate_cargo_view", IntegrateCargoView, self.handle_integrate)
        rospy.Service("~get_cargo_map_stats", GetCargoMapStats, self.handle_stats)

        self._mapper.publish_params(rospy)
        self._publish_visualization()

        if self._publish_viz and octomap_module is None and not self._octomap_warned:
            rospy.logwarn(
                "python3-octomap unavailable; publishing MarkerArray only "
                "(install python3-octomap or ros-noetic-octomap-msgs in Docker)"
            )
            self._octomap_warned = True

    def _build_mapper(self):
        inner = inner_dimensions(self._config)
        base_xyz, base_rpy = container_in_base_link(self._config)
        import math

        yaw = base_rpy[2]
        inner_l, inner_w, inner_h = inner
        cx = base_xyz[0] + (inner_l * 0.5) * math.cos(yaw)
        cy = base_xyz[1] + (inner_l * 0.5) * math.sin(yaw)
        cz = base_xyz[2] + inner_h * 0.5
        return CargoVolumeMapper(
            inner_size=inner,
            center_base=[cx, cy, cz],
            yaw=yaw,
            resolution=self._resolution,
        )

    def _publish_visualization(self):
        if not self._publish_viz:
            return

        stamp = rospy.Time.now()
        markers = self._mapper.to_marker_array(
            self._viz_frame,
            stamp,
            container_config=self._config,
            show_free=self._viz_show_free,
            show_unknown=self._viz_show_unknown,
        )
        if self._markers_pub is not None:
            self._markers_pub.publish(markers)

        if self._octomap_pub is None:
            return

        octomap_msg = self._mapper.to_octomap_msg(
            self._viz_frame,
            stamp,
            include_free=self._viz_include_free_octomap,
        )
        if octomap_msg is not None:
            self._octomap_pub.publish(octomap_msg)
        elif not self._octomap_warned:
            rospy.logwarn_throttle(
                30.0,
                "Skipping /luggage/cargo_map/octomap publish: octomap Python bindings missing",
            )
            self._octomap_warned = True

    def _on_cloud(self, msg):
        self._latest_cloud = msg

    def _lookup_optical_origin(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._optical_frame,
                rospy.Time(0),
                rospy.Duration(1.0),
            )
            t = transform.transform.translation
            return (t.x, t.y, t.z)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            rospy.logwarn("TF lookup failed for %s: %s", self._optical_frame, exc)
            return None

    def _cloud_points_in_base(self, msg):
        from geometry_msgs.msg import PointStamped

        raw = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            raw.append((float(point[0]), float(point[1]), float(point[2])))
        if msg.header.frame_id == self._base_frame:
            return raw

        transformed = []
        for x, y, z in raw:
            pt = PointStamped()
            pt.header.frame_id = msg.header.frame_id
            pt.header.stamp = msg.header.stamp
            pt.point.x = x
            pt.point.y = y
            pt.point.z = z
            try:
                out = self._tf_buffer.transform(
                    pt, self._base_frame, rospy.Duration(0.5)
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as exc:
                rospy.logwarn_throttle(5.0, "Point transform failed: %s", exc)
                continue
            transformed.append((out.point.x, out.point.y, out.point.z))
        return transformed

    def handle_reset(self, _req):
        self._mapper.reset()
        self._mapper.publish_params(rospy)
        self._publish_visualization()
        return ResetCargoMapResponse(success=True, message="Cargo map reset")

    def handle_integrate(self, req):
        if self._latest_cloud is None:
            return IntegrateCargoViewResponse(
                success=False,
                message="No depth cloud received on %s" % self._depth_topic,
                unknown_ratio=1.0,
                occupancy_ratio=0.0,
            )

        origin = self._lookup_optical_origin()
        points = self._cloud_points_in_base(self._latest_cloud)
        if not points:
            return IntegrateCargoViewResponse(
                success=False,
                message="Depth cloud empty after transform",
                unknown_ratio=self._mapper.stats()["unknown_ratio"],
                occupancy_ratio=self._mapper.stats()["occupancy_ratio"],
            )

        frame_count = max(1, int(req.frame_count)) if req.frame_count else 1
        for _ in range(frame_count):
            self._mapper.integrate_points(points, origin=origin)

        self._mapper.publish_params(rospy)
        self._publish_visualization()
        stats = self._mapper.stats()
        return IntegrateCargoViewResponse(
            success=True,
            message="integrated %d points" % len(points),
            unknown_ratio=stats["unknown_ratio"],
            occupancy_ratio=stats["occupancy_ratio"],
        )

    def handle_stats(self, _req):
        stats = self._mapper.stats()
        self._mapper.publish_params(rospy)
        self._publish_visualization()
        return GetCargoMapStatsResponse(
            success=True,
            message="ok",
            unknown_ratio=stats["unknown_ratio"],
            occupancy_ratio=stats["occupancy_ratio"],
            free_volume=stats["free_volume"],
            unknown_count=stats["unknown_count"],
            free_count=stats["free_count"],
            occupied_count=stats["occupied_count"],
            frontier_count=stats["frontier_count"],
            total_voxels=stats["total_voxels"],
        )


def main():
    rospy.init_node("cargo_volume_mapper")
    CargoVolumeMapperNode()
    rospy.loginfo("cargo_volume_mapper ready")
    rospy.spin()


if __name__ == "__main__":
    main()
