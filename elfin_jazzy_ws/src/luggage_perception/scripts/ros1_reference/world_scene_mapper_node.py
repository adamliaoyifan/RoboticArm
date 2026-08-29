#!/usr/bin/env python3
"""Continuous world-scene occupancy mapper from RGBD depth observations.

Maintains a bounded voxel grid around the robot workspace, fusing each depth
frame automatically. Publishes obstacle clusters, an OctoMap for RViz, and
stats for downstream consumers (dynamic scene manager, replanning).

Unlike cargo_volume_mapper (container-local, service-triggered), this node
runs continuously and covers the full robot-reachable workspace.
"""

from __future__ import division

import math
import os
import sys
import time

import rospy
import rospkg
import tf2_ros
from geometry_msgs.msg import Point, PointStamped, Quaternion
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray

try:
    import octomap
except ImportError:
    octomap = None

DESC_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_description"), "scripts")
PERC_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_perception"), "scripts")
for path in (DESC_SCRIPTS, PERC_SCRIPTS):
    if path not in sys.path:
        sys.path.insert(0, path)

from depth_realism_filter import DepthRealismFilter  # noqa: E402
from known_scene_point_filter import KnownScenePointFilter  # noqa: E402
from motion_stability_filter import MotionStabilityGate  # noqa: E402
from robot_self_point_filter import RobotSelfPointFilter  # noqa: E402
from world_scene_mapper import FREE, OCCUPIED, UNKNOWN, WorldSceneMapper  # noqa: E402


class WorldSceneMapperNode:
    def __init__(self):
        self._base_frame = rospy.get_param("~base_frame", "elfin_base_link")
        self._resolution = float(rospy.get_param("~resolution", 0.05))
        self._marker_viz_scale = float(rospy.get_param("~marker_viz_scale", 1.25))
        self._stale_seconds = float(rospy.get_param("~stale_seconds", 30.0))
        self._max_range = float(rospy.get_param("~max_range", 3.0))
        self._update_rate = float(rospy.get_param("~update_rate", 2.0))
        self._depth_topic = rospy.get_param("~depth_points_topic", "/camera/depth/points")
        # Semantic filter integration: when enabled, subscribe to the
        # semantic-filtered obstacle point cloud instead of the raw depth
        # cloud. See .cursor/plans/semantic_perception_pipeline_4ade087b.plan.md.
        self._use_semantic_filter = bool(
            rospy.get_param("~use_semantic_filter", False)
        )
        if self._use_semantic_filter:
            self._depth_topic = rospy.get_param(
                "~semantic_depth_topic", "/luggage/semantic/obstacle_points"
            )
            rospy.loginfo(
                "world_scene_mapper: semantic filter ENABLED, input=%s",
                self._depth_topic,
            )
        self._publish_viz = bool(rospy.get_param("~publish_viz", True))
        self._exclude_container = bool(rospy.get_param("~exclude_container_interior", True))

        x_range = rospy.get_param("~bounds_x", [-1.0, 3.0])
        y_range = rospy.get_param("~bounds_y", [-2.0, 2.0])
        z_range = rospy.get_param("~bounds_z", [-0.5, 2.5])
        bounds = [x_range, y_range, z_range]

        self._scene_config = self._load_scene_config()
        self._container_bounds = self._load_container_bounds()

        self._mapper = WorldSceneMapper(
            bounds,
            self._resolution,
            self._stale_seconds,
            occupancy_params=self._occupancy_params(),
        )
        self._self_filter = self._build_self_filter()
        self._static_filter = KnownScenePointFilter.from_scene_config(
            self._scene_config,
            padding=float(rospy.get_param("~known_scene_filter/padding", 0.03)),
            enabled=bool(rospy.get_param("~known_scene_filter/enabled", True)),
            filter_ground=bool(
                rospy.get_param("~known_scene_filter/filter_ground", True)
            ),
        )
        self._depth_filter = self._build_depth_filter()
        self._motion_gate = MotionStabilityGate(
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
        self._latest_cloud = None
        self._last_integrated_stamp = None
        self._integration_count = 0
        self._last_integrate_time = 0.0
        self._drop_stats = {
            "motion_gate_drops": 0,
            "tf_drops": 0,
            "duplicate_frame_drops": 0,
        }

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._octomap_pub = None
        self._markers_pub = None
        self._obstacles_pub = None
        if self._publish_viz:
            if octomap is not None:
                from octomap_msgs.msg import Octomap as OctomapMsg
                self._octomap_pub = rospy.Publisher(
                    "/luggage/world_scene/octomap", OctomapMsg, queue_size=1, latch=True
                )
            self._markers_pub = rospy.Publisher(
                "/luggage/world_scene/markers", MarkerArray, queue_size=1, latch=True
            )
        self._obstacles_pub = rospy.Publisher(
            "/luggage/world_scene/obstacles", MarkerArray, queue_size=1, latch=True
        )

        rospy.Subscriber(self._depth_topic, PointCloud2, self._on_cloud, queue_size=1)
        rospy.Subscriber(
            rospy.get_param("~motion_gate/joint_states_topic", "/joint_states"),
            JointState,
            self._on_joint_state,
            queue_size=1,
        )
        rospy.Service("~reset", Trigger, self._handle_reset)
        rospy.Service("~get_stats", Trigger, self._handle_stats)

        if self._update_rate > 0:
            period = 1.0 / self._update_rate
            rospy.Timer(rospy.Duration(period), self._on_timer)

        rospy.loginfo(
            "world_scene_mapper ready: res=%.2f bounds_x=%s bounds_y=%s bounds_z=%s "
            "grid=%dx%dx%d stale=%.0fs rate=%.1fHz",
            self._resolution, x_range, y_range, z_range,
            self._mapper.nx, self._mapper.ny, self._mapper.nz,
            self._stale_seconds, self._update_rate,
        )

    def _occupancy_params(self):
        return {
            "p_hit": float(rospy.get_param("~occupancy/p_hit", 0.70)),
            "p_miss": float(rospy.get_param("~occupancy/p_miss", 0.40)),
            "log_odds_min": float(rospy.get_param("~occupancy/log_odds_min", -2.0)),
            "log_odds_max": float(rospy.get_param("~occupancy/log_odds_max", 3.5)),
            "occupied_threshold": float(
                rospy.get_param("~occupancy/occupied_threshold", 1.2)
            ),
            "free_threshold": float(
                rospy.get_param("~occupancy/free_threshold", -0.35)
            ),
            "enabled": bool(rospy.get_param("~occupancy/enabled", True)),
        }

    def _load_scene_config(self):
        from scene_tf_config_utils import (
            load_scene_tf_config,
            resolve_scene_tf_config_path,
        )
        config_path = rospy.get_param(
            "~scene_tf_config",
            rospy.get_param(
                "/luggage/scene_tf_config", resolve_scene_tf_config_path()
            ),
        )
        return load_scene_tf_config(config_path)

    def _load_container_bounds(self):
        """Load container interior bounds in base frame for exclusion filtering."""
        if not self._exclude_container:
            return None
        try:
            from scene_tf_config_utils import (
                container_usable_center_in_base_link,
                container_usable_dimensions,
            )
            from scene_tf_config_utils import container_in_base_link
            center = container_usable_center_in_base_link(self._scene_config)
            _base_xyz, base_rpy = container_in_base_link(self._scene_config)
            inner_l, inner_w, inner_h = container_usable_dimensions(self._scene_config)
            cx, cy, cz = center
            yaw = base_rpy[2]
            rospy.loginfo(
                "world_scene: excluding container interior at [%.2f,%.2f,%.2f] size=[%.2f,%.2f,%.2f]",
                cx, cy, cz, inner_l, inner_w, inner_h,
            )
            return {
                "center": [cx, cy, cz],
                "half_size": [inner_l * 0.5, inner_w * 0.5, inner_h * 0.5],
                "yaw": yaw,
            }
        except Exception as exc:
            rospy.logwarn("world_scene: cannot load container bounds: %s", exc)
            return None

    def _point_in_container(self, x, y, z):
        if self._container_bounds is None:
            return False
        cb = self._container_bounds
        cx, cy, cz = cb["center"]
        hx, hy, hz = cb["half_size"]
        yaw = cb["yaw"]
        dx = x - cx
        dy = y - cy
        local_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        local_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy
        local_z = z - cz
        return abs(local_x) <= hx and abs(local_y) <= hy and abs(local_z) <= hz

    def _build_self_filter(self):
        config_path = rospy.get_param(
            "~self_filter_config",
            os.path.join(
                rospkg.RosPack().get_path("luggage_description"),
                "config",
                "robot_self_filter.yaml.example",
            ),
        )
        enabled = bool(rospy.get_param("~self_filter/enabled", True))
        try:
            filt = RobotSelfPointFilter.load_yaml(config_path)
            filt.enabled = enabled
            filt.allow_latest_fallback = bool(
                rospy.get_param("~self_filter/allow_latest_tf_fallback", False)
            )
            rospy.loginfo(
                "Robot self-filter: enabled=%s bodies=%d config=%s",
                filt.enabled,
                len(filt.bodies),
                config_path,
            )
            return filt
        except (IOError, OSError, ValueError) as exc:
            rospy.logwarn("Robot self-filter disabled — config error: %s", exc)
            return RobotSelfPointFilter(enabled=False)

    def _build_depth_filter(self):
        return DepthRealismFilter(
            enabled=bool(rospy.get_param("~depth_realism/enabled", False)),
            max_reliable_range=float(rospy.get_param("~depth_realism/max_reliable_range", 2.5)),
            hard_max_range=float(rospy.get_param("~depth_realism/hard_max_range", 3.0)),
            range_noise_sigma=float(rospy.get_param("~depth_realism/range_noise_sigma", 0.004)),
            dropout_rate=float(rospy.get_param("~depth_realism/dropout_rate", 0.02)),
            random_seed=rospy.get_param("~depth_realism/random_seed", None),
        )

    def _on_cloud(self, msg):
        self._latest_cloud = msg

    def _on_joint_state(self, msg):
        stamp = msg.header.stamp
        if stamp == rospy.Time():
            stamp = rospy.Time.now()
        self._motion_gate.update(
            msg.name,
            msg.position,
            msg.velocity,
            stamp=stamp,
            now=rospy.Time.now(),
        )

    def _lookup_optical_origin(self, msg):
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame,
                msg.header.frame_id,
                msg.header.stamp,
                rospy.Duration(0.5),
            )
            t = transform.transform.translation
            return (t.x, t.y, t.z)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            rospy.logwarn_throttle(10.0, "world_scene TF lookup failed: %s", exc)
            self._drop_stats["tf_drops"] += 1
            return None

    def _cloud_points_in_base(self, msg):
        """Read cloud and transform to base frame.

        Returns (points, labels) where labels is a parallel list of per-point
        semantic class IDs, or None if the field is absent.
        """
        field_names_set = {f.name for f in msg.fields}
        has_label = "label" in field_names_set
        read_fields = ("x", "y", "z", "label") if has_label else ("x", "y", "z")

        raw = []
        raw_labels = [] if has_label else None
        for point in pc2.read_points(msg, field_names=read_fields, skip_nans=True):
            raw.append((float(point[0]), float(point[1]), float(point[2])))
            if has_label:
                raw_labels.append(int(point[3]))

        if msg.header.frame_id == self._base_frame:
            return raw, raw_labels

        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame,
                msg.header.frame_id,
                msg.header.stamp,
                rospy.Duration(0.5),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            rospy.logwarn_throttle(10.0, "world_scene TF failed: %s", exc)
            self._drop_stats["tf_drops"] += 1
            return [], None

        from tf2_geometry_msgs import do_transform_point

        transformed = []
        kept_labels = [] if has_label else None
        for i, (x, y, z) in enumerate(raw):
            pt = PointStamped()
            pt.header = msg.header
            pt.point.x = x
            pt.point.y = y
            pt.point.z = z
            out = do_transform_point(pt, transform)
            transformed.append((out.point.x, out.point.y, out.point.z))
            if has_label:
                kept_labels.append(raw_labels[i])
        return transformed, kept_labels

    def _on_timer(self, _event):
        if self._latest_cloud is None:
            return
        now = time.time()
        if now - self._last_integrate_time < (0.9 / max(self._update_rate, 0.1)):
            return
        self._integrate_latest()
        self._last_integrate_time = now

    def _integrate_latest(self):
        cloud = self._latest_cloud
        if cloud is None:
            return
        now_ros = rospy.Time.now()
        if not self._motion_gate.accepts_cloud(cloud.header.stamp, now=now_ros):
            self._drop_stats["motion_gate_drops"] += 1
            rospy.set_param(
                "/luggage/world_scene/motion_gate_stats",
                dict(
                    self._motion_gate.diagnostics(now_ros),
                    drops=self._drop_stats["motion_gate_drops"],
                ),
            )
            return
        stamp_key = (cloud.header.stamp.secs, cloud.header.stamp.nsecs)
        if stamp_key == self._last_integrated_stamp:
            self._drop_stats["duplicate_frame_drops"] += 1
            rospy.set_param(
                "/luggage/world_scene/duplicate_frame_stats",
                {
                    "duplicate_drops": self._drop_stats[
                        "duplicate_frame_drops"
                    ],
                    "last_stamp": list(stamp_key),
                },
            )
            return

        origin = self._lookup_optical_origin(cloud)
        if origin is None:
            return

        points, labels = self._cloud_points_in_base(cloud)
        if not points:
            return

        points, kept_indices = self._self_filter.filter_points_with_indices(
            points, self._tf_buffer, cloud.header.stamp
        )
        if labels is not None:
            labels = [labels[i] for i in kept_indices]
        self_stats = self._self_filter.last_stats
        rospy.set_param("/luggage/world_scene/self_filter_stats", self_stats)
        if self._self_filter.enabled and self_stats.get("tf_missing_links"):
            self._drop_stats["tf_drops"] += 1
            rospy.set_param(
                "/luggage/world_scene/tf_drop_stats",
                {
                    "dropped": self._drop_stats["tf_drops"],
                    "reason": "self-filter exact-stamp TF unavailable",
                    "missing_links": self_stats["tf_missing_links"],
                },
            )
            return

        points, static_indices = self._static_filter.filter_points(points)
        if labels is not None:
            labels = [labels[i] for i in static_indices]
        rospy.set_param(
            "/luggage/world_scene/static_filter_stats",
            self._static_filter.last_stats,
        )

        points = self._depth_filter.filter_points(points, origin=origin)
        if not points:
            return

        # Depth filter may change the point set; drop labels if mismatch.
        if labels is not None and len(labels) != len(points):
            labels = None

        if self._exclude_container and self._container_bounds is not None:
            kept = [(i, (x, y, z)) for i, (x, y, z) in enumerate(points)
                    if not self._point_in_container(x, y, z)]
            if not kept:
                return
            indices, points = zip(*kept)
            points = list(points)
            if labels is not None:
                labels = [labels[i] for i in indices]

        now = time.time()
        self._mapper.integrate_points(points, origin, now, labels=labels)
        self._last_integrated_stamp = stamp_key
        rospy.set_param(
            "/luggage/world_scene/tf_drop_stats",
            {"dropped": self._drop_stats["tf_drops"]},
        )
        cleared = self._mapper.clear_stale(now)
        self._integration_count += 1

        stats = self._mapper.stats()
        rospy.set_param("/luggage/world_scene/stats", stats)
        rospy.set_param("/luggage/world_scene/integration_count", self._integration_count)

        if self._integration_count % 5 == 0:
            # Periodic mapper accounting; the numbers are also on
            # /luggage/world_scene/stats, so throttle the console copy.
            rospy.loginfo_throttle(
                10.0,
                "world_scene: frame=%d occ=%d free=%d cleared=%d pts=%d self_dropped=%d",
                self._integration_count,
                stats["occupied_count"],
                stats["free_count"],
                cleared,
                len(points),
                self_stats.get("dropped_self", 0),
            )

        self._publish_obstacles()
        if self._publish_viz and self._integration_count % 3 == 0:
            self._publish_viz_data()

    _LABEL_COLORS = {
        0: ColorRGBA(r=0.9, g=0.2, b=0.1, a=0.6),
        1: ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.5),
        2: ColorRGBA(r=0.1, g=0.85, b=0.2, a=0.7),
        3: ColorRGBA(r=0.1, g=0.8, b=0.9, a=0.5),
        4: ColorRGBA(r=1.0, g=0.55, b=0.0, a=0.7),
    }

    def _publish_obstacles(self):
        obstacles = self._mapper.obstacle_clusters_with_labels()
        rospy.set_param("/luggage/world_scene/obstacle_count", len(obstacles))
        rospy.set_param(
            "/luggage/world_scene/obstacles_xyz",
            [[o[0], o[1], o[2]] for o in obstacles[:200]],
        )

        ma = MarkerArray()
        stamp = rospy.Time.now()
        if obstacles:
            marker = Marker()
            marker.header.frame_id = self._base_frame
            marker.header.stamp = stamp
            marker.ns = "world_obstacles"
            marker.id = 0
            marker.type = Marker.CUBE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation = Quaternion(w=1.0)
            marker.scale.x = self._resolution * self._marker_viz_scale
            marker.scale.y = self._resolution * self._marker_viz_scale
            marker.scale.z = self._resolution * self._marker_viz_scale
            marker.color = ColorRGBA(r=0.9, g=0.2, b=0.1, a=0.6)
            for x, y, z, lbl in obstacles:
                marker.points.append(Point(x=x, y=y, z=z))
                marker.colors.append(
                    self._LABEL_COLORS.get(lbl, self._LABEL_COLORS[0]))
            ma.markers.append(marker)
        else:
            clear = Marker()
            clear.header.frame_id = self._base_frame
            clear.header.stamp = stamp
            clear.ns = "world_obstacles"
            clear.id = 0
            clear.action = Marker.DELETE
            ma.markers.append(clear)

        self._obstacles_pub.publish(ma)

    def _publish_viz_data(self):
        if self._octomap_pub is not None and octomap is not None:
            try:
                from octomap_msgs.msg import Octomap as OctomapMsg
                tree = octomap.OcTree(self._resolution)
                for ix in range(self._mapper.nx):
                    for iy in range(self._mapper.ny):
                        for iz in range(self._mapper.nz):
                            state = self._mapper._grid[self._mapper._index(ix, iy, iz)]
                            if state == UNKNOWN:
                                continue
                            x, y, z = self._mapper._voxel_center(ix, iy, iz)
                            tree.updateNode((x, y, z), state == OCCUPIED)
                msg = OctomapMsg()
                msg.header.frame_id = self._base_frame
                msg.header.stamp = rospy.Time.now()
                msg.binary = True
                msg.id = "OcTree"
                msg.resolution = self._resolution
                binary_data = tree.writeBinary()
                if isinstance(binary_data, str):
                    msg.data = [ord(ch) for ch in binary_data]
                else:
                    msg.data = list(bytearray(binary_data))
                self._octomap_pub.publish(msg)
            except Exception as exc:
                rospy.logwarn_throttle(30.0, "OctoMap viz publish failed: %s", exc)

    def _handle_reset(self, _req):
        self._mapper.reset()
        self._integration_count = 0
        self._last_integrated_stamp = None
        for key in self._drop_stats:
            self._drop_stats[key] = 0
        rospy.loginfo("world_scene_mapper reset")
        return TriggerResponse(success=True, message="World scene map reset")

    def _handle_stats(self, _req):
        stats = self._mapper.stats()
        msg = "occ=%d free=%d unknown=%d integrations=%d" % (
            stats["occupied_count"],
            stats["free_count"],
            stats["unknown_count"],
            self._integration_count,
        )
        return TriggerResponse(success=True, message=msg)



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
    rospy.init_node("world_scene_mapper", log_level=resolve_log_level())
    WorldSceneMapperNode()
    rospy.spin()


if __name__ == "__main__":
    main()
