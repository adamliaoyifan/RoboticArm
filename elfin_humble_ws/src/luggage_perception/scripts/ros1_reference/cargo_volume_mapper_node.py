#!/usr/bin/env python3
"""ROS node: Cargo interior voxel occupancy from depth point clouds."""

from __future__ import division

import os
import sys
import threading
import time
import math
import json
from collections import deque

import rospy
import rospkg
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  registers PointStamped/PoseStamped with tf2_ros.transform()
from octomap_msgs.msg import Octomap
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import CameraInfo, JointState, PointCloud2
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from luggage_msgs.srv import (
    AddPlacedBox,
    AddPlacedBoxResponse,
    EvaluateCargoViews,
    EvaluateCargoViewsResponse,
    GetCargoMapStats,
    GetCargoMapStatsResponse,
    IntegrateCargoView,
    IntegrateCargoViewResponse,
    ResetCargoMap,
    ResetCargoMapResponse,
    RemovePlacedBox,
    RemovePlacedBoxResponse,
)

DESC_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_description"), "scripts")
PERC_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_perception"), "scripts")
PLAN_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_planning"), "scripts")
for path in (DESC_SCRIPTS, PERC_SCRIPTS, PLAN_SCRIPTS):
    if path not in sys.path:
        sys.path.insert(0, path)

from scene_tf_config_utils import (  # noqa: E402
    container_in_base_link,
    container_usable_center_in_base_link,
    container_usable_dimensions,
    load_scene_tf_config,
    resolve_scene_tf_config_path,
)
from cargo_volume_mapper import CargoVolumeMapper, octomap as octomap_module  # noqa: E402
from depth_realism_filter import DepthRealismFilter  # noqa: E402
from motion_stability_filter import MotionStabilityGate  # noqa: E402
from robot_self_point_filter import RobotSelfPointFilter  # noqa: E402
from known_scene_point_filter import KnownScenePointFilter  # noqa: E402
from interior_view_scorer import (  # noqa: E402
    CameraIntrinsics,
    ContainerFloor,
    RaycastConfig,
    floor_coverage_metrics,
    raycast_information_gain,
)


class _CargoOccupancyAccessor:
    """Adapt CargoVolumeMapper to the scorer's deterministic grid contract."""

    def __init__(self, mapper):
        self.mapper = mapper
        self.resolution = mapper.resolution

    def world_to_index(self, point):
        return self.mapper._local_to_voxel(
            *self.mapper._world_to_local(*point))

    def contains_index(self, index):
        return (
            index is not None
            and len(index) == 3
            and 0 <= index[0] < self.mapper.nx
            and 0 <= index[1] < self.mapper.ny
            and 0 <= index[2] < self.mapper.nz
        )

    def occupancy(self, index):
        if not self.contains_index(index):
            return None
        state = self.mapper._grid[self.mapper._index(*index)]
        return {0: "unknown", 1: "free", 2: "occupied"}[state]

    def cell_center(self, index):
        return self.mapper._local_to_world(
            *self.mapper._voxel_center_local(*index))

    def normal(self, _index):
        return None

    def ray_indices(self, origin, direction, max_range):
        step = self.resolution * 0.25
        previous = None
        distance = 0.0
        while distance <= max_range + 1e-9:
            point = [
                origin[i] + direction[i] * distance for i in range(3)]
            index = self.world_to_index(point)
            if index is None:
                if previous is not None:
                    break
            elif index != previous:
                yield index, distance
                previous = index
            distance += step


def _quaternion_axes(pose):
    q = pose.orientation
    x, y, z, w = q.x, q.y, q.z, q.w
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError("candidate pose has zero quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    rotation = (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )
    forward = [rotation[i][2] for i in range(3)]
    camera_up = [-rotation[i][1] for i in range(3)]
    return forward, camera_up


class CargoVolumeMapperNode:
    def __init__(self):
        self._scene_config = load_scene_tf_config(
            rospy.get_param(
                "~scene_tf_config",
                rospy.get_param("/luggage/scene_tf_config", resolve_scene_tf_config_path()),
            )
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
        # Semantic filter integration: when enabled, subscribe to the
        # semantic-filtered cargo point cloud instead of the raw depth cloud.
        # See .cursor/plans/semantic_perception_pipeline_4ade087b.plan.md.
        self._use_semantic_filter = bool(
            rospy.get_param("~use_semantic_filter", False)
        )
        if self._use_semantic_filter:
            self._depth_topic = rospy.get_param(
                "~semantic_depth_topic", "/luggage/semantic/cargo_points"
            )
            rospy.loginfo(
                "cargo_volume_mapper: semantic filter ENABLED, input=%s",
                self._depth_topic,
            )
        self._publish_viz = bool(rospy.get_param("~publish_viz", True))
        self._viz_show_free = bool(rospy.get_param("~viz_show_free", False))
        self._viz_show_unknown = bool(rospy.get_param("~viz_show_unknown", True))
        self._viz_include_free_octomap = bool(
            rospy.get_param("~viz_include_free_octomap", True)
        )

        self._mapper = self._build_mapper()
        self._reset_preserve_placed = bool(rospy.get_param(
            "~reset_preserve_placed", True))
        self._self_filter = self._build_self_filter()
        self._known_scene_filter = self._build_known_scene_filter()
        self._depth_filter = self._build_depth_filter()
        self._latest_cloud = None
        self._camera_info = None
        self._cloud_buffer = deque(
            maxlen=max(1, int(rospy.get_param("~cloud_buffer_size", 12)))
        )
        self._cloud_condition = threading.Condition()
        self._integrated_stamps = set()
        self._frame_wait_timeout = float(
            rospy.get_param("~frame_wait_timeout_sec", 1.5)
        )
        self._allow_stable_latest_tf_fallback = bool(rospy.get_param(
            "~allow_stable_latest_tf_fallback", True))
        # How long integrate_cargo_view waits for the arm to settle (motion
        # gate "stable") before giving up. The orchestrator often calls
        # integrate right after a move, before the post-move joint velocity
        # has decayed below the gate threshold, so we wait instead of failing
        # immediately. Tunable via the occupancy_fusion config yaml.
        self._integrate_settle_wait_sec = float(
            rospy.get_param("~integrate_settle_wait_sec", 4.0)
        )
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
        self._octomap_warned = False

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._octomap_pub = None
        self._markers_pub = None
        self._stats_pub = rospy.Publisher(
            "~stats_json", String, queue_size=1, latch=True)
        self._floor_coverage_pub = rospy.Publisher(
            "/luggage/debug/floor_coverage",
            MarkerArray,
            queue_size=1,
            latch=True,
        )
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
        rospy.Subscriber(
            rospy.get_param(
                "~camera_info_topic", "/camera/depth/camera_info"),
            CameraInfo,
            self._on_camera_info,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~motion_gate/joint_states_topic", "/joint_states"),
            JointState,
            self._on_joint_state,
            queue_size=1,
        )

        rospy.Service("~reset_cargo_map", ResetCargoMap, self.handle_reset)
        rospy.Service("~integrate_cargo_view", IntegrateCargoView, self.handle_integrate)
        rospy.Service("~get_cargo_map_stats", GetCargoMapStats, self.handle_stats)
        rospy.Service(
            "~evaluate_cargo_views",
            EvaluateCargoViews,
            self.handle_evaluate_views,
        )
        rospy.Service("~mark_placed_box", AddPlacedBox, self.handle_mark_placed)
        rospy.Service(
            "~unmark_placed_box", RemovePlacedBox,
            self.handle_unmark_placed)

        self._mapper.publish_params(rospy)
        self._publish_visualization()
        self._publish_realism_config()

        if self._publish_viz and octomap_module is None and not self._octomap_warned:
            rospy.logwarn(
                "python3-octomap unavailable; publishing MarkerArray only "
                "(install python3-octomap or ros-noetic-octomap-msgs in Docker)"
            )
            self._octomap_warned = True

    def _build_mapper(self):
        inner = container_usable_dimensions(self._scene_config)
        base_xyz, base_rpy = container_in_base_link(self._scene_config)

        yaw = base_rpy[2]
        return CargoVolumeMapper(
            inner_size=inner,
            center_base=container_usable_center_in_base_link(self._scene_config),
            yaw=yaw,
            resolution=self._resolution,
            occupancy_params={
                "p_hit": float(rospy.get_param("~occupancy/p_hit", 0.70)),
                "p_miss": float(rospy.get_param("~occupancy/p_miss", 0.40)),
                "log_odds_min": float(
                    rospy.get_param("~occupancy/log_odds_min", -2.0)
                ),
                "log_odds_max": float(
                    rospy.get_param("~occupancy/log_odds_max", 3.5)
                ),
                "occupied_threshold": float(
                    rospy.get_param("~occupancy/occupied_threshold", 1.2)
                ),
                "free_threshold": float(
                    rospy.get_param("~occupancy/free_threshold", -0.35)
                ),
                "enabled": bool(rospy.get_param("~occupancy/enabled", True)),
            },
            max_raycast_points=float(
                rospy.get_param("~raycast_max_points", 20000)
            ),
        )

    def _publish_realism_config(self):
        cfg = {
            "enabled": self._depth_filter.enabled,
            "max_reliable_range": self._depth_filter.max_reliable_range,
            "hard_max_range": self._depth_filter.hard_max_range,
            "range_noise_sigma": self._depth_filter.range_noise_sigma,
            "dropout_rate": self._depth_filter.dropout_rate,
        }
        rospy.set_param("/luggage/cargo_map/realism_config", cfg)
        if self._depth_filter.enabled:
            rospy.loginfo(
                "Depth realism ENABLED: reliable=%.1fm hard_max=%.1fm noise=%.4f dropout=%.2f",
                cfg["max_reliable_range"],
                cfg["hard_max_range"],
                cfg["range_noise_sigma"],
                cfg["dropout_rate"],
            )
        else:
            rospy.loginfo("Depth realism DISABLED (ideal mode)")

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

    def _build_known_scene_filter(self):
        """Known static geometry filter (container shell, pedestal, platform, ground).

        Matches the world_scene_mapper standard so the cargo map doesn't
        accumulate container-wall / pedestal points as cargo occupancy.
        """
        try:
            return KnownScenePointFilter.from_scene_config(
                self._scene_config,
                padding=float(
                    rospy.get_param("~known_scene_filter/padding", 0.03)),
                enabled=bool(
                    rospy.get_param("~known_scene_filter/enabled", True)),
                filter_ground=bool(
                    rospy.get_param("~known_scene_filter/filter_ground", True)),
            )
        except Exception as exc:
            rospy.logwarn(
                "cargo_volume_mapper: known_scene_filter unavailable: %s", exc)
            return None

    def _build_depth_filter(self):
        return DepthRealismFilter(
            enabled=bool(rospy.get_param("~depth_realism/enabled", True)),
            max_reliable_range=float(rospy.get_param("~depth_realism/max_reliable_range", 2.5)),
            hard_max_range=float(rospy.get_param("~depth_realism/hard_max_range", 3.0)),
            range_noise_sigma=float(rospy.get_param("~depth_realism/range_noise_sigma", 0.004)),
            dropout_rate=float(rospy.get_param("~depth_realism/dropout_rate", 0.02)),
            edge_dropout_rate=float(rospy.get_param("~depth_realism/edge_dropout_rate", 0.0)),
            random_seed=rospy.get_param("~depth_realism/random_seed", None),
        )

    def _publish_visualization(self):
        if not self._publish_viz:
            return

        stamp = rospy.Time.now()
        markers = self._mapper.to_marker_array(
            self._viz_frame,
            stamp,
            scene_config=self._scene_config,
            show_free=self._viz_show_free,
            show_unknown=self._viz_show_unknown,
        )
        if self._markers_pub is not None:
            self._markers_pub.publish(markers)

        if self._octomap_pub is None:
            return

        # Visualization must never fail integrate/reset services. The
        # python-octomap binding in this image has no Point3d; a throw here
        # previously aborted InitialExploreCargo after a successful cloud
        # integrate and sent the orchestrator to Idle.
        try:
            octomap_msg = self._mapper.to_octomap_msg(
                self._viz_frame,
                stamp,
                include_free=self._viz_include_free_octomap,
            )
        except Exception as exc:
            if not self._octomap_warned:
                rospy.logwarn(
                    "Skipping /luggage/cargo_map/octomap publish: %s", exc)
                self._octomap_warned = True
            return
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
        stamp_key = (msg.header.stamp.secs, msg.header.stamp.nsecs)
        with self._cloud_condition:
            if not self._cloud_buffer or self._cloud_buffer[-1][0] != stamp_key:
                self._cloud_buffer.append((stamp_key, msg))
                self._cloud_condition.notify_all()

    def _on_camera_info(self, msg):
        self._camera_info = msg

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
        # Wake any integrate_cargo_view waiter polling the gate state.
        with self._cloud_condition:
            self._cloud_condition.notify_all()

    def _lookup_optical_origin(self, stamp):
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._optical_frame,
                stamp,
                rospy.Duration(1.0),
            )
            t = transform.transform.translation
            return (t.x, t.y, t.z)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            if (
                    self._allow_stable_latest_tf_fallback
                    and self._motion_gate.state(rospy.Time.now())
                    in ("stable", "disabled")):
                try:
                    transform = self._tf_buffer.lookup_transform(
                        self._base_frame, self._optical_frame,
                        rospy.Time(0), rospy.Duration(0.5))
                    rospy.logwarn_throttle(
                        5.0,
                        "Using latest optical TF for stable cloud; exact "
                        "stamp unavailable: %s", exc)
                    t = transform.transform.translation
                    return (t.x, t.y, t.z)
                except (
                    tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException,
                ):
                    pass
            rospy.logwarn(
                "TF lookup failed for %s: %s", self._optical_frame, exc)
            return None

    def _cloud_points_in_base(self, msg):
        """Read cloud and transform to base frame.

        Returns (points, labels, instance_ids) where labels and instance_ids
        are lists parallel to points. They are None if the fields are absent.
        """
        from geometry_msgs.msg import PointStamped

        field_names = [f.name for f in msg.fields]
        has_label = "label" in field_names
        has_instance = "instance_id" in field_names

        if has_label and has_instance:
            read_fields = ("x", "y", "z", "label", "instance_id")
        elif has_label:
            read_fields = ("x", "y", "z", "label")
        else:
            read_fields = ("x", "y", "z")

        raw_pts = []
        raw_labels = [] if has_label else None
        raw_instances = [] if has_instance else None
        for point in pc2.read_points(msg, field_names=read_fields, skip_nans=True):
            raw_pts.append((float(point[0]), float(point[1]), float(point[2])))
            if has_label:
                raw_labels.append(int(point[3]))
            if has_instance:
                raw_instances.append(int(point[4] if has_label else point[3]))

        if msg.header.frame_id == self._base_frame:
            return raw_pts, raw_labels, raw_instances

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
            if (
                    self._allow_stable_latest_tf_fallback
                    and self._motion_gate.state(rospy.Time.now())
                    in ("stable", "disabled")):
                try:
                    transform = self._tf_buffer.lookup_transform(
                        self._base_frame, msg.header.frame_id,
                        rospy.Time(0), rospy.Duration(0.5))
                    rospy.logwarn_throttle(
                        5.0,
                        "Using latest point TF for stable cloud; exact stamp "
                        "unavailable: %s", exc)
                except (
                    tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException,
                ):
                    return [], None, None
            else:
                rospy.logwarn_throttle(
                    5.0, "Point transform failed: %s", exc)
                return [], None, None

        from tf2_geometry_msgs import do_transform_point

        transformed = []
        for x, y, z in raw_pts:
            pt = PointStamped()
            pt.header = msg.header
            pt.point.x = x
            pt.point.y = y
            pt.point.z = z
            out = do_transform_point(pt, transform)
            transformed.append((out.point.x, out.point.y, out.point.z))
        return transformed, raw_labels, raw_instances

    def handle_reset(self, _req):
        self._mapper.reset(preserve_placed=self._reset_preserve_placed)
        self._integrated_stamps.clear()
        self._mapper.publish_params(rospy)
        self._publish_visualization()
        return ResetCargoMapResponse(
            success=True,
            message="Cargo map reset (preserve_placed=%s, committed=%d)"
            % (
                self._reset_preserve_placed,
                len(self._mapper._placed_boxes),
            ),
        )

    def _available_stable_clouds(self):
        now_ros = rospy.Time.now()
        if self._motion_gate.state(now_ros) not in ("stable", "disabled"):
            return []
        with self._cloud_condition:
            snapshot = list(self._cloud_buffer)
        result = []
        for stamp_key, cloud in snapshot:
            if stamp_key in self._integrated_stamps:
                continue
            if not self._motion_gate.accepts_cloud(cloud.header.stamp, now=now_ros):
                continue
            result.append((stamp_key, cloud))
        return result

    def _wait_for_arm_stable(self, timeout):
        """Wait for the motion gate to report stable (or disabled).

        Polls the gate up to ``timeout`` seconds, waking on each new
        joint-state/cloud callback. Returns the final gate state. Used by
        handle_integrate so a call that arrives while the arm is still
        settling does not fail instantly - it gives the arm time to drop
        below the velocity threshold.
        """
        deadline = time.time() + max(0.0, timeout)
        while not rospy.is_shutdown():
            state = self._motion_gate.state(rospy.Time.now())
            if state in ("stable", "disabled"):
                return state
            remaining = deadline - time.time()
            if remaining <= 0.0:
                return state
            with self._cloud_condition:
                self._cloud_condition.wait(timeout=min(0.1, remaining))
        return self._motion_gate.state(rospy.Time.now())

    def _wait_for_stable_clouds(self, count):
        deadline = time.time() + max(0.0, self._frame_wait_timeout)
        while not rospy.is_shutdown():
            clouds = self._available_stable_clouds()
            if len(clouds) >= count or time.time() >= deadline:
                return clouds[:count]
            with self._cloud_condition:
                self._cloud_condition.wait(
                    timeout=min(0.1, max(0.0, deadline - time.time()))
                )
        return []

    def _integrate_cloud(self, cloud):
        origin = self._lookup_optical_origin(cloud.header.stamp)
        if origin is None:
            return False, "exact-stamp optical TF unavailable"
        points, labels, instance_ids = self._cloud_points_in_base(cloud)
        if not points:
            return False, "depth cloud empty after exact-stamp transform"

        points, kept_indices = self._self_filter.filter_points_with_indices(
            points, self._tf_buffer, cloud.header.stamp
        )
        if labels is not None:
            labels = [labels[i] for i in kept_indices]
        if instance_ids is not None:
            instance_ids = [instance_ids[i] for i in kept_indices]
        self_stats = self._self_filter.last_stats
        rospy.set_param("/luggage/cargo_map/self_filter_stats", self_stats)
        if self._self_filter.enabled and self_stats.get("tf_missing_links"):
            return False, (
                "self-filter exact-stamp TF unavailable for %s"
                % ",".join(self_stats["tf_missing_links"])
            )

        # Known-scene filter (container shell, pedestal, platform, ground) -
        # matches the world_scene_mapper standard so static geometry doesn't
        # pollute the cargo occupancy map.
        if self._known_scene_filter is not None and self._known_scene_filter.enabled:
            points, static_indices = self._known_scene_filter.filter_points(points)
            if labels is not None:
                labels = [labels[i] for i in static_indices]
            if instance_ids is not None:
                instance_ids = [instance_ids[i] for i in static_indices]
            rospy.set_param(
                "/luggage/cargo_map/known_scene_filter_stats",
                self._known_scene_filter.last_stats,
            )

        points = self._depth_filter.filter_points(points, origin=origin)
        filter_stats = self._depth_filter.last_stats
        rospy.set_param("/luggage/cargo_map/realism_stats", filter_stats)
        if labels is not None and len(labels) != len(points):
            labels = None
            instance_ids = None
        if not points:
            return False, "all points filtered by depth realism"

        self._mapper.integrate_points(
            points,
            origin=origin,
            labels=labels,
            instance_ids=instance_ids,
        )
        return True, "integrated %d points" % len(points)

    def handle_integrate(self, req):
        if self._latest_cloud is None:
            return IntegrateCargoViewResponse(
                success=False,
                message="No depth cloud received on %s" % self._depth_topic,
                unknown_ratio=1.0,
                occupancy_ratio=0.0,
            )

        # Wait for the arm to settle instead of failing the instant the gate
        # reports "moving": the orchestrator often calls integrate before the
        # post-move velocity has decayed below the gate threshold.
        t0 = time.time()
        gate_state = self._wait_for_arm_stable(self._integrate_settle_wait_sec)
        rospy.loginfo(
            "integrate_cargo_view: arm %s after %.2fs (settle_wait=%.1fs)",
            gate_state, time.time() - t0, self._integrate_settle_wait_sec,
        )
        if gate_state not in ("stable", "disabled"):
            diag = self._motion_gate.diagnostics(rospy.Time.now())
            # "settling" means max_vel is already <= threshold but hasn't
            # stayed there for settle_time_sec yet; only "moving" means the
            # threshold is actively exceeded. Report the comparison that
            # actually holds instead of hardcoding ">".
            comparison = (
                ">" if diag["max_velocity"] > diag["velocity_threshold"]
                else "<=")
            return IntegrateCargoViewResponse(
                success=False,
                message=(
                    "Arm not stable after %.1fs (motion gate: %s, "
                    "max_vel=%.3f rad/s %s %.3f)" % (
                        self._integrate_settle_wait_sec, gate_state,
                        diag["max_velocity"], comparison,
                        diag["velocity_threshold"],
                    )
                ),
                unknown_ratio=self._mapper.stats()["unknown_ratio"],
                occupancy_ratio=self._mapper.stats()["occupancy_ratio"],
            )

        requested = max(1, int(req.frame_count)) if req.frame_count else 1
        clouds = self._wait_for_stable_clouds(requested)
        rospy.loginfo(
            "integrate_cargo_view: %d stable cloud(s) after %.2fs total",
            len(clouds), time.time() - t0,
        )
        if not clouds:
            return IntegrateCargoViewResponse(
                success=False,
                message="No distinct post-settle depth frame available",
                unknown_ratio=self._mapper.stats()["unknown_ratio"],
                occupancy_ratio=self._mapper.stats()["occupancy_ratio"],
            )
        used = 0
        failures = []
        for stamp_key, cloud in clouds:
            t_cloud = time.time()
            success, message = self._integrate_cloud(cloud)
            rospy.loginfo(
                "integrate_cargo_view: cloud %s %s in %.2fs (%d pts)",
                stamp_key, "ok" if success else "FAIL: " + message,
                time.time() - t_cloud, cloud.width * cloud.height,
            )
            if success:
                self._integrated_stamps.add(stamp_key)
                used += 1
            else:
                # Do not let one stale/TF-missing frame starve all subsequent
                # verification retries; advance to the next buffered cloud.
                self._integrated_stamps.add(stamp_key)
                failures.append("%s:%s" % (stamp_key, message))
        if used == 0:
            return IntegrateCargoViewResponse(
                success=False,
                message="No frames integrated: %s" % "; ".join(failures),
                unknown_ratio=self._mapper.stats()["unknown_ratio"],
                occupancy_ratio=self._mapper.stats()["occupancy_ratio"],
            )

        self._mapper.publish_params(rospy)
        self._publish_visualization()
        stats = self._mapper.stats()
        rospy.loginfo(
            "integrate_cargo_view: done %d/%d in %.2fs total (unknown=%.2f occ=%.2f)",
            used, requested, time.time() - t0,
            stats["unknown_ratio"], stats["occupancy_ratio"],
        )
        return IntegrateCargoViewResponse(
            success=True,
            message=(
                "integrated %d/%d distinct post-settle frames%s"
                % (
                    used,
                    requested,
                    ("; skipped: %s" % "; ".join(failures)) if failures else "",
                )
            ),
            unknown_ratio=stats["unknown_ratio"],
            occupancy_ratio=stats["occupancy_ratio"],
        )

    def handle_mark_placed(self, req):
        """Mark a just-placed box as occupied so the 2.5D map reflects it."""
        import math

        slot = req.slot
        center = [
            slot.place_pose.position.x,
            slot.place_pose.position.y,
            slot.place_pose.position.z,
        ]
        size = [slot.width, slot.depth, slot.height]
        q = slot.place_pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._mapper.mark_placed_box(center, size, yaw=yaw)
        self._mapper.publish_params(rospy)
        self._publish_visualization()
        return AddPlacedBoxResponse(
            success=True,
            message="marked placed box at %s" % [round(v, 3) for v in center],
        )

    def handle_unmark_placed(self, req):
        slot = req.slot
        removed = self._mapper.unmark_placed_box(
            [
                slot.place_pose.position.x,
                slot.place_pose.position.y,
                slot.place_pose.position.z,
            ],
            [slot.width, slot.depth, slot.height],
        )
        self._mapper.publish_params(rospy)
        self._publish_visualization()
        return RemovePlacedBoxResponse(
            success=removed,
            message=(
                "unmarked placed box"
                if removed else "placed box record not found"),
        )

    def handle_stats(self, _req):
        stats = self._mapper.stats()
        self._stats_pub.publish(String(data=json.dumps(
            stats, sort_keys=True)))
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
            map_revision=stats["map_revision"],
        )

    def _publish_floor_coverage(self, floor, coverage):
        """Draw the inner-floor grid tinted by what one camera pose can see."""
        stamp = rospy.Time.now()
        covered = set(tuple(cell) for cell in coverage["covered_cells"])
        blocked = set(tuple(cell) for cell in coverage["blocked_cells"])
        green = ColorRGBA(r=0.1, g=0.9, b=0.2, a=0.75)
        red = ColorRGBA(r=0.95, g=0.25, b=0.1, a=0.6)
        grey = ColorRGBA(r=0.45, g=0.45, b=0.45, a=0.3)

        cells = Marker()
        cells.header.frame_id = self._viz_frame
        cells.header.stamp = stamp
        cells.ns = "floor_coverage"
        cells.id = 0
        cells.type = Marker.CUBE_LIST
        cells.action = Marker.ADD
        cells.pose.orientation = Quaternion(w=1.0)
        cells.scale.x = cells.scale.y = floor.resolution * 0.9
        cells.scale.z = 0.005
        cells.lifetime = rospy.Duration(0)
        for ix in range(floor.nx):
            for iy in range(floor.ny):
                center = floor.cell_center_base(ix, iy, height=0.003)
                cells.points.append(
                    Point(x=center[0], y=center[1], z=center[2]))
                if (ix, iy) in covered:
                    cells.colors.append(green)
                elif (ix, iy) in blocked:
                    cells.colors.append(red)
                else:
                    cells.colors.append(grey)

        label = Marker()
        label.header.frame_id = self._viz_frame
        label.header.stamp = stamp
        label.ns = "floor_coverage_label"
        label.id = 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        anchor = floor.cell_center_base(
            floor.nx // 2, floor.ny // 2, height=floor.inner_h + 0.05)
        label.pose.position = Point(x=anchor[0], y=anchor[1], z=anchor[2])
        label.pose.orientation = Quaternion(w=1.0)
        label.scale.z = 0.05
        label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        label.text = (
            "floor coverage %.0f%% (%d/%d cells, %d blocked)\n"
            "inside FOV %.0f%%  unknown gain %.0f%%" % (
                100.0 * coverage["floor_xy_coverage"],
                coverage["floor_cells_covered"],
                coverage["floor_cells_total"],
                coverage["floor_cells_blocked"],
                100.0 * coverage["inside_container_fov_ratio"],
                100.0 * coverage["floor_unknown_gain"],
            ))
        label.lifetime = rospy.Duration(0)
        self._floor_coverage_pub.publish(
            MarkerArray(markers=[cells, label]))

    def handle_evaluate_views(self, req):
        if self._camera_info is None:
            return EvaluateCargoViewsResponse(
                success=False, message="camera_info unavailable",
                expected_information_gain=[], visible_unknown_count=[],
                occlusion_ratio=[], corridor_free_confidence=[],
            )
        if req.insertion_depths and (
                len(req.insertion_depths) != len(req.camera_poses)):
            return EvaluateCargoViewsResponse(
                success=False,
                message="insertion_depths length must match camera_poses",
                expected_information_gain=[], visible_unknown_count=[],
                occlusion_ratio=[], corridor_free_confidence=[],
            )
        info = self._camera_info
        intrinsics = CameraIntrinsics(
            info.width, info.height, info.K[0], info.K[4],
            info.K[2], info.K[5],
        )
        raycast_cfg = RaycastConfig(
            max_range=float(req.max_range) if req.max_range > 0.0 else 2.5,
            pixel_stride=max(
                1, int(rospy.get_param("~view_scoring/pixel_stride", 16))),
            range_decay=float(rospy.get_param(
                "~view_scoring/range_decay", 2.5)),
            grazing_power=float(rospy.get_param(
                "~view_scoring/grazing_power", 1.0)),
        )
        accessor = _CargoOccupancyAccessor(self._mapper)
        opening = [
            req.opening_pose.position.x,
            req.opening_pose.position.y,
            req.opening_pose.position.z,
        ]
        floor = None
        if getattr(req, "evaluate_floor_coverage", False):
            floor = ContainerFloor(
                center_base=self._mapper.center,
                yaw=self._mapper.yaw,
                inner_size=(
                    self._mapper.inner_l,
                    self._mapper.inner_w,
                    self._mapper.inner_h,
                ),
                resolution=self._mapper.resolution,
            )
        gains = []
        unknown_counts = []
        occlusion_ratios = []
        corridor_confidences = []
        floor_coverage = []
        floor_unknown_gain = []
        inside_fov_ratio = []
        try:
            for pose in req.camera_poses:
                camera = [
                    pose.position.x, pose.position.y, pose.position.z]
                forward, camera_up = _quaternion_axes(pose)
                candidate = {
                    "camera_xyz": camera,
                    "look_at": [
                        camera[i] + forward[i] for i in range(3)],
                    "camera_up": camera_up,
                }
                result = raycast_information_gain(
                    candidate, accessor, intrinsics, raycast_cfg)
                gains.append(result["information_gain"])
                unknown_counts.append(result["visible_unknown_voxels"])
                occlusion_ratios.append(
                    float(result["occluded_rays"])
                    / float(max(1, result["rays_cast"])))
                corridor_confidences.append(
                    self._mapper.corridor_free_confidence(
                        opening, camera, radius=req.corridor_radius))
                if floor is not None:
                    coverage = floor_coverage_metrics(
                        candidate, accessor, intrinsics, floor, raycast_cfg)
                    floor_coverage.append(coverage["floor_xy_coverage"])
                    floor_unknown_gain.append(coverage["floor_unknown_gain"])
                    inside_fov_ratio.append(
                        coverage["inside_container_fov_ratio"])
                    if (
                            len(floor_coverage) == 1
                            and getattr(
                                req, "publish_floor_coverage_debug", False)):
                        self._publish_floor_coverage(floor, coverage)
            return EvaluateCargoViewsResponse(
                success=True,
                message="evaluated %d candidate views" % len(gains),
                expected_information_gain=gains,
                visible_unknown_count=unknown_counts,
                occlusion_ratio=occlusion_ratios,
                corridor_free_confidence=corridor_confidences,
                floor_xy_coverage=floor_coverage,
                floor_unknown_gain=floor_unknown_gain,
                inside_container_fov_ratio=inside_fov_ratio,
            )
        except Exception as exc:
            return EvaluateCargoViewsResponse(
                success=False,
                message="view evaluation failed: %s" % exc,
                expected_information_gain=[], visible_unknown_count=[],
                occlusion_ratio=[], corridor_free_confidence=[],
            )



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
    rospy.init_node("cargo_volume_mapper", log_level=resolve_log_level())
    CargoVolumeMapperNode()
    rospy.loginfo("cargo_volume_mapper ready")
    rospy.spin()


if __name__ == "__main__":
    main()
