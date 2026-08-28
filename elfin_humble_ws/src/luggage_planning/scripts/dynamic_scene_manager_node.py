#!/usr/bin/env python3
"""Sync dynamic observed obstacles from world_scene_mapper into MoveIt PlanningScene.

Keeps dynamic obstacle collision objects separate from static scene objects
(container, pedestal, placed boxes) managed by scene_manager_node.py.

Uses the world_scene_mapper's obstacle clusters to add/update/remove collision
boxes in the MoveIt PlanningScene, enabling reactive replanning when new
obstacles are detected.

Obstacle clusters whose center falls inside a robot self-filter body (arm,
suction panel, camera) are skipped before being added to the planning scene.
This prevents depth points that leaked past the perception self-filter from
becoming `dyn_obs_*` MoveIt boxes that collide with the robot's current state
and trigger START_STATE_IN_COLLISION (-10) planning failures.
"""

import os
import sys
import math

import rospy
import moveit_commander
import tf2_ros
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_commander import PlanningSceneInterface
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import MarkerArray


def _load_robot_self_filter(config_path):
    """Import RobotSelfPointFilter lazily and build a filter from YAML.

    Done as a function (not a top-level import) so that `rospkg` path
    resolution and the perception scripts import only happen when the
    dynamic scene manager actually needs the self-filter, keeping the
    module importable in test environments without a full ROS stack.
    """
    import rospkg  # noqa: WPS433

    desc_scripts = os.path.join(
        rospkg.RosPack().get_path("luggage_description"), "scripts"
    )
    perc_scripts = os.path.join(
        rospkg.RosPack().get_path("luggage_perception"), "scripts"
    )
    for path in (desc_scripts, perc_scripts):
        if path not in sys.path:
            sys.path.insert(0, path)

    from robot_self_point_filter import RobotSelfPointFilter  # noqa: E402,WPS433

    return RobotSelfPointFilter.load_yaml(config_path)


class DynamicSceneManager:
    OBSTACLE_PREFIX = "dyn_obs_"

    def __init__(self):
        self._base_frame = rospy.get_param("~base_frame", "elfin_base_link")
        self._obstacle_size = float(rospy.get_param("~obstacle_box_size", 0.08))
        self._update_rate = float(rospy.get_param("~update_rate", 1.0))
        self._min_obstacle_height = float(rospy.get_param("~min_obstacle_height", -0.3))
        self._max_obstacle_height = float(rospy.get_param("~max_obstacle_height", 2.0))
        self._cluster_radius = float(rospy.get_param("~cluster_radius", 0.1))
        self._max_obstacles = int(rospy.get_param("~max_obstacles", 50))

        self._scene = None
        self._active_ids = set()
        self._enabled = bool(rospy.get_param("~enabled", True))

        self._self_filter = self._build_self_filter()
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._dropped_robot_last = 0
        self._filter_missing_last = []
        # PR6: canonical task_cloud_filter status for secondary leak prevention.
        self._filter_status = {}
        self._filter_status_stamp = 0.0
        from std_msgs.msg import String as StringMsg
        rospy.Subscriber(
            "/task_cloud_filter/stats_json", StringMsg,
            self._on_filter_stats, queue_size=1)

        rospy.Service("~sync_dynamic_scene", Trigger, self._handle_sync)
        rospy.Service("~clear_dynamic_scene", Trigger, self._handle_clear)
        rospy.Service("~enable", Trigger, self._handle_enable)
        rospy.Service("~disable", Trigger, self._handle_disable)

        if self._update_rate > 0 and self._enabled:
            period = 1.0 / self._update_rate
            rospy.Timer(rospy.Duration(period), self._on_timer)

        rospy.loginfo(
            "dynamic_scene_manager ready: rate=%.1fHz obs_size=%.3f enabled=%s self_filter=%s",
            self._update_rate, self._obstacle_size, self._enabled,
            self._self_filter is not None,
        )

    def _build_self_filter(self):
        """Load the robot self-filter used to reject near-arm obstacle clusters."""
        import rospkg  # noqa: WPS433

        config_path = rospy.get_param(
            "~robot_self_filter_config",
            os.path.join(
                rospkg.RosPack().get_path("luggage_description"),
                "config",
                "robot_self_filter.yaml.example",
            ),
        )
        enabled = bool(rospy.get_param("~robot_self_filter/enabled", True))
        try:
            filt = _load_robot_self_filter(config_path)
            filt.enabled = enabled
            return filt
        except (IOError, OSError, ValueError, ImportError) as exc:
            rospy.logwarn(
                "dynamic_scene: robot self-filter unavailable (%s); "
                "near-arm obstacles will NOT be filtered",
                exc,
            )
            return None

    # ── PR6: canonical filter status + secondary leak prevention ───────

    def _on_filter_stats(self, msg):
        """Cache the latest task_cloud_filter stats_json."""
        import json
        try:
            self._filter_status = json.loads(msg.data)
            self._filter_status_stamp = rospy.Time.now().to_sec()
        except (ValueError, TypeError):
            rospy.logwarn_throttle(5.0, "dynamic_scene: invalid filter stats JSON")

    def _canonical_filter_ok(self):
        """Check whether the canonical task_cloud_filter is deployed and ready.

        Returns (ok, reason):
          - (True, "ok") if filter is ready or not deployed (backward-compat).
          - (False, reason) if deployed but not ready / model mismatch / stale.
        """
        import json
        fs = getattr(self, "_filter_status", {})
        if not fs:
            return True, "filter_not_deployed"  # backward-compat: no canonical filter
        stamp = getattr(self, "_filter_status_stamp", 0.0)
        if rospy.Time.now().to_sec() - stamp > 2.0:
            return False, "self_filter_unavailable"
        if not fs.get("ready", False):
            return False, fs.get("reason", "self_filter_not_ready")
        # Model hash consistency: filter's robot_model must match /robot_name.
        filter_model = fs.get("robot_model", "")
        expected_model = rospy.get_param("/robot_name", "")
        if expected_model and filter_model and filter_model != expected_model:
            return False, "model_mismatch"
        return True, "ok"

    def _ensure_scene(self):
        if self._scene is None:
            moveit_commander.roscpp_initialize([])
            self._scene = PlanningSceneInterface(synchronous=True)
        return self._scene

    def _get_obstacles_from_param(self):
        """Read world_scene obstacle positions from rosparam."""
        try:
            stats = rospy.get_param("/luggage/world_scene/stats", {})
            if not stats:
                return []
        except KeyError:
            return []

        obstacles_topic_data = []
        try:
            obstacle_count = rospy.get_param("/luggage/world_scene/obstacle_count", 0)
            if obstacle_count == 0:
                return []
        except KeyError:
            return []

        return None

    def _cluster_obstacles(self, points):
        """Simple grid-based clustering of occupied voxel centers."""
        if not points:
            return []
        grid = {}
        cell_size = self._cluster_radius
        for x, y, z in points:
            if z < self._min_obstacle_height or z > self._max_obstacle_height:
                continue
            key = (
                int(x / cell_size),
                int(y / cell_size),
                int(z / cell_size),
            )
            if key not in grid:
                grid[key] = []
            grid[key].append((x, y, z))

        clusters = []
        for key, pts in grid.items():
            cx = sum(p[0] for p in pts) / len(pts)
            cy = sum(p[1] for p in pts) / len(pts)
            cz = sum(p[2] for p in pts) / len(pts)
            size = max(self._obstacle_size, self._cluster_radius * len(pts) ** 0.33)
            clusters.append({"center": [cx, cy, cz], "size": size})

        clusters.sort(key=lambda c: -c["size"])
        return clusters[: self._max_obstacles]

    def _is_cluster_on_robot(self, center, size=0.0):
        """Return True if a base-frame cluster center lies inside a robot body.

        The cluster is represented in MoveIt as a cube. Expand each robot body
        by the cube's half diagonal so center-outside/body-overlap cases are
        rejected too. Missing filter geometry or TF fails closed.
        """
        if self._self_filter is None or not self._self_filter.enabled:
            self._filter_missing_last = ["self_filter_unavailable"]
            return True
        cx, cy, cz = center
        if not all(math.isfinite(v) for v in (cx, cy, cz)):
            return True
        extra_padding = max(0.0, float(size)) * math.sqrt(3.0) * 0.5
        intersects, missing = self._self_filter.point_intersects_robot(
            (cx, cy, cz), self._tf_buffer, rospy.Time(0),
            extra_padding=extra_padding,
        )
        self._filter_missing_last = list(missing)
        return bool(intersects or missing)

    def _sync_scene(self):
        """Read world scene obstacles and sync to MoveIt."""
        if not self._enabled:
            return 0

        # PR6: if the canonical task_cloud_filter is deployed but not ready,
        # the upstream cloud may contain unfiltered robot points. Fail closed:
        # remove stale obstacles but do NOT add new ones.
        canonical_ok, canonical_reason = self._canonical_filter_ok()
        if not canonical_ok:
            scene = self._ensure_scene()
            self._clear_all(scene)
            self._active_ids = set()
            rospy.logwarn_throttle(
                5.0, "dynamic_scene: canonical filter not ready (%s); "
                "skipped new obstacles this cycle", canonical_reason)
            return 0

        scene = self._ensure_scene()

        marker_data = rospy.get_param("/luggage/world_scene/obstacles_xyz", None)
        if marker_data is None:
            self._clear_all(scene)
            return 0

        clusters = self._cluster_obstacles(
            [(p[0], p[1], p[2]) for p in marker_data if len(p) == 3]
        )

        new_ids = set()
        dropped_robot = 0
        missing_frames = set()
        for i, cluster in enumerate(clusters):
            if self._is_cluster_on_robot(
                    cluster["center"], size=cluster["size"]):
                dropped_robot += 1
                missing_frames.update(self._filter_missing_last)
                continue
            obj_id = "%s%d" % (self.OBSTACLE_PREFIX, i)
            new_ids.add(obj_id)
            pose = PoseStamped()
            pose.header.frame_id = self._base_frame
            pose.pose.position = Point(
                x=cluster["center"][0],
                y=cluster["center"][1],
                z=cluster["center"][2],
            )
            pose.pose.orientation = Quaternion(w=1.0)
            size = cluster["size"]
            scene.add_box(obj_id, pose, size=(size, size, size))

        stale_ids = self._active_ids - new_ids
        for obj_id in stale_ids:
            scene.remove_world_object(obj_id)

        self._active_ids = new_ids
        if dropped_robot != self._dropped_robot_last:
            rospy.loginfo(
                "dynamic_scene: skipped %d robot/unclassified clusters%s",
                dropped_robot,
                (
                    " (missing=%s)" % ",".join(sorted(missing_frames))
                    if missing_frames else ""
                ),
            )
            self._dropped_robot_last = dropped_robot
        return len(new_ids)

    def _clear_all(self, scene=None):
        if scene is None:
            scene = self._ensure_scene()
        for obj_id in self._active_ids:
            scene.remove_world_object(obj_id)
        self._active_ids = set()

    def _on_timer(self, _event):
        if not self._enabled:
            return
        try:
            self._sync_scene()
        except Exception as exc:
            rospy.logwarn_throttle(10.0, "dynamic_scene sync failed: %s", exc)

    def _handle_sync(self, _req):
        count = self._sync_scene()
        return TriggerResponse(success=True, message="synced %d dynamic obstacles" % count)

    def _handle_clear(self, _req):
        self._clear_all()
        return TriggerResponse(success=True, message="cleared all dynamic obstacles")

    def _handle_enable(self, _req):
        self._enabled = True
        return TriggerResponse(success=True, message="dynamic scene enabled")

    def _handle_disable(self, _req):
        self._enabled = False
        self._clear_all()
        return TriggerResponse(success=True, message="dynamic scene disabled and cleared")



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
    rospy.init_node("dynamic_scene_manager", log_level=resolve_log_level())
    DynamicSceneManager()
    rospy.spin()


if __name__ == "__main__":
    main()
