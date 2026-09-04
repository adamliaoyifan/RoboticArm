#!/usr/bin/env python3
"""ROS node: plan next Cargo exploration view."""

from __future__ import division

import os
import sys
import math
import json
import time

import rospy
import rospkg
import tf2_ros
try:
    import tf2_geometry_msgs  # noqa: F401
except ImportError:  # Pure-Python harnesses do not install ROS TF adapters.
    tf2_geometry_msgs = None

from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.srv import (
    GetPositionIK,
    GetStateValidity,
    GetStateValidityRequest,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from luggage_msgs.srv import (
    EvaluateCargoViews,
    EvaluateCargoViewsRequest,
    GetCargoMapStats,
    PlanNextCargoView,
    PlanNextCargoViewResponse,
)

DESC_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_description"), "scripts")
PLAN_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_planning"), "scripts")
for path in (DESC_SCRIPTS, PLAN_SCRIPTS):
    if path not in sys.path:
        sys.path.insert(0, path)

from exploration_config_utils import (  # noqa: E402
    exploration_joint_names,
    load_exploration_config,
    smart_explore_config,
    interior_probe_config,
    view_planning_constraints,
    default_exploration_path,
    downward_constraints_config,
)
from scene_tf_config_utils import (  # noqa: E402
    container_in_base_link,
    container_inner_box_in_base_link,
    container_opening_aperture_corners,
    container_opening_axes_in_base_link,
    container_opening_dimensions,
    container_opening_in_base_link,
    load_scene_tf_config,
    resolve_scene_tf_config_path,
)

# RViz debug marker sizes for interior-probe scene tuning.
DEBUG_CANDIDATE_SPHERE = 0.12
DEBUG_CANDIDATE_LABEL = 0.09
DEBUG_APERTURE_LINE = 0.055
DEBUG_PROBE_PATH_LINE = 0.065
DEBUG_FRAME_AXIS_LEN = 0.45
DEBUG_FRAME_AXIS_WIDTH = 0.035
DEBUG_INNER_BOX_LINE = 0.028
DEBUG_FRAME_LABEL = 0.09
from container_aim_utils import look_at_quaternion  # noqa: E402
from constrained_view_planner import coverage_score  # noqa: E402
from interior_probe_planner import (  # noqa: E402
    evaluate_probe_termination,
    rank_probe_candidates,
)
from smart_explore_termination import (  # noqa: E402
    phase0_gain_exhausted,
    phase0_low_fov,
)
from luggage_planning.ik_diagnostics import format_event  # noqa: E402
from geometry_view_generator import (  # noqa: E402
    generate_opening_views,
    generate_interior_views,
    generate_interior_downward_views,
    generate_uncertainty_aware_corridor_views,
    filter_valid_views,
    filter_geometry_valid_views,
    filter_reachable_views,
    distance_from_base,
)
from downward_constraint_utils import (  # noqa: E402
    align_tilt_azimuth,
    compute_downward_orientations,
    feasibility_check,
)


def _filter_with_count(views, filter_fn, *args, **kwargs):
    """Apply a view filter and return (kept_views, rejected_count).

    Used by ``_build_smart_views`` to record per-stage filter diagnostics so
    an empty candidate pool can be explained instead of reported as a vague
    "no candidates".
    """
    kept = filter_fn(views, *args, **kwargs)
    return kept, len(views) - len(kept)


class CargoExplorationPlannerNode:
    def __init__(self):
        self._config = load_exploration_config(
            rospy.get_param("~exploration_config", default_exploration_path())
        )
        self._joint_names = exploration_joint_names(self._config)
        mapper_ns = rospy.get_param("~mapper_ns", "cargo_volume_mapper")
        rospy.wait_for_service("/%s/get_cargo_map_stats" % mapper_ns)
        self._get_stats = rospy.ServiceProxy(
            "/%s/get_cargo_map_stats" % mapper_ns, GetCargoMapStats
        )
        self._evaluate_views = rospy.ServiceProxy(
            "/%s/evaluate_cargo_views" % mapper_ns, EvaluateCargoViews
        )

        self._ik_service = rospy.get_param("~ik_service", "/compute_ik")
        self._ik_group = rospy.get_param("~ik_group", "elfin_arm")
        self._ik_link = rospy.get_param("~ik_link", "camera_depth_optical_frame")
        self._base_frame = rospy.get_param("~base_frame", "elfin_base_link")
        self._ik_timeout = float(rospy.get_param("~ik_timeout", 1.0))
        self._ik_attempts = int(rospy.get_param("~ik_attempts", 5))
        self._ik_wait_timeout = float(rospy.get_param("~ik_wait_timeout", 60.0))
        self._ik_diagnostics_enabled = bool(
            rospy.get_param("~ik_diagnostics/enabled", False))
        self._ik_diagnostics_run_id = str(
            rospy.get_param("~ik_diagnostics/run_id", ""))
        self._ik_diagnostics_sequence = 0
        self._ik = None
        self._state_validity_service = rospy.get_param(
            "~state_validity_service", "/check_state_validity")
        self._state_validity = rospy.ServiceProxy(
            self._state_validity_service, GetStateValidity)
        self._observe_seed = self._load_observe_seed()

        self._suction_frame = rospy.get_param("~suction_frame", "suction_contact_frame")
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        # Dual camera-down + suction-down path constraints for smart explore.
        # The suction-down camera orientation is derived from the fixed
        # camera<->suction TF so both axes can stay near base -Z despite their
        # ~12.25 deg fixed mount offset. See downward_constraint_utils.
        self._downward_cfg = downward_constraints_config(self._config)
        rospy.set_param(
            "/luggage/downward/align_before_probe",
            bool(self._downward_cfg["align_before_probe"]),
        )
        self._downward_orientations = None  # cached TF-derived quats
        self._downward_enabled = bool(self._downward_cfg["enabled"])
        self._init_downward_constraints()

        # Smart explore: geometry-driven two-phase viewpoint generation.
        self._smart_cfg = smart_explore_config(self._config)
        self._probe_cfg = interior_probe_config(self._config)
        self._loop_cfg = self._config.get("interior_loop", {}) or {}
        envelope_cfg = self._loop_cfg.get("camera_envelope", {}) or {}
        self._use_sensed_opening = bool(
            rospy.get_param("~use_sensed_opening", True))
        self._opening_hardware_strict = bool(
            rospy.get_param("~opening_hardware_strict", False))
        self._opening_geometry = None
        self._opening_geometry_version = 0
        self._corridor_min_confidence = float(
            rospy.get_param(
                "~corridor_min_confidence",
                self._loop_cfg.get("corridor_min_confidence", 0.95)))
        self._corridor_bootstrap_free_depth = float(
            rospy.get_param("~corridor_bootstrap_free_depth", 0.20))
        self._camera_half_width = float(
            rospy.get_param(
                "~camera_envelope/half_width",
                envelope_cfg.get("half_width", 0.08)))
        self._camera_half_height = float(
            rospy.get_param(
                "~camera_envelope/half_height",
                envelope_cfg.get("half_height", 0.05)))
        self._physical_clearance = float(
            rospy.get_param(
                "~camera_envelope/physical_clearance",
                envelope_cfg.get("physical_clearance", 0.05)))
        self._smart_views = []       # flat ordered list of candidate views
        self._smart_used = set()     # committed/accepted indices
        self._smart_rejected = set()
        self._smart_pending_index = None
        self._smart_committed_views = 0
        self._smart_rejection_counts = {}
        self._smart_active_lane = None
        self._smart_active_depth = 0.0
        self._smart_phase0_count = 0
        self._smart_phase1_count = 0
        # Phase0 (opening-arc) marginal-gain early stop: indices skipped once
        # gains dry up, tracked separately from _smart_rejected so they never
        # pollute the motion-rejection diagnostics counters.
        self._smart_phase0_skipped = set()
        self._phase0_used = 0
        self._phase0_last_unknown = None
        self._phase0_stagnant = 0
        self._smart_filter_stats = {}
        self._smart_last_status = "init"
        self._build_smart_views()

        self._probe_views = []
        self._probe_used = set()
        self._probe_pending_index = None
        self._probe_committed_views = 0
        self._probe_last_unknown = None
        self._probe_stagnant_count = 0
        self._scene_config = None
        self._build_interior_probe_views()

        self._debug_pub = rospy.Publisher(
            "/luggage/debug/explore_targets", MarkerArray, queue_size=1, latch=True
        )
        self._candidate_pub = rospy.Publisher(
            "/luggage/debug/explore_candidates", MarkerArray, queue_size=1, latch=True
        )
        self._aperture_pub = rospy.Publisher(
            "/luggage/debug/opening_aperture", MarkerArray, queue_size=1, latch=True
        )
        self._sensed_opening_pub = rospy.Publisher(
            "/luggage/debug/sensed_opening", MarkerArray, queue_size=1,
            latch=True,
        )
        self._selection_pub = rospy.Publisher(
            "~selection_diagnostics", String, queue_size=1, latch=True)
        self._marker_id = 0
        self._sensed_opening_stamp = None
        self._publish_probe_geometry()
        self._publish_sensed_opening(
            "prior_only",
            "sensed opening disabled" if not self._use_sensed_opening
            else "awaiting first estimate")
        from luggage_msgs.msg import ContainerOpeningEstimate
        rospy.Subscriber(
            rospy.get_param(
                "~opening_estimate_topic",
                "/container_opening_estimator/opening_estimate",
            ),
            ContainerOpeningEstimate,
            self._on_opening_estimate,
            queue_size=1,
        )

        rospy.Service("~plan_next_cargo_view", PlanNextCargoView, self.handle_plan)

    def _load_observe_seed(self):
        """Load observe joint values from robot_poses config as IK seed."""
        try:
            import rospkg
            desc_pkg = rospkg.RosPack().get_path("luggage_description")
            import yaml
            poses_path = os.path.join(desc_pkg, "config", "robot_poses.yaml.example")
            with open(poses_path) as f:
                poses_cfg = yaml.safe_load(f)
            observe = poses_cfg.get("poses", {}).get("observe", {})
            values = observe.get("values", [])
            if values:
                rospy.loginfo("Loaded observe seed for IK: %s", [round(v, 3) for v in values])
            return values
        except Exception as exc:
            rospy.logwarn("Cannot load observe seed: %s", exc)
            return []

    def _init_downward_constraints(self):
        """Eagerly verify the downward mount feasibility at startup.

        TF (camera<->suction) may not be up yet; the lazy
        ``_downward_orientation_quat`` retries on first use.
        """
        if not self._downward_enabled:
            rospy.loginfo("downward_constraints disabled in config")
            return
        if self._downward_orientation_quat() is not None:
            rospy.loginfo("downward_constraints ready at startup")

    def _downward_orientation_quat(self):
        """Return the suction-down camera quaternion (x, y, z, w) or None.

        Computed lazily from the fixed camera_depth_optical_frame <->
        suction_contact_frame TF and cached. This orientation tilts the camera
        by the inter-axis angle so the suction normal lands on base -Z, letting
        the dual camera/suction downward constraints be satisfied jointly.
        Returns None if downward is disabled, the mount is infeasible, or TF is
        unavailable (smart explore then falls back to look-at orientation).
        """
        if not self._downward_enabled:
            return None
        if self._downward_orientations is not None:
            return self._downward_orientations["camera_down_quat"]
        try:
            orientations = compute_downward_orientations(
                self._tf_buffer, self._ik_link, self._suction_frame,
                self._base_frame, timeout=2.0,
            )
        except Exception as exc:
            rospy.logwarn("downward_constraints: camera<->suction TF unavailable: %s", exc)
            return None
        ok, msg = feasibility_check(
            orientations["inter_axis_deg"],
            self._downward_cfg["camera_max_tilt_deg"],
            self._downward_cfg["suction_max_tilt_deg"],
        )
        if not ok:
            rospy.logerr("downward_constraints: %s; disabling strict-down", msg)
            self._downward_enabled = False
            return None
        orientations["camera_down_quat"] = self._aim_downward_quat_into_container(
            orientations["camera_down_quat"])
        self._downward_orientations = orientations
        rospy.loginfo("downward_constraints enabled: %s", msg)
        return orientations["camera_down_quat"]

    def _aim_downward_quat_into_container(self, camera_down_quat):
        """Re-yaw the suction-down camera quat so its residual tilt points
        into the container instead of an arbitrary (mount-dependent) azimuth.

        ``quaternion_align_vectors`` (used to build ``camera_down_quat``) is a
        minimal rotation with a free yaw about the down axis; left uncorrected
        it can point the camera's small residual tilt at the robot's own base
        instead of the container interior (observed: opening-arc views seeing
        mostly the pedestal, unknown_ratio barely moving after each view).
        ``align_tilt_azimuth`` only changes the horizontal direction of that
        residual tilt (see ``yaw_about_down``): it cannot change the achieved
        camera/suction tilt-from-down, so it cannot violate the budgets
        ``feasibility_check`` already validated above.

        Independent scene_tf load: this can run before ``self._scene_config``
        exists (``_init_downward_constraints`` fires at startup, ahead of
        ``_build_smart_views``). A load failure falls back to the unaligned
        (but still tilt-valid) quat rather than blocking downward-orientation
        altogether.
        """
        try:
            scene_config = self._load_scene_config()
            normal, _lateral, _up = container_opening_axes_in_base_link(
                scene_config)
        except Exception as exc:
            rospy.logwarn(
                "downward_constraints: cannot load scene_tf to aim camera "
                "into container (%s); keeping unaligned tilt azimuth", exc)
            return camera_down_quat
        toward_container = (-float(normal[0]), -float(normal[1]))
        return align_tilt_azimuth(camera_down_quat, toward_container)

    @staticmethod
    def _pose_axes(pose):
        q = pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm < 1e-12:
            raise ValueError("opening estimate has zero quaternion")
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        rotation = (
            (1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
             2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
             2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w),
             1 - 2 * (x * x + y * y)),
        )
        return (
            [rotation[i][0] for i in range(3)],
            [rotation[i][1] for i in range(3)],
            [rotation[i][2] for i in range(3)],
        )

    def _on_opening_estimate(self, msg):
        """Cache fresh sensed geometry in base frame and rebuild phase1."""
        if not self._use_sensed_opening:
            return
        if not msg.valid:
            if self._opening_hardware_strict:
                self._opening_geometry = None
                self._build_smart_views()
            self._publish_sensed_opening(
                "rejected", getattr(msg, "rejection_reason", "") or "invalid")
            return
        if (
                self._opening_geometry is not None
                and int(msg.geometry_version)
                == self._opening_geometry_version):
            self._opening_geometry["age"] = max(
                0.0, (rospy.Time.now() - msg.header.stamp).to_sec())
            self._opening_geometry["confidence"] = float(msg.confidence)
            self._refresh_sensed_opening_markers()
            return
        try:
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = msg.opening_pose.pose
            if msg.header.frame_id != self._base_frame:
                # Use latest TF: startup estimates often carry sim-time stamps
                # before the camera chain is published.
                pose_stamped.header.stamp = rospy.Time(0)
                pose_stamped = self._tf_buffer.transform(
                    pose_stamped, self._base_frame, rospy.Duration(0.5))
            lateral, up, normal = self._pose_axes(pose_stamped.pose)
            points = list(msg.aperture.points)
            if len(points) < 3:
                raise ValueError("opening aperture has fewer than three points")
            aperture_width = max(p.x for p in points) - min(
                p.x for p in points)
            aperture_height = max(p.y for p in points) - min(
                p.y for p in points)
            try:
                scene_config = self._load_scene_config()
                scene_w, scene_h = container_opening_dimensions(scene_config)
                aperture_width = max(aperture_width, float(scene_w))
                aperture_height = max(aperture_height, float(scene_h))
            except Exception:
                pass
            covariance = list(msg.opening_pose.covariance)
            sigma = math.sqrt(max(
                0.0, covariance[0], covariance[7], covariance[14]))
            if msg.source == "depth_only":
                sigma = min(sigma, 0.02)
            age = max(
                0.0, (rospy.Time.now() - msg.header.stamp).to_sec())
            self._opening_geometry = {
                "opening_xyz": [
                    pose_stamped.pose.position.x,
                    pose_stamped.pose.position.y,
                    pose_stamped.pose.position.z,
                ],
                "normal": normal,
                "lateral": lateral,
                "up": up,
                "aperture_width": aperture_width,
                "aperture_height": aperture_height,
                "inner_depth": max(float(msg.inner_size[0]),
                                   float(msg.inner_size[1])),
                "uncertainty_margin": 3.0 * sigma,
                "geometry_version": int(msg.geometry_version),
                "source": msg.source,
                "age": age,
                "confidence": float(msg.confidence),
            }
            self._opening_geometry_version = int(msg.geometry_version)
            self._build_smart_views()
            self._publish_sensed_opening("valid")
            rospy.loginfo(
                "smart_explore rebuilt from sensed opening g%d "
                "(source=%s confidence=%.2f uncertainty=%.3fm)",
                msg.geometry_version, msg.source, msg.confidence, 3.0 * sigma,
            )
        except Exception as exc:
            rospy.logwarn_throttle(
                2.0, "Cannot consume opening estimate: %s", exc)
            self._publish_sensed_opening("rejected", str(exc))

    def _refresh_sensed_opening_markers(self, period=1.0):
        """Republish the sensed opening at most once per period.

        Estimates arrive far faster than the geometry changes; only the age
        and confidence in the label move, so throttle instead of flooding.
        """
        now = rospy.Time.now()
        last = getattr(self, "_sensed_opening_stamp", None)
        if last is not None and (now - last).to_sec() < period:
            return
        self._sensed_opening_stamp = now
        self._publish_sensed_opening("valid")

    def _build_smart_views(self):
        """Generate and pre-filter all candidate views for smart mode.

        Phase 0 (opening) views are generated first and IK-validated up
        front so the orchestrator can sequence them without per-view IK
        latency. Phase 1 (interior) views are generated and tilt/reach
        filtered but NOT IK-validated — they are solved on demand during
        NBV selection so the frontier signal can rank them first.
        """
        old_views = list(getattr(self, "_smart_views", []))
        old_used_names = {
            old_views[i].get("candidate_id", old_views[i]["name"])
            for i in getattr(self, "_smart_used", set())
            if 0 <= i < len(old_views)
        }
        old_rejected_names = {
            old_views[i].get("candidate_id", old_views[i]["name"])
            for i in getattr(self, "_smart_rejected", set())
            if 0 <= i < len(old_views)
        }
        old_phase0_skipped_names = {
            old_views[i].get("candidate_id", old_views[i]["name"])
            for i in getattr(self, "_smart_phase0_skipped", set())
            if 0 <= i < len(old_views)
        }
        cfg = self._smart_cfg
        if not cfg["enabled"]:
            rospy.loginfo("smart_explore disabled in config; smart mode will return done")
            return

        try:
            scene_tf_path = rospy.get_param(
                "~scene_tf_config",
                rospy.get_param("/luggage/scene_tf_config", resolve_scene_tf_config_path()),
            )
            scene_config = load_scene_tf_config(scene_tf_path)
        except Exception as exc:
            rospy.logwarn("smart_explore: cannot load scene_tf config: %s", exc)
            return

        p0 = cfg["phase0"]
        p1 = cfg["phase1"]
        reach = cfg["arm_reach"]
        opening_geometry = getattr(self, "_opening_geometry", None)

        # Phase 0: opening arc.
        opening_views = generate_opening_views(
            scene_config,
            num_views=p0["num_views"],
            arc_radius=p0["arc_radius"],
            height_above_opening=p0["height_above_opening"],
            max_tilt_deg=p0["max_tilt_deg"],
        )
        if self._downward_enabled and self._downward_orientations is not None:
            down_q = self._downward_orientations["camera_down_quat"]
            down_tilt = float(
                self._downward_orientations["inter_axis_deg"])
            for view in opening_views:
                # The executed phase0 orientation is suction-down, not the
                # legacy look-at quaternion generated above. Filter against
                # that real orientation so valid opening positions are not
                # discarded before the override is applied.
                view["orientation_quat"] = list(down_q)
                view["tilt_deg"] = down_tilt
                view["valid_tilt"] = (
                    down_tilt <= p0["max_tilt_deg"] + 1e-6)
        p0_generated = len(opening_views)
        opening_views, p0_tilt_rejected = _filter_with_count(
            opening_views, filter_valid_views)
        opening_views, p0_reach_rejected = _filter_with_count(
            opening_views, filter_reachable_views, max_reach=reach)

        # Phase 1: interior grid. In camera_down_mode (strict-down), use the
        # camera-down interior geometry whose optical origin enters the box;
        # the planner overrides each candidate's orientation with the
        # TF-derived suction-down quaternion. Otherwise use the tilted
        # look-into-interior grid.
        if (
                opening_geometry is not None
                and p1.get("camera_down_mode")
                and self._downward_enabled):
            down_q = self._downward_orientation_quat()
            interior_views = generate_uncertainty_aware_corridor_views(
                opening_geometry,
                down_q or [1.0, 0.0, 0.0, 0.0],
                observed_free_depth=opening_geometry["inner_depth"],
                num_lateral=p1["lateral_steps"],
                min_depth=self._probe_cfg["depth_min_from_opening"],
                depth_step=float(getattr(
                    self, "_loop_cfg", {}).get("depth_step", 0.15)),
                max_depth=(
                    opening_geometry["inner_depth"]
                    * self._probe_cfg["depth_max_ratio"]),
                camera_half_width=self._camera_half_width,
                camera_half_height=self._camera_half_height,
                physical_clearance=self._physical_clearance,
                uncertainty_margin=opening_geometry[
                    "uncertainty_margin"],
            )
            if not interior_views:
                rospy.logwarn(
                    "smart_explore: sensed corridor empty "
                    "(source=%s w=%.2f h=%.2f); "
                    "falling back to scene_tf interior grid",
                    opening_geometry.get("source", "unknown"),
                    opening_geometry.get("aperture_width", 0.0),
                    opening_geometry.get("aperture_height", 0.0),
                )
                interior_views = generate_interior_downward_views(
                    scene_config,
                    num_lateral=p1["lateral_steps"],
                    num_depth=p1["height_steps"],
                )
                if down_q is not None:
                    for view in interior_views:
                        view["orientation_quat"] = list(down_q)
                interior_views = filter_geometry_valid_views(interior_views)
        elif (
                getattr(self, "_opening_hardware_strict", False)
                and getattr(self, "_use_sensed_opening", False)):
            interior_views = []
        elif p1.get("camera_down_mode") and self._downward_enabled:
            interior_views = generate_interior_downward_views(
                scene_config,
                num_lateral=p1["lateral_steps"],
                num_depth=p1["height_steps"],
            )
            interior_views = filter_geometry_valid_views(interior_views)
        else:
            interior_views = generate_interior_views(
                scene_config,
                num_lateral=p1["lateral_steps"],
                num_height=p1["height_steps"],
                standoff_values=p1["standoff_values"],
                max_tilt_deg=p1["max_tilt_deg"],
                look_depth_ratio=p1["look_depth_ratio"],
            )
            interior_views = filter_valid_views(interior_views)
        p1_generated = len(interior_views)
        interior_views, p1_reach_rejected = _filter_with_count(
            interior_views, filter_reachable_views, max_reach=reach)

        self._smart_phase0_count = len(opening_views)
        self._smart_phase1_count = len(interior_views)
        new_views = opening_views + interior_views
        self._smart_filter_stats = {
            "p0_generated": p0_generated,
            "p0_tilt_rejected": p0_tilt_rejected,
            "p0_reach_rejected": p0_reach_rejected,
            "p1_generated": p1_generated,
            "p1_reach_rejected": p1_reach_rejected,
        }
        if not new_views and old_views:
            # Don't silently clear a valid pool with an empty rebuild (e.g. a
            # depth-only sensed opening eroded the aperture to nothing). Keep
            # the previous pool so exploration can continue with prior candidates.
            rospy.logwarn(
                "smart_explore: rebuild produced 0 candidates (phase0=%d "
                "phase1=%d, filter=%s); keeping previous %d candidates",
                len(opening_views), len(interior_views),
                self._smart_filter_stats, len(old_views))
            self._smart_views = old_views
            self._smart_last_status = "pool_rebuild_empty_kept_old"
        else:
            self._smart_views = new_views
            self._smart_last_status = (
                "pool_empty" if not new_views else "pool_ok")
            identity_to_index = {
                view.get("candidate_id", view["name"]): i
                for i, view in enumerate(self._smart_views)
            }
            self._smart_used = {
                identity_to_index[name] for name in old_used_names
                if name in identity_to_index
            }
            self._smart_rejected = {
                identity_to_index[name] for name in old_rejected_names
                if name in identity_to_index
            }
            self._smart_phase0_skipped = {
                identity_to_index[name] for name in old_phase0_skipped_names
                if name in identity_to_index
            }
            self._smart_pending_index = None
        rospy.set_param("/luggage/workspace/explore_summary", {
            "phase0_candidates": len(opening_views),
            "phase1_candidates": len(interior_views),
            "candidate_count": len(self._smart_views),
            "arm_reach": float(reach),
            "camera_down_mode": bool(
                p1.get("camera_down_mode") and self._downward_enabled),
            "used": 0,
            "rejected": 0,
            "phase0_skipped": len(self._smart_phase0_skipped),
            "filter_stats": self._smart_filter_stats,
            "status": self._smart_last_status,
        })
        rospy.loginfo(
            "smart_explore: %d phase0 (opening) + %d phase1 (interior%s) = %d candidates "
            "(reach<=%.1fm, tilt p0<=%.0f deg p1<=%.0f deg, filter=%s, status=%s)",
            len(opening_views), len(interior_views),
            " camera_down" if p1.get("camera_down_mode") and self._downward_enabled else "",
            len(self._smart_views),
            reach, p0["max_tilt_deg"], p1["max_tilt_deg"],
            self._smart_filter_stats, self._smart_last_status,
        )

    def _load_scene_config(self):
        path = rospy.get_param(
            "~scene_tf_config",
            rospy.get_param("/luggage/scene_tf_config", resolve_scene_tf_config_path()),
        )
        return load_scene_tf_config(path)

    def _build_interior_probe_views(self):
        """Build camera-down candidates, retaining invalid ones for diagnostics."""
        if not self._probe_cfg["enabled"]:
            return
        try:
            self._scene_config = self._load_scene_config()
            cfg = self._probe_cfg
            self._probe_views = generate_interior_downward_views(
                self._scene_config,
                num_lateral=cfg["lateral_steps"],
                num_depth=cfg["depth_steps"],
                camera_z=cfg["camera_z"],
                depth_min_from_opening=cfg["depth_min_from_opening"],
                depth_max_ratio=cfg["depth_max_ratio"],
                wall_clearance=cfg["wall_clearance"],
                aperture_margin=cfg["aperture_margin"],
                look_down=cfg["look_down"],
            )
            if self._downward_enabled:
                down_q = self._downward_orientation_quat()
                if down_q is not None:
                    for view in self._probe_views:
                        view["orientation_quat"] = list(down_q)
            valid_count = len(filter_geometry_valid_views(self._probe_views))
            rospy.loginfo(
                "interior_probe: built %d candidates (%d geometry-valid)",
                len(self._probe_views), valid_count,
            )
        except Exception as exc:
            self._probe_views = []
            rospy.logwarn("interior_probe: cannot build candidates: %s", exc)

    @staticmethod
    def _pose_for_view(view):
        q = view["orientation_quat"]
        xyz = view["camera_xyz"]
        return Pose(
            position=Point(x=xyz[0], y=xyz[1], z=xyz[2]),
            orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
        )

    def _probe_waypoints(self, view):
        """Return pre-opening, aperture and internal optical-frame poses."""
        normal = view.get("opening_normal")
        if normal is None:
            normal, _lateral, _vertical = container_opening_axes_in_base_link(
                self._scene_config
            )
        aperture = list(view["aperture_xyz"])
        pre = [
            aperture[i] + normal[i] * self._probe_cfg["opening_clearance"]
            for i in range(3)
        ]
        q = view["orientation_quat"]

        def make_pose(xyz):
            return Pose(
                position=Point(x=xyz[0], y=xyz[1], z=xyz[2]),
                orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
            )
        return [make_pose(pre), make_pose(aperture), self._pose_for_view(view)]

    @staticmethod
    def _pose_xyz(pose):
        return [pose.position.x, pose.position.y, pose.position.z]

    def _probe_entry_waypoints_reachable(self, view, waypoints, ik_cfg,
                                         seed_values=None,
                                         diagnostic_context=None):
        """Check entry waypoints before returning a probe path for execution.

        The final internal view IK is not enough: the aperture/pre-opening
        points can be farther from the robot than the internal probe and would
        otherwise fail later as a long MoveIt pose-planning timeout.
        """
        seed = seed_values if seed_values is not None else self._observe_seed
        checks = (
            ("pre_ik_failed", waypoints[0]),
            ("aperture_ik_failed", waypoints[1]),
        )
        solutions = []
        for reason, pose in checks:
            xyz = self._pose_xyz(pose)
            context = dict(diagnostic_context or {})
            context["waypoint"] = reason.replace("_ik_failed", "")
            joints = self._solve_view_ik(
                xyz,
                [xyz[0], xyz[1], xyz[2] - self._probe_cfg["look_down"]],
                seed_values=seed,
                timeout=ik_cfg["timeout"],
                attempts=ik_cfg["attempts"],
                avoid_collisions=ik_cfg["avoid_collisions"],
                orientation_quat=view["orientation_quat"],
                diagnostic_context=context,
            )
            if joints is None:
                return False, reason, [], []
            solutions.append(joints)
            seed = joints
        return True, "", solutions[0], solutions[1]

    @staticmethod
    def _rpy_rotation_matrix(roll, pitch, yaw):
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]

    @classmethod
    def _rpy_axis_directions(cls, rpy):
        rot = cls._rpy_rotation_matrix(*rpy)
        return (
            [rot[i][0] for i in range(3)],
            [rot[i][1] for i in range(3)],
            [rot[i][2] for i in range(3)],
        )

    @staticmethod
    def _append_axis_triplet(
            marker_array, stamp, frame_id, origin, axis_dirs, ns, marker_id,
            axis_length, line_width):
        colors = (
            ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0),
            ColorRGBA(r=0.2, g=1.0, b=0.2, a=1.0),
            ColorRGBA(r=0.2, g=0.4, b=1.0, a=1.0),
        )
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation = Quaternion(w=1.0)
        marker.scale.x = line_width
        marker.lifetime = rospy.Duration(0)
        for axis_dir, color in zip(axis_dirs, colors):
            marker.points.append(Point(x=origin[0], y=origin[1], z=origin[2]))
            marker.points.append(Point(
                x=origin[0] + axis_dir[0] * axis_length,
                y=origin[1] + axis_dir[1] * axis_length,
                z=origin[2] + axis_dir[2] * axis_length,
            ))
            marker.colors.extend([color, color])
        marker_array.markers.append(marker)

    @staticmethod
    def _append_text_marker(
            marker_array, stamp, frame_id, position, text, ns, marker_id,
            scale, color):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position = Point(x=position[0], y=position[1], z=position[2])
        marker.pose.orientation = Quaternion(w=1.0)
        marker.scale.z = scale
        marker.color = color
        marker.text = text
        marker.lifetime = rospy.Duration(0)
        marker_array.markers.append(marker)

    @staticmethod
    def _append_aabb_wireframe(
            marker_array, stamp, frame_id, min_corner, max_corner, ns, marker_id,
            color, line_width):
        corners = [
            [min_corner[0], min_corner[1], min_corner[2]],
            [max_corner[0], min_corner[1], min_corner[2]],
            [max_corner[0], max_corner[1], min_corner[2]],
            [min_corner[0], max_corner[1], min_corner[2]],
            [min_corner[0], min_corner[1], max_corner[2]],
            [max_corner[0], min_corner[1], max_corner[2]],
            [max_corner[0], max_corner[1], max_corner[2]],
            [min_corner[0], max_corner[1], max_corner[2]],
        ]
        edges = (
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        )
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation = Quaternion(w=1.0)
        marker.scale.x = line_width
        marker.color = color
        marker.lifetime = rospy.Duration(0)
        for start_idx, end_idx in edges:
            start = corners[start_idx]
            end = corners[end_idx]
            marker.points.append(Point(x=start[0], y=start[1], z=start[2]))
            marker.points.append(Point(x=end[0], y=end[1], z=end[2]))
        marker_array.markers.append(marker)

    def _append_frame_debug_markers(self, marker_array, stamp):
        """Publish container/opening frame axes, labels, and inner volume bounds."""
        scene = self._scene_config
        container_xyz, container_rpy = container_in_base_link(scene)
        opening_xyz, opening_rpy = container_opening_in_base_link(scene)
        normal, _lateral, _vertical = container_opening_axes_in_base_link(scene)
        inner_min, inner_max = container_inner_box_in_base_link(scene)

        self._append_axis_triplet(
            marker_array, stamp, self._base_frame, container_xyz,
            self._rpy_axis_directions(container_rpy),
            "container_frame_axes", 10, DEBUG_FRAME_AXIS_LEN, DEBUG_FRAME_AXIS_WIDTH,
        )
        self._append_axis_triplet(
            marker_array, stamp, self._base_frame, opening_xyz,
            self._rpy_axis_directions(opening_rpy),
            "opening_frame_axes", 20, DEBUG_FRAME_AXIS_LEN, DEBUG_FRAME_AXIS_WIDTH,
        )
        self._append_aabb_wireframe(
            marker_array, stamp, self._base_frame, inner_min, inner_max,
            "container_inner_box", 30,
            ColorRGBA(r=0.8, g=0.5, b=1.0, a=0.8), DEBUG_INNER_BOX_LINE,
        )

        label_color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        clearance = float(self._probe_cfg.get("opening_clearance", 0.20))
        margin = float(self._probe_cfg.get("aperture_margin", 0.10))
        labels = (
            (40, container_xyz, "container_link"),
            (41, opening_xyz, "container_opening_frame"),
            (42, [
                opening_xyz[i] + normal[i] * 0.12 for i in range(3)
            ], "opening_normal"),
            (43, [
                opening_xyz[0], opening_xyz[1], opening_xyz[2] + 0.18,
            ], "opening_clearance=%.2f aperture_margin=%.2f" % (clearance, margin)),
        )
        for marker_id, position, text in labels:
            self._append_text_marker(
                marker_array, stamp, self._base_frame, position, text,
                "frame_labels", marker_id, DEBUG_FRAME_LABEL, label_color,
            )

    @staticmethod
    def _append_rect_outline(
            marker_array, stamp, frame_id, center, lateral, up,
            half_width, half_height, ns, marker_id, color, line_width):
        """Draw a rectangle spanned by two in-plane axes around a center."""
        corners = []
        for sign_w, sign_h in ((1, 1), (-1, 1), (-1, -1), (1, -1), (1, 1)):
            corners.append(Point(
                x=center[0] + lateral[0] * sign_w * half_width
                + up[0] * sign_h * half_height,
                y=center[1] + lateral[1] * sign_w * half_width
                + up[1] * sign_h * half_height,
                z=center[2] + lateral[2] * sign_w * half_width
                + up[2] * sign_h * half_height,
            ))
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation = Quaternion(w=1.0)
        marker.scale.x = line_width
        marker.color = color
        marker.points = corners
        marker.lifetime = rospy.Duration(0)
        marker_array.markers.append(marker)

    def _publish_sensed_opening(self, status="valid", reason=""):
        """Publish what Phase 0 actually sensed and what Phase 1 may use.

        Overlays the sensed aperture, its 3-sigma shrink and the resulting
        Phase 1 insertion window on top of the static ``scene_tf`` geometry
        drawn by /luggage/debug/opening_aperture, so calibration drift and the
        ``max(sensed, scene)`` aperture fallback are visible side by side.
        """
        publisher = getattr(self, "_sensed_opening_pub", None)
        if publisher is None:
            return
        stamp = rospy.Time.now()
        white = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        colors = {
            "valid": ColorRGBA(r=0.1, g=1.0, b=0.3, a=1.0),
            "rejected": ColorRGBA(r=1.0, g=0.15, b=0.1, a=1.0),
            "prior_only": ColorRGBA(r=1.0, g=0.85, b=0.0, a=1.0),
        }
        color = colors.get(status, colors["prior_only"])
        geometry = self._opening_geometry
        ma = MarkerArray()

        if geometry is None:
            anchor = [0.0, 0.0, 0.0]
            try:
                anchor = list(container_opening_in_base_link(
                    self._load_scene_config())[0])
            except Exception:
                pass
            self._append_text_marker(
                ma, stamp, self._base_frame,
                [anchor[0], anchor[1], anchor[2] + 0.25],
                "sensed opening: %s\n%s" % (status, reason or "no estimate"),
                "sensed_opening_status", 0, DEBUG_FRAME_LABEL, color,
            )
            publisher.publish(ma)
            return

        center = [float(v) for v in geometry["opening_xyz"]]
        lateral = [float(v) for v in geometry["lateral"]]
        up = [float(v) for v in geometry["up"]]
        normal = [float(v) for v in geometry["normal"]]
        half_width = 0.5 * float(geometry["aperture_width"])
        half_height = 0.5 * float(geometry["aperture_height"])
        uncertainty = float(geometry.get("uncertainty_margin", 0.0))

        self._append_rect_outline(
            ma, stamp, self._base_frame, center, lateral, up,
            half_width, half_height, "sensed_aperture", 0, color,
            DEBUG_APERTURE_LINE)
        if uncertainty > 0.0:
            self._append_rect_outline(
                ma, stamp, self._base_frame, center, lateral, up,
                max(0.0, half_width - uncertainty),
                max(0.0, half_height - uncertainty),
                "sensed_uncertainty", 1,
                ColorRGBA(r=1.0, g=0.6, b=0.0, a=0.9), DEBUG_APERTURE_LINE)

        # Same formula as generate_uncertainty_aware_corridor_views(): what
        # Phase 1 is actually allowed to fly the camera through.
        clearance = self._physical_clearance + uncertainty
        safe_half_width = half_width - self._camera_half_width - clearance
        safe_half_height = half_height - self._camera_half_height - clearance
        if safe_half_width > 0.0 and safe_half_height > 0.0:
            self._append_rect_outline(
                ma, stamp, self._base_frame, center, lateral, up,
                safe_half_width, safe_half_height,
                "sensed_insertion_window", 2,
                ColorRGBA(r=0.0, g=0.9, b=1.0, a=1.0), DEBUG_APERTURE_LINE)
            window_text = "insertion window %.3fx%.3f m" % (
                2.0 * safe_half_width, 2.0 * safe_half_height)
        else:
            window_text = "insertion window CLOSED (camera + clearance)"

        self._append_axis_triplet(
            ma, stamp, self._base_frame, center, (lateral, up, normal),
            "sensed_opening_axes", 3,
            DEBUG_FRAME_AXIS_LEN * 0.5, DEBUG_FRAME_AXIS_WIDTH * 0.6)

        self._append_text_marker(
            ma, stamp, self._base_frame,
            [center[0], center[1], center[2] + half_height + 0.12],
            "sensed opening g%d [%s]\n"
            "source=%s confidence=%.2f age=%.2fs\n"
            "aperture %.3fx%.3f m  3sigma=%.3f m\n%s%s" % (
                int(geometry.get("geometry_version", 0)),
                status,
                geometry.get("source", "unknown"),
                float(geometry.get("confidence", 0.0)),
                float(geometry.get("age", 0.0)),
                2.0 * half_width, 2.0 * half_height, uncertainty,
                window_text,
                "\nrejected: %s" % reason if reason else "",
            ),
            "sensed_opening_status", 4, DEBUG_FRAME_LABEL,
            color if status != "valid" else white,
        )
        publisher.publish(ma)

    def _publish_probe_geometry(self, selected_view=None, candidate_states=None):
        """Publish candidate grid, aperture safe window and selected probe path."""
        if self._scene_config is None:
            try:
                self._scene_config = self._load_scene_config()
            except Exception as exc:
                rospy.logwarn("probe geometry: scene config unavailable: %s", exc)
                return
        stamp = rospy.Time.now()
        states = candidate_states or {}
        if self._probe_views:
            ma = MarkerArray()
            for idx, view in enumerate(self._probe_views):
                marker = Marker()
                marker.header.frame_id = self._base_frame
                marker.header.stamp = stamp
                marker.ns = "probe_candidates"
                marker.id = idx
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position = Point(
                    x=view["camera_xyz"][0],
                    y=view["camera_xyz"][1],
                    z=view["camera_xyz"][2],
                )
                marker.pose.orientation = Quaternion(w=1.0)
                marker.scale.x = marker.scale.y = marker.scale.z = DEBUG_CANDIDATE_SPHERE
                state = states.get(idx, "")
                if selected_view is view:
                    marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
                elif not view.get("valid_geometry", True) or state == "ik_failed":
                    marker.color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.9)
                elif state == "low_score" or state.startswith("score="):
                    marker.color = ColorRGBA(r=1.0, g=0.75, b=0.0, a=0.8)
                else:
                    marker.color = ColorRGBA(r=0.65, g=0.65, b=0.65, a=0.65)
                marker.lifetime = rospy.Duration(0)
                ma.markers.append(marker)
                label = Marker()
                label.header.frame_id = self._base_frame
                label.header.stamp = stamp
                label.ns = "probe_candidate_labels"
                label.id = 1000 + idx
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose.position = Point(
                    x=view["camera_xyz"][0],
                    y=view["camera_xyz"][1],
                    z=view["camera_xyz"][2] + 0.14,
                )
                label.pose.orientation = Quaternion(w=1.0)
                label.scale.z = DEBUG_CANDIDATE_LABEL
                reason = state or view.get("reject_reason", "") or "candidate"
                label.text = "%s: %s" % (view["name"], reason)
                label.color = marker.color
                label.lifetime = rospy.Duration(0)
                ma.markers.append(label)
            self._candidate_pub.publish(ma)

        aperture_ma = MarkerArray()
        self._append_frame_debug_markers(aperture_ma, stamp)
        for marker_id, margin, color, ns in (
                (0, 0.0, ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0), "opening"),
                (1, self._probe_cfg["aperture_margin"],
                 ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0), "opening_safe")):
            corners = container_opening_aperture_corners(
                self._scene_config, margin=margin
            )
            line = Marker()
            line.header.frame_id = self._base_frame
            line.header.stamp = stamp
            line.ns = ns
            line.id = marker_id
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.points = [
                Point(x=p[0], y=p[1], z=p[2]) for p in corners + [corners[0]]
            ]
            line.scale.x = DEBUG_APERTURE_LINE
            line.color = color
            line.pose.orientation = Quaternion(w=1.0)
            line.lifetime = rospy.Duration(0)
            aperture_ma.markers.append(line)

        if selected_view is not None:
            waypoints = self._probe_waypoints(selected_view)
            path = Marker()
            path.header.frame_id = self._base_frame
            path.header.stamp = stamp
            path.ns = "probe_path"
            path.id = 2
            path.type = Marker.LINE_STRIP
            path.action = Marker.ADD
            path.points = [pose.position for pose in waypoints]
            path.scale.x = DEBUG_PROBE_PATH_LINE
            path.color = ColorRGBA(r=0.1, g=1.0, b=0.2, a=1.0)
            path.pose.orientation = Quaternion(w=1.0)
            path.lifetime = rospy.Duration(0)
            aperture_ma.markers.append(path)
        self._aperture_pub.publish(aperture_ma)

    def _ensure_ik(self):
        if self._ik is not None:
            return self._ik
        try:
            rospy.loginfo("Waiting for IK service %s (timeout %.0fs)...",
                          self._ik_service, self._ik_wait_timeout)
            rospy.wait_for_service(self._ik_service, timeout=self._ik_wait_timeout)
            self._ik = rospy.ServiceProxy(self._ik_service, GetPositionIK)
            rospy.loginfo("IK service %s ready", self._ik_service)
        except rospy.ROSException:
            rospy.logwarn("IK service %s unavailable after %.0fs",
                          self._ik_service, self._ik_wait_timeout)
        return self._ik

    def _current_joint_state(self):
        """Live arm state for IK seeding.

        MoveIt rejects a name-less JointState with an ERROR log on every call,
        so an unseeded IK request is both noisy and starts from the model
        default rather than from where the arm actually is.
        """
        try:
            return rospy.wait_for_message(
                "/joint_states", JointState, timeout=1.0)
        except rospy.ROSException:
            rospy.logwarn_throttle(
                30.0, "no /joint_states for IK seeding; using model default")
            return JointState()

    def _emit_ik_diagnostic(self, payload):
        if not self._ik_diagnostics_enabled:
            return
        self._ik_diagnostics_sequence += 1
        event = dict(payload)
        event.update({
            "schema_version": 1,
            "run_id": self._ik_diagnostics_run_id,
            "sequence": self._ik_diagnostics_sequence,
        })
        rospy.loginfo(format_event(event))

    def _solve_view_ik(self, camera_xyz, look_at, seed_values=None,
                       timeout=None, attempts=None, avoid_collisions=None,
                       orientation_quat=None, diagnostic_context=None):
        """Solve IK for camera_depth_optical_frame at camera_xyz looking at look_at.

        ``timeout``, ``attempts``, and ``avoid_collisions`` override the
        node-level defaults (used by smart mode to do fast 0.2s / 3-attempt
        pre-filtering instead of the slow 1.0s / 5-attempt default).
        """
        ik = self._ensure_ik()
        if ik is None:
            return None

        q = (
            [float(v) for v in orientation_quat]
            if orientation_quat is not None
            else look_at_quaternion(camera_xyz, look_at)
        )
        req = PositionIKRequest()
        req.group_name = self._ik_group
        req.ik_link_name = self._ik_link
        req.pose_stamped = PoseStamped()
        req.pose_stamped.header.frame_id = self._base_frame
        req.pose_stamped.pose = Pose(
            position=Point(x=camera_xyz[0], y=camera_xyz[1], z=camera_xyz[2]),
            orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
        )
        if avoid_collisions is not None:
            req.avoid_collisions = bool(avoid_collisions)
        else:
            req.avoid_collisions = bool(rospy.get_param("~ik_avoid_collisions", True))
        ik_timeout = float(timeout) if timeout is not None else self._ik_timeout
        ik_attempts = int(attempts) if attempts is not None else self._ik_attempts
        req.timeout = rospy.Duration(ik_timeout)
        seed_source = "provided_seed"
        if seed_values and len(seed_values) == len(self._joint_names):
            req.robot_state = RobotState(
                joint_state=JointState(name=list(self._joint_names), position=list(seed_values))
            )
        else:
            # A default-constructed RobotState makes MoveIt log
            # "Found empty JointState message" at ERROR on every IK call.
            # Seed from the live arm state instead.
            req.robot_state = RobotState(
                joint_state=self._current_joint_state())
            seed_source = "live_joint_state"

        context = dict(diagnostic_context or {})
        seed_source = context.get("seed_source", seed_source)
        seed_positions = [float(v) for v in req.robot_state.joint_state.position]

        rospy.loginfo(
            "IK request: link=%s camera=%s look_at=%s q=[%.3f,%.3f,%.3f,%.3f] "
            "collisions=%s timeout=%.2fs attempts=%d",
            self._ik_link,
            [round(v, 3) for v in camera_xyz],
            [round(v, 3) for v in look_at],
            q[0], q[1], q[2], q[3],
            req.avoid_collisions, ik_timeout, ik_attempts,
        )

        for attempt in range(max(1, ik_attempts)):
            started = time.monotonic()
            try:
                resp = ik(req)
            except rospy.ServiceException as exc:
                self._emit_ik_diagnostic(dict(context, event="ik_attempt",
                    target_xyz=[float(v) for v in camera_xyz], orientation=q,
                    seed_source=seed_source, seed_values=seed_positions,
                    timeout_sec=ik_timeout, avoid_collisions=req.avoid_collisions,
                    attempt=attempt + 1, success=False, error_code=None,
                    elapsed_ms=round(1000.0 * (time.monotonic() - started), 3),
                    exception=str(exc)))
                rospy.logwarn("IK service call failed: %s", exc)
                return None
            elapsed_ms = round(1000.0 * (time.monotonic() - started), 3)
            success = resp.error_code.val == resp.error_code.SUCCESS
            event = dict(context, event="ik_attempt",
                target_xyz=[float(v) for v in camera_xyz], orientation=q,
                seed_source=seed_source, seed_values=seed_positions,
                timeout_sec=ik_timeout, avoid_collisions=req.avoid_collisions,
                attempt=attempt + 1, success=success,
                error_code=int(resp.error_code.val), elapsed_ms=elapsed_ms)
            if resp.error_code.val == resp.error_code.SUCCESS:
                joint_map = dict(
                    zip(resp.solution.joint_state.name, resp.solution.joint_state.position)
                )
                values = [float(joint_map.get(n, 0.0)) for n in self._joint_names]
                event["solution"] = values
                self._emit_ik_diagnostic(event)
                rospy.loginfo(
                    "IK solved view: camera=%s look_at=%s -> joints=%s (attempt %d)",
                    [round(v, 3) for v in camera_xyz],
                    [round(v, 3) for v in look_at],
                    [round(v, 3) for v in values],
                    attempt + 1,
                )
                return values
            self._emit_ik_diagnostic(event)
            if attempt == 0:
                rospy.loginfo("IK attempt 1 failed (error_code=%d), retrying...",
                              resp.error_code.val)
        rospy.logwarn(
            "IK failed for view camera=%s look_at=%s after %d attempts (last error=%d)",
            [round(v, 3) for v in camera_xyz],
            [round(v, 3) for v in look_at],
            ik_attempts,
            resp.error_code.val,
        )
        return None

    def _next_marker_id(self):
        self._marker_id += 1
        return self._marker_id

    def _publish_explore_markers(
            self, camera_xyz, look_at, view_name, ik_success, error_code=0,
            tilt_deg=None):
        """Publish debug markers for an explore view attempt."""
        ma = MarkerArray()
        stamp = rospy.Time.now()
        frame = self._base_frame
        green = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        red = ColorRGBA(r=1.0, g=0.2, b=0.0, a=1.0)
        color = green if ik_success else red

        # Camera target sphere
        cam_marker = Marker()
        cam_marker.header.frame_id = frame
        cam_marker.header.stamp = stamp
        cam_marker.ns = "explore_camera"
        cam_marker.id = self._next_marker_id()
        cam_marker.type = Marker.SPHERE
        cam_marker.action = Marker.ADD
        cam_marker.pose.position = Point(x=camera_xyz[0], y=camera_xyz[1], z=camera_xyz[2])
        cam_marker.pose.orientation = Quaternion(w=1.0)
        cam_marker.scale.x = 0.10
        cam_marker.scale.y = 0.10
        cam_marker.scale.z = 0.10
        cam_marker.color = color
        cam_marker.lifetime = rospy.Duration(0)
        ma.markers.append(cam_marker)

        # Look-at target sphere
        look_marker = Marker()
        look_marker.header.frame_id = frame
        look_marker.header.stamp = stamp
        look_marker.ns = "explore_look_at"
        look_marker.id = self._next_marker_id()
        look_marker.type = Marker.SPHERE
        look_marker.action = Marker.ADD
        look_marker.pose.position = Point(x=look_at[0], y=look_at[1], z=look_at[2])
        look_marker.pose.orientation = Quaternion(w=1.0)
        look_marker.scale.x = 0.08
        look_marker.scale.y = 0.08
        look_marker.scale.z = 0.08
        look_marker.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=1.0)
        look_marker.lifetime = rospy.Duration(0)
        ma.markers.append(look_marker)

        # Arrow from camera to look-at
        arrow = Marker()
        arrow.header.frame_id = frame
        arrow.header.stamp = stamp
        arrow.ns = "explore_ray"
        arrow.id = self._next_marker_id()
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.orientation = Quaternion(w=1.0)
        arrow.points.append(Point(x=camera_xyz[0], y=camera_xyz[1], z=camera_xyz[2]))
        arrow.points.append(Point(x=look_at[0], y=look_at[1], z=look_at[2]))
        arrow.scale.x = 0.03
        arrow.scale.y = 0.06
        arrow.scale.z = 0.0
        arrow.color = color
        arrow.lifetime = rospy.Duration(0)
        ma.markers.append(arrow)

        # Short optical +Z axis arrow, useful for verifying camera-down probes.
        dx = look_at[0] - camera_xyz[0]
        dy = look_at[1] - camera_xyz[1]
        dz = look_at[2] - camera_xyz[2]
        norm = math.sqrt(dx * dx + dy * dy + dz * dz)
        if norm > 1e-9:
            optical = Marker()
            optical.header.frame_id = frame
            optical.header.stamp = stamp
            optical.ns = "explore_optical_z"
            optical.id = self._next_marker_id()
            optical.type = Marker.ARROW
            optical.action = Marker.ADD
            optical.pose.orientation = Quaternion(w=1.0)
            optical.points = [
                Point(x=camera_xyz[0], y=camera_xyz[1], z=camera_xyz[2]),
                Point(
                    x=camera_xyz[0] + 0.35 * dx / norm,
                    y=camera_xyz[1] + 0.35 * dy / norm,
                    z=camera_xyz[2] + 0.35 * dz / norm,
                ),
            ]
            optical.scale.x = 0.045
            optical.scale.y = 0.08
            optical.color = ColorRGBA(r=0.1, g=0.8, b=1.0, a=1.0)
            optical.lifetime = rospy.Duration(0)
            ma.markers.append(optical)

        # Text label
        label = Marker()
        label.header.frame_id = frame
        label.header.stamp = stamp
        label.ns = "explore_label"
        label.id = self._next_marker_id()
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position = Point(
            x=camera_xyz[0], y=camera_xyz[1], z=camera_xyz[2] + 0.18
        )
        label.pose.orientation = Quaternion(w=1.0)
        label.scale.z = 0.09
        if ik_success:
            label.text = "%s: IK OK" % view_name
        else:
            label.text = "%s: IK FAIL (err=%d)" % (view_name, error_code)
        if tilt_deg is not None:
            label.text += " tilt=%.1f deg" % float(tilt_deg)
        label.color = color
        label.lifetime = rospy.Duration(0)
        ma.markers.append(label)

        self._debug_pub.publish(ma)

    def handle_plan(self, req):
        mode = (req.mode or "smart").strip().lower()
        views_used = int(req.views_used) if req.views_used >= 0 else 0
        current = list(req.current_joint_values) if req.current_joint_values else []
        preview_only = bool(getattr(req, "preview_only", False))

        try:
            stats_resp = self._get_stats()
        except rospy.ServiceException as exc:
            return PlanNextCargoViewResponse(
                success=False,
                done=True,
                message="get_cargo_map_stats failed: %s" % exc,
                joint_values=[],
                joint_names=self._joint_names,
                view_index=-1,
            )

        if not stats_resp.success:
            return PlanNextCargoViewResponse(
                success=False,
                done=True,
                message=stats_resp.message,
                joint_values=[],
                joint_names=self._joint_names,
                view_index=-1,
            )

        stats = {
            "unknown_ratio": stats_resp.unknown_ratio,
            "frontier_count": stats_resp.frontier_count,
            "map_revision": int(getattr(stats_resp, "map_revision", 0)),
        }
        frontier_points = rospy.get_param("/luggage/cargo_map/frontier_points", [])

        if bool(getattr(req, "reset_session", False)) and not preview_only:
            self._smart_used = set()
            self._smart_rejected = set()
            self._smart_phase0_skipped = set()
            self._phase0_used = 0
            self._phase0_last_unknown = None
            self._phase0_stagnant = 0
            self._smart_pending_index = None
            self._smart_committed_views = views_used
            self._smart_rejection_counts = {}
            self._smart_active_lane = None
            self._smart_active_depth = 0.0
            self._probe_used = set()
            self._probe_pending_index = None
            self._probe_committed_views = 0
            self._probe_last_unknown = None
            self._probe_stagnant_count = 0

        if mode == "smart" and not preview_only:
            self._update_smart_pending(views_used, stats)

        if mode == "smart":
            state_ok, state_diag = self._current_state_preflight(current)
            self._emit_ik_diagnostic({
                "event": "start_state",
                "phase": "smart",
                "seed_source": "request_current_joints",
                "seed_values": [float(v) for v in current],
                "state_valid": bool(state_ok),
                "state_diagnostic": state_diag,
            })
            if not state_ok:
                result = {
                    "success": False,
                    "done": False,
                    "joint_values": [],
                    "joint_names": self._joint_names,
                    "view_index": -1,
                    "message": state_diag,
                    "diagnostics": state_diag,
                }
            else:
                result = self._plan_smart_view(
                    stats, frontier_points, views_used,
                    current_joints=current, preview_only=preview_only,
                )
        elif mode == "interior_probe":
            result = self._plan_interior_probe(stats, frontier_points, views_used)
        else:
            result = {
                "success": False,
                "done": True,
                "joint_values": [],
                "joint_names": self._joint_names,
                "view_index": -1,
                "message": "unsupported exploration mode: %s" % mode,
            }

        if mode == "smart":
            summary = rospy.get_param(
                "/luggage/workspace/explore_summary", {})
            summary["accepted"] = len(self._smart_used)
            summary["rejected"] = len(self._smart_rejected)
            summary["phase0_skipped"] = len(self._smart_phase0_skipped)
            summary["pending_index"] = (
                -1 if self._smart_pending_index is None
                else int(self._smart_pending_index))
            summary["rejection_reasons"] = dict(
                self._smart_rejection_counts)
            summary["preview_only"] = preview_only
            summary["remaining"] = max(
                0,
                len(self._smart_views)
                - len(self._smart_used)
                - len(self._smart_rejected)
                - len(self._smart_phase0_skipped)
                - (1 if self._smart_pending_index is not None else 0),
            )
            summary["last_result"] = result.get("message", "")
            rospy.set_param("/luggage/workspace/explore_summary", summary)

        selected_view = {}
        selected_index = int(result.get("view_index", -1))
        if mode == "smart" and 0 <= selected_index < len(self._smart_views):
            selected_view = self._smart_views[selected_index]
            self._request_floor_coverage_render(selected_view)
        selection_pub = getattr(self, "_selection_pub", None)
        if selection_pub is not None:
            selection_pub.publish(String(data=json.dumps({
            "success": bool(result.get("success", True)),
            "done": bool(result["done"]),
            "candidate_id": str(selected_view.get(
                "candidate_id", selected_view.get("name", ""))),
            "lane_id": str(selected_view.get("lane_id", "")),
            "insertion_depth": float(selected_view.get("depth", 0.0)),
            "map_revision": stats["map_revision"],
            "geometry_version": int(selected_view.get(
                "geometry_version", 0)),
            "information_gain": float(selected_view.get(
                "information_gain", 0.0)),
            "corridor_free_confidence": float(selected_view.get(
                "corridor_free_confidence", 0.0)),
            "floor_xy_coverage": float(selected_view.get(
                "floor_xy_coverage", 0.0)),
            "floor_unknown_gain": float(selected_view.get(
                "floor_unknown_gain", 0.0)),
            "inside_container_fov_ratio": float(selected_view.get(
                "inside_container_fov_ratio", 0.0)),
            "diagnostics": result.get("diagnostics", ""),
            }, sort_keys=True)))
        return PlanNextCargoViewResponse(
            success=bool(result.get("success", True)),
            done=result["done"],
            message=result["message"],
            joint_values=result["joint_values"],
            joint_names=result["joint_names"],
            view_index=result["view_index"],
            camera_pose=result.get("camera_pose", Pose()),
            waypoints=result.get("waypoints", []),
            plan_type=result.get("plan_type", "joint_target"),
            stage=result.get("stage", mode),
            diagnostics=result.get("diagnostics", ""),
            candidate_id=str(selected_view.get(
                "candidate_id", selected_view.get("name", ""))),
            lane_id=str(selected_view.get("lane_id", "")),
            insertion_depth=float(selected_view.get("depth", 0.0)),
            map_revision=stats["map_revision"],
        )

    def _plan_interior_probe(self, stats, frontier_points, views_used):
        """Select a collision-aware, camera-down internal probe trajectory."""
        cfg = self._probe_cfg
        completed_new_probe = (
                self._probe_pending_index is not None
                and views_used > self._probe_committed_views
        )
        if completed_new_probe:
            self._probe_used.add(self._probe_pending_index)
            self._probe_pending_index = None
            self._probe_committed_views = views_used
        if not cfg["enabled"] or not self._probe_views:
            return {
                "done": True, "joint_values": [], "joint_names": self._joint_names,
                "view_index": -1, "message": "interior_probe disabled or no candidates",
                "plan_type": "cartesian_probe", "stage": "interior_probe",
                "diagnostics": "no_candidates",
            }
        termination = evaluate_probe_termination(
            stats["unknown_ratio"],
            views_used,
            cfg,
            last_unknown=(
                self._probe_last_unknown if completed_new_probe else None
            ),
            stagnant_count=self._probe_stagnant_count,
        )
        self._probe_last_unknown = termination["last_unknown"]
        self._probe_stagnant_count = termination["stagnant_count"]
        if termination["done"]:
            return {
                "done": True, "joint_values": [], "joint_names": self._joint_names,
                "view_index": -1, "message": termination["message"],
                "plan_type": "cartesian_probe", "stage": "interior_probe",
                "diagnostics": "%s stagnation_count=%d" % (
                    termination["reason"], self._probe_stagnant_count,
                ),
            }

        scored, states = rank_probe_candidates(
            self._probe_views,
            frontier_points,
            self._probe_used,
            cfg["coverage_radius"],
        )

        ik_cfg = cfg["ik"]
        for score, idx, view in scored:
            diagnostic_context = {
                "phase": "interior_probe",
                "candidate_id": str(view.get("candidate_id", view.get("name", ""))),
                "lane_id": str(view.get("lane_id", "")),
                "seed_source": "observe_seed",
                "waypoint": "interior",
            }
            joints = self._solve_view_ik(
                view["camera_xyz"], view["look_at"],
                seed_values=self._observe_seed,
                timeout=ik_cfg["timeout"],
                attempts=ik_cfg["attempts"],
                avoid_collisions=ik_cfg["avoid_collisions"],
                orientation_quat=view["orientation_quat"],
                diagnostic_context=diagnostic_context,
            )
            if joints is None:
                states[idx] = "ik_failed"
                continue
            waypoints = self._probe_waypoints(view)
            entry_ok, entry_reason, pre_joints, _aperture_joints = (
                self._probe_entry_waypoints_reachable(
                    view, waypoints, ik_cfg,
                    diagnostic_context=diagnostic_context,
                )
            )
            if not entry_ok:
                states[idx] = entry_reason
                continue
            self._probe_pending_index = idx
            self._publish_probe_geometry(view, states)
            self._publish_explore_markers(
                view["camera_xyz"], view["look_at"], view["name"],
                ik_success=True, tilt_deg=view["tilt_deg"],
            )
            return {
                "done": False,
                # Return the pre-opening IK branch; execution uses pose goals,
                # but callers and diagnostics now retain the exact entry seed
                # rather than the unrelated internal-view solution.
                "joint_values": pre_joints,
                "joint_names": self._joint_names,
                "view_index": idx,
                "camera_pose": self._pose_for_view(view),
                "waypoints": waypoints,
                "plan_type": "cartesian_probe",
                "stage": "interior_probe",
                "message": "interior probe %s coverage=%.2f" % (view["name"], score),
                "diagnostics": "coverage=%.3f depth=%.3f lateral=%.3f" % (
                    score, view["depth"], view["lateral_offset"],
                ),
            }

        self._publish_probe_geometry(candidate_states=states)
        return {
            "done": True, "joint_values": [], "joint_names": self._joint_names,
            "view_index": -1, "message": "all interior probe candidates rejected",
            "plan_type": "cartesian_probe", "stage": "interior_probe",
            "diagnostics": ";".join(
                "%d:%s" % (idx, state) for idx, state in sorted(states.items())
            ),
        }

    def _ik_seed(self, current_joints=None):
        """Prefer the arm's current joints for IK; fall back to observe pose."""
        if (
                current_joints
                and len(current_joints) == len(self._joint_names)):
            return list(current_joints)
        return self._observe_seed

    def _current_state_preflight(self, current_joints):
        """Return (valid, diagnostic) for the actual IK seed/start state."""
        if (
                not current_joints
                or len(current_joints) != len(self._joint_names)):
            return True, "start_state_unavailable"
        req = GetStateValidityRequest()
        req.group_name = self._ik_group
        req.robot_state = RobotState(joint_state=JointState(
            name=list(self._joint_names),
            position=list(current_joints),
        ))
        try:
            rospy.wait_for_service(self._state_validity_service, timeout=2.0)
            response = self._state_validity(req)
        except (rospy.ServiceException, rospy.ROSException) as exc:
            return False, "state_validity_unavailable: %s" % exc
        if response.valid:
            return True, "start_state_valid"
        pairs = []
        for contact in getattr(response, "contacts", []):
            pair = "%s<->%s" % (
                getattr(contact, "contact_body_1", "?"),
                getattr(contact, "contact_body_2", "?"),
            )
            if pair not in pairs:
                pairs.append(pair)
        return False, "start_state_invalid: %s" % (
            "; ".join(pairs) if pairs else "unknown contact")

    def _count_smart_rejection(self, reason):
        self._smart_rejection_counts[reason] = (
            self._smart_rejection_counts.get(reason, 0) + 1)

    def _update_smart_pending(self, views_used, stats=None):
        """Commit or reject the plan returned by the previous request."""
        if self._smart_pending_index is None:
            return
        pending_index = self._smart_pending_index
        if views_used > self._smart_committed_views:
            self._smart_used.add(pending_index)
            views = getattr(self, "_smart_views", [])
            if 0 <= pending_index < len(views):
                committed = views[pending_index]
                if committed.get("lane_id"):
                    self._smart_active_lane = committed["lane_id"]
                    self._smart_active_depth = float(
                        committed.get("depth", 0.0))
            self._smart_committed_views = views_used
            if stats is not None and pending_index < self._smart_phase0_count:
                self._evaluate_phase0_gain(
                    pending_index, float(stats.get("unknown_ratio", 0.0)))
        else:
            self._smart_rejected.add(pending_index)
            self._count_smart_rejection("motion_rejected")
        self._smart_pending_index = None

    def _evaluate_phase0_gain(self, committed_index, unknown_ratio):
        """After executing a phase0 (opening-arc) view, check marginal gain.

        Mirrors ``interior_probe_planner``'s stagnation logic: once the
        unknown_ratio improvement between phase0 views falls below
        ``min_improvement`` for ``stagnation_limit`` views in a row, the
        remaining un-run opening views are moved into
        ``_smart_phase0_skipped`` (never ``_smart_rejected``, so they do not
        pollute the motion-rejection diagnostics) and phase1 proceeds
        immediately. Always requires at least one executed phase0 view.
        """
        self._phase0_used += 1
        p0_cfg = self._smart_cfg["phase0"]
        exhausted, next_stagnant, reason = phase0_gain_exhausted(
            last_unknown=self._phase0_last_unknown,
            unknown_ratio=unknown_ratio,
            phase0_used=self._phase0_used,
            stagnant_count=self._phase0_stagnant,
            min_improvement=p0_cfg["min_improvement"],
            stagnation_limit=p0_cfg["stagnation_limit"],
        )
        self._phase0_last_unknown = unknown_ratio
        self._phase0_stagnant = next_stagnant
        if not exhausted:
            return
        newly_skipped = {
            idx for idx in range(self._smart_phase0_count)
            if idx not in self._smart_used
            and idx not in self._smart_rejected
            and idx not in self._smart_phase0_skipped
        }
        if not newly_skipped:
            return
        self._smart_phase0_skipped |= newly_skipped
        rospy.loginfo(
            "smart phase0: gain exhausted after %d view(s) (unknown=%.3f, "
            "reason=%s); skipping %d remaining opening view(s), proceeding "
            "to phase1",
            self._phase0_used, unknown_ratio, reason, len(newly_skipped),
        )

    @staticmethod
    def _view_with_orientation(view, orientation_quat):
        """Return a shallow copy of ``view`` with an updated orientation."""
        updated = dict(view)
        updated["orientation_quat"] = [float(v) for v in orientation_quat]
        return updated

    def _ensure_scene_config(self):
        if self._scene_config is None:
            self._scene_config = self._load_scene_config()
        return self._scene_config

    def _smart_interior_probe_waypoints(self, view, orientation_quat):
        """Build pre-opening / aperture / internal poses for a smart interior view."""
        self._ensure_scene_config()
        oriented = self._view_with_orientation(view, orientation_quat)
        return self._probe_waypoints(oriented)

    def _build_view_evaluation_request(self, views):
        """Build an EvaluateCargoViews request for a list of candidate views."""
        request = EvaluateCargoViewsRequest()
        request.camera_poses = [self._pose_for_view(view) for view in views]
        request.insertion_depths = [
            float(view.get("depth", 0.0)) for view in views]
        request.opening_pose.position = Point(
            x=self._opening_geometry["opening_xyz"][0],
            y=self._opening_geometry["opening_xyz"][1],
            z=self._opening_geometry["opening_xyz"][2],
        )
        request.opening_pose.orientation.w = 1.0
        request.corridor_radius = max(
            self._camera_half_width,
            self._camera_half_height,
        ) + self._physical_clearance
        view_scoring = self._config.get("view_scoring", {}) or {}
        request.max_range = float(view_scoring.get("max_range", 2.5))
        request.evaluate_floor_coverage = bool(
            view_scoring.get("evaluate_floor_coverage", True))
        return request

    def _request_floor_coverage_render(self, view):
        """Ask the mapper to render the selected view's inner-floor coverage.

        Only the mapper knows which floor cells a pose can reach, so the
        heat map is drawn there; this is a one-pose follow-up call made after
        the winning candidate is known.
        """
        if not view or self._opening_geometry is None:
            return
        if "orientation_quat" not in view or "camera_xyz" not in view:
            return
        request = self._build_view_evaluation_request([view])
        if not request.evaluate_floor_coverage:
            return
        request.publish_floor_coverage_debug = True
        try:
            self._evaluate_views(request)
        except (rospy.ServiceException, rospy.ROSException) as exc:
            rospy.logdebug("floor coverage visualization skipped: %s", exc)

    def _phase0_inside_fov_ratio(self, view, orientation_quat):
        """Return an about-to-run phase0 candidate's inside_container_fov_ratio.

        Called from the phase0 selection loop after IK succeeds, before the
        candidate is handed off to ``motion_planner`` -- a low ratio here
        means the FOV mostly misses the container (e.g. lands on the robot's
        own pedestal), so it is worth skipping before paying the OMPL
        solve/simplify cost, not just after the fact.

        Returns ``None`` (fail-open; caller must never treat this as "low
        FOV") when the metric cannot be computed: opening geometry not yet
        known, the mapper's evaluate service is unreachable/erroring, or the
        response omits the field (older mapper).
        """
        if self._opening_geometry is None or orientation_quat is None:
            return None
        oriented = self._view_with_orientation(view, orientation_quat)
        try:
            request = self._build_view_evaluation_request([oriented])
        except (KeyError, TypeError, IndexError):
            return None
        if not request.evaluate_floor_coverage:
            return None
        try:
            response = self._evaluate_views(request)
        except (rospy.ServiceException, rospy.ROSException) as exc:
            rospy.logwarn_throttle(
                2.0, "smart phase0: FOV evaluation unavailable: %s", exc)
            return None
        if not response.success:
            return None
        values = getattr(response, "inside_container_fov_ratio", None)
        if not values:
            return None
        return float(values[0])

    def _evaluate_interior_views(self, indexed_views):
        """Return mapper information/corridor metrics keyed by smart index."""
        if not indexed_views or self._opening_geometry is None:
            return {}
        request = self._build_view_evaluation_request(
            [view for _idx, view in indexed_views])
        try:
            response = self._evaluate_views(request)
        except (rospy.ServiceException, rospy.ROSException) as exc:
            rospy.logwarn_throttle(
                2.0, "cargo view evaluation unavailable: %s", exc)
            return {}
        if not response.success:
            rospy.logwarn_throttle(
                2.0, "cargo view evaluation rejected: %s", response.message)
            return {}
        metrics = {}
        for offset, (smart_index, _view) in enumerate(indexed_views):
            entry = {
                "information_gain": float(
                    response.expected_information_gain[offset]),
                "visible_unknown_count": int(
                    response.visible_unknown_count[offset]),
                "occlusion_ratio": float(
                    response.occlusion_ratio[offset]),
                "corridor_free_confidence": float(
                    response.corridor_free_confidence[offset]),
            }
            # Observation-only FOV quality. Absent on older mappers, and never
            # consumed by _plan_smart_view() scoring or gating.
            entry.update(self._floor_metrics(response, offset))
            metrics[smart_index] = entry
        return metrics

    @staticmethod
    def _floor_metrics(response, offset):
        """Extract optional floor-coverage metrics for one evaluated view."""
        fields = (
            "floor_xy_coverage",
            "floor_unknown_gain",
            "inside_container_fov_ratio",
        )
        entry = {}
        for name in fields:
            values = getattr(response, name, None)
            if values is None or offset >= len(values):
                continue
            entry[name] = float(values[offset])
        if "inside_container_fov_ratio" in entry:
            entry["outside_container_ratio"] = (
                1.0 - entry["inside_container_fov_ratio"])
        return entry

    def _plan_smart_view(self, stats, frontier_points, views_used,
                         current_joints=None, preview_only=False):
        """Two-phase geometry-driven exploration.

        Phase 0 (opening): return the next pre-generated opening view in
        order. These are IK-validated lazily on first request and cached.
        Phase 1 (interior): rank remaining interior views by frontier
        coverage, IK-validate the best one, and return it. Skip any view
        whose IK fails or that has already been used.
        """
        cfg = self._smart_cfg
        if not cfg["enabled"]:
            return {
                "done": True,
                "joint_values": [],
                "joint_names": self._joint_names,
                "view_index": -1,
                "message": "smart_disabled",
            }
        if not self._smart_views:
            return {
                "done": True,
                "joint_values": [],
                "joint_names": self._joint_names,
                "view_index": -1,
                "message": "pool_empty (phase0=%d phase1=%d status=%s filter=%s)" % (
                    getattr(self, "_smart_phase0_count", 0),
                    getattr(self, "_smart_phase1_count", 0),
                    getattr(self, "_smart_last_status", "unknown"),
                    getattr(self, "_smart_filter_stats", {})),
            }

        max_views = cfg["termination"]["max_views"]
        if views_used >= max_views:
            return {
                "done": True,
                "joint_values": [],
                "joint_names": self._joint_names,
                "view_index": -1,
                "message": "max smart views reached (%d)" % max_views,
            }

        unknown_threshold = cfg["termination"]["unknown_threshold"]
        if stats["unknown_ratio"] <= unknown_threshold and views_used > 0:
            return {
                "done": True,
                "joint_values": [],
                "joint_names": self._joint_names,
                "view_index": -1,
                "message": "unknown below threshold (%.2f)" % stats["unknown_ratio"],
            }

        # Strict-down orientation (suction-down camera quaternion) for
        # pose_target smart views. None -> legacy joint_target behaviour.
        down_q = self._downward_orientation_quat()
        inter_axis = (
            self._downward_orientations["inter_axis_deg"]
            if self._downward_orientations is not None else 0.0
        )
        ik_seed = self._ik_seed(current_joints)
        p1 = cfg["phase1"]
        use_interior_probe_path = bool(
            p1.get("camera_down_mode") and self._downward_enabled
        )
        probe_defaults = self._probe_cfg.get("ik", cfg["ik"])
        # A smart request may reject several candidates before returning one.
        # Cap each internal/pre/aperture IK check to the smart-mode budget;
        # otherwise 5 candidates * 3 poses * interior_probe retries can block
        # the service for well over a minute.
        probe_ik_cfg = {
            "timeout": min(
                float(probe_defaults["timeout"]),
                float(cfg["ik"]["timeout"])),
            "attempts": min(
                int(probe_defaults["attempts"]),
                int(cfg["ik"]["attempts"])),
            "avoid_collisions": bool(
                probe_defaults.get(
                    "avoid_collisions",
                    cfg["ik"].get("avoid_collisions", True))),
        }
        result_rejection = [None]

        def _result(view, idx, phase, joint_values, score=None):
            """Build a smart view result; pose_target or cartesian_probe."""
            orient = down_q if down_q is not None else view.get("orientation_quat")
            if (
                    phase == "phase1"
                    and use_interior_probe_path
                    and view.get("stage") == "interior_probe"
                    and orient is not None
                    and view.get("aperture_xyz") is not None
            ):
                oriented = self._view_with_orientation(view, orient)
                waypoints = self._smart_interior_probe_waypoints(view, orient)
                entry_ok, entry_reason, pre_joints, _aperture_joints = (
                    self._probe_entry_waypoints_reachable(
                        oriented, waypoints, probe_ik_cfg,
                        seed_values=ik_seed,
                        diagnostic_context={
                            "phase": "phase1",
                            "candidate_id": str(view.get("candidate_id", view.get("name", ""))),
                            "lane_id": str(view.get("lane_id", "")),
                            "seed_source": "live_seed" if current_joints else "observe_seed",
                        },
                    )
                )
                if not entry_ok:
                    result_rejection[0] = entry_reason
                    return None
                coverage = "" if score is None else " coverage=%.2f" % score
                return {
                    "done": False,
                    "joint_values": pre_joints,
                    "joint_names": self._joint_names,
                    "view_index": idx,
                    "camera_pose": self._pose_for_view(oriented),
                    "waypoints": waypoints,
                    "plan_type": "cartesian_probe",
                    "stage": "phase1",
                    "message": "smart %s: %s (probe path)%s" % (
                        phase, view["name"], coverage),
                    "diagnostics": "camera_tilt=%.2f suction_tilt=0.00%s" % (
                        inter_axis, coverage),
                }
            if down_q is not None:
                camera_pose = Pose(
                    position=Point(
                        x=view["camera_xyz"][0], y=view["camera_xyz"][1],
                        z=view["camera_xyz"][2]),
                    orientation=Quaternion(
                        x=down_q[0], y=down_q[1], z=down_q[2], w=down_q[3]),
                )
                coverage = "" if score is None else " coverage=%.2f" % score
                return {
                    "done": False,
                    "joint_values": joint_values,
                    "joint_names": self._joint_names,
                    "view_index": idx,
                    "camera_pose": camera_pose,
                    "plan_type": "pose_target",
                    "stage": phase,
                    "message": "smart %s: %s tilt=%.1f deg%s" % (
                        phase, view["name"], inter_axis, coverage),
                    "diagnostics": "camera_tilt=%.2f suction_tilt=0.00%s" % (
                        inter_axis, coverage),
                }
            return {
                "done": False,
                "joint_values": joint_values,
                "joint_names": self._joint_names,
                "view_index": idx,
                "plan_type": "joint_target",
                "stage": phase,
                "message": "smart %s: %s tilt=%.1f deg" % (
                    phase, view["name"], view.get("tilt_deg", 0.0)),
            }

        excluded = (
            set(self._smart_used)
            | set(self._smart_rejected)
            | set(self._smart_phase0_skipped)
        )
        preview_rejected = set()

        # Phase 0: opening views in order, independent of views_used. The
        # latter counts integrated views and is not a candidate-array index.
        for idx in range(self._smart_phase0_count):
            if idx in excluded or idx in preview_rejected:
                continue
            view = self._smart_views[idx]
            joint_values = self._solve_view_ik(
                view["camera_xyz"], view["look_at"],
                seed_values=ik_seed,
                orientation_quat=down_q,
                diagnostic_context={
                    "phase": "phase0",
                    "candidate_id": str(view.get("candidate_id", view.get("name", ""))),
                    "lane_id": str(view.get("lane_id", "")),
                    "seed_source": "live_seed" if current_joints else "observe_seed",
                    "waypoint": "target",
                },
            )
            if joint_values is not None:
                fov_ratio = self._phase0_inside_fov_ratio(view, down_q)
                min_inside_fov = cfg["phase0"].get("min_inside_fov", 0.0)
                if phase0_low_fov(self._phase0_used, fov_ratio, min_inside_fov):
                    newly_skipped = {
                        i for i in range(idx, self._smart_phase0_count)
                        if i not in self._smart_used
                        and i not in self._smart_rejected
                    }
                    if not preview_only:
                        self._smart_phase0_skipped |= newly_skipped
                    else:
                        preview_rejected |= newly_skipped
                    rospy.loginfo(
                        "smart phase0: low inside-container FOV (%.2f < "
                        "%.2f) at %s; skipping %d remaining opening view(s), "
                        "proceeding to phase1",
                        fov_ratio, min_inside_fov, view["name"],
                        len(newly_skipped),
                    )
                    break
                self._publish_explore_markers(
                    view["camera_xyz"], view["look_at"],
                    view["name"], ik_success=True,
                )
                result = _result(view, idx, "phase0", joint_values)
                if result is not None:
                    if not preview_only:
                        self._smart_pending_index = idx
                    return result
            self._publish_explore_markers(
                view["camera_xyz"], view["look_at"],
                view["name"], ik_success=False, error_code=-31,
            )
            if not preview_only:
                self._smart_rejected.add(idx)
                self._count_smart_rejection("view_ik_failed")
            else:
                preview_rejected.add(idx)
            rospy.logwarn(
                "smart phase0: IK failed for %s; trying next opening view",
                view["name"],
            )

        # Phase 1: rank interior views by frontier coverage, IK-validate
        # the best unused one.
        interior_start = self._smart_phase0_count
        interior_indices = [
            i for i in range(interior_start, len(self._smart_views))
            if i not in excluded and i not in preview_rejected
            and (
                getattr(self, "_smart_active_lane", None) is None
                or (
                    self._smart_views[i].get("lane_id")
                    == self._smart_active_lane
                    and float(self._smart_views[i].get("depth", 0.0))
                    > self._smart_active_depth + 1e-6
                )
            )
        ]
        active_lane = getattr(self, "_smart_active_lane", None)
        if active_lane is None:
            shallowest_by_lane = {}
            for index in interior_indices:
                view = self._smart_views[index]
                lane = view.get("lane_id", "")
                if (
                        lane not in shallowest_by_lane
                        or view.get("depth", 0.0)
                        < self._smart_views[
                            shallowest_by_lane[lane]].get("depth", 0.0)):
                    shallowest_by_lane[lane] = index
            interior_indices = sorted(shallowest_by_lane.values())
        elif interior_indices:
            next_depth = min(
                float(self._smart_views[index].get("depth", 0.0))
                for index in interior_indices)
            interior_indices = [
                index for index in interior_indices
                if abs(float(self._smart_views[index].get("depth", 0.0))
                       - next_depth) <= 1e-6
            ]
        if not interior_indices:
            return {
                "success": False,
                "done": True,
                "joint_values": [],
                "joint_names": self._joint_names,
                "view_index": -1,
                "message": "no feasible smart interior candidates remain",
                "diagnostics": "candidate_pool_exhausted",
            }

        # Skip candidates too close to an already-used view (downward config).
        min_sep = float(self._downward_cfg.get("candidate_min_separation_m", 0.0))
        used_positions = [
            self._smart_views[i]["camera_xyz"]
            for i in self._smart_used
            if 0 <= i < len(self._smart_views)
        ]

        def _too_close(xyz):
            if min_sep <= 0.0 or not used_positions:
                return False
            for p in used_positions:
                if math.sqrt(sum((xyz[k] - p[k]) ** 2 for k in range(3))) <= min_sep:
                    return True
            return False

        # Sensed openings use camera-model voxel raycasting and a hard
        # observed-free corridor gate. Config-only fallback retains the legacy
        # frontier approximation for simulation compatibility.
        vp = view_planning_constraints(self._config)
        sensed_geometry = getattr(self, "_opening_geometry", None) is not None
        view_metrics = (
            self._evaluate_interior_views([
                (idx, self._smart_views[idx]) for idx in interior_indices])
            if sensed_geometry else {}
        )
        if (
                sensed_geometry
                and getattr(self, "_opening_hardware_strict", False)
                and not view_metrics):
            return {
                "success": False,
                "done": False,
                "joint_values": [],
                "joint_names": self._joint_names,
                "view_index": -1,
                "message": "sensed view evaluation unavailable",
                "diagnostics": "information_gain_unavailable",
            }
        score_cfg = self._config.get("view_scoring", {}) or {}
        gain_weight = float(score_cfg.get("information_gain_weight", 1.0))
        depth_weight = float(score_cfg.get("depth_weight", 0.15))
        risk_weight = float(score_cfg.get("risk_weight", 0.50))
        min_gain = float(score_cfg.get("min_information_gain", 0.0))
        scored = []
        for idx in interior_indices:
            view = self._smart_views[idx]
            if _too_close(view["camera_xyz"]):
                if not preview_only:
                    self._smart_rejected.add(idx)
                    self._count_smart_rejection("too_close")
                continue
            metrics = view_metrics.get(idx)
            if metrics is not None:
                confidence = metrics["corridor_free_confidence"]
                view.update(metrics)
                if confidence < getattr(
                        self, "_corridor_min_confidence", 0.95):
                    continue
                if metrics["information_gain"] < min_gain:
                    continue
                score = (
                    gain_weight * metrics["information_gain"] * confidence
                    + depth_weight * float(view.get("depth", 0.0))
                    - risk_weight * metrics["occlusion_ratio"]
                )
            else:
                cand = {
                    "camera_xyz": view["camera_xyz"],
                    "look_at": view["look_at"],
                }
                score = coverage_score(cand, frontier_points, vp)
            scored.append((score, idx, view))
        if not scored:
            return {
                "success": False,
                "done": True,
                "joint_values": [],
                "joint_names": self._joint_names,
                "view_index": -1,
                "message": "no feasible smart interior candidates",
                "diagnostics": "separation_or_corridor_or_information_gain",
            }
        scored.sort(key=lambda t: (-t[0], t[1]))

        ik_cfg = cfg["ik"]
        active_ik_cfg = probe_ik_cfg if use_interior_probe_path else ik_cfg
        for score, idx, view in scored:
            joint_values = self._solve_view_ik(
                view["camera_xyz"], view["look_at"],
                seed_values=ik_seed,
                timeout=active_ik_cfg["timeout"],
                attempts=active_ik_cfg["attempts"],
                avoid_collisions=active_ik_cfg["avoid_collisions"],
                orientation_quat=down_q,
                diagnostic_context={
                    "phase": "phase1",
                    "candidate_id": str(view.get("candidate_id", view.get("name", ""))),
                    "lane_id": str(view.get("lane_id", "")),
                    "seed_source": "live_seed" if current_joints else "observe_seed",
                    "waypoint": "interior",
                },
            )
            if joint_values is not None:
                result = _result(view, idx, "phase1", joint_values, score=score)
                if result is None:
                    self._publish_explore_markers(
                        view["camera_xyz"], view["look_at"],
                        view["name"], ik_success=False, error_code=-31,
                    )
                    rospy.logwarn(
                        "smart phase1: entry waypoints unreachable for %s "
                        "(score=%.2f); trying next",
                        view["name"], score,
                    )
                    reason = result_rejection[0] or "entry_ik_failed"
                    if not preview_only:
                        self._smart_rejected.add(idx)
                        self._count_smart_rejection(reason)
                    else:
                        preview_rejected.add(idx)
                    continue
                if not preview_only:
                    self._smart_pending_index = idx
                self._publish_explore_markers(
                    view["camera_xyz"], view["look_at"],
                    view["name"], ik_success=True,
                )
                return result
            self._publish_explore_markers(
                view["camera_xyz"], view["look_at"],
                view["name"], ik_success=False, error_code=-31,
            )
            rospy.logwarn(
                "smart phase1: IK failed for %s (score=%.2f); trying next",
                view["name"], score,
            )
            if not preview_only:
                self._smart_rejected.add(idx)
                self._count_smart_rejection("view_ik_failed")

        reasons = ", ".join(
            "%s=%d" % item
            for item in sorted(self._smart_rejection_counts.items())
        )
        return {
            "success": False,
            "done": True,
            "joint_values": [],
            "joint_names": self._joint_names,
            "view_index": -1,
            "message": "no feasible smart candidates%s" % (
                ": " + reasons if reasons else ""),
            "diagnostics": reasons or "no_feasible_candidates",
        }


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
    rospy.init_node("cargo_exploration_planner", log_level=resolve_log_level())
    CargoExplorationPlannerNode()
    rospy.loginfo("cargo_exploration_planner ready")
    rospy.spin()


if __name__ == "__main__":
    main()
