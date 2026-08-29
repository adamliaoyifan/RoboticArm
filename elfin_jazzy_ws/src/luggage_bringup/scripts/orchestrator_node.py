#!/usr/bin/env python3
"""Top-level loading orchestrator.

The legacy flow remains available by leaving ~active_loading false. Active
loading uses a fixed pickup source and computes the placement before picking.
"""

import json
import math
import os
import sys

import rospkg

import rospy
import tf2_ros
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from tf2_geometry_msgs import do_transform_pose
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, Trigger
try:
    from gazebo_msgs.srv import GetModelState
except ImportError:
    GetModelState = None
from luggage_msgs.msg import LoadTaskStatus, MotionSegment, SlotSpec
from luggage_msgs.srv import (
    AddPlacedBox,
    BuildMotionSequence,
    ClearCurrentBox,
    ComputePlacement,
    DetectLuggage,
    FinalizeCurrentBox,
    GetCargoMapStats,
    GetCurrentBox,
    GetNextSlot,
    GoToJointValues,
    GoToRobotPose,
    InspectContainer,
    IntegrateCargoView,
    OrchestratorStep,
    OrchestratorStepResponse,
    PlanMotion,
    PlanNextCargoView,
    ResetCargoMap,
    RemovePlacedBox,
    SpawnNextBox,
    SyncStaticScene,
    SyncPickupBox,
    ValidateMotionSequence,
    VacuumCommand,
    VerifyPlacedBox,
)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)
from size_uncertainty import (
    box_aabb,
    find_overlaps,
    inflate_size,
    inflated_center_z,
)
from run_events_recorder import RunEventsRecorder, is_terminal_success
from step_gate import StepGate
from probe_execution_utils import build_interior_probe_segments
from explore_execution_utils import build_smart_explore_segment
from interior_explore_loop import (
    BOOTSTRAP_OPENING,
    REPLAN_DEPTH,
    RETREAT,
    SELECT_CORRIDOR,
    InteriorExploreLoop,
)

ARM_JOINT_NAMES = [
    "elfin_joint1", "elfin_joint2", "elfin_joint3",
    "elfin_joint4", "elfin_joint5", "elfin_joint6",
]


class Orchestrator:
    STATES = (
        "Idle",
        "ResetObserve",
        "InitialExploreCargo",
        "ReturnObserveAfterInitialExplore",
        "SyncScene",
        "SpawnCurrentBox",
        "DetectPickupBox",
        "ReturnObserveBeforeDetect",
        "ExploreCargo",
        "InspectContainer",
        "ComputePlacement",
        "Detect",
        "PlanPick",
        "ExecPick",
        "PlanPlace",
        "ExecPlace",
        "UpdateOccupancy",
    )

    def __init__(self):
        self.placed = []
        self.current_luggage = None
        self.current_slot = None
        self.pick_segments = []
        self.place_segments = []
        self._segment_index = 0
        self._phase = None
        self._place_released = False
        self._released_at_contact = False
        self._retreat_after_release = False
        self._placement_backups = []
        self._placement_attempt = 0
        self._max_placement_attempts = max(1, int(rospy.get_param(
            "~max_placement_attempts", 5)))
        self._current_candidate_id = ""
        self.max_placed = rospy.get_param("~max_placed", 1)
        self.skip_reset = rospy.get_param("~skip_reset", False)
        self.observe_pose = rospy.get_param("~observe_pose", "observe")
        self.pickup_observe_pose = rospy.get_param(
            "~pickup_observe_pose", "pickup_observe")
        self.post_pick_stage_pose = rospy.get_param(
            "~post_pick_stage_pose", "observe")
        self.active_loading = rospy.get_param("~active_loading", False)
        self.inspect_mode = rospy.get_param("~inspect_mode", "gazebo_gt")
        self.use_post_place_inspect = rospy.get_param("~use_post_place_inspect", False)
        self.post_place_remap = rospy.get_param("~post_place_remap", True)
        self.strict_perception = bool(rospy.get_param(
            "~strict_perception", False))
        self.post_place_verify = bool(rospy.get_param(
            "~post_place_verify", self.strict_perception))
        self.post_place_settle_sec = float(rospy.get_param(
            "~post_place_settle_sec", 1.5))
        self.post_place_height_tolerance_voxels = float(rospy.get_param(
            "~post_place_height_tolerance_voxels", 1.5))
        self.post_place_verify_retries = max(1, int(rospy.get_param(
            "~post_place_verify_retries", 3)))
        self.post_place_verify_pose = rospy.get_param(
            "~post_place_verify_pose", self.observe_pose)
        self.verify_actual_pose = bool(rospy.get_param(
            "~verify_actual_pose", self.strict_perception))
        self.simulation_pose_oracle = bool(rospy.get_param(
            "~simulation_pose_oracle", False))
        self.actual_xy_tolerance = float(rospy.get_param(
            "~actual_pose/xy_tolerance", 0.04))
        self.actual_z_tolerance = float(rospy.get_param(
            "~actual_pose/z_tolerance", 0.03))
        self.actual_yaw_tolerance = math.radians(float(rospy.get_param(
            "~actual_pose/yaw_tolerance_deg", 5.0)))
        self.actual_rp_tolerance = math.radians(float(rospy.get_param(
            "~actual_pose/roll_pitch_tolerance_deg", 2.0)))
        self.actual_drift_tolerance = float(rospy.get_param(
            "~actual_pose/drift_tolerance", 0.02))
        self.actual_drift_wait_sec = float(rospy.get_param(
            "~actual_pose/drift_wait_sec", 3.0))
        self._base_frame = rospy.get_param(
            "~base_frame", "elfin_base_link")
        self._world_frame = rospy.get_param("~world_frame", "world")
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._get_model_state = None
        if self.simulation_pose_oracle and GetModelState is not None:
            rospy.wait_for_service("/gazebo/get_model_state")
            self._get_model_state = rospy.ServiceProxy(
                "/gazebo/get_model_state", GetModelState)
        self.detect_retries = max(1, int(rospy.get_param(
            "~detect_retries", 3)))
        # Plausibility envelope for a perception-owned size. Loaded from the
        # same catalog the spawner samples from, so the gate tracks the box
        # population instead of a hand-copied constant.
        self._size_tolerance = float(rospy.get_param(
            "~size_plausibility_tolerance", 0.06))
        self._size_range = self._load_size_range()
        # Conservative margin applied to what gets recorded, not to what gets
        # grasped. Sized from the measured per-axis p95 error (Phase 6a).
        self._size_xy_margin = float(rospy.get_param(
            "~size_uncertainty/xy_margin", 0.02))
        self._size_z_margin = float(rospy.get_param(
            "~size_uncertainty/z_margin", 0.01))
        self._overlap_tolerance_m3 = float(rospy.get_param(
            "~size_uncertainty/overlap_tolerance_m3", 1.0e-5))
        self._size_drift_warn_m = float(rospy.get_param(
            "~size_uncertainty/drift_warn_m", 0.02))
        # When set, ComputePlacement uses the 2.5D surface-map placement planner
        # (and optional MoveIt reachability filter) instead of the bin_packer.
        self.placement_ns = rospy.get_param("~placement_ns", "")
        self.filter_ns = rospy.get_param("~filter_ns", "")
        self.dry_run_motion = rospy.get_param("~dry_run_motion", False)
        # InitialExploreCargo and per-box exploration share one mode.
        self.exploration_mode = rospy.get_param("~exploration_mode", "smart")
        self.run_initial_explore = rospy.get_param("~run_initial_explore", False)
        self.settle_time_sec = float(rospy.get_param("~settle_time_sec", 0.5))
        self.explore_candidate_retries = int(
            rospy.get_param("~explore_candidate_retries", 4))
        self._interior_loop = InteriorExploreLoop(
            max_depth_steps=int(rospy.get_param(
                "~interior_loop/max_depth_steps", 4)),
            max_views=int(rospy.get_param(
                "~interior_loop/max_views", 12)),
            max_seconds=float(rospy.get_param(
                "~interior_loop/max_seconds", 120.0)),
            max_candidate_attempts=self.explore_candidate_retries,
            stagnation_limit=int(rospy.get_param(
                "~interior_loop/stagnation_limit", 2)),
        )
        self._active_probe_retreat = None
        self.detect_settle_sec = float(rospy.get_param("~detect_settle_sec", 0.3))
        # PR5: task-cloud-filter status cache for perception gates.
        self._filter_status = {}
        self._filter_status_age_sec = float(
            rospy.get_param("~filter_status_max_age", 1.0))
        from std_msgs.msg import String as StringMsg
        rospy.Subscriber(
            "/task_cloud_filter/stats_json", StringMsg,
            self._on_filter_stats, queue_size=1)
        self.require_vacuum_attach = bool(rospy.get_param("~require_vacuum_attach", True))
        self._vacuum_attached_param = rospy.get_param(
            "~vacuum_attached_param", "/luggage/vacuum/attached"
        )
        self._spawner_gt_box = None
        self._spawned_manifest = None
        self._explore_views_used = 0
        self._initial_explore_views_used = 0
        self._initial_explore_done = False
        self._current_joint_values = []
        self._joint_state_topic = rospy.get_param(
            "~joint_state_topic", "/joint_states")
        self._joint_state_timeout = float(rospy.get_param(
            "~joint_state_timeout", 1.0))

        self.status_pub = rospy.Publisher("~status", LoadTaskStatus, queue_size=1, latch=True)

        # Manual stepping and run events. Both are inert by default: auto mode
        # makes the gate a no-op and an empty events path disables recording,
        # so an existing acceptance run is byte-for-byte unaffected.
        self._step_gate = StepGate(
            mode=rospy.get_param("~run_mode", "auto"),
            breakpoints=rospy.get_param("~step_breakpoints", []),
            should_abort=rospy.is_shutdown,
        )
        self._events = RunEventsRecorder(rospy.get_param("~events_path", ""))
        self._cycle_started = None
        self._current_state = ""
        rospy.Service("~step", OrchestratorStep, self._handle_step)

        detector_ns = rospy.get_param("~detector_ns", "luggage_detector")
        packer_ns = rospy.get_param("~packer_ns", "bin_packer")
        scene_ns = rospy.get_param("~scene_ns", "scene_manager")
        wpt_ns = rospy.get_param("~waypoint_ns", "waypoint_generator")
        motion_ns = rospy.get_param("~motion_ns", "motion_planner")
        vacuum_ns = rospy.get_param("~vacuum_ns", "vacuum_simulator")
        inspector_ns = rospy.get_param("~inspector_ns", "container_inspector")
        mapper_ns = rospy.get_param("~mapper_ns", "cargo_volume_mapper")
        verifier_ns = rospy.get_param(
            "~placed_pose_verifier_ns", "placed_pose_verifier")
        self._mapper_ns = mapper_ns
        self._mark_placed_in_map = None
        self._unmark_placed_in_map = None
        explore_ns = rospy.get_param("~explore_ns", "cargo_exploration_planner")
        spawner_ns = rospy.get_param("~spawner_ns", "pickup_box_spawner")

        rospy.wait_for_service("/%s/detect_luggage" % detector_ns)
        rospy.wait_for_service("/%s/get_next_slot" % packer_ns)
        rospy.wait_for_service("/%s/sync_static_scene" % scene_ns)
        rospy.wait_for_service("/%s/sync_pickup_box" % scene_ns)
        if self.strict_perception:
            rospy.wait_for_service(
                "/%s/sync_detected_pickup_box" % scene_ns)
        rospy.wait_for_service("/%s/add_placed_box" % scene_ns)
        rospy.wait_for_service("/%s/remove_placed_box" % scene_ns)
        rospy.wait_for_service("/%s/build_motion_sequence" % wpt_ns)
        rospy.wait_for_service("/%s/plan_motion" % motion_ns)
        rospy.wait_for_service("/%s/vacuum_command" % vacuum_ns)
        if self.active_loading:
            rospy.wait_for_service("/%s/spawn_next_box" % spawner_ns)
            rospy.wait_for_service("/%s/get_current_box" % spawner_ns)
            rospy.wait_for_service("/%s/clear_current_box" % spawner_ns)
            rospy.wait_for_service("/%s/finalize_current_box" % spawner_ns)
            if self.placement_ns:
                rospy.wait_for_service("/%s/plan_placements" % self.placement_ns)
            else:
                rospy.wait_for_service("/%s/compute_placement" % packer_ns)
        if self.active_loading:
            rospy.wait_for_service("/%s/inspect_container" % inspector_ns)
        if self._needs_mapper_services():
            rospy.wait_for_service("/%s/reset_cargo_map" % mapper_ns)
            rospy.wait_for_service("/%s/integrate_cargo_view" % mapper_ns)
            rospy.wait_for_service("/%s/get_cargo_map_stats" % mapper_ns)
        if self.post_place_verify:
            rospy.wait_for_service("/%s/verify" % verifier_ns)
        if self._needs_explore_services():
            rospy.wait_for_service("/%s/plan_next_cargo_view" % explore_ns)
            rospy.wait_for_service("/%s/go_to_joint_values" % motion_ns)
            rospy.wait_for_service(
                "/%s/validate_motion_sequence" % motion_ns)
        if not self.skip_reset or self.run_initial_explore or self.active_loading:
            rospy.wait_for_service("/%s/go_to_robot_pose" % motion_ns)

        self.detect = rospy.ServiceProxy("/%s/detect_luggage" % detector_ns, DetectLuggage)
        self.next_slot = rospy.ServiceProxy("/%s/get_next_slot" % packer_ns, GetNextSlot)
        self.compute_placement = None
        self.plan_placements = None
        self.filter_placements = None
        self.spawn_next_box = None
        self.get_current_box = None
        self.clear_current_box = None
        self.finalize_current_box = None
        if self.active_loading:
            if self.placement_ns:
                self.plan_placements = rospy.ServiceProxy(
                    "/%s/plan_placements" % self.placement_ns, Trigger
                )
            else:
                self.compute_placement = rospy.ServiceProxy(
                    "/%s/compute_placement" % packer_ns, ComputePlacement
                )
            self.spawn_next_box = rospy.ServiceProxy(
                "/%s/spawn_next_box" % spawner_ns, SpawnNextBox
            )
            self.get_current_box = rospy.ServiceProxy(
                "/%s/get_current_box" % spawner_ns, GetCurrentBox
            )
            self.clear_current_box = rospy.ServiceProxy(
                "/%s/clear_current_box" % spawner_ns, ClearCurrentBox
            )
            self.finalize_current_box = rospy.ServiceProxy(
                "/%s/finalize_current_box" % spawner_ns,
                FinalizeCurrentBox,
            )
        self.clear_octomap = rospy.ServiceProxy("/clear_octomap", Empty)
        self.clear_dynamic_scene = rospy.ServiceProxy(
            "/dynamic_scene_manager/clear_dynamic_scene", Trigger
        )
        self.sync_dynamic_scene = rospy.ServiceProxy(
            "/dynamic_scene_manager/sync_dynamic_scene", Trigger
        )
        self.sync_scene = rospy.ServiceProxy("/%s/sync_static_scene" % scene_ns, SyncStaticScene)
        self.sync_pickup_box = rospy.ServiceProxy("/%s/sync_pickup_box" % scene_ns, SyncStaticScene)
        self.sync_detected_pickup_box = rospy.ServiceProxy(
            "/%s/sync_detected_pickup_box" % scene_ns, SyncPickupBox)
        self.add_placed = rospy.ServiceProxy("/%s/add_placed_box" % scene_ns, AddPlacedBox)
        self.remove_placed = rospy.ServiceProxy(
            "/%s/remove_placed_box" % scene_ns, RemovePlacedBox)
        self.build_sequence = rospy.ServiceProxy("/%s/build_motion_sequence" % wpt_ns, BuildMotionSequence)
        self.plan_motion = rospy.ServiceProxy("/%s/plan_motion" % motion_ns, PlanMotion)
        self.vacuum = rospy.ServiceProxy("/%s/vacuum_command" % vacuum_ns, VacuumCommand)
        self.inspect_container = None
        if self.active_loading:
            self.inspect_container = rospy.ServiceProxy(
                "/%s/inspect_container" % inspector_ns, InspectContainer
            )
        self.go_to_pose = None
        self.go_to_joints = None
        self.reset_cargo_map = None
        self.integrate_cargo_view = None
        self.get_cargo_map_stats = None
        self.verify_placed_rgbd = None
        self.plan_next_cargo_view = None
        self.validate_motion_sequence = None
        if self._needs_mapper_services():
            self.reset_cargo_map = rospy.ServiceProxy(
                "/%s/reset_cargo_map" % mapper_ns, ResetCargoMap
            )
            self.integrate_cargo_view = rospy.ServiceProxy(
                "/%s/integrate_cargo_view" % mapper_ns, IntegrateCargoView
            )
            self.get_cargo_map_stats = rospy.ServiceProxy(
                "/%s/get_cargo_map_stats" % mapper_ns, GetCargoMapStats
            )
        if self.post_place_verify:
            self.verify_placed_rgbd = rospy.ServiceProxy(
                "/%s/verify" % verifier_ns, VerifyPlacedBox)
        if self._needs_explore_services():
            self.plan_next_cargo_view = rospy.ServiceProxy(
                "/%s/plan_next_cargo_view" % explore_ns, PlanNextCargoView
            )
            self.go_to_joints = rospy.ServiceProxy(
                "/%s/go_to_joint_values" % motion_ns, GoToJointValues
            )
            self.validate_motion_sequence = rospy.ServiceProxy(
                "/%s/validate_motion_sequence" % motion_ns,
                ValidateMotionSequence,
            )
        if not self.skip_reset or self.run_initial_explore or self.active_loading:
            self.go_to_pose = rospy.ServiceProxy(
                "/%s/go_to_robot_pose" % motion_ns, GoToRobotPose
            )

    # States whose message is acceptance evidence rather than progress chatter.
    # These survive a WARN-level run so an acceptance log stays readable while
    # still containing every field the harness and a human reviewer need.
    _EVIDENCE_STATES = ("Idle",)

    def publish_status(self, state, message, level="info"):
        msg = LoadTaskStatus(state=state, message=message, placed_count=len(self.placed))
        self.status_pub.publish(msg)
        # Idle is both the failure sink and the normal end of a run, so the
        # failure record is emitted here rather than at ~30 call sites.
        if state == "Idle" and not is_terminal_success(message):
            self._events.failure(message, state=self._current_state)
        if level == "warn" or state in self._EVIDENCE_STATES:
            rospy.logwarn(
                "[%s] %s (placed=%d)", state, message, len(self.placed))
        else:
            rospy.loginfo(
                "[%s] %s (placed=%d)", state, message, len(self.placed))

    def _handle_step(self, request):
        """Manual step control. Also the only way to mark a session tainted."""
        command = str(request.command or "").strip().lower()
        success, message = self._step_gate.command(
            command,
            target_state=request.target_state,
            breakpoints=list(request.breakpoints),
            clear_breakpoints=bool(request.clear_breakpoints),
            reason=request.reason,
        )
        if success and command == "taint":
            self._events.taint(request.reason)
            rospy.logwarn(
                "SESSION_TAINTED out-of-band call: %s", request.reason)
        snapshot = self._step_gate.snapshot()
        return OrchestratorStepResponse(
            success=success,
            message=message,
            paused_state=snapshot["paused_state"],
            paused=snapshot["paused"],
            placed_count=len(self.placed),
            run_mode=snapshot["mode"],
            probe_touched=snapshot["probe_touched"],
            active_breakpoints=snapshot["breakpoints"],
        )

    def _await_step_permission(self, state):
        """Block before ``state`` when a front-end is stepping the pipeline.

        Auto mode returns immediately, so the automated path is unchanged. An
        abort comes back as Idle, which drops the caller into the normal
        terminal path instead of leaving a half-committed placement.
        """
        self._current_state = state
        if self._step_gate.mode != "manual":
            return state
        granted = self._step_gate.wait(state)
        if granted != state:
            self.publish_status(
                "Idle", "RUN_ABORTED by step control before %s" % state)
        return granted

    def _needs_explore_services(self):
        return self.run_initial_explore or self.exploration_mode in (
            "smart", "interior_probe"
        )

    def _needs_mapper_services(self):
        return self._needs_explore_services() or (
            self.active_loading and self.post_place_verify)

    def _ensure_interior_loop(self):
        if not hasattr(self, "_interior_loop"):
            self._interior_loop = InteriorExploreLoop()
        if not hasattr(self, "_active_probe_retreat"):
            self._active_probe_retreat = None
        return self._interior_loop

    @staticmethod
    def _slot_to_dict(slot):
        return {
            "layer": slot.layer,
            "row": slot.row,
            "col": slot.col,
            "width": slot.width,
            "height": slot.height,
            "depth": slot.depth,
            "place_pose": {
                "position": {
                    "x": slot.place_pose.position.x,
                    "y": slot.place_pose.position.y,
                    "z": slot.place_pose.position.z,
                },
                "orientation": {
                    "x": slot.place_pose.orientation.x,
                    "y": slot.place_pose.orientation.y,
                    "z": slot.place_pose.orientation.z,
                    "w": slot.place_pose.orientation.w,
                },
            },
        }

    @staticmethod
    def _best_param_to_slot(best):
        if not best or not best.get("feasible", False):
            return None
        center = best["center_base"]
        local = best.get("center_local", [0.0, 0.0, 0.0])
        size = best["size"]
        yaw = float(best.get("yaw", 0.0))
        return SlotSpec(
            layer=int(round(local[2] * 1000.0)),
            row=int(round((local[1] + 10.0) * 1000.0)),
            col=int(round(local[0] * 1000.0)),
            width=size[0],
            depth=size[1],
            height=size[2],
            place_pose=Pose(
                position=Point(x=center[0], y=center[1], z=center[2]),
                orientation=Quaternion(z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5)),
            ),
        )

    def _run_motion_filter(self):
        if not self.filter_ns:
            return None
        if self.filter_placements is None:
            service = "/%s/filter_placements" % self.filter_ns
            try:
                rospy.wait_for_service(service, timeout=2.0)
                self.filter_placements = rospy.ServiceProxy(service, Trigger)
            except rospy.ROSException:
                rospy.logwarn_throttle(30.0, "motion filter %s unavailable", service)
                return None
        try:
            return self.filter_placements()
        except rospy.ServiceException as exc:
            rospy.logwarn("motion filter failed: %s", exc)
            return None

    def _compute_placement_pipeline(self):
        """Generate surface-map candidates, optionally filter, select best."""
        try:
            plan = self.plan_placements()
        except rospy.ServiceException as exc:
            return False, "placement planner failed: %s" % exc
        if not plan.success:
            return False, "placement planner: %s" % plan.message
        self.publish_status(
            "ComputePlacement", "candidates: %s" % plan.message, level="warn")

        fr = self._run_motion_filter()
        if fr is not None:
            self.publish_status(
                "ComputePlacement", "motion filter: %s" % fr.message,
                level="warn")

        best = rospy.get_param("/luggage/placement/best", {})
        slot = self._best_param_to_slot(best)
        backups = rospy.get_param("/luggage/placement/backups", [])
        self._placement_backups = [
            candidate for candidate in backups
            if candidate.get("feasible", False)
        ]
        self._placement_attempt = 1 if slot is not None else 0
        self._current_candidate_id = str(best.get("candidate_id", ""))
        explore_summary = rospy.get_param(
            "/luggage/workspace/explore_summary", {})
        placement_summary = rospy.get_param(
            "/luggage/workspace/placement_summary", {})
        rospy.set_param("/luggage/workspace/joint_summary", {
            "explore": explore_summary,
            "placement": placement_summary,
            "jointly_viable": bool(
                int(explore_summary.get("phase1_candidates", 0)) > 0
                and int(placement_summary.get("reachable", 0)) > 0
            ),
        })
        if slot is None:
            return False, "no feasible placement candidate"
        self.current_slot = slot
        return True, (
            "selected candidate=%s score=%.2f backups=%d"
            % (
                self._current_candidate_id,
                float(best.get("score", 0.0)),
                len(self._placement_backups),
            ))

    def _select_next_placement_backup(self, reason):
        no_good = list(rospy.get_param(
            "/luggage/placement/no_good_ids", []))
        if self._current_candidate_id:
            no_good.append(self._current_candidate_id)
        no_good = sorted(set(no_good))
        rospy.set_param("/luggage/placement/no_good_ids", no_good)
        if self._placement_attempt >= self._max_placement_attempts:
            return False, "placement attempts exhausted"
        for candidate in self._placement_backups:
            candidate_id = str(candidate.get("candidate_id", ""))
            if not candidate_id or candidate_id in no_good:
                continue
            slot = self._best_param_to_slot(candidate)
            if slot is None:
                continue
            response = self.build_sequence(
                self.current_luggage, slot, "place")
            if not response.success or not response.segments:
                no_good.append(candidate_id)
                rospy.set_param(
                    "/luggage/placement/no_good_ids",
                    sorted(set(no_good)),
                )
                continue
            self.current_slot = slot
            self._current_candidate_id = candidate_id
            self._placement_attempt += 1
            self.place_segments = response.segments
            self._segment_index = 0
            self._phase = "place"
            self._place_released = False
            self._released_at_contact = False
            self._retreat_after_release = False
            return True, (
                "retry candidate=%s attempt=%d after %s"
                % (candidate_id, self._placement_attempt, reason))
        return False, "no placement backup remains"

    def _remap_placed_box(self, slot):
        """Mark the just-placed box into the cargo voxel map (best-effort)."""
        if not self.post_place_remap or slot is None:
            return True, "post-place remap disabled"
        if self._mark_placed_in_map is None:
            service = "/%s/mark_placed_box" % self._mapper_ns
            try:
                rospy.wait_for_service(service, timeout=2.0)
                self._mark_placed_in_map = rospy.ServiceProxy(service, AddPlacedBox)
            except rospy.ROSException:
                rospy.logwarn_throttle(
                    30.0, "post-place remap skipped: %s unavailable", service
                )
                return False, "%s unavailable" % service
        try:
            resp = self._mark_placed_in_map(slot)
            self.publish_status(
                "UpdateOccupancy", "post-place remap: %s" % resp.message,
                level="warn")
            return bool(resp.success), resp.message
        except rospy.ServiceException as exc:
            rospy.logwarn("post-place remap failed: %s", exc)
            return False, str(exc)

    def _rollback_placed_map(self, slot):
        service = "/%s/unmark_placed_box" % self._mapper_ns
        if self._unmark_placed_in_map is None:
            try:
                rospy.wait_for_service(service, timeout=2.0)
                self._unmark_placed_in_map = rospy.ServiceProxy(
                    service, RemovePlacedBox)
            except rospy.ROSException:
                return False, "%s unavailable" % service
        try:
            response = self._unmark_placed_in_map(slot)
            return bool(response.success), response.message
        except rospy.ServiceException as exc:
            return False, str(exc)

    def _verify_post_place_surface(self, slot):
        """Integrate RGBD and confirm a top surface at the planned footprint."""
        if not self.post_place_verify:
            return True, "post-place verification disabled"
        if self.integrate_cargo_view is None:
            return False, "VERIFY_MAPPER_UNAVAILABLE"
        # The retreat service can return while Gazebo/controller velocities are
        # still just above the mapper's strict motion gate.
        rospy.sleep(self.post_place_settle_sec)
        if self.verify_placed_rgbd is not None:
            try:
                verified = self.verify_placed_rgbd(slot)
            except rospy.ServiceException as exc:
                return False, "VERIFY_RGBD_SERVICE: %s" % exc
            rospy.set_param("/luggage/verification/rgbd_pose", {
                "success": bool(verified.success),
                "message": verified.message,
                "xy_error": verified.xy_error,
                "z_error": verified.z_error,
                "yaw_error": verified.yaw_error,
                "roll": verified.roll,
                "pitch": verified.pitch,
                "drift": verified.drift,
                "point_count": verified.point_count,
                "measured_size": [
                    verified.measured_width,
                    verified.measured_depth,
                    verified.measured_height,
                ],
            })
            self._record_size_drift(slot, verified)
            if not verified.success:
                surface_only_ok = (
                    self.simulation_pose_oracle
                    and verified.point_count >= 100
                    and verified.drift <= self.actual_drift_tolerance
                )
                if not surface_only_ok:
                    return False, verified.message
                return True, (
                    "RGBD surface verified; Gazebo oracle owns simulation "
                    "XY/yaw: %s" % verified.message)
            if not self.simulation_pose_oracle:
                slot.place_pose = verified.actual_pose
            return True, verified.message
        last_message = "no response"
        for attempt in range(self.post_place_verify_retries):
            try:
                integrate = self.integrate_cargo_view(1)
            except rospy.ServiceException as exc:
                if attempt + 1 >= self.post_place_verify_retries:
                    return False, "VERIFY_INTEGRATE_SERVICE: %s" % exc
                rospy.sleep(0.5)
                continue
            last_message = integrate.message
            if not integrate.success:
                if attempt + 1 >= self.post_place_verify_retries:
                    return False, "VERIFY_INTEGRATE_FAILED: %s" % last_message
                rospy.sleep(0.5)
                continue
            try:
                sample = self._sample_surface_footprint(slot)
            except (KeyError, TypeError, ValueError, IndexError) as exc:
                return False, "VERIFY_SURFACE_INVALID: %s" % exc
            if sample is None:
                if attempt + 1 >= self.post_place_verify_retries:
                    return False, "VERIFY_SURFACE_UNKNOWN"
                rospy.sleep(0.5)
                continue
            observed_top, expected_top, tolerance, count = sample
            if abs(observed_top - expected_top) > tolerance:
                return False, (
                    "VERIFY_HEIGHT_MISMATCH observed=%.3f expected=%.3f "
                    "tol=%.3f" % (observed_top, expected_top, tolerance))
            return True, (
                "verified surface footprint cells=%d top=%.3f expected=%.3f"
                % (count, observed_top, expected_top))
        return False, "VERIFY_INTEGRATE_FAILED: %s" % last_message

    def _sample_surface_footprint(self, slot):
        """Return a robust top estimate from known cells in the box footprint."""
        surface = rospy.get_param("/luggage/cargo_map/surface_2d", {})
        resolution = float(surface["resolution"])
        nx, ny = int(surface["nx"]), int(surface["ny"])
        map_center = [float(v) for v in surface["center_base"]]
        map_yaw = float(surface["yaw"])
        origin = surface.get("origin_local", [
            -float(surface["inner_size"][0]) * 0.5,
            -float(surface["inner_size"][1]) * 0.5,
        ])
        dx = slot.place_pose.position.x - map_center[0]
        dy = slot.place_pose.position.y - map_center[1]
        lx = math.cos(-map_yaw) * dx - math.sin(-map_yaw) * dy
        ly = math.sin(-map_yaw) * dx + math.cos(-map_yaw) * dy
        ix = int((lx - float(origin[0])) / resolution)
        iy = int((ly - float(origin[1])) / resolution)
        if not (0 <= ix < nx and 0 <= iy < ny):
            raise ValueError("target outside map")
        _roll, _pitch, box_yaw = self._quaternion_to_rpy(
            slot.place_pose.orientation)
        radius = max(1, int(math.ceil(
            math.hypot(slot.width, slot.depth) * 0.5 / resolution)))
        observed = []
        for sx in range(max(0, ix - radius), min(nx, ix + radius + 1)):
            for sy in range(max(0, iy - radius), min(ny, iy + radius + 1)):
                cell_lx = float(origin[0]) + (sx + 0.5) * resolution
                cell_ly = float(origin[1]) + (sy + 0.5) * resolution
                bx = (
                    map_center[0] + math.cos(map_yaw) * cell_lx
                    - math.sin(map_yaw) * cell_ly)
                by = (
                    map_center[1] + math.sin(map_yaw) * cell_lx
                    + math.cos(map_yaw) * cell_ly)
                rdx = bx - slot.place_pose.position.x
                rdy = by - slot.place_pose.position.y
                box_x = (
                    math.cos(-box_yaw) * rdx
                    - math.sin(-box_yaw) * rdy)
                box_y = (
                    math.sin(-box_yaw) * rdx
                    + math.cos(-box_yaw) * rdy)
                if (
                        abs(box_x) <= slot.width * 0.4
                        and abs(box_y) <= slot.depth * 0.4
                        and surface["state"][sx][sy] != "unknown"):
                    observed.append(float(surface["height"][sx][sy]))
        if not observed:
            return None
        observed.sort()
        top_quartile = observed[max(0, int(len(observed) * 0.75)):]
        observed_top = top_quartile[len(top_quartile) // 2]
        map_floor_base_z = (
            map_center[2] - float(surface["inner_size"][2]) * 0.5)
        expected_top = (
            slot.place_pose.position.z + slot.height * 0.5
            - map_floor_base_z)
        tolerance = self.post_place_height_tolerance_voxels * resolution
        return observed_top, expected_top, tolerance, len(observed)

    def _verify_committed_map_top(self, slot):
        try:
            sample = self._sample_surface_footprint(slot)
            if sample is None:
                return False, "MAP_RECONCILE_SURFACE_UNKNOWN"
            observed, expected, _tolerance, count = sample
            resolution = float(rospy.get_param(
                "/luggage/cargo_map/surface_2d/resolution"))
            error = abs(observed - expected)
            if error > resolution:
                return False, (
                    "MAP_RECONCILE_TOP error=%.3f resolution=%.3f"
                    % (error, resolution))
            return True, (
                "map reconciled cells=%d top_error=%.3f"
                % (count, error))
        except (KeyError, TypeError, ValueError, IndexError) as exc:
            return False, "MAP_RECONCILE_INVALID: %s" % exc

    @staticmethod
    def _quaternion_to_rpy(quaternion):
        x, y, z, w = (
            quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        sinr = 2.0 * (w * x + y * z)
        cosr = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr, cosr)
        sinp = 2.0 * (w * y - z * x)
        pitch = (
            math.copysign(math.pi * 0.5, sinp)
            if abs(sinp) >= 1.0 else math.asin(sinp))
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        return roll, pitch, math.atan2(siny, cosy)

    @staticmethod
    def _yaw_error_mod_pi(actual, expected):
        delta = actual - expected
        return abs(math.atan2(
            math.sin(2.0 * delta), math.cos(2.0 * delta))) * 0.5

    def _gazebo_model_pose_in_base(self, model_name):
        if self._get_model_state is None:
            return None
        response = self._get_model_state(model_name, self._world_frame)
        if not response.success:
            return None
        stamped = PoseStamped()
        stamped.header.frame_id = self._world_frame
        stamped.header.stamp = rospy.Time(0)
        stamped.pose = response.pose
        transform = self._tf_buffer.lookup_transform(
            self._base_frame, self._world_frame,
            rospy.Time(0), rospy.Duration(1.0))
        return do_transform_pose(stamped, transform).pose

    def _verify_actual_placement(self, slot):
        """Verify and adopt the settled physical model pose before commit."""
        if not self.verify_actual_pose:
            return True, "actual pose verification disabled"
        if not self.simulation_pose_oracle or self._get_model_state is None:
            if self.verify_placed_rgbd is not None:
                return True, "actual pose deferred to production RGBD verifier"
            return False, "ACTUAL_POSE_SOURCE_UNAVAILABLE"
        model_name = getattr(self._spawned_manifest, "id", "")
        if not model_name:
            return False, "ACTUAL_MODEL_ID_MISSING"
        try:
            first = self._gazebo_model_pose_in_base(model_name)
            if first is None:
                return False, "ACTUAL_MODEL_POSE_MISSING"
            rospy.sleep(self.actual_drift_wait_sec)
            actual = self._gazebo_model_pose_in_base(model_name)
        except (
                rospy.ServiceException,
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            return False, "ACTUAL_POSE_QUERY_FAILED: %s" % exc
        if actual is None:
            return False, "ACTUAL_MODEL_POSE_MISSING_AFTER_WAIT"
        drift = math.sqrt(
            (actual.position.x - first.position.x) ** 2
            + (actual.position.y - first.position.y) ** 2
            + (actual.position.z - first.position.z) ** 2)
        dx = actual.position.x - slot.place_pose.position.x
        dy = actual.position.y - slot.place_pose.position.y
        dz = actual.position.z - slot.place_pose.position.z
        xy_error = math.hypot(dx, dy)
        roll, pitch, actual_yaw = self._quaternion_to_rpy(
            actual.orientation)
        _pr, _pp, planned_yaw = self._quaternion_to_rpy(
            slot.place_pose.orientation)
        yaw_error = self._yaw_error_mod_pi(actual_yaw, planned_yaw)
        diagnostics = {
            "model_name": model_name,
            "xy_error": xy_error,
            "z_error": abs(dz),
            "yaw_error": yaw_error,
            "roll": roll,
            "pitch": pitch,
            "drift": drift,
            "actual_position": [
                actual.position.x, actual.position.y, actual.position.z],
        }
        rospy.set_param("/luggage/verification/actual_pose", diagnostics)
        failures = []
        if xy_error > self.actual_xy_tolerance:
            failures.append("xy=%.3f>%.3f" % (
                xy_error, self.actual_xy_tolerance))
        if abs(dz) > self.actual_z_tolerance:
            failures.append("z=%.3f>%.3f" % (
                abs(dz), self.actual_z_tolerance))
        if yaw_error > self.actual_yaw_tolerance:
            failures.append("yaw=%.3f>%.3f" % (
                yaw_error, self.actual_yaw_tolerance))
        if max(abs(roll), abs(pitch)) > self.actual_rp_tolerance:
            failures.append("rp=(%.3f,%.3f)>%.3f" % (
                roll, pitch, self.actual_rp_tolerance))
        if drift > self.actual_drift_tolerance:
            failures.append("drift=%.3f>%.3f" % (
                drift, self.actual_drift_tolerance))
        if failures:
            return False, "ACTUAL_POSE_GATE: %s" % ", ".join(failures)
        # Commit the measured physical pose, not the planned ghost.
        slot.place_pose = actual
        return True, (
            "actual pose verified xy=%.3f z=%.3f yaw=%.3f rp=(%.3f,%.3f) "
            "drift=%.3f" % (
                xy_error, abs(dz), yaw_error, roll, pitch, drift))

    @staticmethod
    def _load_size_range():
        """Per-axis sampling envelope from the box catalog."""
        try:
            desc_scripts = os.path.join(
                rospkg.RosPack().get_path("luggage_description"), "scripts")
            if desc_scripts not in sys.path:
                sys.path.insert(0, desc_scripts)
            from box_catalog_utils import box_size_range, load_box_catalog
            return box_size_range(load_box_catalog())
        except Exception as exc:  # noqa: BLE001 - degrade to a permissive gate
            rospy.logwarn(
                "box catalog unavailable for size plausibility (%s); "
                "using a permissive envelope", exc)
            return ((0.05, 2.0), (0.05, 2.0), (0.05, 2.0))

    def _check_size_plausible(self, box):
        """Physical sanity gate on a perception-owned size.

        Replaces the old manifest comparison. Continuous box sizes mean there
        is no expected size to check against, so the gate asks the only
        question that can be asked without the answer key: could a real
        suitcase have these dimensions? A gross estimator failure (a sliver of
        a partially visible top face, or the platform plane) fails this; an
        ordinary few-centimetre error passes and is handled downstream by the
        size-uncertainty margin.
        """
        dims = (("width", box.width), ("depth", box.depth),
                ("height", box.height))
        for axis, (name, value) in enumerate(dims):
            low, high = self._size_range[axis]
            if not (low - self._size_tolerance <= value
                    <= high + self._size_tolerance):
                return False, "%s=%.3f outside [%.3f, %.3f]" % (
                    name, value, low, high)
        return True, "ok"

    def _record_size_eval(self, box):
        """Publish detected-vs-spawned size for metrics only, never for control."""
        spawned = rospy.get_param(
            "/luggage/perception/size_eval/spawned", {}) or {}
        orientation = box.pose.orientation
        record = {
            "detected": [box.width, box.depth, box.height],
            "detected_yaw": round(math.atan2(
                2.0 * (orientation.w * orientation.z
                       + orientation.x * orientation.y),
                1.0 - 2.0 * (orientation.y * orientation.y
                             + orientation.z * orientation.z)), 4),
        }
        if spawned:
            truth = [
                float(spawned.get("width", 0.0)),
                float(spawned.get("depth", 0.0)),
                float(spawned.get("height", 0.0)),
            ]
            record["spawned"] = truth
            record["errors"] = [
                round(record["detected"][i] - truth[i], 4) for i in range(3)]
        rospy.set_param("/luggage/perception/size_eval/latest", record)
        rospy.logwarn("SIZE_EVAL %s", json.dumps(record, sort_keys=True))
        return record

    def _record_size_drift(self, slot, verified):
        """Compare the placed box's measured extents with the pickup estimate.

        This closes the loop the size-uncertainty margin is guessing at: if the
        placed box measures consistently larger than what was grasped, the
        margin is too small and the next box will be planned into it. Recorded
        as telemetry rather than a gate -- the top-face view is noisier than the
        pickup view, so failing a run on it would trade a real defect for a
        measurement artifact.
        """
        measured = [
            verified.measured_width,
            verified.measured_depth,
            verified.measured_height,
        ]
        if min(measured) <= 0.0:
            return None
        planned = [slot.width, slot.depth, slot.height]
        # Pickup and placement axes may be swapped by a 90 degree yaw, so
        # compare the footprint as an unordered pair.
        deltas = [
            round(max(measured[0], measured[1]) - max(planned[0], planned[1]), 4),
            round(min(measured[0], measured[1]) - min(planned[0], planned[1]), 4),
            round(measured[2] - planned[2], 4),
        ]
        record = {
            "planned": [round(v, 4) for v in planned],
            "measured": [round(v, 4) for v in measured],
            "deltas": deltas,
            "max_abs_delta": round(max(abs(v) for v in deltas), 4),
        }
        rospy.set_param("/luggage/verification/size_drift", record)
        if record["max_abs_delta"] > self._size_drift_warn_m:
            rospy.logwarn("SIZE_DRIFT %s", json.dumps(record, sort_keys=True))
        else:
            rospy.loginfo("SIZE_DRIFT %s", json.dumps(record, sort_keys=True))
        return record

    def _commit_slot(self, slot):
        """Conservative copy of ``slot`` for scene / map / free-space commits.

        The motion has already run against the measured size; what gets
        recorded is deliberately larger so a size underestimate cannot let the
        next box be planned into this one. See size_uncertainty for why the
        margin is asymmetric.
        """
        if self._size_xy_margin <= 0.0 and self._size_z_margin <= 0.0:
            return slot
        width, depth, height = inflate_size(
            (slot.width, slot.depth, slot.height),
            self._size_xy_margin, self._size_z_margin)
        pose = Pose(
            position=Point(
                x=slot.place_pose.position.x,
                y=slot.place_pose.position.y,
                z=inflated_center_z(
                    slot.place_pose.position.z, self._size_z_margin),
            ),
            orientation=slot.place_pose.orientation,
        )
        return SlotSpec(
            layer=slot.layer, row=slot.row, col=slot.col,
            width=width, depth=depth, height=height, place_pose=pose)

    def _check_commit_overlap(self, slot):
        """Reject a commit that would intersect an already committed box.

        This is the direct evidence that the size margin is large enough: if
        the margin is too small the overlap shows up here rather than as a box
        being nudged off its support several placements later.
        """
        boxes = [
            ("placed_%d" % index, box_aabb(
                (item.place_pose.position.x, item.place_pose.position.y,
                 item.place_pose.position.z),
                (item.width, item.depth, item.height)))
            for index, item in enumerate(self.placed)
        ]
        boxes.append(("candidate", box_aabb(
            (slot.place_pose.position.x, slot.place_pose.position.y,
             slot.place_pose.position.z),
            (slot.width, slot.depth, slot.height))))
        overlaps = [
            entry for entry in find_overlaps(
                boxes, tolerance=self._overlap_tolerance_m3)
            if "candidate" in (entry[0], entry[1])
        ]
        if not overlaps:
            return True, "no overlap with %d committed boxes" % len(self.placed)
        worst = max(overlaps, key=lambda entry: entry[2])
        return False, "COMMIT_AABB_OVERLAP with %s volume=%.5f m3" % (
            worst[0] if worst[1] == "candidate" else worst[1], worst[2])

    def _publish_placed_param(self):
        rospy.set_param(
            "/luggage/container_inspection/placed_boxes",
            [self._slot_to_dict(slot) for slot in self.placed],
        )

    def _log_placement_commit(self, slot):
        """Emit one machine-readable record per committed placement.

        Utilization acceptance needs the support height, footprint and whether
        a floor candidate was still available; scraping those back out of free
        text is brittle, so the run log carries the same schema the bag harness
        consumes.
        """
        best = rospy.get_param("/luggage/placement/best", {}) or {}
        candidates = rospy.get_param("/luggage/placement/candidates", []) or []
        floor_available = sum(
            1 for cand in candidates
            if cand.get("feasible")
            and float(cand.get("peak", 0.0)) <= 1e-3
            and float(cand.get("reachability_score", -1.0)) > 0.0)
        record = {
            "kind": "placement",
            "index": len(self.placed),
            "candidate_id": best.get("candidate_id", ""),
            "map_revision": best.get("map_revision", -1),
            "peak": round(float(best.get("peak", 0.0)), 4),
            "footprint": [
                round(float(v), 4)
                for v in best.get("footprint", [slot.width, slot.depth])],
            "size": [
                round(slot.width, 4), round(slot.depth, 4),
                round(slot.height, 4)],
            "center_base": [
                round(slot.place_pose.position.x, 4),
                round(slot.place_pose.position.y, 4),
                round(slot.place_pose.position.z, 4)],
            "container_x": round(float(best.get(
                "center_local", [0.0, 0.0, 0.0])[0]), 4),
            "container_y": round(float(best.get(
                "center_local", [0.0, 0.0, 0.0])[1]), 4),
            "atlas_status": int(best.get("atlas_status", -1)),
            "pose_gate_passed": True,
            "floor_candidates_available": floor_available,
            "stale_revision": False,
        }
        rospy.logwarn("PLACEMENT_COMMIT %s", json.dumps(record, sort_keys=True))
        # The harness cycle_budget gate reads cycle_sec, which the log record
        # never carried. Measured spawn -> commit, so it covers the whole box.
        cycle_sec = None
        if self._cycle_started is not None:
            cycle_sec = max(0.0, rospy.get_time() - self._cycle_started)
        self._events.placement(record, cycle_sec=cycle_sec)
        self._events.map_event("commit", record["map_revision"])
        return record

    def _next_after_scene_sync(self):
        if self.active_loading:
            return "SpawnCurrentBox"
        return "Detect"

    def _next_after_reset(self):
        if self.run_initial_explore and not self._initial_explore_done:
            return "InitialExploreCargo"
        return "SyncScene"

    def _next_after_spawn(self):
        if self.exploration_mode in ("smart", "interior_probe"):
            return "ExploreCargo"
        return "InspectContainer"

    def _next_after_inspect(self):
        return "ComputePlacement" if self.active_loading else "Detect"

    def _attempt_probe_retreat(self, segment):
        """Best-effort safety retreat that also handles lost service responses."""
        try:
            response = self.plan_motion(segment)
        except Exception as exc:
            rospy.logerr("Probe retreat service call failed: %s", exc)
            return False, str(exc)
        if not response.success:
            rospy.logerr("Probe retreat failed: %s", response.message)
            return False, response.message
        return True, response.message

    def _retry_explore_candidate(
            self, state, mode, views_used, done_state, reset_on_start,
            candidate_attempt, message):
        max_retries = int(getattr(self, "explore_candidate_retries", 4))
        if mode == "smart" and candidate_attempt < max_retries:
            self.publish_status(
                state,
                "candidate %d rejected (%s); trying next"
                % (candidate_attempt + 1, message),
            )
            return self._run_explore_sequence(
                state, mode, views_used, done_state,
                reset_on_start=False,
                candidate_attempt=candidate_attempt + 1,
            )
        self.publish_status("Idle", message)
        return "Idle"

    def _refresh_explore_scene(self, diagnostic=""):
        """Prune stale dyn_obs and clear only a proven octomap self-contact."""
        try:
            response = self.sync_dynamic_scene()
            if not response.success:
                rospy.logwarn("dynamic scene sync failed: %s", response.message)
        except (rospy.ServiceException, rospy.ROSException) as exc:
            rospy.logwarn("dynamic scene sync unavailable: %s", exc)
        if "<octomap>" in diagnostic:
            try:
                self.clear_octomap()
                rospy.logwarn(
                    "Cleared octomap after confirmed start-state contact")
            except (rospy.ServiceException, rospy.ROSException) as exc:
                rospy.logwarn("clear octomap unavailable: %s", exc)

    # ── PR5: perception (self-filter + ROI) gates ──────────────────────

    def _on_filter_stats(self, msg):
        """Cache the latest task_cloud_filter stats_json."""
        import json
        try:
            self._filter_status = json.loads(msg.data)
            self._filter_status_stamp = rospy.Time.now().to_sec()
        except (ValueError, TypeError):
            rospy.logwarn_throttle(5.0, "Invalid filter stats JSON")

    def _check_perception_gate(self):
        """Return (ok, reason) for the self-filter + ROI readiness gate.

        When the task_cloud_filter is not deployed (no stats received), the
        gate passes (backward-compat with legacy bags / no-filter runs).
        """
        import json
        fs = getattr(self, "_filter_status", {})
        if not fs:
            return True, "filter_status_unavailable"
        # Stale stats -> treat as not ready.
        stamp = getattr(self, "_filter_status_stamp", 0.0)
        if rospy.Time.now().to_sec() - stamp > self._filter_status_age_sec:
            return False, "self_filter_not_ready"
        if not fs.get("ready", False):
            return False, fs.get("reason", "self_filter_not_ready")
        if not fs.get("task_roi_ready", False):
            return False, "task_roi_unavailable"
        if int(fs.get("unsafe_passthrough_count", 0)) > 0:
            return False, "unsafe_passthrough"
        if float(fs.get("cloud_age", 999.0)) > 0.50:
            return False, "task_cloud_stale"
        return True, "ok"

    def _fresh_arm_joints(self):
        """Read a complete current arm state; retain the last seed on timeout."""
        try:
            msg = rospy.wait_for_message(
                getattr(self, "_joint_state_topic", "/joint_states"),
                JointState,
                timeout=float(getattr(self, "_joint_state_timeout", 1.0)),
            )
        except rospy.ROSException as exc:
            rospy.logwarn(
                "Explore joint seed unavailable; using cached/observe seed: %s",
                exc,
            )
            return list(self._current_joint_values)
        values = dict(zip(msg.name, msg.position))
        missing = [name for name in ARM_JOINT_NAMES if name not in values]
        if missing:
            rospy.logwarn(
                "Explore joint seed missing %s; using cached/observe seed",
                ", ".join(missing),
            )
            return list(self._current_joint_values)
        self._current_joint_values = [
            float(values[name]) for name in ARM_JOINT_NAMES]
        return list(self._current_joint_values)

    def _run_explore_sequence(
            self, state, mode, views_used, done_state, reset_on_start=True,
            candidate_attempt=0):
        self._ensure_interior_loop()
        # PR5: if inserted and the perception gate fails, retreat immediately.
        if self._interior_loop.inserted:
            gate_ok, gate_reason = self._check_perception_gate()
            if not gate_ok:
                self._interior_loop.request_retreat(gate_reason)
                retreat_ok, retreat_message = (
                    self._attempt_probe_retreat(self._active_probe_retreat))
                self._interior_loop.retreated(retreat_ok, retreat_message)
                return done_state if retreat_ok else "Idle"
        if views_used == 0 and reset_on_start:
            self.publish_status(state, "resetting cargo voxel map")
            self._interior_loop.reset()
            self._active_probe_retreat = None
            reset = self.reset_cargo_map()
            if not reset.success:
                self.publish_status("Idle", "reset cargo map failed: %s" % reset.message)
                return "Idle"

        current_joints = self._fresh_arm_joints()
        plan = self.plan_next_cargo_view(
            mode,
            current_joints,
            ARM_JOINT_NAMES if current_joints else [],
            views_used,
            bool(views_used == 0 and candidate_attempt == 0),
            bool(self.dry_run_motion),
        )
        if not plan.success:
            if (
                    "start_state_invalid" in plan.message
                    and candidate_attempt < self.explore_candidate_retries):
                self._refresh_explore_scene(plan.message)
                return self._run_explore_sequence(
                    state, mode, views_used, done_state,
                    reset_on_start=False,
                    candidate_attempt=candidate_attempt + 1,
                )
            if self._interior_loop.inserted:
                self._interior_loop.request_retreat("planner_unavailable")
                retreat_ok, retreat_message = self._attempt_probe_retreat(
                    self._active_probe_retreat)
                self._interior_loop.retreated(
                    retreat_ok, retreat_message)
            self.publish_status("Idle", "explore unavailable: %s" % plan.message)
            return "Idle"
        if plan.done:
            if self._interior_loop.inserted:
                self._interior_loop.request_retreat("planner_done")
                retreat_ok, retreat_message = self._attempt_probe_retreat(
                    self._active_probe_retreat)
                self._interior_loop.retreated(
                    retreat_ok, retreat_message)
                if not retreat_ok:
                    self.publish_status(
                        "Idle", "probe retreat failed: %s" % retreat_message)
                    return "Idle"
            self.publish_status(state, "explore finished: %s" % plan.message)
            return done_state

        joint_names = list(plan.joint_names) if plan.joint_names else []
        joint_values = list(plan.joint_values)
        is_probe = getattr(plan, "plan_type", "") == "cartesian_probe"
        closed_loop_probe = bool(
            is_probe
            and getattr(plan, "candidate_id", "")
            and getattr(plan, "lane_id", "")
            and getattr(self, "validate_motion_sequence", None) is not None
        )
        pre_stats = (
            self.get_cargo_map_stats()
            if closed_loop_probe and not self.dry_run_motion else None)
        if not joint_values and not is_probe:
            self.publish_status("Idle", "explore plan returned empty joints")
            return "Idle"

        validation_segments = []
        if is_probe:
            try:
                validation_segments = build_interior_probe_segments(
                    plan.waypoints, MotionSegment,
                    require_tool_down=(getattr(plan, "stage", "") == "phase1"),
                    align_before_probe=bool(rospy.get_param(
                        "/luggage/downward/align_before_probe", True)),
                )
                if closed_loop_probe and self._interior_loop.inserted:
                    validation_segments = [
                        segment for segment in validation_segments
                        if segment.name in ("probe_inside", "retreat_opening")
                    ]
            except ValueError as exc:
                return self._retry_explore_candidate(
                    state, mode, views_used, done_state,
                    reset_on_start, candidate_attempt, str(exc))
        elif getattr(plan, "plan_type", "") == "pose_target":
            validation_segments = [build_smart_explore_segment(
                plan.camera_pose, MotionSegment,
                phase=getattr(plan, "stage", "phase1"),
            )]
        if validation_segments and getattr(
                self, "validate_motion_sequence", None) is not None:
            try:
                validation = self.validate_motion_sequence(
                    validation_segments)
            except Exception as exc:
                return self._retry_explore_candidate(
                    state, mode, views_used, done_state,
                    reset_on_start, candidate_attempt,
                    "explore validation service failed: %s" % exc,
                )
            if not validation.success:
                return self._retry_explore_candidate(
                    state, mode, views_used, done_state,
                    reset_on_start, candidate_attempt,
                    "explore validation rejected: %s (%s)" % (
                        validation.message, validation.rejection_reason),
                )
            if closed_loop_probe and not self.dry_run_motion:
                if self._interior_loop.state == BOOTSTRAP_OPENING:
                    gate_ok, gate_reason = self._check_perception_gate()
                    opening_ok = bool(plan.candidate_id and plan.lane_id)
                    self._interior_loop.geometry_ready(
                        opening_ok and gate_ok,
                        gate_reason if not gate_ok
                        else "opening_geometry_or_lane_missing",
                    )
                if self._interior_loop.state in (
                        SELECT_CORRIDOR, REPLAN_DEPTH):
                    loop_state = self._interior_loop.select_candidate(
                        plan.candidate_id,
                        plan.lane_id,
                        plan.insertion_depth,
                        validation.retreat_feasible,
                        hard_feasible=validation.success,
                    )
                    if loop_state == RETREAT:
                        retreat_ok, retreat_message = (
                            self._attempt_probe_retreat(
                                self._active_probe_retreat))
                        self._interior_loop.retreated(
                            retreat_ok, retreat_message)
                        return done_state if retreat_ok else "Idle"

        if self.dry_run_motion:
            self.publish_status(
                state,
                "dry-run view %d validated: %s (%d waypoints)" % (
                    plan.view_index, plan.message,
                    len(getattr(plan, "waypoints", [])),
                ),
            )
            return state, views_used
        else:
            self.publish_status(state, "moving to explore view %d" % plan.view_index)
            probe_retreat = None
            if is_probe:
                segments = validation_segments
                probe_retreat = segments[-1]
                for segment in segments[:-1]:
                    may_be_inserted = segment.name in (
                        "enter_opening", "probe_inside")
                    # PR5: re-check perception gate before entering the container.
                    if segment.name == "enter_opening":
                        gate_ok, gate_reason = self._check_perception_gate()
                        if not gate_ok:
                            if self._interior_loop.inserted:
                                self._interior_loop.request_retreat(
                                    gate_reason)
                                retreat_ok, retreat_message = (
                                    self._attempt_probe_retreat(probe_retreat))
                                self._interior_loop.retreated(
                                    retreat_ok, retreat_message)
                                return "Idle"
                            return self._retry_explore_candidate(
                                state, mode, views_used, done_state,
                                reset_on_start, candidate_attempt,
                                "perception_gate: %s" % gate_reason)
                    try:
                        # align_down uses the pre-opening IK branch from the
                        # explorer. Pose planning often picks an equivalent
                        # wrist-heavy OMPL path that fails wrist quality.
                        if segment.name == "align_down":
                            move = self.go_to_joints(
                                list(plan.joint_values),
                                list(plan.joint_names or []),
                            )
                        else:
                            move = self.plan_motion(segment)
                    except Exception as exc:
                        if may_be_inserted:
                            self._attempt_probe_retreat(probe_retreat)
                            self.publish_status(
                                "Idle", "explore %s service failed: %s" % (
                                    segment.name, exc,
                                ),
                            )
                            return "Idle"
                        return self._retry_explore_candidate(
                            state, mode, views_used, done_state,
                            reset_on_start, candidate_attempt,
                            "explore %s service failed: %s"
                            % (segment.name, exc),
                        )
                    if not move.success:
                        if may_be_inserted:
                            self._attempt_probe_retreat(probe_retreat)
                            self.publish_status(
                                "Idle", "explore %s failed: %s" % (
                                    segment.name, move.message,
                                ),
                            )
                            return "Idle"
                        return self._retry_explore_candidate(
                            state, mode, views_used, done_state,
                            reset_on_start, candidate_attempt,
                            "explore %s failed: %s"
                            % (segment.name, move.message),
                        )
            elif getattr(plan, "plan_type", "") == "pose_target":
                # Smart downward view: plan_motion with dual downward
                # constraints (camera + suction near base -Z). The planner has
                # already IK-validated and consumed this candidate, so on
                # motion failure we stop and let the next explore step take
                # the following candidate.
                segment = validation_segments[0]
                try:
                    move = self.plan_motion(segment)
                except Exception as exc:
                    return self._retry_explore_candidate(
                        state, mode, views_used, done_state,
                        reset_on_start, candidate_attempt,
                        "explore motion service failed: %s" % exc,
                    )
                if not move.success:
                    return self._retry_explore_candidate(
                        state, mode, views_used, done_state,
                        reset_on_start, candidate_attempt,
                        "explore motion failed: %s" % move.message,
                    )
            else:
                move = self.go_to_joints(joint_values, joint_names)
                if not move.success:
                    self.publish_status("Idle", "explore move failed: %s" % move.message)
                    return "Idle"
            if closed_loop_probe:
                self._active_probe_retreat = probe_retreat
                if (
                        self._interior_loop.pending_candidate_id is None
                        and self._interior_loop.state in (
                            SELECT_CORRIDOR, REPLAN_DEPTH)):
                    self._interior_loop.select_candidate(
                        plan.candidate_id,
                        plan.lane_id,
                        plan.insertion_depth,
                        retreat_valid=True,
                        hard_feasible=True,
                    )
                self._interior_loop.entered()
            rospy.sleep(self.settle_time_sec)
        integrate = None
        integration_error = None
        try:
            integrate = self.integrate_cargo_view(1)
        except Exception as exc:
            integration_error = str(exc)
        if is_probe and not closed_loop_probe and not self.dry_run_motion:
            retreat_ok, retreat_message = self._attempt_probe_retreat(
                probe_retreat)
            if not retreat_ok:
                self.publish_status(
                    "Idle", "probe retreat failed: %s" % retreat_message)
                return "Idle"
        if integration_error is not None:
            if closed_loop_probe and self._interior_loop.inserted:
                self._interior_loop.request_retreat("integration_service_failed")
                retreat_ok, retreat_message = self._attempt_probe_retreat(
                    self._active_probe_retreat)
                self._interior_loop.retreated(
                    retreat_ok, retreat_message)
            self.publish_status(
                "Idle", "integrate view service failed: %s" % integration_error
            )
            return "Idle"
        if not integrate.success:
            if closed_loop_probe and self._interior_loop.inserted:
                self._interior_loop.request_retreat("integration_failed")
                retreat_ok, retreat_message = self._attempt_probe_retreat(
                    self._active_probe_retreat)
                self._interior_loop.retreated(
                    retreat_ok, retreat_message)
            self.publish_status("Idle", "integrate view failed: %s" % integrate.message)
            return "Idle"

        if is_probe:
            self._fresh_arm_joints()
        else:
            self._current_joint_values = joint_values
        views_used += 1
        stats = self.get_cargo_map_stats()
        self.publish_status(
            state,
            "view %d integrated unknown=%.0f%% occupancy=%.0f%%"
            % (
                views_used,
                integrate.unknown_ratio * 100.0,
                integrate.occupancy_ratio * 100.0,
            ),
        )

        if closed_loop_probe:
            improvement = (
                float(pre_stats.unknown_ratio) - float(stats.unknown_ratio)
                if pre_stats is not None
                and pre_stats.success and stats.success else 0.0
            )
            done = bool(
                stats.success
                and stats.unknown_ratio <= float(
                    rospy.get_param("~unknown_threshold", 0.15))
            )
            loop_state = self._interior_loop.observation_committed(
                int(getattr(stats, "map_revision", plan.map_revision)),
                improvement,
                geometry_valid=True,
                done=done,
            )
            if loop_state == RETREAT:
                retreat_ok, retreat_message = self._attempt_probe_retreat(
                    self._active_probe_retreat)
                self._interior_loop.retreated(
                    retreat_ok, retreat_message)
                if not retreat_ok:
                    self.publish_status(
                        "Idle", "probe retreat failed: %s" % retreat_message)
                    return "Idle"
                return done_state

        if not is_probe and stats.success and stats.unknown_ratio <= float(
            rospy.get_param("~unknown_threshold", 0.15)
        ):
            return done_state
        return state, views_used

    def _run_initial_explore_cargo(self, state):
        if self._initial_explore_views_used == 0:
            self.publish_status(state, "syncing planning scene before initial explore")
            sync_resp = self.sync_scene()
            if not sync_resp.success:
                rospy.logwarn("Scene sync before explore failed: %s", sync_resp.message)
        result = self._run_explore_sequence(
            state,
            self.exploration_mode,
            self._initial_explore_views_used,
            "ReturnObserveAfterInitialExplore",
        )
        if isinstance(result, tuple):
            next_state, views_used = result
            self._initial_explore_views_used = views_used
            return next_state
        self._initial_explore_views_used = 0
        self._initial_explore_done = True
        return result

    def _run_explore_cargo(self, state):
        result = self._run_explore_sequence(
            state,
            self.exploration_mode,
            self._explore_views_used,
            "InspectContainer",
        )
        if isinstance(result, tuple):
            next_state, views_used = result
            self._explore_views_used = views_used
            return next_state
        self._explore_views_used = 0
        return result

    def _vacuum_attached(self):
        return bool(rospy.get_param(self._vacuum_attached_param, False))

    def _attempt_vacuum_attach(self):
        resp = self.vacuum(True)
        if not resp.success:
            self.publish_status("Idle", "vacuum attach failed: %s" % resp.message)
            return False
        if self.require_vacuum_attach and not self._vacuum_attached():
            self.publish_status("Idle", "vacuum attach did not report attached state")
            return False
        return True

    def _execute_segment_or_advance(self, segments, next_state, state):
        if self._segment_index >= len(segments):
            return next_state
        seg = segments[self._segment_index]
        if self.dry_run_motion:
            self.publish_status(state, "dry-run segment: %s" % seg.name)
            self._segment_index += 1
            return state
        self.publish_status(state, "executing %s segment: %s" % (self._phase, seg.name))
        resp = self.plan_motion(seg)
        if resp.success:
            self.publish_status(state, "%s segment done: %s" % (seg.name, resp.message))
            self._segment_index += 1
            return state
        self.publish_status("Idle", "%s motion failed: %s" % (self._phase, resp.message))
        return "Idle"

    def _clear_pick_dynamic_obstacles(self):
        """Clear perception-derived obstacles before planning the known pickup box.

        Active loading knows the current box geometry from pickup_box_spawner and
        syncs it into MoveIt as current_pickup_box. During pick, the raw
        world-scene/octomap obstacles often include the target box and the robot's
        own camera mount, which makes the goal state invalid. Use the explicit
        pickup-box collision object for pick planning and drop stale perception
        obstacles before the pick sequence.
        """
        try:
            self.clear_octomap()
            self.publish_status("PlanPick", "cleared MoveIt octomap before pick")
        except (rospy.ServiceException, rospy.ROSException) as exc:
            rospy.logwarn("clear octomap unavailable before pick: %s", exc)

        try:
            resp = self.clear_dynamic_scene()
            if resp.success:
                self.publish_status("PlanPick", resp.message)
            else:
                rospy.logwarn("clear dynamic scene failed: %s", resp.message)
        except (rospy.ServiceException, rospy.ROSException) as exc:
            rospy.logwarn("clear dynamic scene unavailable before pick: %s", exc)

    def run(self):
        if self.skip_reset and self.run_initial_explore:
            state = "InitialExploreCargo"
        else:
            state = "SyncScene" if self.skip_reset else "ResetObserve"
        self._write_session_record(state)
        while not rospy.is_shutdown() and state != "Idle":
            # Manual stepping hooks in here and nowhere else: the state bodies
            # below stay exactly as the automated path runs them.
            state = self._await_step_permission(state)
            if state == "Idle":
                break

            if state == "ResetObserve":
                self.publish_status(state, "moving to observe pose")
                resp = self.go_to_pose(self.observe_pose)
                if resp.success:
                    msg = resp.message
                    if resp.already_there:
                        msg = "already at observe pose"
                    self.publish_status(state, msg)
                    state = self._next_after_reset()
                else:
                    self.publish_status("Idle", "observe reset failed: %s" % resp.message)
                    state = "Idle"

            elif state == "InitialExploreCargo":
                state = self._run_initial_explore_cargo(state)

            elif state == "ReturnObserveAfterInitialExplore":
                self.publish_status(state, "returning to observe after initial explore")
                self._refresh_explore_scene()
                resp = self.go_to_pose(self.observe_pose)
                if resp.success:
                    msg = resp.message
                    if resp.already_there:
                        msg = "already at observe pose"
                    self.publish_status(state, msg)
                    state = "SyncScene"
                else:
                    self.publish_status(
                        "Idle",
                        "return observe after initial explore failed: %s" % resp.message,
                    )
                    state = "Idle"

            elif state == "SyncScene":
                self.publish_status(state, "syncing static scene")
                resp = self.sync_scene()
                if not resp.success:
                    self.publish_status("Idle", "scene sync failed: %s" % resp.message)
                    state = "Idle"
                else:
                    self._publish_placed_param()
                    state = self._next_after_scene_sync()

            elif state == "SpawnCurrentBox":
                if len(self.placed) >= self.max_placed:
                    self.publish_status("Idle", "max placed reached — done")
                    state = "Idle"
                    continue
                self.publish_status(state, "spawning fixed-source pickup box")
                self._cycle_started = rospy.get_time()
                resp = self.spawn_next_box()
                if resp.success:
                    self._spawned_manifest = resp.box
                    self._spawner_gt_box = (
                        None if self.strict_perception else resp.box)
                    self.current_slot = None
                    rospy.set_param("/luggage/placement/no_good_ids", [])
                    self.publish_status(
                        state,
                        "%s size=%.2fx%.2fx%.2f"
                        % (resp.box.id, resp.box.width, resp.box.depth, resp.box.height),
                    )
                    state = "DetectPickupBox"
                else:
                    self.publish_status("Idle", "spawn failed: %s" % resp.message)
                    state = "Idle"

            elif state == "DetectPickupBox":
                self.publish_status(state, "detecting pickup box via perception")
                if self.strict_perception:
                    view = self.go_to_pose(self.pickup_observe_pose)
                    if not view.success:
                        self.publish_status(
                            "Idle",
                            "DETECT_VIEW_FAILED: %s" % view.message)
                        state = "Idle"
                        continue
                    self.publish_status(
                        state, "pickup observation pose ready")
                rospy.sleep(self.detect_settle_sec)
                det_resp = None
                for attempt in range(self.detect_retries):
                    try:
                        candidate = self.detect()
                    except rospy.ServiceException as exc:
                        self.publish_status(
                            state,
                            "detect service failed attempt %d/%d: %s"
                            % (attempt + 1, self.detect_retries, exc),
                        )
                        candidate = None
                    if candidate and candidate.success and candidate.luggage:
                        det_resp = candidate
                        break
                    if candidate is not None:
                        self.publish_status(
                            state,
                            "detect rejected attempt %d/%d: %s"
                            % (attempt + 1, self.detect_retries,
                               candidate.message),
                        )
                    if attempt + 1 < self.detect_retries:
                        rospy.sleep(self.detect_settle_sec)

                if det_resp and det_resp.success and det_resp.luggage:
                    self.current_luggage = det_resp.luggage[0]
                    if self.strict_perception:
                        # Perception owns the size. Boxes are sampled
                        # continuously, so there is no manifest size to fall
                        # back on and no catalog entry to snap to; the only
                        # meaningful gate is that the measurement is physically
                        # plausible. Comparing against the spawned size would
                        # be grading perception with the answer key.
                        plausible, reason = self._check_size_plausible(
                            self.current_luggage)
                        if not plausible:
                            self.publish_status(
                                "Idle", "DETECT_IMPLAUSIBLE_SIZE %s" % reason)
                            state = "Idle"
                            continue
                        self._record_size_eval(self.current_luggage)
                    # Acceptance evidence: carries the detector's own message,
                    # which is how a run proves perception (not GT fallback)
                    # produced the pose.
                    self._events.detection(
                        True, "perception", det_resp.message)
                    self.publish_status(
                        state,
                        "perceived box at (%.3f, %.3f, %.3f) %s"
                        % (
                            self.current_luggage.pose.position.x,
                            self.current_luggage.pose.position.y,
                            self.current_luggage.pose.position.z,
                            det_resp.message,
                        ),
                        level="warn",
                    )
                else:
                    if self.strict_perception:
                        self._events.detection(
                            False, "perception",
                            "strict RGBD exhausted %d attempts"
                            % self.detect_retries)
                        self.publish_status(
                            "Idle",
                            "DETECT_FAILED strict RGBD exhausted %d attempts"
                            % self.detect_retries,
                        )
                        state = "Idle"
                        continue
                    rospy.logwarn(
                        "Perception failed — falling back to spawner GT")
                    self._events.detection(
                        True, "gt_fallback", "perception unavailable")
                    self.current_luggage = self._spawner_gt_box
                    self.publish_status(
                        state, "using spawner GT (perception unavailable)")

                try:
                    sync_pickup = (
                        self.sync_detected_pickup_box(self.current_luggage)
                        if self.strict_perception
                        else self.sync_pickup_box()
                    )
                    if sync_pickup.success:
                        self.publish_status(state, sync_pickup.message)
                    else:
                        self.publish_status("Idle", "pickup scene sync failed: %s" % sync_pickup.message)
                        state = "Idle"
                        continue
                except rospy.ServiceException as exc:
                    self.publish_status("Idle", "pickup scene sync failed: %s" % exc)
                    state = "Idle"
                    continue
                state = self._next_after_spawn()

            elif state == "ReturnObserveBeforeDetect":
                self.publish_status(state, "returning to observe before detect")
                self._refresh_explore_scene()
                resp = self.go_to_pose(self.observe_pose)
                if resp.success:
                    self.publish_status(state, resp.message)
                    state = "SpawnCurrentBox"
                else:
                    self.publish_status("Idle", "return observe failed: %s" % resp.message)
                    state = "Idle"

            elif state == "ExploreCargo":
                state = self._run_explore_cargo(state)

            elif state == "InspectContainer":
                self.publish_status(state, "inspecting container interior")
                resp = self.inspect_container(mode=self.inspect_mode)
                if resp.success:
                    self.publish_status(
                        state,
                        "%s (free_volume=%.2f occupancy=%.0f%%)"
                        % (resp.message, resp.free_volume, resp.occupancy_ratio * 100.0),
                    )
                    state = self._next_after_inspect()
                else:
                    self.publish_status("Idle", "container inspect failed: %s" % resp.message)
                    state = "Idle"

            elif state == "ComputePlacement":
                self.publish_status(state, "computing placement before pick")
                if self.placement_ns:
                    ok, message = self._compute_placement_pipeline()
                    if ok:
                        self.publish_status(state, message)
                        state = "PlanPick"
                    else:
                        self.publish_status("Idle", "placement failed: %s" % message)
                        state = "Idle"
                else:
                    resp = self.compute_placement(self.current_luggage, self.placed)
                    if resp.success:
                        self.current_slot = resp.slot
                        self.publish_status(state, resp.message)
                        state = "PlanPick"
                    else:
                        self.publish_status("Idle", "cargo full: %s" % resp.message)
                        state = "Idle"

            elif state == "Detect":
                self.publish_status(state, "detecting luggage")
                resp = self.detect()
                if not resp.success or not resp.luggage:
                    self.publish_status("Idle", "no luggage — done")
                    state = "Idle"
                elif len(self.placed) >= self.max_placed:
                    self.publish_status("Idle", "max placed reached — done")
                    state = "Idle"
                else:
                    self.current_luggage = resp.luggage[0]
                    state = "PlanPick"

            elif state == "PlanPick":
                self.publish_status(state, "planning pick sequence")
                self._clear_pick_dynamic_obstacles()
                try:
                    sync_pickup = self.sync_pickup_box()
                    if sync_pickup.success:
                        self.publish_status(state, sync_pickup.message)
                    else:
                        self.publish_status("Idle", "pickup scene sync failed: %s" % sync_pickup.message)
                        state = "Idle"
                        continue
                except rospy.ServiceException as exc:
                    self.publish_status("Idle", "pickup scene sync failed: %s" % exc)
                    state = "Idle"
                    continue
                resp = self.build_sequence(self.current_luggage, SlotSpec(), "pick")
                if resp.success and resp.segments:
                    self.pick_segments = resp.segments
                    self._segment_index = 0
                    self._phase = "pick"
                    state = "ExecPick"
                else:
                    self.publish_status("Idle", "pick sequence failed: %s" % resp.message)
                    state = "Idle"

            elif state == "ExecPick":
                if self._segment_index >= len(self.pick_segments):
                    if (
                        not self.dry_run_motion
                        and self.require_vacuum_attach
                        and not self._vacuum_attached()
                    ):
                        self.publish_status("Idle", "pick finished without vacuum attach")
                        state = "Idle"
                    else:
                        if (
                                self.strict_perception
                                and not self.dry_run_motion
                                and self.post_pick_stage_pose):
                            staged = self.go_to_pose(
                                self.post_pick_stage_pose)
                            if not staged.success:
                                self.publish_status(
                                    "Idle",
                                    "POST_PICK_STAGE_FAILED: %s"
                                    % staged.message,
                                )
                                state = "Idle"
                                continue
                            self.publish_status(
                                "ExecPick",
                                "payload staged at pose '%s'"
                                % self.post_pick_stage_pose,
                            )
                        state = "PlanPlace"
                else:
                    seg = self.pick_segments[self._segment_index]
                    state = self._execute_segment_or_advance(
                        self.pick_segments, "PlanPlace", state
                    )
                    if state == "Idle":
                        continue
                    if seg.name == "attach" and not self.dry_run_motion:
                        if not self._attempt_vacuum_attach():
                            state = "Idle"

            elif state == "PlanPlace":
                if (
                    not self.dry_run_motion
                    and self.require_vacuum_attach
                    and not self._vacuum_attached()
                ):
                    self.publish_status("Idle", "cannot plan place without vacuum attach")
                    state = "Idle"
                    continue
                self.publish_status(state, "planning place sequence")
                if self.placement_ns:
                    # Re-run reachability now that the suitcase is attached in
                    # MoveIt's planning scene. This makes collision-aware IK
                    # account for the payload and can select a different slot
                    # than the pre-pick estimate.
                    ok, message = self._compute_placement_pipeline()
                    if not ok:
                        self.publish_status(
                            "Idle", "attached-payload placement failed: %s"
                            % message)
                        state = "Idle"
                        continue
                if not self.active_loading:
                    resp = self.next_slot(self.placed)
                    if not resp.success:
                        self.publish_status("Idle", "slot selection failed: %s" % resp.message)
                        state = "Idle"
                        continue
                    self.current_slot = resp.slot
                if self.current_slot is None:
                    self.publish_status("Idle", "no precomputed placement slot")
                    state = "Idle"
                else:
                    resp = self.build_sequence(self.current_luggage, self.current_slot, "place")
                    if resp.success and resp.segments:
                        self.place_segments = resp.segments
                        self._segment_index = 0
                        self._phase = "place"
                        self._place_released = False
                        self._released_at_contact = False
                        self._retreat_after_release = False
                        state = "ExecPlace"
                    else:
                        self.publish_status("Idle", "place sequence failed: %s" % resp.message)
                        state = "Idle"

            elif state == "ExecPlace":
                if self._segment_index >= len(self.place_segments):
                    if self.dry_run_motion:
                        state = "UpdateOccupancy"
                        continue
                    if not self._place_released:
                        resp = self.vacuum(False)
                        if not resp.success:
                            self.publish_status(
                                "Idle", "RELEASE_FAILED: %s" % resp.message)
                            state = "Idle"
                            continue
                        self._place_released = True
                    # Records what the sequence actually did, not what it was
                    # supposed to do: a release that happened after retreat
                    # instead of at contact must show up as a gate failure.
                    self._events.release(
                        self._released_at_contact, self._retreat_after_release)
                    state = "UpdateOccupancy"
                else:
                    segment = self.place_segments[self._segment_index]
                    state = self._execute_segment_or_advance(
                        self.place_segments, "UpdateOccupancy", state
                    )
                    # Every segment before insertion is "we could not reach
                    # this candidate", so all of them may fall back to a
                    # backup candidate. insert/descend/retreat must not: by
                    # then the box is already going down and a retry would
                    # leave a partially committed placement.
                    if state == "Idle" and segment.name in (
                            "stage_mid", "stage_late", "stage",
                            "transit", "traverse"):
                        recovered, recovery_message = (
                            self._select_next_placement_backup(
                                "%s failure" % segment.name))
                        if recovered:
                            self.publish_status(
                                "PlanPlace", recovery_message)
                            state = "ExecPlace"
                            continue
                    if (
                            state != "Idle"
                            and segment.name == "descend"
                            and not self.dry_run_motion
                            and not self._place_released):
                        release = self.vacuum(False)
                        if not release.success:
                            self.publish_status(
                                "Idle",
                                "RELEASE_FAILED_AT_CONTACT: %s"
                                % release.message,
                            )
                            state = "Idle"
                            continue
                        self._place_released = True
                        self._released_at_contact = True
                        self.publish_status(
                            "ExecPlace",
                            "released payload at contact before retreat",
                        )
                        rospy.sleep(self.post_place_settle_sec)
                    elif (
                            state != "Idle"
                            and segment.name == "retreat"
                            and self._place_released):
                        self._retreat_after_release = True

            elif state == "UpdateOccupancy":
                self.publish_status(state, "updating scene and occupancy")
                actual_ok, actual_message = self._verify_actual_placement(
                    self.current_slot)
                if not actual_ok:
                    self.publish_status(
                        "Idle", "VERIFY_ACTUAL_FAILED: %s" % actual_message)
                    state = "Idle"
                    continue
                self.publish_status(
                    state, "actual pose: %s" % actual_message)
                if self.active_loading and not self.dry_run_motion:
                    finalized = self.finalize_current_box()
                    if not finalized.success:
                        self.publish_status(
                            "Idle",
                            "FINALIZE_MODEL_FAILED: %s"
                            % finalized.message,
                        )
                        state = "Idle"
                        continue
                    rospy.set_param(
                        "/luggage/last_finalized_model",
                        finalized.model_name)
                    self.publish_status(
                        state, finalized.message)
                if self.post_place_verify and self.go_to_pose is not None:
                    verify_pose = self.go_to_pose(
                        self.post_place_verify_pose)
                    if not verify_pose.success:
                        self.publish_status(
                            "Idle",
                            "VERIFY_OBSERVE_POSE_FAILED: %s"
                            % verify_pose.message,
                        )
                        state = "Idle"
                        continue
                    self.publish_status(
                        state,
                        "post-place observe: %s" % verify_pose.message)
                verified, verify_message = self._verify_post_place_surface(
                    self.current_slot)
                if not verified:
                    self.publish_status(
                        "Idle", "VERIFY_FAILED: %s" % verify_message)
                    state = "Idle"
                    continue
                self.publish_status(
                    state, "post-place verification: %s" % verify_message,
                    level="warn")
                # From here on the conservative extents are what gets recorded;
                # the motion already ran against the measured ones.
                commit_slot = self._commit_slot(self.current_slot)
                clear, overlap_message = self._check_commit_overlap(commit_slot)
                if not clear:
                    self.publish_status("Idle", overlap_message)
                    state = "Idle"
                    continue
                resp = self.add_placed(commit_slot)
                if resp.success:
                    remap_ok, remap_message = self._remap_placed_box(
                        commit_slot)
                    if not remap_ok:
                        rollback = self.remove_placed(commit_slot)
                        self.publish_status(
                            "Idle",
                            "COMMIT_MAP_FAILED: %s; rollback=%s"
                            % (remap_message, rollback.message),
                        )
                        state = "Idle"
                        continue
                    map_ok, map_message = self._verify_committed_map_top(
                        commit_slot)
                    if not map_ok:
                        map_rollback = self._rollback_placed_map(commit_slot)
                        scene_rollback = self.remove_placed(commit_slot)
                        self.publish_status(
                            "Idle",
                            "%s; map_rollback=%s; scene_rollback=%s"
                            % (
                                map_message,
                                map_rollback[1],
                                scene_rollback.message,
                            ),
                        )
                        state = "Idle"
                        continue
                    self.publish_status(state, map_message)
                    self._log_placement_commit(commit_slot)
                    self.placed.append(commit_slot)
                    self._events.status(len(self.placed), state=state)
                    self._cycle_started = None
                    self._publish_placed_param()
                    if self.use_post_place_inspect and self.inspect_container is not None:
                        post = self.inspect_container(mode=self.inspect_mode)
                        self.publish_status(state, "post-place inspect: %s" % post.message)
                    if self.active_loading:
                        state = "ReturnObserveBeforeDetect"
                    else:
                        state = "Detect"
                else:
                    self.publish_status("Idle", "scene update failed: %s" % resp.message)
                    state = "Idle"

        self.publish_status("Idle", "orchestrator finished")
        self._events.status(len(self.placed), state="Idle")
        self._events.close()

    def _write_session_record(self, first_state):
        """Configuration snapshot that makes two runs comparable."""
        self._events.session(
            first_state=first_state,
            run_mode=self._step_gate.mode,
            max_placed=int(self.max_placed),
            active_loading=bool(self.active_loading),
            strict_perception=bool(self.strict_perception),
            dry_run_motion=bool(self.dry_run_motion),
            exploration_mode=str(self.exploration_mode),
            inspect_mode=str(self.inspect_mode),
            placement_ns=str(self.placement_ns),
            filter_ns=str(self.filter_ns),
            post_place_verify=bool(self.post_place_verify),
            size_xy_margin=float(self._size_xy_margin),
            size_z_margin=float(self._size_z_margin),
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
    rospy.init_node("orchestrator", log_level=resolve_log_level())
    orch = Orchestrator()
    rospy.sleep(0.5)
    orch.run()
    rospy.loginfo("orchestrator skeleton cycle complete")


if __name__ == "__main__":
    main()
