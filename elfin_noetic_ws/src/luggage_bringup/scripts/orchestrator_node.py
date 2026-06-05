#!/usr/bin/env python3
"""Top-level loading orchestrator.

The legacy flow remains available by leaving ~active_loading false. Active
loading uses a fixed pickup source and computes the placement before picking.
"""

import rospy
from luggage_msgs.msg import LoadTaskStatus, SlotSpec
from luggage_msgs.srv import (
    AddPlacedBox,
    AimCameraAtContainer,
    BuildMotionSequence,
    ClearCurrentBox,
    ComputePlacement,
    DetectLuggage,
    GetCargoMapStats,
    GetCurrentBox,
    GetNextSlot,
    GoToJointValues,
    GoToRobotPose,
    InspectContainer,
    IntegrateCargoView,
    PlanMotion,
    PlanNextCargoView,
    ResetCargoMap,
    SpawnNextBox,
    SyncStaticScene,
    VacuumCommand,
)


class Orchestrator:
    STATES = (
        "Idle",
        "ResetObserve",
        "InitialExploreCargo",
        "ReturnObserveAfterInitialExplore",
        "SyncScene",
        "SpawnCurrentBox",
        "AimContainer",
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
        self.max_placed = rospy.get_param("~max_placed", 1)
        self.skip_reset = rospy.get_param("~skip_reset", False)
        self.skip_container_aim = rospy.get_param("~skip_container_aim", True)
        self.observe_pose = rospy.get_param("~observe_pose", "observe")
        self.active_loading = rospy.get_param("~active_loading", False)
        self.inspect_mode = rospy.get_param("~inspect_mode", "gazebo_gt")
        self.use_post_place_inspect = rospy.get_param("~use_post_place_inspect", False)
        self.dry_run_motion = rospy.get_param("~dry_run_motion", False)
        self.exploration_mode = rospy.get_param("~exploration_mode", "none")
        self.run_initial_explore = rospy.get_param("~run_initial_explore", False)
        self.initial_exploration_mode = rospy.get_param(
            "~initial_exploration_mode", "initial_fixed_scan"
        )
        self.settle_time_sec = float(rospy.get_param("~settle_time_sec", 0.5))
        self._explore_views_used = 0
        self._initial_explore_views_used = 0
        self._initial_explore_done = False
        self._current_joint_values = []

        self.status_pub = rospy.Publisher("~status", LoadTaskStatus, queue_size=1, latch=True)

        detector_ns = rospy.get_param("~detector_ns", "luggage_detector")
        packer_ns = rospy.get_param("~packer_ns", "bin_packer")
        scene_ns = rospy.get_param("~scene_ns", "scene_manager")
        wpt_ns = rospy.get_param("~waypoint_ns", "waypoint_generator")
        motion_ns = rospy.get_param("~motion_ns", "motion_planner")
        vacuum_ns = rospy.get_param("~vacuum_ns", "vacuum_simulator")
        inspector_ns = rospy.get_param("~inspector_ns", "container_inspector")
        mapper_ns = rospy.get_param("~mapper_ns", "cargo_volume_mapper")
        explore_ns = rospy.get_param("~explore_ns", "cargo_exploration_planner")
        spawner_ns = rospy.get_param("~spawner_ns", "pickup_box_spawner")

        rospy.wait_for_service("/%s/detect_luggage" % detector_ns)
        rospy.wait_for_service("/%s/get_next_slot" % packer_ns)
        rospy.wait_for_service("/%s/sync_static_scene" % scene_ns)
        rospy.wait_for_service("/%s/add_placed_box" % scene_ns)
        rospy.wait_for_service("/%s/build_motion_sequence" % wpt_ns)
        rospy.wait_for_service("/%s/plan_motion" % motion_ns)
        rospy.wait_for_service("/%s/vacuum_command" % vacuum_ns)
        if self.active_loading:
            rospy.wait_for_service("/%s/spawn_next_box" % spawner_ns)
            rospy.wait_for_service("/%s/get_current_box" % spawner_ns)
            rospy.wait_for_service("/%s/clear_current_box" % spawner_ns)
            rospy.wait_for_service("/%s/compute_placement" % packer_ns)
        if self.active_loading or not self.skip_container_aim:
            rospy.wait_for_service("/%s/inspect_container" % inspector_ns)
        if not self.skip_container_aim:
            rospy.wait_for_service("/%s/aim_camera_at_container" % motion_ns)
        if self._needs_explore_services():
            rospy.wait_for_service("/%s/reset_cargo_map" % mapper_ns)
            rospy.wait_for_service("/%s/integrate_cargo_view" % mapper_ns)
            rospy.wait_for_service("/%s/get_cargo_map_stats" % mapper_ns)
            rospy.wait_for_service("/%s/plan_next_cargo_view" % explore_ns)
            rospy.wait_for_service("/%s/go_to_joint_values" % motion_ns)
        if not self.skip_reset or self.run_initial_explore:
            rospy.wait_for_service("/%s/go_to_robot_pose" % motion_ns)

        self.detect = rospy.ServiceProxy("/%s/detect_luggage" % detector_ns, DetectLuggage)
        self.next_slot = rospy.ServiceProxy("/%s/get_next_slot" % packer_ns, GetNextSlot)
        self.compute_placement = None
        self.spawn_next_box = None
        self.get_current_box = None
        self.clear_current_box = None
        if self.active_loading:
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
        self.sync_scene = rospy.ServiceProxy("/%s/sync_static_scene" % scene_ns, SyncStaticScene)
        self.add_placed = rospy.ServiceProxy("/%s/add_placed_box" % scene_ns, AddPlacedBox)
        self.build_sequence = rospy.ServiceProxy("/%s/build_motion_sequence" % wpt_ns, BuildMotionSequence)
        self.plan_motion = rospy.ServiceProxy("/%s/plan_motion" % motion_ns, PlanMotion)
        self.vacuum = rospy.ServiceProxy("/%s/vacuum_command" % vacuum_ns, VacuumCommand)
        self.inspect_container = None
        self.aim_camera = None
        if self.active_loading or not self.skip_container_aim:
            self.inspect_container = rospy.ServiceProxy(
                "/%s/inspect_container" % inspector_ns, InspectContainer
            )
        if not self.skip_container_aim:
            self.aim_camera = rospy.ServiceProxy(
                "/%s/aim_camera_at_container" % motion_ns, AimCameraAtContainer
            )
        self.go_to_pose = None
        self.go_to_joints = None
        self.reset_cargo_map = None
        self.integrate_cargo_view = None
        self.get_cargo_map_stats = None
        self.plan_next_cargo_view = None
        if self._needs_explore_services():
            self.reset_cargo_map = rospy.ServiceProxy(
                "/%s/reset_cargo_map" % mapper_ns, ResetCargoMap
            )
            self.integrate_cargo_view = rospy.ServiceProxy(
                "/%s/integrate_cargo_view" % mapper_ns, IntegrateCargoView
            )
            self.get_cargo_map_stats = rospy.ServiceProxy(
                "/%s/get_cargo_map_stats" % mapper_ns, GetCargoMapStats
            )
            self.plan_next_cargo_view = rospy.ServiceProxy(
                "/%s/plan_next_cargo_view" % explore_ns, PlanNextCargoView
            )
            self.go_to_joints = rospy.ServiceProxy(
                "/%s/go_to_joint_values" % motion_ns, GoToJointValues
            )
        if not self.skip_reset or self.run_initial_explore:
            self.go_to_pose = rospy.ServiceProxy(
                "/%s/go_to_robot_pose" % motion_ns, GoToRobotPose
            )

    def publish_status(self, state, message):
        msg = LoadTaskStatus(state=state, message=message, placed_count=len(self.placed))
        self.status_pub.publish(msg)
        rospy.loginfo("[%s] %s (placed=%d)", state, message, len(self.placed))

    def _needs_explore_services(self):
        return self.run_initial_explore or self.exploration_mode in ("fixed_scan", "nbv")

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

    def _publish_placed_param(self):
        rospy.set_param(
            "/luggage/container_inspection/placed_boxes",
            [self._slot_to_dict(slot) for slot in self.placed],
        )

    def _next_after_scene_sync(self):
        if self.active_loading:
            return "SpawnCurrentBox"
        if self.skip_container_aim:
            return "Detect"
        return "AimContainer"

    def _next_after_reset(self):
        if self.run_initial_explore and not self._initial_explore_done:
            return "InitialExploreCargo"
        return "SyncScene"

    def _next_after_spawn(self):
        if self.skip_container_aim:
            if self.exploration_mode in ("fixed_scan", "nbv"):
                return "ExploreCargo"
            return "InspectContainer"
        return "AimContainer"

    def _next_after_aim(self):
        if self.exploration_mode in ("fixed_scan", "nbv"):
            return "ExploreCargo"
        return "InspectContainer"

    def _next_after_inspect(self):
        return "ComputePlacement" if self.active_loading else "Detect"

    def _run_explore_sequence(self, state, mode, views_used, done_state, reset_on_start=True):
        if views_used == 0 and reset_on_start:
            self.publish_status(state, "resetting cargo voxel map")
            reset = self.reset_cargo_map()
            if not reset.success:
                self.publish_status("Idle", "reset cargo map failed: %s" % reset.message)
                return "Idle"

        plan = self.plan_next_cargo_view(
            mode,
            self._current_joint_values,
            [],
            views_used,
        )
        if not plan.success and plan.done:
            self.publish_status(state, "explore finished: %s" % plan.message)
            return done_state

        if plan.done:
            self.publish_status(state, "explore finished: %s" % plan.message)
            return done_state

        joint_names = list(plan.joint_names) if plan.joint_names else []
        joint_values = list(plan.joint_values)
        if not joint_values:
            self.publish_status("Idle", "explore plan returned empty joints")
            return "Idle"

        if self.dry_run_motion:
            self.publish_status(
                state,
                "dry-run view %d: %s" % (plan.view_index, plan.message),
            )
        else:
            self.publish_status(state, "moving to explore view %d" % plan.view_index)
            move = self.go_to_joints(joint_values, joint_names)
            if not move.success:
                self.publish_status("Idle", "explore move failed: %s" % move.message)
                return "Idle"
            rospy.sleep(self.settle_time_sec)

        integrate = self.integrate_cargo_view(1)
        if not integrate.success:
            self.publish_status("Idle", "integrate view failed: %s" % integrate.message)
            return "Idle"

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

        if stats.success and stats.unknown_ratio <= float(
            rospy.get_param("~unknown_threshold", 0.15)
        ) and mode != "initial_fixed_scan":
            return "InspectContainer"
        return state, views_used

    def _run_initial_explore_cargo(self, state):
        result = self._run_explore_sequence(
            state,
            self.initial_exploration_mode,
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
            self._segment_index += 1
            return state
        self.publish_status("Idle", "%s motion failed: %s" % (self._phase, resp.message))
        return "Idle"

    def run(self):
        if self.skip_reset and self.run_initial_explore:
            state = "InitialExploreCargo"
        else:
            state = "SyncScene" if self.skip_reset else "ResetObserve"
        while not rospy.is_shutdown() and state != "Idle":
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
                resp = self.spawn_next_box()
                if resp.success:
                    self.current_luggage = resp.box
                    self.current_slot = None
                    self.publish_status(
                        state,
                        "%s size=%.2fx%.2fx%.2f"
                        % (resp.box.id, resp.box.width, resp.box.depth, resp.box.height),
                    )
                    state = self._next_after_spawn()
                else:
                    self.publish_status("Idle", "spawn failed: %s" % resp.message)
                    state = "Idle"

            elif state == "AimContainer":
                self.publish_status(state, "aiming camera at container opening")
                resp = self.aim_camera(
                    container_frame="container_opening_frame",
                    link6_xy_tolerance=0.03,
                    link6_z_tolerance=0.15,
                    execute=True,
                )
                if resp.success:
                    self.publish_status(state, resp.message)
                    state = self._next_after_aim()
                else:
                    self.publish_status("Idle", "container aim failed: %s" % resp.message)
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
                    if self.dry_run_motion:
                        state = "PlanPlace"
                        continue
                    resp = self.vacuum(True)
                    state = "PlanPlace" if resp.success else "Idle"
                else:
                    state = self._execute_segment_or_advance(
                        self.pick_segments, "PlanPlace", state
                    )

            elif state == "PlanPlace":
                self.publish_status(state, "planning place sequence")
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
                        state = "ExecPlace"
                    else:
                        self.publish_status("Idle", "place sequence failed: %s" % resp.message)
                        state = "Idle"

            elif state == "ExecPlace":
                if self._segment_index >= len(self.place_segments):
                    if self.dry_run_motion:
                        state = "UpdateOccupancy"
                        continue
                    resp = self.vacuum(False)
                    state = "UpdateOccupancy" if resp.success else "Idle"
                else:
                    state = self._execute_segment_or_advance(
                        self.place_segments, "UpdateOccupancy", state
                    )

            elif state == "UpdateOccupancy":
                self.publish_status(state, "updating scene and occupancy")
                resp = self.add_placed(self.current_slot)
                if resp.success:
                    self.placed.append(self.current_slot)
                    self._publish_placed_param()
                    if self.active_loading:
                        self.clear_current_box()
                    if self.use_post_place_inspect and self.inspect_container is not None:
                        post = self.inspect_container(mode=self.inspect_mode)
                        self.publish_status(state, "post-place inspect: %s" % post.message)
                    if self.active_loading:
                        state = "SpawnCurrentBox"
                    else:
                        state = "Detect"
                else:
                    self.publish_status("Idle", "scene update failed: %s" % resp.message)
                    state = "Idle"

        self.publish_status("Idle", "orchestrator finished")


def main():
    rospy.init_node("orchestrator")
    orch = Orchestrator()
    rospy.sleep(0.5)
    orch.run()
    rospy.loginfo("orchestrator skeleton cycle complete")


if __name__ == "__main__":
    main()
