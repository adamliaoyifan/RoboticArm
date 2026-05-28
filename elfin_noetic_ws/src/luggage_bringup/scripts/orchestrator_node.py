#!/usr/bin/env python3
"""Phase 0 orchestrator — state machine only, calls module services."""

import rospy
from luggage_msgs.msg import LoadTaskStatus, SlotSpec
from luggage_msgs.srv import (
    AddPlacedBox,
    AimCameraAtContainer,
    BuildMotionSequence,
    DetectLuggage,
    GetNextSlot,
    GoToRobotPose,
    InspectContainer,
    PlanMotion,
    SyncStaticScene,
    VacuumCommand,
)


class Orchestrator:
    STATES = (
        "Idle",
        "ResetObserve",
        "SyncScene",
        "AimContainer",
        "InspectContainer",
        "Detect",
        "PlanPick",
        "ExecPick",
        "PlanPlace",
        "ExecPlace",
        "UpdateScene",
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

        self.status_pub = rospy.Publisher("~status", LoadTaskStatus, queue_size=1, latch=True)

        detector_ns = rospy.get_param("~detector_ns", "luggage_detector")
        packer_ns = rospy.get_param("~packer_ns", "bin_packer")
        scene_ns = rospy.get_param("~scene_ns", "scene_manager")
        wpt_ns = rospy.get_param("~waypoint_ns", "waypoint_generator")
        motion_ns = rospy.get_param("~motion_ns", "motion_planner")
        vacuum_ns = rospy.get_param("~vacuum_ns", "vacuum_simulator")
        inspector_ns = rospy.get_param("~inspector_ns", "container_inspector")

        rospy.wait_for_service("/%s/detect_luggage" % detector_ns)
        rospy.wait_for_service("/%s/get_next_slot" % packer_ns)
        rospy.wait_for_service("/%s/sync_static_scene" % scene_ns)
        rospy.wait_for_service("/%s/add_placed_box" % scene_ns)
        rospy.wait_for_service("/%s/build_motion_sequence" % wpt_ns)
        rospy.wait_for_service("/%s/plan_motion" % motion_ns)
        rospy.wait_for_service("/%s/vacuum_command" % vacuum_ns)
        if not self.skip_container_aim:
            rospy.wait_for_service("/%s/inspect_container" % inspector_ns)
            rospy.wait_for_service("/%s/aim_camera_at_container" % motion_ns)
        if not self.skip_reset:
            rospy.wait_for_service("/%s/go_to_robot_pose" % motion_ns)

        self.detect = rospy.ServiceProxy("/%s/detect_luggage" % detector_ns, DetectLuggage)
        self.next_slot = rospy.ServiceProxy("/%s/get_next_slot" % packer_ns, GetNextSlot)
        self.sync_scene = rospy.ServiceProxy("/%s/sync_static_scene" % scene_ns, SyncStaticScene)
        self.add_placed = rospy.ServiceProxy("/%s/add_placed_box" % scene_ns, AddPlacedBox)
        self.build_sequence = rospy.ServiceProxy("/%s/build_motion_sequence" % wpt_ns, BuildMotionSequence)
        self.plan_motion = rospy.ServiceProxy("/%s/plan_motion" % motion_ns, PlanMotion)
        self.vacuum = rospy.ServiceProxy("/%s/vacuum_command" % vacuum_ns, VacuumCommand)
        self.inspect_container = None
        self.aim_camera = None
        if not self.skip_container_aim:
            self.inspect_container = rospy.ServiceProxy(
                "/%s/inspect_container" % inspector_ns, InspectContainer
            )
            self.aim_camera = rospy.ServiceProxy(
                "/%s/aim_camera_at_container" % motion_ns, AimCameraAtContainer
            )
        self.go_to_pose = None
        if not self.skip_reset:
            self.go_to_pose = rospy.ServiceProxy(
                "/%s/go_to_robot_pose" % motion_ns, GoToRobotPose
            )

    def publish_status(self, state, message):
        msg = LoadTaskStatus(state=state, message=message, placed_count=len(self.placed))
        self.status_pub.publish(msg)
        rospy.loginfo("[%s] %s (placed=%d)", state, message, len(self.placed))

    def run(self):
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
                    state = "SyncScene"
                else:
                    self.publish_status("Idle", "observe reset failed: %s" % resp.message)
                    state = "Idle"

            elif state == "SyncScene":
                self.publish_status(state, "syncing static scene")
                resp = self.sync_scene()
                if not resp.success:
                    self.publish_status("Idle", "scene sync failed: %s" % resp.message)
                    state = "Idle"
                elif self.skip_container_aim:
                    state = "Detect"
                else:
                    state = "AimContainer"

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
                    state = "InspectContainer"
                else:
                    self.publish_status("Idle", "container aim failed: %s" % resp.message)
                    state = "Idle"

            elif state == "InspectContainer":
                self.publish_status(state, "inspecting container interior")
                resp = self.inspect_container(mode="gazebo_gt")
                if resp.success:
                    self.publish_status(
                        state,
                        "%s (free_volume=%.2f occupancy=%.0f%%)"
                        % (resp.message, resp.free_volume, resp.occupancy_ratio * 100.0),
                    )
                    state = "Detect"
                else:
                    self.publish_status("Idle", "container inspect failed: %s" % resp.message)
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
                    state = "Idle"

            elif state == "ExecPick":
                if self._segment_index >= len(self.pick_segments):
                    resp = self.vacuum(True)
                    state = "PlanPlace" if resp.success else "Idle"
                else:
                    seg = self.pick_segments[self._segment_index]
                    self.publish_status(state, "executing pick segment: %s" % seg.name)
                    resp = self.plan_motion(seg)
                    if resp.success:
                        self._segment_index += 1
                    else:
                        state = "Idle"

            elif state == "PlanPlace":
                self.publish_status(state, "planning place sequence")
                resp = self.next_slot(self.placed)
                if not resp.success:
                    state = "Idle"
                else:
                    self.current_slot = resp.slot
                    resp = self.build_sequence(self.current_luggage, self.current_slot, "place")
                    if resp.success and resp.segments:
                        self.place_segments = resp.segments
                        self._segment_index = 0
                        self._phase = "place"
                        state = "ExecPlace"
                    else:
                        state = "Idle"

            elif state == "ExecPlace":
                if self._segment_index >= len(self.place_segments):
                    resp = self.vacuum(False)
                    state = "UpdateScene" if resp.success else "Idle"
                else:
                    seg = self.place_segments[self._segment_index]
                    self.publish_status(state, "executing place segment: %s" % seg.name)
                    resp = self.plan_motion(seg)
                    if resp.success:
                        self._segment_index += 1
                    else:
                        state = "Idle"

            elif state == "UpdateScene":
                self.publish_status(state, "updating scene")
                resp = self.add_placed(self.current_slot)
                if resp.success:
                    self.placed.append(self.current_slot)
                state = "Detect"

        self.publish_status("Idle", "orchestrator finished")


def main():
    rospy.init_node("orchestrator")
    orch = Orchestrator()
    rospy.sleep(0.5)
    orch.run()
    rospy.loginfo("orchestrator skeleton cycle complete")


if __name__ == "__main__":
    main()
