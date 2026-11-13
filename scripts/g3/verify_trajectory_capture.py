#!/usr/bin/env python3
"""Durable OFFLINE recorder self-check. No nodes, clients, or robot motion.

Usage (ROS messages and source package on PYTHONPATH):
  python3 scripts/g3/verify_trajectory_capture.py NEW_OUTPUT_DIRECTORY
The directory must not exist. All inputs are synthetic, labelled accordingly.
"""

import argparse
import json
from pathlib import Path
import statistics
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from luggage_msgs.msg import RouteLegV2
from luggage_msgs.srv import ValidateRouteV2
from moveit_msgs.action import ExecuteTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from unique_identifier_msgs.msg import UUID

from luggage_planning.capture.ros_boundary_capture import RosBoundaryRecorder, load_boundary_record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    recorder = RosBoundaryRecorder(args.output, run_id="offline-synthetic-capture-check")
    cases = []
    timings = []
    for index, accelerations in enumerate([[], [0.] * 6, [.1, -.2, .3, -.4, .5, -.6]]):
        request = FollowJointTrajectory.Impl.SendGoalService.Request()
        request.goal_id = UUID(uuid=[index] + list(range(1, 16)))
        trajectory = request.goal.trajectory
        trajectory.header.frame_id = "offline-input-header"
        trajectory.header.stamp.sec = 12345678
        trajectory.header.stamp.nanosec = 987654321
        trajectory.joint_names = ["elfin_joint%d" % j for j in [6, 1, 5, 2, 4, 3]]
        trajectory.points = [JointTrajectoryPoint(
            positions=[.12345678912345678] * 6, velocities=[.01] * 6,
            accelerations=accelerations, effort=[],
            time_from_start=Duration(sec=sec, nanosec=ns))
            for sec, ns in [(0, 1), (1, 999999999)]]
        certificate = ValidateRouteV2.Request()
        certificate.request_id = "offline-case-%d" % index
        certificate.legs = [RouteLegV2(edge_id="transit-edge", knots=trajectory)]
        execute = ExecuteTrajectory.Impl.SendGoalService.Request()
        execute.goal_id = UUID(uuid=[index + 16] + list(range(1, 16)))
        execute.goal.trajectory.joint_trajectory = trajectory
        status = GoalStatusArray(status_list=[GoalStatus(status=GoalStatus.STATUS_SUCCEEDED)])
        status.status_list[0].goal_info.goal_id = request.goal_id
        status.status_list[0].goal_info.stamp.sec = 42
        status.status_list[0].goal_info.stamp.nanosec = 123456789
        feedback = JointState(name=list(trajectory.joint_names), position=[.125] * 6,
                              velocity=[0.] * 6, effort=[])
        feedback.header.stamp.sec = 44
        feedback.header.stamp.nanosec = 234567891
        saved = []
        for label, message in [("certification.request", certificate),
                               ("executor.send_goal.request", execute),
                               ("fjt.send_goal.request", request),
                               ("fjt.status", status), ("joint_states", feedback)]:
            before = time.monotonic_ns()
            path = recorder.record(label, message, correlation=dict(
                synthetic=True, request_id=certificate.request_id,
                leg="transit", case=index))
            timings.append(time.monotonic_ns() - before)
            decoded = load_boundary_record(path, type(message))
            assert decoded == message
            saved.append(str(path.relative_to(args.output)))
        cases.append(dict(case=index, accelerations=accelerations,
                          exact_message_roundtrips=True, records=saved))
    recorder.close()
    result = dict(input_kind="synthetic_offline_messages", simulations_started=0,
                  ros_nodes_created=0, records=15, cases=cases,
                  recorder_roundtrip_verified=True, live_boundary_wiring_verified=False,
                  run_capture_complete=False, dispatch_ready=False,
                  synchronous_write_ns=dict(min=min(timings), median=statistics.median(timings),
                                            max=max(timings)),
                  overhead_scope="local filesystem only, not live callback overhead")
    (args.output / "selfcheck.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
