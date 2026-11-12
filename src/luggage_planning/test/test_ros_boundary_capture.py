"""Offline persistence tests using actual ROS message serialization, no ROS graph."""

from concurrent.futures import ThreadPoolExecutor
import json
import math

import pytest

pytest.importorskip("control_msgs")
from action_msgs.msg import GoalStatus, GoalStatusArray
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from moveit_msgs.action import ExecuteTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from unique_identifier_msgs.msg import UUID

from luggage_planning.motion_frame_adapters import timed_trajectory
from luggage_planning.ros_boundary_capture import RosBoundaryRecorder, load_boundary_record


def action_request(accelerations):
    request = FollowJointTrajectory.Impl.SendGoalService.Request()
    request.goal_id = UUID(uuid=list(range(16)))
    trajectory = request.goal.trajectory
    trajectory.header.frame_id = "preserve-input-frame"
    trajectory.header.stamp.sec = 12345678
    trajectory.header.stamp.nanosec = 987654321
    trajectory.joint_names = ["elfin_joint%d" % i for i in [6, 1, 5, 2, 4, 3]]
    for seconds, nanos in [(0, 1), (1, 999999999)]:
        point = JointTrajectoryPoint(
            positions=[-0.0, .12345678912345678, 2., -3., 4., 1e-17],
            velocities=[.01 * i for i in range(6)],
            accelerations=list(accelerations), effort=[],
            time_from_start=Duration(sec=seconds, nanosec=nanos))
        trajectory.points.append(point)
    request.goal.goal_time_tolerance = Duration(sec=5, nanosec=7)
    return request


@pytest.mark.parametrize("accelerations", [[], [0.] * 6, [.1, -.2, .3, -.4, .5, -.6]])
def test_full_action_roundtrip_preserves_empty_vs_populated_arrays(tmp_path, accelerations):
    request = action_request(accelerations)
    recorder = RosBoundaryRecorder(tmp_path / "capture", run_id="offline-array-cases")
    folder = recorder.record("fjt.send_goal.request", request,
                             correlation={"leg": "transit", "request_id": "req-123"})
    recovered = load_boundary_record(folder, type(request))
    assert recovered == request
    # Do not normalize joint order, nanoseconds, negative zero or absent effort.
    assert math.copysign(1., recovered.goal.trajectory.points[0].positions[0]) == -1.
    fields = json.loads((folder / "fields.json").read_text())
    assert fields["goal"]["trajectory"]["points"][0]["accelerations"] == accelerations
    assert fields["goal"]["trajectory"]["points"][0]["effort"] == []
    assert fields["goal"]["trajectory"]["header"]["stamp"]["nanosec"] == 987654321
    assert fields["goal"]["trajectory"]["points"][1]["time_from_start"]["nanosec"] == 999999999
    entry = json.loads((folder / "index.json").read_text())
    assert entry["goal_id"] == bytes(range(16)).hex()
    assert entry["correlation"] == {"leg": "transit", "request_id": "req-123"}
    # Prove recording is a snapshot, independent of later caller mutation.
    request.goal.trajectory.points[0].positions[0] = 99.
    request.goal.trajectory.points[0].accelerations = [42.] * 6
    again = load_boundary_record(folder, type(request))
    assert again.goal.trajectory.points[0].positions[0] == 0.
    assert list(again.goal.trajectory.points[0].accelerations) == accelerations
    recorder.close()
    health = json.loads((folder.parent / "health.json").read_text())
    assert health["recorder_io_complete"]
    assert not health["run_capture_complete"]


def test_existing_typed_submit_conversion_preserves_explicit_empty_acceleration():
    from types import SimpleNamespace
    admitted = SimpleNamespace(joint_names=["j"], joint_points=[dict(
        positions=[.1], velocities=[.2], accelerations=[], effort=[],
        time_from_start_ns=1234567891)])
    names, points = timed_trajectory(admitted)
    assert names == ["j"]
    assert list(points[0].accelerations) == []
    assert points[0].time_from_start.sec == 1
    assert points[0].time_from_start.nanosec == 234567891


@pytest.mark.parametrize("accelerations", [[], [.1] * 6])
def test_typed_admission_and_executor_submit_preserve_acceleration_semantics(accelerations):
    from types import SimpleNamespace
    import threading
    from luggage_planning.motion_executor import MotionExecutor
    from luggage_planning.motion_frame_adapters import record_from_message
    from luggage_planning.motion_frame_contracts import ExecutionContext, TARGET_LINK
    from luggage_planning.place_service import _route_boundary, _route_motion_messages
    from luggage_planning.routing.producer import _leg_record

    trajectory = action_request(accelerations).goal.trajectory
    source = SimpleNamespace(joint_names=trajectory.joint_names,
        positions=[list(p.positions) for p in trajectory.points],
        velocities=[list(p.velocities) for p in trajectory.points],
        accelerations=[list(p.accelerations) for p in trajectory.points],
        time_from_start_ns=[p.time_from_start.sec*10**9+p.time_from_start.nanosec
                            for p in trajectory.points])
    record = _leg_record("transit", source, "offline-req", "offline-scene", 0)
    message = _route_motion_messages([record], dict(request_id="offline-req",
                                                  scene_revision="offline-scene"))[0]
    context = ExecutionContext(_route_boundary("offline-req", "offline-scene", "robot_state"),
                               TARGET_LINK, "world")
    executor = object.__new__(MotionExecutor)
    executor._frame, executor._link = "world", TARGET_LINK
    executor._v2_lock = threading.Lock()
    submitted = []
    executor._timed_submit = lambda goal: submitted.append(goal) or True
    executor.execute_segment_v2(record_from_message(message), context)
    points = submitted[0].joint_trajectory.points
    assert [list(p.accelerations) for p in points] == [accelerations, accelerations]
    assert [(p.time_from_start.sec,p.time_from_start.nanosec) for p in points] == [(0,1),(1,999999999)]


def test_certification_knots_use_same_lossless_recorder(tmp_path):
    from luggage_msgs.msg import RouteLegV2
    from luggage_msgs.srv import ValidateRouteV2
    request = ValidateRouteV2.Request()
    request.request_id = "offline-certificate"
    request.route_digest = "offline-route"
    request.legs = [RouteLegV2(edge_id="transit-edge", knots=action_request([]).goal.trajectory)]
    recorder = RosBoundaryRecorder(tmp_path / "capture", run_id="certification")
    folder = recorder.record("certification.request", request)
    decoded = load_boundary_record(folder, type(request))
    assert decoded == request
    assert list(decoded.legs[0].knots.points[0].accelerations) == []
    recorder.close()


def test_same_writer_handles_executor_status_and_source_stamped_feedback(tmp_path):
    recorder = RosBoundaryRecorder(tmp_path / "capture", run_id="offline-boundaries")
    request = action_request([])
    upstream = ExecuteTrajectory.Impl.SendGoalService.Request()
    upstream.goal_id = UUID(uuid=list(reversed(range(16))))
    upstream.goal.trajectory.joint_trajectory = request.goal.trajectory
    status = GoalStatusArray()
    status.status_list = [GoalStatus(status=GoalStatus.STATUS_SUCCEEDED)]
    status.status_list[0].goal_info.goal_id = request.goal_id
    status.status_list[0].goal_info.stamp.nanosec = 314159265
    state = JointState(name=["elfin_joint1"], position=[.3], velocity=[.01], effort=[float("nan")])
    state.header.stamp.sec = 284
    state.header.stamp.nanosec = 230000001
    controller = JointTrajectoryControllerState()
    controller.header = state.header
    controller.joint_names = state.name
    controller.reference.positions = [.2]
    controller.feedback.positions = [.3]
    for label, message in [("executor.send_goal.request", upstream),
                           ("fjt.status", status), ("joint_states", state),
                           ("controller_state", controller)]:
        folder = recorder.record(label, message)
        decoded = load_boundary_record(folder, type(message))
        if label == "joint_states":
            assert decoded.header.stamp == state.header.stamp
            assert list(decoded.position) == [.3] and math.isnan(decoded.effort[0])
        else:
            assert decoded == message
    recorder.close()


def test_rejects_conflicting_goal_id_and_keeps_error_sticky(tmp_path):
    recorder = RosBoundaryRecorder(tmp_path / "capture", run_id="mismatch")
    request = action_request([])
    with pytest.raises(ValueError, match="goal ID mismatch"):
        recorder.record("fjt.send_goal.request", request, goal_id=list(reversed(range(16))))
    recorder.record("fjt.send_goal.request", request)
    recorder.close()
    health = json.loads((tmp_path / "capture/health.json").read_text())
    assert health["errors"] == 1 and health["records"] == 1
    assert not health["recorder_io_complete"]


def test_budget_exhaustion_never_commits_partial_or_truncated_message(tmp_path):
    recorder = RosBoundaryRecorder(tmp_path / "capture", run_id="budget", max_bytes=1)
    with pytest.raises(RuntimeError, match="BUDGET_EXHAUSTED"):
        recorder.record("fjt.send_goal.request", action_request([]))
    recorder.close()
    assert not list((tmp_path / "capture").glob("[0-9]*"))
    assert not list((tmp_path / "capture").glob(".pending-*"))
    assert not json.loads((tmp_path / "capture/health.json").read_text())["recorder_io_complete"]
    with pytest.raises(FileExistsError):
        RosBoundaryRecorder(tmp_path / "capture", run_id="do-not-overwrite")


def test_corrupt_payload_rejected_before_deserialization(tmp_path):
    recorder = RosBoundaryRecorder(tmp_path / "capture", run_id="corruption")
    message = action_request([])
    folder = recorder.record("fjt.send_goal.request", message)
    (folder / "message.cdr").write_bytes(b"damaged")
    with pytest.raises(ValueError, match="hash mismatch"):
        load_boundary_record(folder, type(message))


def test_concurrent_boundaries_get_unique_committed_records(tmp_path):
    recorder = RosBoundaryRecorder(tmp_path / "capture", run_id="threads")
    with ThreadPoolExecutor(max_workers=4) as pool:
        folders = list(pool.map(lambda i: recorder.record(
            "fjt.send_goal.request", action_request([]), correlation={"sample": i}), range(12)))
    assert len(set(folders)) == 12
    assert {json.loads((f / "index.json").read_text())["correlation"]["sample"] for f in folders} == set(range(12))
    for folder in folders:
        assert load_boundary_record(folder, FollowJointTrajectory.Impl.SendGoalService.Request) == action_request([])
    recorder.close()
