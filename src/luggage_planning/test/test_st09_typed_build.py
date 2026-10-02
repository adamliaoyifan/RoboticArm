"""Typed build output is executable without legacy fallback or field loss."""
import math
import pytest
from rclpy.serialization import serialize_message, deserialize_message
from luggage_msgs.srv import BuildMotionSequenceV2
from luggage_msgs.msg import MotionSegmentV2
from luggage_planning.place_service import handle_build_v2, _build_motion_messages, pose_message
from luggage_planning.pose import MotionSegment, Pose, Point, Quaternion
from luggage_planning.ros_message_adapters import pick_from_detected


def identity():
    return dict(contract_version=2, kind="pose", frame_id="world", length_unit="m",
                stamp_ns=9007199254740993, time_basis="acquisition", scene_revision="scene",
                request_id="request", reference="box_center", orientation_semantics="box_axis_pi",
                position=[0.5, 0.1, 0.4], quaternion_xyzw=[0.0, 0.0, 0.0, 1.0])


def test_pick_build_typed_output():
    request = BuildMotionSequenceV2.Request()
    request.phase = "pick"
    request.request_id = "request"
    request.scene_revision = "scene"
    record = identity()
    request.pick_pose = pose_message(record)
    box = request.pick
    box.id = "box"
    box.header.frame_id = "world"
    box.header.stamp.sec, box.header.stamp.nanosec = divmod(record["stamp_ns"], 10**9)
    box.pose = request.pick_pose.pose
    box.width, box.depth, box.height = 0.8, 0.5, 0.32
    box.height_valid = True
    from luggage_planning.pick_authorization import HEIGHT_SOURCE_MEASURED_SUPPORT
    box.height_source = HEIGHT_SOURCE_MEASURED_SUPPORT
    from luggage_planning.route_producer import _leg_record
    from st09_contracts.fixtures import make_timed_trajectory

    def encoder(pick, *, request_id, scene_revision, stamp_ns):
        del pick
        trajectory = make_timed_trajectory()
        return [
            _leg_record(name, trajectory, request_id, scene_revision,
                        stamp_ns)
            for name in ("pre_grasp", "approach", "attach",
                         "pick_retreat")]

    result = handle_build_v2(
        request, BuildMotionSequenceV2.Response(),
        pick=pick_from_detected(box), pick_encoder=encoder)
    assert result.success, result.message
    assert [segment.name for segment in result.motion_segments] == [
        "pre_grasp", "approach", "attach", "pick_retreat"]
    for segment in result.motion_segments:
        assert segment.request_id == "request"
        assert segment.scene_revision == "scene"
        assert segment.stamp_ns == record["stamp_ns"]
        assert segment.segment_type == "joint_path"
        assert len(segment.joint_trajectory.trajectory.points) >= 2


def test_place_build_refuses_named():
    from luggage_description.frame_contracts import ContractError

    request = BuildMotionSequenceV2.Request()
    request.phase = "place"
    request.request_id = "request"
    response = handle_build_v2(request, BuildMotionSequenceV2.Response())
    assert not response.success
    assert response.reason_code == "PLANNING_ROUTE_REQUIRED"


def test_apply_selected_path_to_v2_rewrites_pose_target():
    from luggage_planning.typed_driver_adapter import (
        apply_selected_path_to_v2, pose_segment_message)

    built = pose_segment_message(
        "transit", [2.505, -0.535, 0.827], [1.0, 0.0, 0.0, 0.0],
        "place-1", "scene-a", segment_type="pose_target",
        allow_ompl_fallback=False)
    apply_selected_path_to_v2(
        built, [[0.86, -0.38, 1.5], [1.005, -0.535, 0.827]])
    assert built.segment_type == "cartesian"
    assert built.allow_ompl_fallback
    assert built.request_id == "place-1"
    assert built.target_pose.pose.position.x == pytest.approx(1.005)
    assert built.target_pose.pose.position.y == pytest.approx(-0.535)
    assert len(built.waypoints) == 1
    assert built.waypoints[0].pose.position.x == pytest.approx(0.86)
    assert built.waypoints[0].identity.request_id == "place-1"


def test_adopt_v2_identity_keeps_selected_path():
    from luggage_planning.typed_driver_adapter import (
        adopt_v2_identity, apply_selected_path_to_v2, pose_segment_message)

    built = pose_segment_message(
        "transit", [2.505, -0.535, 0.827], [1.0, 0.0, 0.0, 0.0],
        "place-1", "scene-a", segment_type="pose_target",
        allow_ompl_fallback=False)
    built.stamp_ns = 42
    built.stamp_valid = True
    built.target_pose.identity.stamp_ns = 42
    selected = pose_segment_message(
        "transit", [0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0],
        "select-id", "other-scene", segment_type="cartesian")
    apply_selected_path_to_v2(
        selected, [[0.86, -0.38, 1.5], [1.005, -0.535, 0.827]])
    adopted = adopt_v2_identity(selected, built)
    assert adopted.request_id == "place-1"
    assert adopted.scene_revision == "scene-a"
    assert adopted.stamp_ns == 42
    assert adopted.target_pose.identity.request_id == "place-1"
    assert adopted.target_pose.pose.position.x == pytest.approx(1.005)
    assert adopted.waypoints[0].identity.request_id == "place-1"
    assert adopted.waypoints[0].pose.position.x == pytest.approx(0.86)


def test_handle_plan_v2_forwards_live_joints():
    from types import SimpleNamespace
    from luggage_msgs.action import PlanMotionV2
    from luggage_planning.motion_v2_service import handle_plan_v2
    from luggage_planning.typed_driver_adapter import pose_segment_message

    captured = {}

    class _Executor(object):
        def execute_segment_v2(self, record, context, current_joints=None,
                               feedback_cb=None, execute_timeout=45.0):
            del record, context, feedback_cb, execute_timeout
            captured["joints"] = list(current_joints or [])
            return SimpleNamespace(
                success=True, message="ok", fraction=1.0,
                used_ompl_fallback=False, moveit_error_code=1)

    segment = pose_segment_message(
        "transit", [1.0, -0.5, 0.8], [1.0, 0.0, 0.0, 0.0],
        "place-1", "scene-a", segment_type="cartesian")
    response = PlanMotionV2.Result()
    handle_plan_v2(
        _Executor(), segment, "scene-a", response, None,
        current_joints=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    assert response.success
    assert captured["joints"] == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
