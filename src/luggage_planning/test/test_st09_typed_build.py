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
    result = handle_build_v2(request, BuildMotionSequenceV2.Response(), pick=pick_from_detected(box))
    assert result.success, result.message
    assert [segment.name for segment in result.motion_segments] == ["pre_grasp", "approach", "attach", "pick_retreat"]
    for segment in result.motion_segments:
        assert segment.request_id == "request"
        assert segment.scene_revision == "scene"
        assert segment.stamp_ns == record["stamp_ns"]
        assert segment.target_pose.pose.position.x == 0.5
        assert segment.target_pose.pose.position.y == 0.1
        assert segment.target_pose.reference == "suction_contact_frame"
        assert segment.target_pose.pose.position.z >= 0.56


@pytest.mark.parametrize("yaw", [0, math.pi/2, math.pi, math.atan2(4, 3)])
@pytest.mark.parametrize("origin", [(0, 0, 0), (1.5, -0.4, 0.2)])
@pytest.mark.parametrize("floor", [0.53, 0.71])
@pytest.mark.parametrize("support", [0, 0.3])
@pytest.mark.parametrize("offset", [0, 0.1])
def test_generated_leg_cdr_preserves_pose_and_policy(yaw, origin, floor, support, offset):
    # Independent direct rigid transform oracle, including all eight corners.
    c, s = math.cos(yaw), math.sin(yaw)
    target = Pose(Point(origin[0]+c*0.2+s*0.1, origin[1]+s*0.2-c*0.1, origin[2]+floor+support),
                  Quaternion(math.cos(yaw/2), math.sin(yaw/2), 0.0, 0.0))
    waypoint = Pose(Point(target.position.x+offset, target.position.y, target.position.z+0.1), target.orientation)
    leg = MotionSegment(name="attach", type="cartesian", target_pose=target, waypoints=[waypoint],
                        keep_tool_down=False, keep_camera_down=True, lock_wrist=True,
                        allow_ompl_fallback=False, required_cartesian_fraction=1.0)
    typed = _build_motion_messages([leg], identity())[0]
    restored = deserialize_message(serialize_message(typed), MotionSegmentV2)
    assert restored == typed
    assert restored.stamp_ns == 9007199254740993
    assert restored.required_cartesian_fraction == 1.0
    assert restored.lock_wrist and restored.keep_camera_down
    assert not restored.allow_ompl_fallback
    assert restored.waypoints[0].pose.position.x == target.position.x+offset
    from luggage_description.frame_transforms import rotate_vector
    q = restored.target_pose.pose.orientation
    p = restored.target_pose.pose.position
    for x in [-0.4, 0.4]:
        for y in [-0.25, 0.25]:
            for z in [-0.16, 0.16]:
                rotated = rotate_vector([q.x,q.y,q.z,q.w], [x,y,z])
                assert [p.x+rotated[0],p.y+rotated[1],p.z+rotated[2]] == pytest.approx(
                    [origin[0]+c*(0.2+x)-s*(-0.1-y), origin[1]+s*(0.2+x)+c*(-0.1-y),
                     origin[2]+floor+support-z], abs=1e-6)


def test_place_build_typed_output():
    from test_frame_migration_selector import (
        _build_message, _request, _service_surface, _surface, PORTAL, TARGET,
    )
    surface = _service_surface(_surface("open"), "svc")
    request = _build_message(_request(surface, PORTAL, TARGET, (0, 0, 0.16)))
    response = handle_build_v2(request, BuildMotionSequenceV2.Response())
    assert response.success, response.message
    assert response.motion_segments
    assert len(response.motion_segments) == len(response.segments)
    for typed, legacy in zip(response.motion_segments, response.segments):
        assert typed.name == legacy.name
        assert typed.target_pose.pose == legacy.target_pose
        assert typed.request_id == "svc"
        assert typed.scene_revision == request.slot.identity.scene_revision
        assert typed.stamp_ns == request.slot.identity.stamp_ns


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
