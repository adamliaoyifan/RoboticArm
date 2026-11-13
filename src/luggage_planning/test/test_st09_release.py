"""Release rejection must occur before any detach effect."""
from copy import deepcopy
from types import SimpleNamespace

import pytest
from luggage_msgs.srv import VacuumCommandV2
from luggage_planning.attachment.attachment_frame_adapters import handle_vacuum_command_v2


def fixture():
    attachment = dict(box_id="box", generation=3, scene_revision="scene", tool_stamp_ns=9,
                      t_tool_box_translation=[0.0, 0.0, 0.2], t_tool_box_quaternion=[0.0, 0.0, 0.0, 1.0])
    tool = dict(contract_version=2, kind="pose", frame_id="world", length_unit="m",
                scene_revision="scene", request_id="release", stamp_ns=10, time_basis="acquisition",
                reference="suction_contact_frame", orientation_semantics="tool_quaternion",
                position=[1.0, 2.0, 3.0], quaternion_xyzw=[0.0, 0.0, 0.0, 1.0])
    request = VacuumCommandV2.Request()
    request.enable = False
    request.request_id = "release"
    request.scene_revision = "scene"
    request.box_id = "box"
    request.generation = 3
    return request, attachment, tool


@pytest.mark.parametrize("fault", ["tool", "attachment", "box", "generation", "scene", "request", "stamp", "quaternion", "relation", "attach_stamp"])
def test_release_refusal_preserves_attachment(fault):
    request, attachment, tool = fixture()
    if fault == "relation": attachment.pop("t_tool_box_translation")
    if fault == "attach_stamp": attachment.pop("tool_stamp_ns")
    if fault == "tool": tool = None
    if fault == "attachment": attachment = None
    if fault == "box": request.box_id = "wrong"
    if fault == "generation": request.generation = 4
    if fault == "scene": request.scene_revision = "wrong"
    if fault == "request": request.request_id = ""
    if fault == "stamp": tool["stamp_ns"] = 8
    if fault == "quaternion": tool["quaternion_xyzw"] = [0.0]*4
    effects = []
    backend = SimpleNamespace(last_attach_record=attachment, last_detach_record=None,
                              detach=lambda payload: (effects.append(payload) or True, "ok"))
    before = deepcopy(attachment)
    result = handle_vacuum_command_v2(request, VacuumCommandV2.Response(), scene_revision="scene",
                                     generation=3, resolve_tool=lambda _: tool, backend=backend)
    assert not result.success
    assert not result.diagnostic.accepted
    assert effects == []
    assert backend.last_attach_record == before
    assert backend.last_detach_record is None


def test_release_measured_transform_retains_stamp():
    request, attachment, tool = fixture()
    effects = []
    backend = SimpleNamespace(last_attach_record=attachment,
                              detach=lambda payload: (effects.append(payload) or True, "ok"))
    result = handle_vacuum_command_v2(request, VacuumCommandV2.Response(), scene_revision="scene",
                                     generation=3, resolve_tool=lambda _: tool, backend=backend)
    assert result.success
    assert len(effects) == 1
    assert backend.last_detach_record["stamp_ns"] == 10
    assert backend.last_detach_record["box_position"] == [1.0, 2.0, 3.2]


def _tool_pose(stamp_ns, position, request_id="attach"):
    return dict(
        contract_version=2, kind="pose", frame_id="world", length_unit="m",
        scene_revision="scene", request_id=request_id, stamp_ns=int(stamp_ns),
        time_basis="acquisition", reference="suction_contact_frame",
        orientation_semantics="tool_quaternion",
        position=list(position), quaternion_xyzw=[1.0, 0.0, 0.0, 0.0])


def test_attach_locks_live_tool_not_detection_stamp():
    """F18 freeze uses live TCP at attach, not pickup_observe TF."""
    from luggage_msgs.msg import DetectedLuggage
    from luggage_planning.attachment.attachment_frame_contracts import compose_box_world
    from luggage_planning.pick.pick_authorization import HEIGHT_SOURCE_MEASURED_SUPPORT
    from luggage_planning.place.place_service import pose_message

    detection_stamp = 1000
    attach_stamp = 2000
    box_xyz = [-1.0, 0.0, 0.9835]
    height = 0.247
    observe_xyz = [-0.986, -0.216, 1.90]
    attach_xyz = [-1.0, 0.0, box_xyz[2] + height * 0.5]
    place_tcp = [1.005, -0.535, 0.829]
    tool_quat = [1.0, 0.0, 0.0, 0.0]

    request = VacuumCommandV2.Request()
    request.enable = True
    request.request_id = "attach"
    request.scene_revision = "scene"
    request.box_id = "box"
    request.generation = 3
    request.box_pose = pose_message({
        "contract_version": 2, "frame_id": "world", "length_unit": "m",
        "stamp_ns": detection_stamp, "time_basis": "acquisition",
        "scene_revision": "scene", "request_id": "attach", "kind": "pose",
        "reference": "box_center", "orientation_semantics": "box_axis_pi",
        "position": box_xyz, "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
    })
    box = DetectedLuggage()
    box.id = "box"
    box.header.frame_id = "world"
    box.header.stamp.nanosec = detection_stamp
    box.pose.position.x, box.pose.position.y, box.pose.position.z = box_xyz
    box.pose.orientation.w = 1.0
    box.width = 0.516
    box.depth = 0.390
    box.height = height
    box.height_valid = True
    box.height_source = HEIGHT_SOURCE_MEASURED_SUPPORT
    request.measured_box = box

    observe_tool = _tool_pose(detection_stamp, observe_xyz)
    attach_tool = _tool_pose(attach_stamp, attach_xyz)
    stamps = []

    def resolve_tool(stamp_ns):
        stamps.append(stamp_ns)
        return attach_tool if stamp_ns is None else observe_tool

    captured = []
    backend = SimpleNamespace(
        last_attach_record=None,
        attach=lambda payload: (captured.append(payload) or True, "ok"))
    result = handle_vacuum_command_v2(
        request, VacuumCommandV2.Response(), scene_revision="scene",
        generation=3, resolve_tool=resolve_tool, backend=backend)
    assert result.success, result.message
    assert stamps == [None]
    record = captured[0]["attach_record"]
    box_world, _ = compose_box_world(record, place_tcp, tool_quat)
    err_xy = ((box_world[0] - place_tcp[0]) ** 2
              + (box_world[1] - place_tcp[1]) ** 2) ** 0.5
    assert err_xy < 0.02
    # The observe-stamp tool would have frozen a ~0.22 m lateral offset.
    observe_record = dict(record)
    from luggage_planning.attachment.vacuum_attach_utils import invert_transform
    from luggage_planning.pick.pick_frame_contracts import grasp_transform
    actual = grasp_transform(
        box_xyz, [0.0, 0.0, 0.0, 1.0], observe_xyz, tool_quat, "scene")
    tool_box = invert_transform(actual["translation"], actual["quaternion_xyzw"])
    observe_record["t_tool_box_translation"] = list(tool_box[0])
    observe_record["t_tool_box_quaternion"] = list(tool_box[1])
    wrong, _ = compose_box_world(observe_record, place_tcp, tool_quat)
    wrong_xy = ((wrong[0] - place_tcp[0]) ** 2
                + (wrong[1] - place_tcp[1]) ** 2) ** 0.5
    assert wrong_xy > 0.20
    assert err_xy < wrong_xy / 5.0
