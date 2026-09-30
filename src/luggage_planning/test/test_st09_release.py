"""Release rejection must occur before any detach effect."""
from copy import deepcopy
from types import SimpleNamespace

import pytest
from luggage_msgs.srv import VacuumCommandV2
from luggage_planning.attachment_frame_adapters import handle_vacuum_command_v2


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
