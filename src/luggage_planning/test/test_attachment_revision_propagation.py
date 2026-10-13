"""Attach revision propagation. The equality check is not relaxed.

Run8 failed VACUUM_ATTACH / CALIBRATION_REVISION_MISMATCH because the
detection record carried the live deploy scene while the vacuum handler
stamped the tool from a different file hash (the example scene, which is
also the revision stored on the older atlas). These tests keep the
refusal and require the diagnostic to name both revisions.
"""
import json

import pytest
from luggage_msgs.msg import DetectedLuggage
from luggage_msgs.srv import VacuumCommandV2
from luggage_description.frame_contracts import ContractError
from luggage_planning.attachment_frame_adapters import (
    attachment_revision_evidence,
    bound_attachment_scene_revision,
    handle_vacuum_command_v2,
    revision_evidence_from_message,
    vacuum_attach_revision_dump,
)
from luggage_planning.attachment_frame_contracts import _freeze
from luggage_planning.pick_authorization import HEIGHT_SOURCE_MEASURED_SUPPORT
from luggage_planning.place_service import pose_message
from types import SimpleNamespace


LIVE = "35f15c4ceb0f2a938600da9b2c52df7a"
STALE = "fb0dfdc1565a208edb5b9ea61c257754"


def _detection(scene_revision, request_id="attach"):
    return {
        "contract_version": 2, "kind": "pose", "frame_id": "world",
        "length_unit": "m", "stamp_ns": 1000, "time_basis": "acquisition",
        "scene_revision": scene_revision, "request_id": request_id,
        "reference": "box_center", "orientation_semantics": "box_axis_pi",
        "position": [-1.0, 0.0, 0.9835],
        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
    }


def _tool(scene_revision, request_id="attach", position=None):
    return {
        "contract_version": 2, "kind": "pose", "frame_id": "world",
        "length_unit": "m", "stamp_ns": 2000, "time_basis": "acquisition",
        "scene_revision": scene_revision, "request_id": request_id,
        "reference": "suction_contact_frame",
        "orientation_semantics": "tool_quaternion",
        "position": list(position or [-1.0, 0.0, 1.107]),
        "quaternion_xyzw": [1.0, 0.0, 0.0, 0.0],
    }


def _request(scene_revision, box_revision=None):
    box_revision = scene_revision if box_revision is None else box_revision
    request = VacuumCommandV2.Request()
    request.enable = True
    request.request_id = "attach"
    request.scene_revision = scene_revision
    request.box_id = "pickup_box_0001_carryon"
    request.generation = 2
    request.box_pose = pose_message(_detection(box_revision))
    box = DetectedLuggage()
    box.id = "pickup_box_0001_carryon"
    box.header.frame_id = "world"
    box.header.stamp.sec = 0
    box.header.stamp.nanosec = 1000
    box.pose.position.x = -1.0
    box.pose.position.z = 0.9835
    box.pose.orientation.w = 1.0
    box.width = 0.516
    box.depth = 0.390
    box.height = 0.247
    box.height_valid = True
    box.height_source = HEIGHT_SOURCE_MEASURED_SUPPORT
    request.measured_box = box
    return request


def _evidence(message):
    evidence = revision_evidence_from_message(message)
    assert evidence is not None, message
    return evidence


def test_freeze_rejects_different_revisions_and_names_both():
    with pytest.raises(ContractError) as caught:
        _freeze(_tool(STALE), _detection(LIVE), {}, None, 2, 2, "box")
    assert caught.value.code == "CALIBRATION_REVISION_MISMATCH"
    assert caught.value.expected == STALE
    assert caught.value.received == LIVE


def test_freeze_same_revision_reaches_later_checks():
    try:
        _freeze(_tool(LIVE), _detection(LIVE), {}, None, 2, 2, "box")
    except ContractError as exc:
        assert exc.code != "CALIBRATION_REVISION_MISMATCH"
    else:
        raise AssertionError("empty measured payload must not pass _freeze")


def test_matching_live_revision_attaches():
    captured = []
    backend = SimpleNamespace(
        attach=lambda payload: (captured.append(payload) or True, "ok"))
    result = handle_vacuum_command_v2(
        _request(LIVE), VacuumCommandV2.Response(), scene_revision=LIVE,
        generation=2, resolve_tool=lambda _stamp: _tool(LIVE), backend=backend)
    assert result.success, result.message
    assert captured
    assert captured[0]["attach_record"]["scene_revision"] == LIVE


def test_run8_stale_tool_bind_is_refused_with_both_revisions():
    """Detection is the live scene; the handler bind and tool are stale."""
    backend = SimpleNamespace(attach=lambda _payload: (_ for _ in ()).throw(
        AssertionError("backend attach must not run")))
    result = handle_vacuum_command_v2(
        _request(LIVE), VacuumCommandV2.Response(), scene_revision=STALE,
        generation=2, resolve_tool=lambda _stamp: _tool(STALE), backend=backend)
    assert not result.success
    assert result.message.startswith("CALIBRATION_REVISION_MISMATCH")
    evidence = _evidence(result.message)
    assert evidence["request_scene_revision"] == LIVE
    assert evidence["box_pose_scene_revision"] == LIVE
    assert evidence["tool_pose_scene_revision"] == STALE
    assert evidence["bound_scene_revision"] == STALE
    assert evidence["box_id"] == "pickup_box_0001_carryon"
    assert evidence["generation"] == 2
    assert evidence["measured_stamp_ns"] == 1000
    diagnostic = result.diagnostic
    compared = diagnostic.expected + diagnostic.received
    assert LIVE in compared
    assert STALE in compared


def test_box_and_tool_revisions_must_match_under_the_live_bind():
    backend = SimpleNamespace(attach=lambda _payload: (_ for _ in ()).throw(
        AssertionError("backend attach must not run")))
    result = handle_vacuum_command_v2(
        _request(LIVE), VacuumCommandV2.Response(), scene_revision=LIVE,
        generation=2, resolve_tool=lambda _stamp: _tool(STALE), backend=backend)
    assert not result.success
    assert result.message.startswith("CALIBRATION_REVISION_MISMATCH")
    evidence = _evidence(result.message)
    assert evidence["box_pose_scene_revision"] == LIVE
    assert evidence["tool_pose_scene_revision"] == STALE
    assert evidence["bound_scene_revision"] == LIVE
    compared = result.diagnostic.expected + result.diagnostic.received
    assert LIVE in compared and STALE in compared


def test_live_identity_is_not_replaced_by_atlas_scene_revision():
    assert bound_attachment_scene_revision(
        {"scene_revision": LIVE}, STALE) == LIVE
    assert bound_attachment_scene_revision(
        {"scene_revision": "  "}, STALE) == ""
    assert bound_attachment_scene_revision(None, STALE) == STALE


def test_driver_dump_names_adapter_bind_and_live_scene():
    evidence = attachment_revision_evidence(
        _request(LIVE), _detection(LIVE), _tool(STALE), STALE)
    message = "CALIBRATION_REVISION_MISMATCH " + json.dumps(
        evidence, sort_keys=True, separators=(",", ":"))
    dump = vacuum_attach_revision_dump(
        message, live_scene_revision=LIVE, fallback={})
    assert dump["request_scene_revision"] == LIVE
    assert dump["box_pose_scene_revision"] == LIVE
    assert dump["tool_pose_scene_revision"] == STALE
    assert dump["adapter_scene_revision"] == STALE
    assert dump["live_scene_revision"] == LIVE
    assert dump["measured_stamp_ns"] == 1000
    assert dump["box_id"] == "pickup_box_0001_carryon"
    assert dump["generation"] == 2
