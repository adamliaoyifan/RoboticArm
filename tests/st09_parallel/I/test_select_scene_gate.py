#!/usr/bin/env python3
"""[ST09-I G0-3] scene_revision is an INDEPENDENT gate at select.

PlanningSnapshot.scene_revision deliberately stays OUT of the map
digest (no digest-format migration this round), so the same map —
identical geometry_hash, map_revision, and grid — offered under a
different scene_revision must still refuse at every scene-carrying
boundary:

- select: request_dict binds slots, grasp, and surface to slots[0]'s
  declared scene; any record stamped with another scene is
  CALIBRATION_REVISION_MISMATCH (this file).
- verify: same digest + map, scene flipped -> IDENTITY_CHANGED
  (test_verify_selected.py::test_scene_identity_change_refuses).
- admission: same revision, scene flipped -> GENERATION_CHANGED
  (test_verify_selected.py::
  test_execution_admission_superseded_certificate_and_identity_refuse).

All three refusals are service-boundary verdicts: plan_place_route
never runs, nothing reaches the controller.
"""
import os
import sys

import pytest

_ROOT = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
for entry in ("src/luggage_planning", "src/luggage_perception"):
    path = os.path.join(_ROOT, entry)
    if path not in sys.path:
        sys.path.append(path)

from luggage_planning.place.place_service import (  # noqa: E402
    boundary_context, pose_message, request_dict,
)
from luggage_description.frame_contracts import ContractError  # noqa: E402
from luggage_description.ros_frame_bindings.surfaces import (  # noqa: E402
    surface_to_message,
)

_SCENE = "scene-A"
_OTHER_SCENE = "scene-B"
_GEOMETRY = "hull-hash-1"
_MAP_REVISION = 5


def _identity_fields(scene):
    return dict(
        contract_version=2, frame_id="container_link", length_unit="m",
        stamp_ns=9007199254740993, time_basis="scene",
        scene_revision=scene, request_id="req-select-1")


def _slot_record(scene, z=0.69):
    record = _identity_fields(scene)
    record.update(
        reference="box_center", orientation_semantics="box_axis_pi",
        position=[0.2, -0.1, z], quaternion_xyzw=[0.0, 0.0, 0.0, 1.0])
    return record


def _tcp_record(scene):
    record = _identity_fields(scene)
    record.update(
        reference="suction_contact_frame",
        orientation_semantics="tool_quaternion",
        position=[-0.1, 0.0, 0.9], quaternion_xyzw=[0.0, 0.0, 0.0, 1.0])
    return record


def _grasp_message(scene):
    from luggage_msgs.msg import PoseContractV2

    record = _identity_fields(scene)
    record.update(
        reference="box", orientation_semantics="suction_contact_frame",
        position=[0.0, 0.0, -0.16], quaternion_xyzw=[0.0, 0.0, 0.0, 1.0])
    message = PoseContractV2()
    message.identity.contract_version = 2
    message.identity.frame_id = str(record["frame_id"])
    message.identity.length_unit = "m"
    message.identity.stamp_valid = True
    message.identity.stamp_ns = int(record["stamp_ns"])
    message.identity.time_basis = "scene"
    message.identity.scene_revision = str(scene)
    message.identity.request_id = str(record["request_id"])
    message.reference = str(record["reference"])
    message.orientation_semantics = str(record["orientation_semantics"])
    message.pose.position.x = float(record["position"][0])
    message.pose.position.y = float(record["position"][1])
    message.pose.position.z = float(record["position"][2])
    message.pose.orientation.x = 0.0
    message.pose.orientation.y = 0.0
    message.pose.orientation.z = 0.0
    message.pose.orientation.w = 1.0
    return message


def _surface_record(scene, floor=0.53):
    return dict(
        contract_version=2, kind="surface", frame_id="container_link",
        length_unit="m", stamp_ns=9007199254740993,
        time_basis="scene", scene_revision=scene,
        request_id="req-map-1", geometry_hash=_GEOMETRY,
        map_revision=_MAP_REVISION, height_datum="container_origin",
        floor_z_container=floor, ceiling_z_container=floor + 2.0,
        resolution_m=0.1, origin_xy_container=[-1.0, -0.5],
        surface_z_container=[[floor, floor + 0.32, None]],
        state=[["free", "occupied", "unknown"]],
        confidence=[["sensor", "sensor", "unknown"]])


def _select_message(slot_scenes, surface_scene):
    """A complete select request: same map/geometry/slots throughout —
    ONLY the scene stamps differ between the named records."""
    from luggage_msgs.srv import SelectPlacePathV2

    message = SelectPlacePathV2.Request()
    message.request_id = "req-select-1"
    for scene in slot_scenes:
        message.slots.append(pose_message(_slot_record(scene)))
    message.slot_scores = [1.0] * len(slot_scenes)
    context = boundary_context(
        "req-map-1", surface_scene, "container_link", "",
        geometry_hash=_GEOMETRY, map_revision=_MAP_REVISION)
    message.surface = surface_to_message(
        _surface_record(surface_scene), context)
    message.box_wdh = [0.55, 0.35, 0.32]
    message.inner_size = [1.49, 0.78, 0.86]
    message.geometry_hash = _GEOMETRY
    message.map_revision = _MAP_REVISION
    message.start_tcp = pose_message(_tcp_record(slot_scenes[0]))
    message.start_joints = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
    message.grasp = _grasp_message(slot_scenes[0])
    message.container_source_frame = "container_link"
    message.container_target_frame = "world"
    message.container_translation = [0.0, 0.0, 0.0]
    message.container_quaternion_xyzw = [0.0, 0.0, 0.0, 1.0]
    message.container_stamp_ns = 0
    message.container_stamp_is_null = True
    message.excluded_ids = []
    from luggage_planning.planning_contract.attachment_geometry import freeze_geometry
    from luggage_planning.planning_contract.payload_envelope import EnvelopeLimits
    from luggage_planning.planning_contract.attachment import invert_transform
    from luggage_planning.attachment.attachment_geometry_transport import geometry_to_message
    p, q = message.grasp.pose.position, message.grasp.pose.orientation
    message.conservative_geometry = geometry_to_message(freeze_geometry(
        message.box_wdh, *invert_transform((p.x,p.y,p.z), (q.x,q.y,q.z,q.w)),
        EnvelopeLimits(5, (.1,)*3, (0,0), .005)))
    return message


def test_same_map_different_scene_surface_refuses_select():
    """[G0-3] the surface carries the SAME map (geometry_hash +
    map_revision + grid) but a different scene stamp: select refuses
    CALIBRATION_REVISION_MISMATCH before plan_place_route runs."""
    message = _select_message([_SCENE], _OTHER_SCENE)
    with pytest.raises(ContractError) as caught:
        request_dict(message)
    assert caught.value.code == "CALIBRATION_REVISION_MISMATCH"
    assert caught.value.field == "scene_revision"


def test_same_map_different_scene_slot_refuses_select():
    """[G0-3] one slot of the same request stamped with the other
    scene: the whole select refuses on that record."""
    message = _select_message([_SCENE, _OTHER_SCENE], _SCENE)
    with pytest.raises(ContractError) as caught:
        request_dict(message)
    assert caught.value.code == "CALIBRATION_REVISION_MISMATCH"
    assert caught.value.field == "scene_revision"


def test_matching_scene_admits_the_request_dict():
    """Positive control: identical records under one scene pass the
    boundary and the request dict carries the declared scene (the
    field the select-side snapshot binds to)."""
    message = _select_message([_SCENE, _SCENE], _SCENE)
    request = request_dict(message)
    assert request["scene_revision"] == _SCENE
    assert request["geometry_hash"] == _GEOMETRY
    assert request["map_revision"] == _MAP_REVISION
    assert len(request["slots"]) == 2
