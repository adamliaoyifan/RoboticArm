#!/usr/bin/env python3
"""[ST09-I S5.7-6 §2] EnsureCycleStartV2 producer adjudication.

Pure authoritative-state verdict, no route planning: the selector's
latched joints + offline world FK + scene config + latched vacuum
classify the suction TCP into the container zone taxonomy. The anchors
mirror the containment tests' (deploy scene, world TCP): the
pickup-observe pose is OUTSIDE, the opening mouth is CORRIDOR, the
interior is INSIDE — with the corridor depth derived planning-side
(wall + camera envelope off the URDF transform + margin).
"""
import os
import sys
from types import SimpleNamespace

_ROOT = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
for entry in ("src/luggage_planning", "src/luggage_perception",
              "src/luggage_description"):
    path = os.path.join(_ROOT, entry)
    if path not in sys.path:
        sys.path.append(path)

import luggage_planning.route_producer as rp  # noqa: E402
from luggage_planning.route_producer import RouteProducer  # noqa: E402

SCENE_PATH = os.path.join(
    _ROOT, "src", "luggage_description", "config", "scene_tf_deploy.yaml")
SCENE_REVISION = "scene-binding-md5"

OUTSIDE_TCP = (0.945, 0.214, 1.748)
CORRIDOR_TCP = (-0.70, 0.6, 1.30)
INSIDE_TCP = (-1.20, 0.6, 1.30)


class _Adjudicator(object):
    """Only what ensure_cycle_start_state touches."""

    def __init__(self, world_tcp=OUTSIDE_TCP, camera=(0.3, 0.0, 0.0)):
        from luggage_description.scene_tf_config_utils import (
            load_scene_tf_config,
        )

        self.scene_config = load_scene_tf_config(SCENE_PATH)
        self.scene_revision_binding = SCENE_REVISION
        self.urdf = object()
        rp._CAMERA_TCP_CACHE[id(self.urdf)] = (
            tuple(camera), (0.0, 0.0, 0.0, 1.0))
        self._world_tcp = tuple(world_tcp)
        self.fk = lambda joints: (
            tuple(self._world_tcp), (0.0, 0.0, 0.0, 1.0))

    ensure_cycle_start_state = RouteProducer.ensure_cycle_start_state


def _vacuum(attached):
    return (None if attached is None else {
        "attached": bool(attached), "vacuum_on": bool(attached),
        "received_ns": 0})


JOINTS = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]


def test_unloaded_outside_is_already_ready():
    verdict = _Adjudicator().ensure_cycle_start_state(
        joints=JOINTS, request_scene_revision=SCENE_REVISION,
        vacuum_state=_vacuum(False))
    ready, reason, zone, attachment, detail = verdict
    assert ready and reason == "OK"
    assert zone == "outside_container"
    assert attachment == "unloaded"
    assert detail["corridor_depth_m"] == 0.44  # 0.04 wall + 0.30 + 0.10
    assert detail["envelope_source"] == "urdf"


def test_unloaded_corridor_and_inside_require_recovery():
    for tcp in (CORRIDOR_TCP, INSIDE_TCP):
        verdict = _Adjudicator(world_tcp=tcp).ensure_cycle_start_state(
            joints=JOINTS, request_scene_revision=SCENE_REVISION,
            vacuum_state=_vacuum(False))
        ready, reason, zone, attachment, _detail = verdict
        assert not ready and reason == "EXIT_ROUTE_UNAVAILABLE"
        assert zone == ("opening_corridor" if tcp is CORRIDOR_TCP
                        else "inside_container")
        assert attachment == "unloaded"


def test_carrying_any_zone_is_carry_exit_unsupported():
    for tcp in (OUTSIDE_TCP, CORRIDOR_TCP, INSIDE_TCP):
        verdict = _Adjudicator(world_tcp=tcp).ensure_cycle_start_state(
            joints=JOINTS, request_scene_revision=SCENE_REVISION,
            vacuum_state=_vacuum(True))
        ready, reason, _zone, attachment, _detail = verdict
        assert not ready and reason == "CARRY_EXIT_UNSUPPORTED"
        assert attachment == "carrying"


def test_unknown_attachment_requires_recovery():
    verdict = _Adjudicator().ensure_cycle_start_state(
        joints=JOINTS, request_scene_revision=SCENE_REVISION,
        vacuum_state=None)
    ready, reason, _zone, attachment, _detail = verdict
    assert not ready and reason == "ATTACHMENT_STATE_UNKNOWN"
    assert attachment == "unknown"


def test_identity_mismatch_is_identity_unknown():
    verdict = _Adjudicator().ensure_cycle_start_state(
        joints=JOINTS, request_scene_revision="other-scene",
        vacuum_state=_vacuum(False))
    ready, reason, zone, attachment, detail = verdict
    assert not ready and reason == "IDENTITY_UNKNOWN"
    assert zone == "unknown"
    # attachment is still honestly reported
    assert attachment == "unloaded"
    assert detail["request"] == "other-scene"


def test_missing_joints_is_named_refusal():
    verdict = _Adjudicator().ensure_cycle_start_state(
        joints=None, request_scene_revision=SCENE_REVISION,
        vacuum_state=_vacuum(False))
    assert verdict[0] is False and verdict[1] == "JOINT_STATE_UNAVAILABLE"
    verdict = _Adjudicator().ensure_cycle_start_state(
        joints=JOINTS[:4], request_scene_revision=SCENE_REVISION,
        vacuum_state=_vacuum(False))
    assert verdict[1] == "JOINT_STATE_UNAVAILABLE"


def test_missing_camera_transform_is_named_refusal():
    adjudicator = _Adjudicator()
    adjudicator.urdf = None
    verdict = adjudicator.ensure_cycle_start_state(
        joints=JOINTS, request_scene_revision=SCENE_REVISION,
        vacuum_state=_vacuum(False))
    ready, reason, zone, _attachment, detail = verdict
    assert not ready and reason == "CAMERA_FRAME_UNAVAILABLE"
    assert zone == "unknown"
    assert detail["suction_base"]


def test_missing_scene_config_is_named_refusal():
    adjudicator = _Adjudicator()
    adjudicator.scene_config = None
    verdict = adjudicator.ensure_cycle_start_state(
        joints=JOINTS, request_scene_revision=SCENE_REVISION,
        vacuum_state=_vacuum(False))
    assert verdict[0] is False
    assert verdict[1] == "SCENE_CONFIG_UNAVAILABLE"


if __name__ == "__main__":
    import unittest

    unittest.main()
