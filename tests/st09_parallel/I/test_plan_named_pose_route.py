#!/usr/bin/env python3
"""[ST09-I S5.7-6] GoToRobotPoseV2 producer gates (plan_named_pose_route).

The caller names an explicit calibrated pose; the producer validates the
SIM site-pose file against the live identity (no silent observe_pose
resolution, no current/here execution), builds ONE MoveIt edge from the
live joints to the FILE values (FK fills the goal pose — no IK), and
returns the COMPLETE certified joint_path under the explicit UNLOADED
admission. These tests pin the named-refusal gates and the pass path.
"""
import os
import sys
from types import SimpleNamespace

_ROOT = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
for entry in ("src/luggage_planning", "src/luggage_perception"):
    path = os.path.join(_ROOT, entry)
    if path not in sys.path:
        sys.path.append(path)

from luggage_planning.routing.producer import RouteProducer  # noqa: E402
from luggage_planning.planning_contract.attachment import UNLOADED_ATTACHMENT_DIGEST  # noqa: E402

JOINTS = ["elfin_joint%d" % i for i in range(1, 7)]
POSE_VALUES = [0.2, -0.2, 0.3, -0.4, 0.5, -0.6]
CURRENT = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]


class _Traj(object):
    joint_names = tuple(JOINTS)

    def __init__(self):
        self.positions = (tuple(CURRENT), tuple(POSE_VALUES))
        self.velocities = ((0.0,) * 6, (0.01,) * 6)
        self.accelerations = ((0.0,) * 6, (0.01,) * 6)
        self.time_from_start_ns = (0, 200_000_000)

    def digest(self):
        return "named-leg-digest"


class _Edge(object):
    edge_id = "edge-named-1"
    attributes = {"edge_class": "PORTAL_TRANSIT"}

    def __init__(self):
        self.trajectory = _Traj()

    def digest(self):
        return "edge-named-digest"


class _Graph(object):
    def register_node(self, q, position, quaternion, name, load, mode):
        return SimpleNamespace(node_id="%s-%s" % (name, mode), q=q)


class _Producer(object):
    """Only what plan_named_pose_route touches."""

    def __init__(self, *, snapshot, site=None, edge=None,
                 revalidation=None, certification=None):
        self.graph = _Graph()
        self.fk = lambda q: ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        self._edge = edge
        self._stub_revalidation = revalidation
        self._stub_certification = certification
        self.latest_snapshot_fn = lambda: snapshot
        self.site_poses_fn = (lambda: site) if site is not None else None
        self.last_selection = None
        # Recorded budget: _stage_timings reports it, never a default.
        self.budget_ns = 60_000_000_000

    def _revalidate(self, route, snapshot, edges, allow_unconfirmed=False):
        self.revalidate_allow_unconfirmed = allow_unconfirmed
        return self._stub_revalidation

    def _certify(self, route, snapshot, attachment, edges, candidate,
                 request, load_state=None):
        self._certify_load_state = load_state
        self._certify_attachment = attachment
        return self._stub_certification

    @property
    def edge_builder(self):
        producer = self

        class _Builder(object):
            def __call__(self, start, goal, klass):
                if producer._edge is None:
                    raise AssertionError("edge unexpectedly built")
                return producer._edge

            def reset_scene(self):
                pass

        return _Builder()

    _fail = RouteProducer._fail
    _promote_selection = RouteProducer._promote_selection
    _stage_timings = RouteProducer._stage_timings

    def _clock_ns(self):
        return 0

    plan_named_pose_route = RouteProducer.plan_named_pose_route


def _site_data(scene="scene-A", valid=True, joint_names=None):
    def _pose(values):
        return {
            "valid": bool(valid),
            "calibration_status": "synthetic",
            "values": list(values),
            "acquisition_stamp_ns": 1,
            "scene_revision_at_capture": scene,
            "provenance": {"source": "synthetic_fixture"},
        }

    return {
        "schema": "luggage.site_poses.sim/1",
        "identity": {
            "scene_revision": scene,
            "joint_names": joint_names or list(JOINTS),
            "urdf_sha256": "",
            "srdf_sha256": "",
        },
        "poses": {
            "pickup_observe_sim": _pose(POSE_VALUES),
            "portal_observe_sim": _pose(
                [0.7, -1.8, -1.0, -1.8, 1.5, -2.4]),
        },
        "defaults": {"observe_pose": "pickup_observe_sim"},
    }, "/fake/robot_poses.yaml"


def _snapshot(revision=5, scene="scene-A"):
    return SimpleNamespace(
        digest=lambda: "snap-digest", scene_revision=scene,
        map_revision=revision)


def _ok_certification():
    return SimpleNamespace(passed=True, digest=lambda: "cert-named-1"), None


def _ok_revalidation(revision=5):
    from luggage_planning.planning_contract.certification import MapDecision

    return SimpleNamespace(
        decision=MapDecision.CONTINUE, reason=None, map_revision=revision)


def _call(producer, *, pose="pickup_observe_sim", latest=5,
          scene="scene-A", joints=CURRENT):
    return producer.plan_named_pose_route(
        request_id="goto-req-1", pose_name=pose,
        current_joints=list(joints), scene_revision=scene,
        latest_map_revision=latest)


def _pass_producer(**overrides):
    kwargs = dict(
        snapshot=_snapshot(), site=_site_data(), edge=_Edge(),
        revalidation=_ok_revalidation(), certification=_ok_certification())
    kwargs.update(overrides)
    return _Producer(**kwargs)


def test_missing_pose_file_refuses():
    producer = _Producer(snapshot=_snapshot(), site=None)
    assert _call(producer)["reason_code"] == "POSE_FILE_MISSING"


def test_current_pose_name_is_not_executable():
    producer = _pass_producer()
    result = _call(producer, pose="current")
    assert result["reason_code"] == "CURRENT_POSE_NOT_EXECUTABLE"
    result = _call(producer, pose="here")
    assert result["reason_code"] == "CURRENT_POSE_NOT_EXECUTABLE"


def test_scene_revision_mismatch_is_scene_stale():
    producer = _pass_producer()
    result = _call(producer, scene="scene-B")
    assert result["reason_code"] == "SCENE_STALE"


def test_wrong_joint_order_refuses():
    producer = _Producer(
        snapshot=_snapshot(),
        site=_site_data(joint_names=list(reversed(JOINTS))),
        edge=_Edge(), revalidation=_ok_revalidation(),
        certification=_ok_certification())
    assert _call(producer)["reason_code"] == "JOINT_ORDER_MISMATCH"


def test_invalid_pose_file_refuses():
    producer = _Producer(
        snapshot=_snapshot(), site=_site_data(valid=False), edge=_Edge(),
        revalidation=_ok_revalidation(), certification=_ok_certification())
    result = _call(producer)
    assert result["reason_code"] == "POSE_NOT_VALID"
    # the validator's issues ride the row detail
    row = result["rows"][0]
    assert row.get("issues")
    # a pose name the file does not carry is the same named refusal
    producer = _pass_producer()
    assert _call(producer, pose="pickup_observe")[
        "reason_code"] == "POSE_NOT_VALID"


def test_already_there_returns_no_legs_and_promotes_nothing():
    producer = _pass_producer()
    result = _call(producer, joints=POSE_VALUES)
    assert result["success"] and result["reason_code"] == "OK"
    assert result["already_there"] is True
    assert result["legs"] == [] and result["path_hash"] == ""
    assert producer.last_selection is None
    # no certification was run for a no-op
    assert not hasattr(producer, "_certify_load_state")
    # wrap-equivalent joints are the SAME physical pose
    import math

    wrapped = [v + 2.0 * math.pi for v in POSE_VALUES]
    result = _call(producer, joints=wrapped)
    assert result["already_there"] is True


def test_pass_path_returns_certified_unloaded_legs():
    from luggage_planning.planning_contract.graph import LoadState

    producer = _pass_producer()
    result = _call(producer)
    assert result["success"] and result["reason_code"] == "OK"
    assert result["already_there"] is False
    assert producer._certify_load_state is LoadState.UNLOADED
    assert producer._certify_attachment is None
    # An unloaded observation move is admitted while a change is pending.
    assert producer.revalidate_allow_unconfirmed is True
    route = result["route"]
    assert route.attachment_digest == UNLOADED_ATTACHMENT_DIGEST
    assert len(result["legs"]) == 1
    assert result["legs"][0]["name"] == "transit"
    selection = producer.last_selection
    assert selection["path_hash"] == route.digest()
    assert selection["attachment"] is None
    assert selection["load_state"] is LoadState.UNLOADED
    assert selection["purpose"] == "named_pose:pickup_observe_sim"


def test_map_semantics_fail_closed():
    producer = _Producer(
        snapshot=None, site=_site_data(), edge=_Edge(),
        revalidation=_ok_revalidation(), certification=_ok_certification())
    assert _call(producer)["reason_code"] == "MAP_REVALIDATION_UNAVAILABLE"
    producer = _Producer(
        snapshot=_snapshot(scene="scene-B"), site=_site_data(), edge=_Edge(),
        revalidation=_ok_revalidation(), certification=_ok_certification())
    assert _call(producer)["reason_code"] == "IDENTITY_CHANGED"
    producer = _Producer(
        snapshot=_snapshot(revision=6), site=_site_data(), edge=_Edge(),
        revalidation=_ok_revalidation(), certification=_ok_certification())
    assert _call(producer, latest=5)["reason_code"] == "MAP_RESTART"
    producer = _Producer(
        snapshot=_snapshot(revision=4), site=_site_data(), edge=_Edge(),
        revalidation=_ok_revalidation(), certification=_ok_certification())
    assert _call(producer, latest=5)["reason_code"] == "MAP_STALE"


def test_edge_and_cert_failures_surface_their_reasons():
    producer = _Producer(
        snapshot=_snapshot(), site=_site_data(), edge=None,
        revalidation=_ok_revalidation(), certification=_ok_certification())
    result = _call(producer)
    assert result["reason_code"] == "EDGE_BUILD_FAILED"
    producer = _pass_producer(certification=(
        SimpleNamespace(
            passed=False,
            reason=SimpleNamespace(value="CERT_COLLISION")), None))
    assert _call(producer)["reason_code"] == "CERT_COLLISION"


def test_start_state_length_is_checked():
    producer = _pass_producer()
    assert _call(producer, joints=CURRENT[:4])[
        "reason_code"] == "START_STATE_MISMATCH"


if __name__ == "__main__":
    import unittest

    unittest.main()


def test_named_route_exports_the_same_frozen_snapshot_as_its_certificate():
    producer = _pass_producer()
    result = _call(producer)
    assert result["success"]
    frozen = result["request"]["frozen_map"]
    assert frozen == {"map_revision": 5, "snapshot_digest": "snap-digest", "scene_revision": "scene-A"}
    assert producer.last_selection["snapshot"].digest() == frozen["snapshot_digest"]
    assert producer.last_selection["request"] is result["request"]
