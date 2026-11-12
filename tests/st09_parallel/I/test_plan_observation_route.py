#!/usr/bin/env python3
"""[ST09-I S5.7-4 §4] PlanObservationRouteV2 producer gates.

The driver + ObservationPolicy declare WHERE (world camera poses, <= 2
per request); the producer owns the camera->TCP conversion, the MoveIt
edge, and the FULL certification under the explicit LoadState.UNLOADED
admission. These tests pin the named-refusal gates and the pass path's
identity fields (unloaded binding, selection bookkeeping for
verify_selected).
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

import luggage_planning.routing.producer as rp  # noqa: E402
from luggage_planning.routing.producer import RouteProducer  # noqa: E402
from st09_contracts.attachment import UNLOADED_ATTACHMENT_DIGEST  # noqa: E402


class _Traj(object):
    joint_names = tuple("elfin_joint%d" % i for i in range(1, 7))

    def __init__(self):
        self.positions = ((0.1, -0.2, 0.3, -0.4, 0.5, -0.6),
                          (0.2, -0.2, 0.3, -0.4, 0.5, -0.6))
        self.velocities = ((0.0,) * 6, (0.01,) * 6)
        self.accelerations = ((0.0,) * 6, (0.01,) * 6)
        self.time_from_start_ns = (0, 200_000_000)

    def digest(self):
        return "obs-leg-digest"


class _Edge(object):
    edge_id = "edge-obs-1"
    attributes = {"edge_class": "PORTAL_TRANSIT"}

    def __init__(self):
        self.trajectory = _Traj()

    def digest(self):
        return "edge-obs-digest"


class _Graph(object):
    def register_node(self, q, position, quaternion, name, load, mode):
        return SimpleNamespace(node_id="%s-%s" % (name, mode), q=q)


class _Producer(object):
    """Only what plan_observation_route touches."""

    def __init__(self, *, snapshot, refine_q=None, edge=None,
                 revalidation=None, certification=None):
        self.urdf = object()
        rp._CAMERA_TCP_CACHE[id(self.urdf)] = (
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        self.graph = _Graph()
        self.fk = lambda q: ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        self._refine_result = refine_q
        self._edge = edge
        self._stub_revalidation = revalidation
        self._stub_certification = certification
        self.latest_snapshot_fn = lambda: snapshot
        self.last_selection = None
        # Recorded budget: _stage_timings reports it, never a default.
        self.budget_ns = 60_000_000_000

    def _refine_to(self, position, orientation, seed):
        return self._refine_result, {}

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
                return producer._edge or _Edge()

            def reset_scene(self):
                pass

        return _Builder()

    _fail = RouteProducer._fail
    _close_row = RouteProducer._close_row
    _promote_selection = RouteProducer._promote_selection
    _stage_timings = RouteProducer._stage_timings

    def _clock_ns(self):
        return 0
    plan_observation_route = RouteProducer.plan_observation_route


def _snapshot(revision=5, scene="scene-A"):
    return SimpleNamespace(
        digest=lambda: "snap-digest", scene_revision=scene,
        map_revision=revision)


def _ok_certification():
    return SimpleNamespace(passed=True, digest=lambda: "cert-obs-1"), None


def _ok_revalidation(revision=5):
    from st09_contracts.certification import MapDecision

    return SimpleNamespace(
        decision=MapDecision.CONTINUE, reason=None, map_revision=revision)


def _call(producer, *, targets=None, latest=5, scene="scene-A",
          joints=(0.1, -0.2, 0.3, -0.4, 0.5, -0.6)):
    if targets is None:
        targets = [((0.5, 0.0, 0.9), (0.0, 0.0, 0.0, 1.0))]
    return producer.plan_observation_route(
        request_id="obs-req-1", scene_revision=scene,
        current_joints=list(joints),
        camera_targets=targets,
        latest_map_revision=latest, purpose="post_place_aim")


def test_rejected_branch_is_not_adopted_and_next_branch_is_certified():
    producer = _Producer(snapshot=_snapshot(),
                         revalidation=_ok_revalidation(),
                         certification=_ok_certification())
    calls = []
    def refine(position, quaternion, seed):
        calls.append(seed)
        return tuple(seed), {}
    producer.refine = refine
    certifications = []
    def certify(*args, **kwargs):
        certifications.append(args)
        return (None, 'CERT_COLLISION') if len(certifications) == 1 else _ok_certification()
    producer._certify = certify
    result = _call(producer)
    assert result['success']
    assert len(calls) == len(certifications) == 2
    assert result['rows'][0]['reason'] == 'CERT_COLLISION'
    assert result['rows'][1]['ik']['seed'].startswith('j6-')
    assert result['rows'][1]['certified']


def test_missing_targets_and_limit_refuse():
    producer = _Producer(snapshot=_snapshot())
    assert _call(producer, targets=[])[
        "reason_code"] == "OBSERVATION_TARGET_MISSING"
    three = [((0.5, 0.0, 0.9), (0.0, 0.0, 0.0, 1.0))] * 3
    assert _call(producer, targets=three)[
        "reason_code"] == "OBSERVATION_TARGET_LIMIT"


def test_map_semantics_fail_closed_without_stale_fallback():
    producer = _Producer(snapshot=None)
    assert _call(producer)["reason_code"] == "MAP_REVALIDATION_UNAVAILABLE"
    producer = _Producer(snapshot=_snapshot(scene="scene-B"))
    assert _call(producer)["reason_code"] == "IDENTITY_CHANGED"
    producer = _Producer(snapshot=_snapshot(revision=6))
    assert _call(producer, latest=5)["reason_code"] == "MAP_RESTART"
    producer = _Producer(snapshot=_snapshot(revision=4))
    assert _call(producer, latest=5)["reason_code"] == "MAP_STALE"


def test_ik_unavailable_is_a_named_row():
    producer = _Producer(
        snapshot=_snapshot(), refine_q=None,
        revalidation=_ok_revalidation(), certification=_ok_certification())
    result = _call(producer)
    assert not result["success"]
    assert result["reason_code"] == "IK_UNAVAILABLE"


def test_pass_path_binds_unloaded_admission_and_selection():
    from st09_contracts.graph import LoadState

    producer = _Producer(
        snapshot=_snapshot(), refine_q=[0.2, -0.2, 0.3, -0.4, 0.5, -0.6],
        edge=_Edge(), revalidation=_ok_revalidation(),
        certification=_ok_certification())
    result = _call(producer)
    assert result["success"] and result["reason_code"] == "OK"
    # The certification ran under the EXPLICIT unloaded admission with
    # no attachment record.
    assert producer._certify_load_state is LoadState.UNLOADED
    assert producer._certify_attachment is None
    assert producer.revalidate_allow_unconfirmed is True
    route = result["route"]
    assert route.attachment_digest == UNLOADED_ATTACHMENT_DIGEST
    # Selection bookkeeping lets verify_selected re-verify by digest.
    selection = producer.last_selection
    assert selection["path_hash"] == route.digest()
    assert selection["attachment"] is None
    assert selection["load_state"] is LoadState.UNLOADED
    assert len(result["legs"]) == 1


def test_recertification_failure_surfaces_its_reason():
    producer = _Producer(
        snapshot=_snapshot(), refine_q=[0.2, -0.2, 0.3, -0.4, 0.5, -0.6],
        edge=_Edge(), revalidation=_ok_revalidation(),
        certification=(
            SimpleNamespace(
                passed=False,
                reason=SimpleNamespace(value="CERT_COLLISION")), None))
    result = _call(producer)
    assert not result["success"]
    assert result["reason_code"] == "CERT_COLLISION"


if __name__ == "__main__":
    import unittest

    unittest.main()
