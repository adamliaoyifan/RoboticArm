#!/usr/bin/env python3
"""ValidateSelectedRoute gate logic (ST09-I S5.7): certify A, execute A.

The probe never generates a second path — these tests pin the named
refusals of the re-verification gates (digest, scene identity, wrapped
start-state match, map regression, re-certification) and the pass path
that returns the per-leg digests the caller must match.
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

from luggage_planning.route_producer import RouteProducer  # noqa: E402


class _Traj(object):
    def __init__(self, first):
        self.positions = [tuple(first)]

    def digest(self):
        return "leg-digest-1"


class _Edge(object):
    def __init__(self, first):
        self.trajectory = _Traj(first)


def _selection(start=(0.1, -0.2, 0.3, -0.4, 0.5, -0.6)):
    return {
        "path_hash": "hash-1",
        "route": SimpleNamespace(),
        "edges": [_Edge(start)],
        "snapshot": SimpleNamespace(
            scene_revision="scene-A", map_revision=5),
        "attachment": None,
        "candidate": None,
        "request": {},
        "certification_id": "cert-0",
    }


class _Producer(object):
    """Only what verify_selected touches."""

    def __init__(self, selection, revalidation=None, certification=None):
        self.last_selection = selection
        self._stub_revalidation = revalidation
        self._stub_certification = certification

    def _revalidate(self, route, snapshot, edges):
        return self._stub_revalidation

    def _certify(self, route, snapshot, attachment, edges, candidate,
                 request):
        return self._stub_certification

    verify_selected = RouteProducer.verify_selected


def _ok_certification():
    return SimpleNamespace(passed=True, digest=lambda: "cert-1"), None


def test_unknown_digest_refuses():
    producer = _Producer(_selection())
    ok, reason, _detail = producer.verify_selected(
        route_digest="other", current_joints=[0.0] * 6,
        latest_map_revision=5, scene_revision="scene-A")
    assert not ok and reason == "DIGEST_UNKNOWN"


def test_scene_identity_change_refuses():
    producer = _Producer(_selection())
    ok, reason, _detail = producer.verify_selected(
        route_digest="hash-1", current_joints=[0.1, -0.2, 0.3, -0.4, 0.5,
                                               -0.6],
        latest_map_revision=5, scene_revision="scene-B")
    assert not ok and reason == "IDENTITY_CHANGED"


def test_start_state_mismatch_refuses_but_wrap_matches():
    producer = _Producer(_selection())
    ok, reason, _detail = producer.verify_selected(
        route_digest="hash-1",
        current_joints=[0.12, -0.2, 0.3, -0.4, 0.5, -0.6],
        latest_map_revision=5, scene_revision="scene-A")
    assert not ok and reason == "START_STATE_MISMATCH"
    # A 2*pi-equivalent branch is the SAME start state (the executor's
    # own wrap discipline).
    import math
    wrapped = [0.1 + 2.0 * math.pi, -0.2, 0.3, -0.4, 0.5, -0.6]
    from st09_contracts.certification import MapDecision
    producer2 = _Producer(
        _selection(),
        revalidation=SimpleNamespace(
            decision=MapDecision.CONTINUE, reason=None, map_revision=5),
        certification=_ok_certification())
    ok, reason, detail = producer2.verify_selected(
        route_digest="hash-1", current_joints=wrapped,
        latest_map_revision=5, scene_revision="scene-A")
    assert ok and reason == "OK"
    assert detail["start_state_delta_rad"] < 1e-6


def test_map_regression_refuses():
    producer = _Producer(_selection())
    ok, reason, _detail = producer.verify_selected(
        route_digest="hash-1",
        current_joints=[0.1, -0.2, 0.3, -0.4, 0.5, -0.6],
        latest_map_revision=4, scene_revision="scene-A")
    assert not ok and reason == "MAP_RESTART"


def test_recertification_failure_surfaces_its_reason():
    from st09_contracts.certification import MapDecision
    producer = _Producer(
        _selection(),
        revalidation=SimpleNamespace(
            decision=MapDecision.CONTINUE, reason=None, map_revision=5),
        certification=(
            SimpleNamespace(
                passed=False, reason=SimpleNamespace(value="CERT_COLLISION")),
            None))
    ok, reason, _detail = producer.verify_selected(
        route_digest="hash-1",
        current_joints=[0.1, -0.2, 0.3, -0.4, 0.5, -0.6],
        latest_map_revision=5, scene_revision="scene-A")
    assert not ok and reason == "CERT_COLLISION"


def test_pass_path_returns_leg_digests():
    from st09_contracts.certification import MapDecision
    producer = _Producer(
        _selection(),
        revalidation=SimpleNamespace(
            decision=MapDecision.CONTINUE, reason=None, map_revision=5),
        certification=_ok_certification())
    ok, reason, detail = producer.verify_selected(
        route_digest="hash-1",
        current_joints=[0.1, -0.2, 0.3, -0.4, 0.5, -0.6],
        latest_map_revision=5, scene_revision="scene-A")
    assert ok and reason == "OK"
    assert detail["certification_id"] == "cert-1"
    assert detail["validated_map_revision"] == 5
    assert len(detail["leg_digests"]) == 1
