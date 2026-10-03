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
    def __init__(self, first, digest="leg-digest-1"):
        self.positions = [tuple(first)]
        self._digest = digest

    def digest(self):
        return self._digest


class _Edge(object):
    def __init__(self, first, digest="leg-digest-1",
                 load_state="carrying"):
        self.trajectory = _Traj(first, digest)
        self.load_state = load_state


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
        self.superseded_admissions = {}
        self._stub_revalidation = revalidation
        self._stub_certification = certification

    _promote_selection = RouteProducer._promote_selection
    _stage_timings = RouteProducer._stage_timings

    def _revalidate(self, route, snapshot, edges):
        return self._stub_revalidation

    def _certify(self, route, snapshot, attachment, edges, candidate,
                 request, load_state=None):
        return self._stub_certification

    def _clock_ns(self):
        return 0

    verify_selected = RouteProducer.verify_selected
    execution_admission_is_live = RouteProducer.execution_admission_is_live


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


def test_latest_map_fetch_failure_refuses_without_stale_fallback():
    """[S5.7 lock] A failed LATEST-map fetch is its own named refusal.

    The producer never silently substitutes the selection-time snapshot
    for the latest map (no stale-map fallback): _revalidate returns
    None on fetch failure and the revalidator is never consulted.
    """
    import types

    class _Recording(object):
        def __init__(self):
            self.calls = []

        def revalidate(self, request):
            self.calls.append(request)
            raise AssertionError("must not revalidate on a failed fetch")

    producer = _Producer(_selection())
    producer.revalidator = _Recording()
    producer.latest_snapshot_fn = lambda: None
    producer._clock_ns = lambda: 0
    producer._revalidate = types.MethodType(
        RouteProducer._revalidate, producer)
    ok, reason, _detail = producer.verify_selected(
        route_digest="hash-1", current_joints=[0.1, -0.2, 0.3, -0.4, 0.5,
                                               -0.6],
        latest_map_revision=5, scene_revision="scene-A")
    assert not ok and reason == "MAP_REVALIDATION_UNAVAILABLE"
    assert producer.revalidator.calls == []


def test_map_revision_advanced_without_edge_impact_allows():
    """[S5.7 lock] Case 3: revision grew but no route edge is affected."""
    from st09_contracts.certification import MapDecision
    producer = _Producer(
        _selection(),
        revalidation=SimpleNamespace(
            decision=MapDecision.CONTINUE, reason=None, map_revision=7),
        certification=_ok_certification())
    ok, reason, detail = producer.verify_selected(
        route_digest="hash-1", current_joints=[0.1, -0.2, 0.3, -0.4, 0.5,
                                               -0.6],
        latest_map_revision=7, scene_revision="scene-A")
    assert ok and reason == "OK"


def test_map_corridor_affected_refuses():
    """[S5.7 lock] Case 4: the new map touches the route corridor."""
    from st09_contracts.certification import MapDecision
    producer = _Producer(
        _selection(),
        revalidation=SimpleNamespace(
            decision=MapDecision.REPLAN,
            reason=SimpleNamespace(value="MAP_REVISION_MISMATCH"),
            map_revision=6))
    ok, reason, _detail = producer.verify_selected(
        route_digest="hash-1", current_joints=[0.1, -0.2, 0.3, -0.4, 0.5,
                                               -0.6],
        latest_map_revision=6, scene_revision="scene-A")
    assert not ok and reason == "MAP_REVISION_MISMATCH"


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

    # [S5.7-3b] the FRESH certificate becomes the live admission the
    # executor checks between legs.
    assert producer.last_selection["certification_id"] == "cert-1"


def _live_producer(revalidation=None):
    from st09_contracts.certification import MapDecision

    producer = _Producer(
        _selection(),
        revalidation=revalidation or SimpleNamespace(
            decision=MapDecision.CONTINUE, reason=None, map_revision=5))
    producer.last_selection["certification_id"] = "cert-1"
    return producer


def _admission(producer, *, digest="hash-1", cert="cert-1", revision=5,
               scene="scene-A", leg_digest=None, leg_load_state=None):
    return producer.execution_admission_is_live(
        route_digest=digest, certification_id=cert,
        admission_map_revision=revision, scene_revision=scene,
        leg_digest=leg_digest, leg_load_state=leg_load_state)


def test_execution_admission_unknown_covers_peer_restart():
    producer = _Producer(_selection())
    producer.last_selection = None
    assert _admission(producer)[:2] == (False, "ADMISSION_UNKNOWN")
    assert _admission(producer, digest="other")[:2] == (
        False, "ADMISSION_UNKNOWN")


def test_execution_admission_superseded_certificate_and_identity_refuse():
    producer = _live_producer()
    assert _admission(producer, cert="cert-old")[:2] == (
        False, "CERTIFICATE_SUPERSEDED")
    assert _admission(producer, scene="scene-B")[:2] == (
        False, "GENERATION_CHANGED")


def test_execution_admission_map_regression_refuses():
    producer = _live_producer()
    assert _admission(producer, revision=4)[:2] == (False, "MAP_RESTART")


def test_execution_admission_fetch_failure_and_corridor_impact_revoke():
    from st09_contracts.certification import MapDecision

    producer = _Producer(_selection())
    producer.last_selection["certification_id"] = "cert-1"
    producer._stub_revalidation = None
    assert _admission(producer)[:2] == (
        False, "MAP_REVALIDATION_UNAVAILABLE")
    producer = _live_producer(revalidation=SimpleNamespace(
        decision=MapDecision.REPLAN,
        reason=SimpleNamespace(value="MAP_REVISION_MISMATCH"),
        map_revision=6))
    assert _admission(producer, revision=6)[:2] == (
        False, "MAP_REVISION_MISMATCH")


def test_execution_admission_live_when_map_advanced_without_impact():
    from st09_contracts.certification import MapDecision

    producer = _live_producer(revalidation=SimpleNamespace(
        decision=MapDecision.CONTINUE, reason=None, map_revision=7))
    live, reason, detail = _admission(producer, revision=5)
    assert live and reason == "OK"
    assert detail["live_map_revision"] == 7


def test_new_selection_immediately_supersedes_old_certificate():
    """[ST09-I S5.7-6 lock] verify B -> B is the ONLY live admission.

    Route A's next leg is refused by name (CERTIFICATE_SUPERSEDED —
    the user's ADMISSION_SUPERSEDED semantics: the certifier side knew
    A, then replaced it), never the generic peer-restart
    ADMISSION_UNKNOWN; and a refused admission means no controller
    submission happens for that leg (the executor gates before submit).
    """
    producer = _live_producer()
    selection_b = _selection()
    selection_b["path_hash"] = "hash-B"
    selection_b["certification_id"] = "cert-B"
    producer._promote_selection(selection_b)
    live, reason, detail = _admission(producer)
    assert (live, reason) == (False, "CERTIFICATE_SUPERSEDED")
    assert detail["demoted_certification_id"] == "cert-1"
    live, reason, _detail = _admission(
        producer, digest="hash-B", cert="cert-B")
    assert live and reason == "OK"
    # a stale certification id for the LIVE route is still superseded
    assert _admission(producer, digest="hash-B", cert="cert-old")[:2] == (
        False, "CERTIFICATE_SUPERSEDED")


def _release_route_producer():
    """One certified place route: carrying legs, release, unloaded legs."""
    from st09_contracts.certification import MapDecision

    selection = _selection()
    selection["certification_id"] = "cert-1"
    selection["edges"] = [
        _Edge((0.1, -0.2, 0.3, -0.4, 0.5, -0.6),
              digest="leg-carrying", load_state="carrying"),
        _Edge((0.1, -0.2, 0.3, -0.4, 0.5, -0.6),
              digest="leg-retreat", load_state="unloaded"),
    ]
    producer = _Producer(
        selection, revalidation=SimpleNamespace(
            decision=MapDecision.CONTINUE, reason=None, map_revision=5))
    return producer


def test_release_boundary_admission_uses_certificate_load_state_index():
    """[ST09-I S5.7-6 lock] carrying -> release transition -> unloaded.

    The post-release retreat executes under the certificate's UNLOADED
    index: the vacuum-off attachment digest change at the release
    boundary is never misread as an identity change (the liveness
    verdict consults graph/map generations, never the live attachment
    state), and a carrying scope can never authorize the post-release
    leg (named LOAD_STATE_MISMATCH, no controller submission).
    """
    producer = _release_route_producer()
    live, reason, _detail = _admission(
        producer, leg_digest="leg-retreat", leg_load_state="unloaded")
    assert live and reason == "OK"
    # the same route's carrying phase stays live under ITS index
    live, reason, _detail = _admission(
        producer, leg_digest="leg-carrying", leg_load_state="carrying")
    assert live and reason == "OK"
    # a carrying claim on the post-release leg refuses by name
    assert _admission(
        producer, leg_digest="leg-retreat",
        leg_load_state="carrying")[:2] == (False, "LOAD_STATE_MISMATCH")
    # a foreign leg is not in the certificate at all
    assert _admission(
        producer, leg_digest="leg-foreign")[:2] == (
        False, "LEG_DIGEST_MISMATCH")


def test_stage_timings_attribution_search_is_the_residue():
    """[ST09-I S5.7-6] frozen-budget evidence: certify/revalidate are
    measured, the graph/edge search share is the residue — recorded
    for attribution, the budgets themselves never move."""
    producer = _Producer(_selection())
    producer._clock_ns = lambda: 5_000_000
    producer._stage_ns = {"certify": 3_000_000, "revalidate": 1_000_000}
    timings = producer._stage_timings(0)
    assert timings["total_ms"] == 5.0
    assert timings["certify_ms"] == 3.0
    assert timings["revalidate_ms"] == 1.0
    assert timings["search_ms"] == 1.0
