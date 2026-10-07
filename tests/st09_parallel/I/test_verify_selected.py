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
        # Recorded budget: _stage_timings reports it, never a default.
        self.budget_ns = 60_000_000_000
        self._stub_revalidation = revalidation
        self._stub_certification = certification
        # [ST09-I S5.7-6] fresh attached vacuum feed (fake _clock_ns
        # returns 0 and received_ns is 0 -> age 0: the carrying-leg
        # LIVE assertions below hold through the new vacuum window).
        self.vacuum_state_fn = lambda: {
            "attached": True, "vacuum_on": True, "received_ns": 0}

    _promote_selection = RouteProducer._promote_selection
    _stage_timings = RouteProducer._stage_timings
    _selection_phase = staticmethod(RouteProducer._selection_phase)
    _live_attachment_matches = RouteProducer._live_attachment_matches
    _live_vacuum_holds = RouteProducer._live_vacuum_holds
    transition_admission_phase = RouteProducer.transition_admission_phase

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


class _Provider(object):
    """Measured-attachment provider fake: live / lost / re-incarnated."""

    def __init__(self, digest="attach-digest"):
        self.digest = digest
        self.lost = False

    def expire_stale(self, request_id, now_ns):
        del request_id, now_ns

    def latest_record(self, request_id):
        del request_id
        if self.lost:
            raise LookupError("record expired")
        return SimpleNamespace(digest=lambda: self.digest)


def _release_route_producer(provider=None):
    """One certified place route: carrying legs, release, unloaded legs."""
    from st09_contracts.certification import MapDecision

    selection = _selection()
    selection["certification_id"] = "cert-1"
    selection["attachment"] = SimpleNamespace(
        digest=lambda: "attach-digest")
    selection["edges"] = [
        _Edge((0.1, -0.2, 0.3, -0.4, 0.5, -0.6),
              digest="leg-carrying", load_state="carrying"),
        _Edge((0.1, -0.2, 0.3, -0.4, 0.5, -0.6),
              digest="leg-retreat", load_state="unloaded"),
    ]
    producer = _Producer(
        selection, revalidation=SimpleNamespace(
            decision=MapDecision.CONTINUE, reason=None, map_revision=5))
    producer.attachment_provider = provider if provider is not None \
        else _Provider()
    return producer


def _transition(producer, **kwargs):
    return producer.transition_admission_phase(
        route_digest="hash-1", certification_id="cert-1",
        target_phase=kwargs.pop("target_phase", "unloaded"),
        detach_confirmed=kwargs.pop("detach_confirmed", True),
        release_evidence=kwargs.pop(
            "release_evidence", "vacuum-off event-7 detach confirmed"),
        **kwargs)


def test_release_boundary_phase_transition_activates_unloaded():
    """[ST09-I S5.7-6 lock] carrying -> release -> detach verified ->
    UNLOADED activated -> retreat.

    Certificate scope and live state are separate gates: the retreat
    needs BOTH the certified unloaded index AND the completed phase
    transition; the vacuum-off digest change at the boundary is never
    an identity change (liveness consults graph/map generations); and
    the carrying claim keeps requiring the live measured attachment.
    """
    producer = _release_route_producer()
    # carrying leg under a live matching attachment: LIVE
    live, reason, _detail = _admission(
        producer, leg_digest="leg-carrying", leg_load_state="carrying")
    assert live and reason == "OK"
    # scope errors stay named
    assert _admission(
        producer, leg_digest="leg-retreat",
        leg_load_state="carrying")[:2] == (False, "LOAD_STATE_MISMATCH")
    assert _admission(
        producer, leg_digest="leg-foreign")[:2] == (
        False, "LEG_DIGEST_MISMATCH")
    # unloaded retreat BEFORE the transition: the certificate saying
    # unloaded is not enough — the phase must be activated
    assert _admission(
        producer, leg_digest="leg-retreat",
        leg_load_state="unloaded")[:2] == (False, "PHASE_TRANSITION_REQUIRED")
    ok, reason, _detail = _transition(producer)
    assert ok and reason == "OK"
    live, reason, _detail = _admission(
        producer, leg_digest="leg-retreat", leg_load_state="unloaded")
    assert live and reason == "OK"
    # a stale carrying claim after the transition refuses by name
    assert _admission(
        producer, leg_digest="leg-carrying",
        leg_load_state="carrying")[:2] == (False, "PHASE_STATE_MISMATCH")


def test_carrying_leg_refuses_when_payload_lost():
    """[S5.7-6 negative] payload lost before a carrying leg: named
    refusal, and the executor gate means zero controller submissions."""
    producer = _release_route_producer()
    producer.attachment_provider.lost = True
    assert _admission(
        producer, leg_digest="leg-carrying",
        leg_load_state="carrying")[:2] == (False, "ATTACHMENT_LOST")


def test_carrying_leg_refuses_on_attachment_incarnation_change():
    """[S5.7-6 negative] an external re-attach (new digest) must not
    keep authorizing the old carrying certificate."""
    producer = _release_route_producer()
    producer.attachment_provider.digest = "attach-digest-reincarnated"
    assert _admission(
        producer, leg_digest="leg-carrying",
        leg_load_state="carrying")[:2] == (
        False, "ATTACHMENT_INCARNATION_CHANGED")


def test_carrying_leg_requires_live_vacuum_state():
    """[S5.7-6 lock] no authoritative vacuum feed -> named refusal.

    Closes the driver-check -> controller-commit window: the admission
    verdict itself must read a live feed (selector-side gate), and an
    old stub without the attribute fails CLOSED, never AttributeError.
    """
    producer = _release_route_producer()
    producer.vacuum_state_fn = None
    assert _admission(
        producer, leg_digest="leg-carrying",
        leg_load_state="carrying")[:2] == (False, "VACUUM_STATE_UNAVAILABLE")
    # the attribute-absent legacy-stub shape fails closed the same way
    producer = _release_route_producer()
    del producer.vacuum_state_fn
    assert _admission(
        producer, leg_digest="leg-carrying",
        leg_load_state="carrying")[:2] == (False, "VACUUM_STATE_UNAVAILABLE")


def test_carrying_leg_refuses_on_stale_vacuum_state():
    """[S5.7-6 negative] receive-time age beyond the 2.0 s window
    refuses by name with the age in the detail."""
    producer = _release_route_producer()
    producer.vacuum_state_fn = lambda: {
        "attached": True, "vacuum_on": True, "received_ns": 0}
    producer._clock_ns = lambda: 5_000_000_000  # 5 s later
    live, reason, detail = _admission(
        producer, leg_digest="leg-carrying", leg_load_state="carrying")
    assert (live, reason) == (False, "VACUUM_STATE_STALE")
    assert detail["age_s"] == 5.0


def test_carrying_leg_refuses_when_vacuum_detached():
    """[S5.7-6 negative] fresh feed, attached False: PAYLOAD_LOST."""
    producer = _release_route_producer()
    producer.vacuum_state_fn = lambda: {
        "attached": False, "vacuum_on": True, "received_ns": 0}
    assert _admission(
        producer, leg_digest="leg-carrying",
        leg_load_state="carrying")[:2] == (False, "PAYLOAD_LOST")


def test_carrying_leg_live_with_fresh_attached_vacuum():
    """[S5.7-6] fresh + attached: the carrying claim stays LIVE through
    the new vacuum window (the gate composes with, not replaces, the
    measured-attachment binding)."""
    producer = _release_route_producer()
    producer.vacuum_state_fn = lambda: {
        "attached": True, "vacuum_on": True, "received_ns": 0}
    producer._clock_ns = lambda: 1_000_000_000  # 1 s < 2.0 s window
    live, reason, _detail = _admission(
        producer, leg_digest="leg-carrying", leg_load_state="carrying")
    assert live and reason == "OK"


def test_fail_reason_with_attached_still_admits():
    """[S5.7-6 lock] the gate keys on ``attached`` ONLY.

    The I-suite mock sim backend sets fail_reason while attached stays
    True (gz set_pose unavailable); keying on fail_reason would fail
    every mock-stack carrying leg. fail_reason is evidence, never the
    admission key.
    """
    producer = _release_route_producer()
    producer.vacuum_state_fn = lambda: {
        "attached": True, "vacuum_on": True, "received_ns": 0,
        "fail_reason": "GZ_SET_POSE_UNAVAILABLE"}
    live, reason, _detail = _admission(
        producer, leg_digest="leg-carrying", leg_load_state="carrying")
    assert live and reason == "OK"


def test_detach_unconfirmed_refuses_the_phase_transition():
    """[S5.7-6 negative] vacuum-off sent but detach not confirmed: the
    unloaded phase never activates (the retreat would be refused)."""
    producer = _release_route_producer()
    ok, reason, _detail = _transition(producer, detach_confirmed=False)
    assert not ok and reason == "DETACH_UNCONFIRMED"
    ok, reason, _detail = _transition(producer, release_evidence="  ")
    assert not ok and reason == "RELEASE_EVIDENCE_MISSING"
    # and the transition target is one-way carrying -> unloaded
    ok, reason, _detail = _transition(producer, target_phase="carrying")
    assert not ok and reason == "PHASE_TRANSITION_INVALID"


def test_unloaded_leg_refuses_while_release_never_happened():
    """[S5.7-6 negative] a live attachment + no transition: claiming
    the unloaded retreat is refused (the release side effect never
    ran), while the attachment itself is still perfectly live."""
    provider = _Provider()
    producer = _release_route_producer(provider=provider)
    assert _admission(
        producer, leg_digest="leg-retreat",
        leg_load_state="unloaded")[:2] == (False, "PHASE_TRANSITION_REQUIRED")
    live, reason, _detail = _admission(
        producer, leg_digest="leg-carrying", leg_load_state="carrying")
    assert live and reason == "OK"


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
