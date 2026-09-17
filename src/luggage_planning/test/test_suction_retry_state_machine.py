#!/usr/bin/env python3
"""Gates C2-C6: bounded attempts, seal success, the exact C4 retry
trace, recovery fault injection, and the pre/carry failure boundary —
plus the reducer transition grid and the C7 dry-run protocol.
"""

import unittest

from harness import suction_fakes as fakes

from luggage_planning.suction_pick_session import (
    EXIT_CANDIDATES_EXHAUSTED,
    EXIT_CARRY_FAULT,
    EXIT_OK,
    EXIT_RETRY_RECOVERY,
    PickSession,
    SessionConfig,
    SessionResult,
)
from luggage_planning.suction_retry_contracts import (
    RELEASE_DEADLINE_SEC,
    RELEASE_LOW_HOLD_SEC,
    RETRY_BUDGET_EXPIRED,
    RETRY_CONTROLLER_FAILURE,
    RETRY_DI0_AMBIGUOUS,
    RETRY_DI0_STUCK_HIGH,
    RETRY_GENERATION_CHANGED,
    RETRY_GRAPH_UNHEALTHY,
    RETRY_RELEASE_LOW_TIMEOUT,
    RETRY_RECOVERY_FRACTION,
    RETRY_VACUUM_SERVICE_ERROR,
    RETRY_BOUNDARIES,
    SUCTION_CANDIDATES_EXHAUSTED,
    SUCTION_RETRY_RECOVERY_FAILED,
    VACUUM_SEAL_LOST,
    FailureBoundary,
    RetryEvent,
    RetryEventType,
    RetryModel,
    RetryState,
    Transition,
    UnhandledRetryTransition,
    audit_trace,
    initial_retry_model,
    is_terminal,
    reduce_retry_event,
)

CONFIG = dict(contact_model_version=1, contact_model_hash="abc123")


def three_candidates():
    return [fakes.make_candidate("C001_001", 1),
            fakes.make_candidate("C002_001", 2),
            fakes.make_candidate("C003_001", 3)]


def run_session(candidates, **kwargs):
    ports = fakes.make_ports(candidates, **kwargs)
    session = PickSession(ports, SessionConfig(**CONFIG))
    result = session.run_request(fakes.make_detection(candidates))
    return session, result, ports


def canonical_names(trace):
    """Trace reduced to the ordered C4-relevant names.

    Order note: ``release_backend`` precedes ``seal_timeout`` because the
    backend completes its DO0=0 + blow-off release *inside* the timed-out
    attach call; the machine's seal-timeout status follows recognition.
    """
    names = []
    for record in trace:
        kind = record["kind"]
        name = record["name"]
        if kind == "effect":
            payload = record.get("payload", {})
            segment = payload.get("segment", "")
            if segment:
                names.append("%s:%s" % (segment, payload.get(
                    "candidate_id", "")))
                continue
            if record.get("effect_type") == "VACUUM_ENABLE":
                names.append("vacuum_on")
                continue
            if record.get("effect_type") == "SCENE_ATTACH":
                names.append("scene_attach")
                continue
        elif kind == "event" and name in (
                "release_backend", "seal_timeout", "release_confirmed",
                "sealed", "settled"):
            names.append(name)
        elif kind == "io" and name in ("identity_ok", "graph_ok"):
            names.append(name)
    return names


class TestReducerTransitionGrid(unittest.TestCase):
    """Every (state, event) pair is handled explicitly or raises typed."""

    def _representative_events(self):
        events = []
        for event_type in RetryEventType:
            events.append(RetryEvent(event_type, candidate_id="C001_001",
                                     segment="approach", ok=True,
                                     reason_code="r", detail="d", t=10.0,
                                     di0=0, fraction=1.0,
                                     ranked_ids=("C001_001",)))
            # di0 sample variants for the window
            events.append(RetryEvent(event_type, t=10.6, di0=1))
            events.append(RetryEvent(event_type, t=11.5, di0=None))
            events.append(RetryEvent(event_type, t=13.0, fraction=0.99,
                                     reason_code="fail"))
        return events

    def test_grid_covers_every_state_event_pair(self):
        handled = set()
        for state in RetryState:
            for event in self._representative_events():
                model = RetryModel(
                    state=state,
                    ranked_ids=("C001_001", "C002_001"),
                    attempted_ids=frozenset({"C001_001"})
                    if state not in (RetryState.IDLE,) else frozenset(),
                    active_candidate="C001_001" if state not in (
                        RetryState.IDLE, RetryState.SELECTING) else "",
                    release_window_started=10.0,
                    di0_confirmed=state in (
                        RetryState.SCENE_ATTACH,
                        RetryState.PICK_RETREAT_MOTION,
                        RetryState.CARRY_FAULT,
                        RetryState.COMPLETED),
                    vacuum_on=state in (RetryState.SCENE_ATTACH,
                                        RetryState.PICK_RETREAT_MOTION),
                )
                try:
                    transition = reduce_retry_event(model, event)
                except UnhandledRetryTransition:
                    continue
                except Exception as error:   # noqa: BLE001 - grid must be clean
                    self.fail("state %s event %s raised %r"
                              % (state.value, event.event_type.value,
                                 error))
                self.assertIsInstance(transition, Transition)
                handled.add((state, event.event_type))
        # every event type is handled in at least one state, and every
        # non-terminal state has at least one handled event
        for event_type in RetryEventType:
            self.assertTrue(
                any(key[1] == event_type for key in handled),
                "event %s never handled" % event_type.value)
        for state in RetryState:
            if is_terminal(state):
                continue
            self.assertTrue(
                any(key[0] == state for key in handled),
                "state %s never handles any event" % state.value)

    def test_duplicate_candidate_selection_raises(self):
        model = initial_retry_model()
        model = reduce_retry_event(
            model, RetryEvent(RetryEventType.REQUEST_START, t=0.0,
                              ranked_ids=("C001_001",))).model
        model = reduce_retry_event(
            model, RetryEvent(RetryEventType.CANDIDATE_SELECTED,
                              candidate_id="C001_001")).model
        with self.assertRaises(UnhandledRetryTransition):
            reduce_retry_event(
                model, RetryEvent(RetryEventType.CANDIDATE_SELECTED,
                                  candidate_id="C001_001"))

    def test_terminal_states_raise_on_any_event(self):
        for state in (RetryState.COMPLETED, RetryState.CANDIDATES_EXHAUSTED,
                      RetryState.RETRY_RECOVERY_FAILED,
                      RetryState.MOTION_FAILED, RetryState.CARRY_FAULT,
                      RetryState.REJECTED):
            with self.assertRaises(UnhandledRetryTransition):
                reduce_retry_event(
                    RetryModel(state=state),
                    RetryEvent(RetryEventType.REQUEST_START, t=0.0,
                               ranked_ids=("C001_001",)))


class TestReducerReleaseWindow(unittest.TestCase):
    def _to_window(self, t0=10.0):
        model = initial_retry_model()
        for event in (
                RetryEvent(RetryEventType.REQUEST_START, t=0.0,
                           ranked_ids=("C001_001",)),
                RetryEvent(RetryEventType.CANDIDATE_SELECTED,
                           candidate_id="C001_001"),
                RetryEvent(RetryEventType.SEGMENT_SUCCEEDED,
                           segment="pre_grasp"),
                RetryEvent(RetryEventType.SEGMENT_SUCCEEDED,
                           segment="approach"),
                RetryEvent(RetryEventType.SEGMENT_SUCCEEDED,
                           segment="attach"),
                RetryEvent(RetryEventType.VACUUM_SEAL_TIMEOUT, t=t0),
        ):
            model = reduce_retry_event(model, event).model
        return model

    def test_low_hold_requires_continuous_half_second(self):
        model = self._to_window()
        model = reduce_retry_event(
            model, RetryEvent(RetryEventType.DI0_SAMPLE, t=10.0, di0=0)).model
        # high glitch at 10.2 resets the hold
        model = reduce_retry_event(
            model, RetryEvent(RetryEventType.DI0_SAMPLE, t=10.2, di0=1)).model
        self.assertEqual(model.state, RetryState.RELEASE_LOW_WAIT)
        model = reduce_retry_event(
            model, RetryEvent(RetryEventType.DI0_SAMPLE, t=10.3, di0=0)).model
        # 0.4 s of low is not enough
        model = reduce_retry_event(
            model, RetryEvent(RetryEventType.DI0_SAMPLE, t=10.7, di0=0)).model
        self.assertEqual(model.state, RetryState.RELEASE_LOW_WAIT)
        # 0.5 s continuous low confirms
        transition = reduce_retry_event(
            model, RetryEvent(RetryEventType.DI0_SAMPLE, t=10.8, di0=0))
        self.assertEqual(transition.model.state,
                         RetryState.RETRY_REVERSE_MOTION)
        self.assertTrue(transition.model.release_confirmed)

    def test_window_deadline_fails_closed(self):
        model = self._to_window()
        model = reduce_retry_event(
            model, RetryEvent(RetryEventType.DI0_SAMPLE, t=10.0, di0=0)).model
        # low but interrupted: never holds 0.5 s inside the deadline
        for t in (10.4, 10.8, 11.2, 11.6, 12.0):
            model = reduce_retry_event(
                model, RetryEvent(RetryEventType.DI0_SAMPLE, t=t,
                                  di0=1 if int(t * 10) % 8 == 0 else 0)).model
        transition = reduce_retry_event(
            model, RetryEvent(RetryEventType.DI0_SAMPLE, t=12.05, di0=0))
        self.assertEqual(transition.model.state,
                         RetryState.RETRY_RECOVERY_FAILED)
        self.assertEqual(transition.model.reason_code,
                         SUCTION_RETRY_RECOVERY_FAILED)

    def test_stuck_high_gives_stuck_detail(self):
        model = self._to_window()
        transition = reduce_retry_event(
            model, RetryEvent(RetryEventType.DI0_SAMPLE, t=12.2, di0=1))
        self.assertEqual(transition.model.state,
                         RetryState.RETRY_RECOVERY_FAILED)
        self.assertEqual(transition.model.detail, RETRY_DI0_STUCK_HIGH)

    def test_unknown_di0_is_ambiguity_fault(self):
        model = self._to_window()
        transition = reduce_retry_event(
            model, RetryEvent(RetryEventType.DI0_SAMPLE, t=10.1, di0=None))
        self.assertEqual(transition.model.state,
                         RetryState.RETRY_RECOVERY_FAILED)
        self.assertEqual(transition.model.detail, RETRY_DI0_AMBIGUOUS)
        # no vacuum command may be issued from the ambiguity branch
        self.assertFalse(any(effect.vacuum for effect in transition.effects))

    def test_plan_constants_match_the_plan(self):
        self.assertEqual(RELEASE_LOW_HOLD_SEC, 0.5)
        self.assertEqual(RELEASE_DEADLINE_SEC, 2.0)


class TestBoundaryTable(unittest.TestCase):
    """C6 boundary authority: the public table matches handler outcomes."""

    def test_every_state_has_exactly_one_boundary(self):
        seen = {}
        for boundary, spec in RETRY_BOUNDARIES.items():
            for state in spec["states"]:
                self.assertNotIn(
                    state, seen,
                    "state %s in two boundaries" % state.value)
                seen[state] = boundary
        for state in RetryState:
            self.assertIn(state, seen, "state %s has no boundary"
                          % state.value)
            self.assertEqual(RetryModel(state=state).boundary, seen[state])

    def test_carry_boundary_preserves_vacuum(self):
        spec = RETRY_BOUNDARIES[FailureBoundary.CARRY]
        self.assertTrue(spec["vacuum_preserved"])

    def test_pre_seal_and_recovery_never_preserve_vacuum(self):
        self.assertFalse(
            RETRY_BOUNDARIES[FailureBoundary.PRE_SEAL]["vacuum_preserved"])
        self.assertFalse(
            RETRY_BOUNDARIES[FailureBoundary.RETRY_RECOVERY][
                "vacuum_preserved"])


class TestSealSuccess(unittest.TestCase):
    """Gate C3: fake DI0 rise delays under the 8.0 s backend timeout."""

    def test_seal_success_for_each_delay(self):
        for delay in (0.0, 1.0, 6.4, 7.9):
            with self.subTest(delay=delay):
                candidates = [fakes.make_candidate("C001_001", 1)]
                session, result, ports = run_session(
                    candidates, seal_delays={"C001_001": delay})
                self.assertEqual(result.exit_code, EXIT_OK,
                                 result.reason_code)
                self.assertTrue(result.completed)
                # exactly one vacuum enable, one scene attach, one retreat
                self.assertEqual(ports.vacuum.enable_calls, 1)
                self.assertEqual(
                    ports.scene.calls.count(("attach", True)), 1)
                self.assertEqual(
                    [name for _cid, name in ports.motion.executed],
                    ["pre_grasp", "approach", "attach", "pick_retreat"])
                # no retry recovery ran
                self.assertNotIn(
                    "retry_reverse",
                    [name for _cid, name in ports.motion.executed])
                self.assertEqual(session.audit.order_violations, 0)
                self.assertEqual(session.audit.duplicate_scene_attach, 0)


class TestRetryTrace(unittest.TestCase):
    """Gate C4: the exact first-fail/second-seal trace."""

    def test_exact_trace(self):
        candidates = [fakes.make_candidate("C001_001", 1),
                      fakes.make_candidate("C002_001", 2)]
        session, result, ports = run_session(
            candidates,
            seal_delays={"C001_001": 99.0, "C002_001": 0.5})
        self.assertEqual(result.exit_code, EXIT_OK)
        self.assertEqual(result.selected_candidate_id, "C002_001")
        names = canonical_names(result.trace)
        self.assertEqual(names, [
            "pre_grasp:C001_001",
            "approach:C001_001",
            "attach:C001_001",
            "vacuum_on",
            "release_backend",
            "seal_timeout",
            "release_confirmed",
            "retry_reverse:C001_001",
            "settled",
            "identity_ok",
            "graph_ok",
            "pre_grasp:C002_001",
            "approach:C002_001",
            "attach:C002_001",
            "vacuum_on",
            "sealed",
            "scene_attach",
            "pick_retreat:C002_001",
        ])
        # all audit counters zero
        audit = session.audit
        self.assertEqual(
            (audit.order_violations,
             audit.lateral_before_release_confirmed,
             audit.vacuum_on_lateral_motion,
             audit.duplicate_scene_attach,
             audit.unauthorized_detect,
             audit.unauthorized_vacuum_during_recovery),
            (0, 0, 0, 0, 0, 0))

    def test_reverse_retreat_is_at_least_80mm(self):
        candidates = [fakes.make_candidate("C001_001", 1),
                      fakes.make_candidate("C002_001", 2)]
        _session, result, _ports = run_session(
            candidates,
            seal_delays={"C001_001": 99.0, "C002_001": 0.5})
        settled = [record for record in result.trace
                   if record["name"] == "settled"]
        self.assertEqual(len(settled), 1)
        self.assertGreaterEqual(
            settled[0]["payload"]["reverse_distance_m"], 0.08)

    def test_lateral_motion_only_after_recovery_confirmed(self):
        candidates = [fakes.make_candidate("C001_001", 1),
                      fakes.make_candidate("C002_001", 2)]
        _session, result, ports = run_session(
            candidates,
            seal_delays={"C001_001": 99.0, "C002_001": 0.5})
        confirmed_at = None
        lateral_at = None
        for record in result.trace:
            if record["name"] == "release_confirmed":
                confirmed_at = record["t"]
            if (record["kind"] == "effect"
                    and record.get("payload", {}).get("segment")
                    == "pre_grasp"
                    and record.get("payload", {}).get("candidate_id")
                    == "C002_001"):
                lateral_at = record["t"]
        self.assertIsNotNone(confirmed_at)
        self.assertIsNotNone(lateral_at)
        self.assertGreaterEqual(lateral_at, confirmed_at)


class TestBoundedAttempts(unittest.TestCase):
    """Gate C2: at most three candidates, one vacuum enable each."""

    def test_exhaustion_after_three_seal_timeouts(self):
        candidates = three_candidates()
        session, result, ports = run_session(
            candidates,
            seal_delays={"C001_001": 99.0, "C002_001": 99.0,
                         "C003_001": 99.0})
        self.assertEqual(result.exit_code, EXIT_CANDIDATES_EXHAUSTED)
        self.assertEqual(result.reason_code, SUCTION_CANDIDATES_EXHAUSTED)
        # three unique candidates, one vacuum enable each
        self.assertEqual(ports.vacuum.enable_candidates,
                         ["C001_001", "C002_001", "C003_001"])
        self.assertEqual(len(set(ports.vacuum.enable_candidates)), 3)
        self.assertEqual(ports.vacuum.enable_calls, 3)
        # inside the 180 s budget on the fake clock
        self.assertLessEqual(ports.clock.now() - 1000.0, 180.0)
        # terminal state: vacuum off, DI0 low, no attached scene object
        self.assertEqual(ports.vacuum.do0, 0)
        self.assertEqual(ports.vacuum.di0(), 0)
        self.assertNotIn(("attach", True), ports.scene.calls)

    def test_five_candidates_still_cap_at_three_attempts(self):
        candidates = [fakes.make_candidate("C%03d_001" % i, i)
                      for i in range(1, 6)]
        _session, result, ports = run_session(
            candidates,
            seal_delays={cid: 99.0 for cid in
                         ("C001_001", "C002_001", "C003_001")})
        self.assertEqual(result.exit_code, EXIT_CANDIDATES_EXHAUSTED)
        self.assertEqual(ports.vacuum.enable_calls, 3)

    def test_max_one_candidate_reproduces_single_attempt(self):
        candidates = three_candidates()
        ports = fakes.make_ports(
            candidates,
            seal_delays={"C001_001": 99.0, "C002_001": 99.0})
        session = PickSession(
            ports, SessionConfig(max_candidates=1, **CONFIG))
        result = session.run_request(fakes.make_detection(candidates))
        self.assertEqual(result.exit_code, EXIT_CANDIDATES_EXHAUSTED)
        self.assertEqual(ports.vacuum.enable_candidates, ["C001_001"])

    def test_request_budget_watchdog_terminates(self):
        candidates = [fakes.make_candidate("C001_001", 1)]
        # 6.4 s seal wait blows the 0.001 s budget; the watchdog fires
        # right after the seal event, inside the carry boundary
        ports = fakes.make_ports(candidates,
                                 seal_delays={"C001_001": 6.4})
        session = PickSession(
            ports, SessionConfig(request_deadline_sec=0.001, **CONFIG))
        result = session.run_request(fakes.make_detection(candidates))
        self.assertEqual(result.reason_code, SUCTION_RETRY_RECOVERY_FAILED)
        self.assertEqual(result.detail, RETRY_BUDGET_EXPIRED)
        self.assertIn(result.exit_code,
                      (EXIT_CANDIDATES_EXHAUSTED, EXIT_RETRY_RECOVERY,
                       EXIT_CARRY_FAULT))


class TestRecoveryFaultInjection(unittest.TestCase):
    """Gate C5: every recovery failure stops before lateral motion."""

    def _assert_recovery_failure(self, result, ports, detail=None):
        self.assertEqual(result.exit_code, EXIT_RETRY_RECOVERY)
        self.assertEqual(result.reason_code, SUCTION_RETRY_RECOVERY_FAILED)
        if detail is not None:
            self.assertEqual(result.detail, detail)
        # no lateral motion was dispatched for a next candidate
        laterals = [name for _cid, name in ports.motion.executed
                    if name == "pre_grasp"]
        self.assertLessEqual(laterals.count("pre_grasp"), 1)
        # no next-candidate probe and no new detection
        probed_candidates = {cid for cid, _seg in ports.probe_fake.calls}
        self.assertLessEqual(len(probed_candidates), 1)
        self.assertEqual(ports.detect_calls_tally, 0)

    @staticmethod
    def _ports(candidates, **kwargs):
        ports = fakes.make_ports(candidates, **kwargs)
        ports.detect_calls_tally = 0
        return ports

    def test_stuck_high_di0(self):
        candidates = three_candidates()
        ports = self._ports(
            candidates,
            seal_delays={"C001_001": 99.0, "C002_001": 0.0},
            stuck_high=True)
        session = PickSession(ports, SessionConfig(**CONFIG))
        result = session.run_request(fakes.make_detection(candidates))
        self._assert_recovery_failure(result, ports, RETRY_DI0_STUCK_HIGH)
        # no vacuum commands after the fault: only the original enable
        self.assertEqual(ports.vacuum.enable_calls, 1)

    def test_unknown_di0(self):
        candidates = three_candidates()
        ports = self._ports(
            candidates,
            seal_delays={"C001_001": 99.0, "C002_001": 0.0},
            unknown=True)
        session = PickSession(ports, SessionConfig(**CONFIG))
        result = session.run_request(fakes.make_detection(candidates))
        self._assert_recovery_failure(result, ports, RETRY_DI0_AMBIGUOUS)
        self.assertEqual(ports.vacuum.enable_calls, 1)

    def test_release_window_timeout(self):
        candidates = three_candidates()
        # intermittent DI0: low never holds 0.5 s inside the 2.0 s window
        ports = self._ports(
            candidates,
            seal_delays={"C001_001": 99.0, "C002_001": 0.0},
            di0_flip_flop_sec=0.2)
        session = PickSession(ports, SessionConfig(**CONFIG))
        result = session.run_request(fakes.make_detection(candidates))
        self._assert_recovery_failure(result, ports,
                                      RETRY_RELEASE_LOW_TIMEOUT)

    def test_recovery_cartesian_fraction_0p99(self):
        candidates = three_candidates()
        from luggage_planning.suction_pick_session import SegmentOutcome
        ports = self._ports(
            candidates,
            seal_delays={"C001_001": 99.0, "C002_001": 0.0},
            segment_outcomes={
                "retry_reverse": SegmentOutcome(
                    ok=False,
                    message="cartesian fraction 0.990 below 1.000",
                    fraction=0.99)})
        session = PickSession(ports, SessionConfig(**CONFIG))
        result = session.run_request(fakes.make_detection(candidates))
        self._assert_recovery_failure(result, ports,
                                      RETRY_RECOVERY_FRACTION)

    def test_controller_failure_on_recovery(self):
        candidates = three_candidates()
        from luggage_planning.suction_pick_session import SegmentOutcome
        ports = self._ports(
            candidates,
            seal_delays={"C001_001": 99.0, "C002_001": 0.0},
            segment_outcomes={
                "retry_reverse": SegmentOutcome(
                    ok=False, message="controller rejected goal",
                    fraction=-1.0)})
        session = PickSession(ports, SessionConfig(**CONFIG))
        result = session.run_request(fakes.make_detection(candidates))
        self._assert_recovery_failure(result, ports,
                                      RETRY_CONTROLLER_FAILURE)

    def test_generation_change(self):
        candidates = three_candidates()
        ports = self._ports(
            candidates,
            seal_delays={"C001_001": 99.0, "C002_001": 0.0},
            identity=("box-1", 8))     # epoch advanced under the arm
        session = PickSession(ports, SessionConfig(**CONFIG))
        result = session.run_request(fakes.make_detection(candidates))
        self._assert_recovery_failure(result, ports,
                                      RETRY_GENERATION_CHANGED)

    def test_identity_unavailable_fails_closed(self):
        candidates = three_candidates()
        ports = self._ports(
            candidates,
            seal_delays={"C001_001": 99.0, "C002_001": 0.0},
            identity=None)
        session = PickSession(ports, SessionConfig(**CONFIG))
        result = session.run_request(fakes.make_detection(candidates))
        self._assert_recovery_failure(result, ports,
                                      RETRY_GENERATION_CHANGED)
        # the fail-closed reason is preserved in the trace
        self.assertTrue(any(
            record["name"] == "identity_unavailable"
            for record in result.trace))

    def test_graph_loss(self):
        candidates = three_candidates()
        ports = self._ports(
            candidates,
            seal_delays={"C001_001": 99.0, "C002_001": 0.0},
            graph_ok=False)
        session = PickSession(ports, SessionConfig(**CONFIG))
        result = session.run_request(fakes.make_detection(candidates))
        self._assert_recovery_failure(result, ports, RETRY_GRAPH_UNHEALTHY)

    def test_vacuum_service_error_stops_cold(self):
        candidates = three_candidates()
        ports = self._ports(
            candidates,
            seal_delays={"C001_001": 0.0},
            service_error=True)
        session = PickSession(ports, SessionConfig(**CONFIG))
        result = session.run_request(fakes.make_detection(candidates))
        self._assert_recovery_failure(result, ports,
                                      RETRY_VACUUM_SERVICE_ERROR)
        # no motion after the fault at all: the arm stays at attach
        self.assertNotIn("retry_reverse",
                         [name for _cid, name in ports.motion.executed])


class TestCarryBoundary(unittest.TestCase):
    """Gate C6: post-DI0 failures preserve vacuum (carry fault)."""

    def test_pick_retreat_failure_preserves_vacuum(self):
        from luggage_planning.suction_pick_session import SegmentOutcome
        candidates = [fakes.make_candidate("C001_001", 1)]
        ports = fakes.make_ports(
            candidates, seal_delays={"C001_001": 0.0},
            segment_outcomes={
                "pick_retreat": SegmentOutcome(
                    ok=False, message="controller aborted", fraction=-1.0)})
        session = PickSession(ports, SessionConfig(**CONFIG))
        result = session.run_request(fakes.make_detection(candidates))
        self.assertEqual(result.exit_code, EXIT_CARRY_FAULT)
        # vacuum preserved: no release, DI0 still high, DO0 on
        self.assertEqual(ports.vacuum.di0(), 1)
        self.assertEqual(ports.vacuum.do0, 1)
        self.assertEqual(ports.vacuum.release_calls, 0)

    def test_pressure_loss_while_carrying(self):
        candidates = [fakes.make_candidate("C001_001", 1)]
        ports = fakes.make_ports(
            candidates, seal_delays={"C001_001": 0.0},
            drop_di0_on_carry=True)
        session = PickSession(ports, SessionConfig(**CONFIG))
        result = session.run_request(fakes.make_detection(candidates))
        self.assertEqual(result.exit_code, EXIT_CARRY_FAULT)
        self.assertEqual(result.reason_code, VACUUM_SEAL_LOST)
        # no further motion was dispatched after the loss
        self.assertEqual(
            [name for _cid, name in ports.motion.executed],
            ["pre_grasp", "approach", "attach"])

    def test_scene_attach_failure_is_carry_fault(self):
        candidates = [fakes.make_candidate("C001_001", 1)]
        scene = fakes.FakeScenePort(fail_attach=True)
        ports = fakes.make_ports(
            candidates, seal_delays={"C001_001": 0.0}, scene=scene)
        session = PickSession(ports, SessionConfig(**CONFIG))
        result = session.run_request(fakes.make_detection(candidates))
        self.assertEqual(result.exit_code, EXIT_CARRY_FAULT)
        self.assertEqual(ports.vacuum.di0(), 1)

    def test_pre_seal_motion_failure_is_not_carry(self):
        from luggage_planning.suction_pick_session import (
            EXIT_SEGMENT, SegmentOutcome)
        candidates = [fakes.make_candidate("C001_001", 1)]
        ports = fakes.make_ports(
            candidates, seal_delays={"C001_001": 0.0},
            segment_outcomes={
                "approach": SegmentOutcome(ok=False,
                                           message="plan failed",
                                           fraction=-1.0)})
        session = PickSession(ports, SessionConfig(**CONFIG))
        result = session.run_request(fakes.make_detection(candidates))
        # pre-seal failure: non-moving stop, vacuum never confirmed
        self.assertEqual(result.exit_code, EXIT_SEGMENT)
        self.assertEqual(ports.vacuum.enable_calls, 0)
        self.assertEqual(ports.vacuum.do0, 0)


class TestNoNewDetection(unittest.TestCase):
    """Retries never trigger a new detection while the arm occludes."""

    def test_detect_never_called_in_any_scenario(self):
        scenarios = [
            dict(seal_delays={"C001_001": 0.0}),
            dict(seal_delays={"C001_001": 99.0, "C002_001": 0.5}),
            dict(seal_delays={"C001_001": 99.0, "C002_001": 99.0,
                              "C003_001": 99.0}),
        ]
        for kwargs in scenarios:
            candidates = three_candidates()
            session, _result, ports = run_session(candidates, **kwargs)
            self.assertEqual(ports.detect_calls, 0)
            self.assertEqual(session.audit.unauthorized_detect, 0)


class TestAuditCounters(unittest.TestCase):
    def test_audit_detects_violating_traces(self):
        base = {"kind": "effect", "effect_type": "EXECUTE_SEGMENT",
                "lateral": False, "detect": False,
                "payload": {"segment": "attach"}}
        records = [
            {"kind": "event", "name": "sealed"},
            dict(base),                                        # attach ok
            {"kind": "effect", "effect_type": "SCENE_ATTACH"},  # 1st ok
            {"kind": "effect", "effect_type": "SCENE_ATTACH"},  # duplicate
            {"kind": "effect", "effect_type": "EXECUTE_SEGMENT",
             "detect": True, "payload": {"segment": "approach"}},
        ]
        counters = audit_trace(records)
        self.assertEqual(counters.duplicate_scene_attach, 1)
        self.assertEqual(counters.unauthorized_detect, 1)

    def test_audit_detects_lateral_before_release(self):
        records = [
            {"kind": "event", "name": "seal_timeout"},
            {"kind": "effect", "effect_type": "EXECUTE_SEGMENT",
             "lateral": True, "payload": {"segment": "pre_grasp"}},
        ]
        counters = audit_trace(records)
        self.assertEqual(counters.lateral_before_release_confirmed, 1)

    def test_audit_detects_vacuum_on_lateral(self):
        records = [
            {"kind": "effect", "effect_type": "VACUUM_ENABLE"},
            {"kind": "effect", "effect_type": "EXECUTE_SEGMENT",
             "lateral": True, "payload": {"segment": "pre_grasp"}},
        ]
        counters = audit_trace(records)
        self.assertEqual(counters.vacuum_on_lateral_motion, 1)

    def test_audit_detects_vacuum_during_recovery(self):
        records = [
            {"kind": "event", "name": "seal_timeout"},
            {"kind": "effect", "effect_type": "VACUUM_ENABLE"},
        ]
        counters = audit_trace(records)
        self.assertEqual(counters.unauthorized_vacuum_during_recovery, 1)


class TestDryRunSession(unittest.TestCase):
    """C7 protocol: detection-only, zero actions."""

    def test_dry_run_touches_no_capability_port(self):
        candidates = [fakes.make_candidate("C001_001", 1),
                      fakes.make_candidate("C002_001", 2)]
        detection = fakes.make_detection(candidates)
        ports = fakes.make_ports(
            candidates, seal_delays={}, dry_run_detections=[
                detection, detection, detection])
        session = PickSession(
            ports, SessionConfig(dry_run=True, dry_run_count=3, **CONFIG))
        result = session.run_dry_run()
        self.assertEqual(result.exit_code, EXIT_OK)
        # three detections, zero probe/execute/vacuum/scene
        self.assertEqual(ports.detect_calls, 3)
        self.assertEqual(len(ports.dry_run_detections), 0)
        self.assertEqual(ports.probe_fake.calls, [])
        self.assertEqual(ports.motion.executed, [])
        self.assertEqual(ports.vacuum.enable_calls, 0)
        self.assertEqual(ports.scene.calls, [])
        # every detection printed ranked candidates with model provenance
        printed = [record for record in result.trace
                   if record["name"] == "dry_run_candidate"]
        self.assertEqual(len(printed), 6)
        for record in printed:
            payload = record["payload"]
            self.assertIn("candidate_id", payload)
            self.assertIn("rank", payload)
            self.assertIn("contact", payload)
            self.assertEqual(payload["model_version"], 1)
            self.assertEqual(payload["model_hash"], "abc123")
        # ranked order per detection
        self.assertEqual([record["payload"]["candidate_id"]
                          for record in printed[:2]],
                         ["C001_001", "C002_001"])


if __name__ == "__main__":
    unittest.main()
