#!/usr/bin/env python3
"""Pick authorization: measured FULL_3D or wait / fail closed."""

import unittest
from types import SimpleNamespace

from luggage_planning.pick_authorization import (
    AUTHORIZE,
    DETECT_FAILED,
    DETECT_FULL_GEOMETRY_REQUIRED,
    HEIGHT_SOURCE_CATALOG_PRIOR,
    HEIGHT_SOURCE_CONFIGURED_SUPPORT,
    HEIGHT_SOURCE_MEASURED_SUPPORT,
    HEIGHT_SOURCE_UNAVAILABLE,
    HOLD_TRACK_SUPPORT,
    REOBSERVE,
    TERMINAL,
    WAIT,
    WAIT_FOR_FULL_3D,
    AuthorizationConfig,
    PickAuthorizationPolicy,
    authorized_pick_boundary,
    is_continuity_hold,
    pick_authorized,
    run_authorization_loop,
)


def _box(**fields):
    box = SimpleNamespace(
        top_surface_valid=fields.get("top_surface_valid", True),
        height_valid=fields.get("height_valid", False),
        height_source=fields.get(
            "height_source", HEIGHT_SOURCE_UNAVAILABLE),
    )
    for name in (
            "support_gate", "support_reason", "pca_source", "pca_reason",
            "source"):
        if name in fields:
            setattr(box, name, fields[name])
    return box


def _hold_reason():
    return (
        "perception estimate (conf=1.00, height_valid=False, "
        "support=%s, gate=hold_track, sides=0.0, raw=hit)"
        % HOLD_TRACK_SUPPORT)


def _measure_ok_reason():
    return (
        "perception estimate (conf=1.00, height_valid=True, "
        "support=ok, gate=n/a, sides=0.80, raw=hit)")


def _unstable_reason():
    return (
        "perception estimate (conf=1.00, height_valid=False, "
        "support=DETECT_SUPPORT_UNSTABLE, gate=n/a, sides=0.10, raw=hit)")


class TestPickAuthorized(unittest.TestCase):

    def test_hold_only_measure_does_not_authorize_with_a_synthesized_id(self):
        # Fresh depth and a passing support check are not a new observation
        # when the only detection is a temporal hold. The instance id only
        # separates points.
        from luggage_perception.semantic_segmenter import (
            LABEL_CARGO, observation_source_for_detections)
        hold = {
            "label": LABEL_CARGO, "prompt": "temporal_hold", "held": True,
            "instance_id": 7, "instance_source": "temporal_hold",
            "source_stamp": 4.752,
            "target_id": "pickup_box_0001_standard",
        }
        source = observation_source_for_detections([hold], "measure")
        self.assertEqual(source, "hold_track")
        box = _box(
            height_valid=True,
            height_source=HEIGHT_SOURCE_MEASURED_SUPPORT,
            pca_source=source,
            support_reason="ok",
            support_gate="n/a")
        reason = (
            "perception estimate (conf=1.00, height_valid=True, "
            "support=ok, gate=n/a, source=hold_track)")
        self.assertTrue(is_continuity_hold(box, reason))
        self.assertFalse(pick_authorized(box, reason))

    def test_unbound_measure_does_not_authorize_even_when_support_passed(self):
        box = _box(
            height_valid=True,
            height_source=HEIGHT_SOURCE_MEASURED_SUPPORT,
            pca_source="measure",
            pca_reason="DETECT_TARGET_ASSOCIATION_AMBIGUOUS")
        self.assertFalse(pick_authorized(
            box, "DETECT_TARGET_ASSOCIATION_AMBIGUOUS"))

    def test_later_bound_frame_authorizes_inside_the_same_window(self):
        # The wide frame would have passed support. It is refused. The next
        # call is a different frame that is bound and support-valid. The
        # earlier unstable 0.630 m frame is not offered to the loop.
        samples = [
            (False, None, "DETECT_TARGET_ASSOCIATION_AMBIGUOUS"),
            (True, _box(
                height_valid=True,
                height_source=HEIGHT_SOURCE_MEASURED_SUPPORT,
                pca_source="measure"), _measure_ok_reason()),
        ]
        clock = {"t": 0.0}

        def detect():
            return samples.pop(0)

        def sleep(dt):
            clock["t"] += float(dt)

        result = run_authorization_loop(
            detect, sleep, lambda: clock["t"],
            policy=PickAuthorizationPolicy(AuthorizationConfig(
                max_elapsed_sec=5.0, wait_period_sec=0.5, max_attempts=11)))
        self.assertTrue(result.authorized)
        self.assertEqual(result.attempts, 2)
        self.assertEqual(result.trace[0]["action"], WAIT)
        self.assertEqual(
            result.trace[0]["reason"], "DETECT_TARGET_ASSOCIATION_AMBIGUOUS")
        self.assertLess(result.decision.elapsed_sec, 5.0)

    def test_window_exhaustion_stops_under_the_association_name(self):
        # Every sample is support-valid and still unbound. The loop must
        # not authorize, and the stop reason stays the association name.
        clock = {"t": 0.0}
        box = _box(
            height_valid=True,
            height_source=HEIGHT_SOURCE_MEASURED_SUPPORT,
            pca_source="measure",
            pca_reason="DETECT_TARGET_ASSOCIATION_AMBIGUOUS")

        def detect():
            return True, box, _measure_ok_reason()

        def sleep(dt):
            clock["t"] += float(dt)

        result = run_authorization_loop(
            detect, sleep, lambda: clock["t"],
            policy=PickAuthorizationPolicy(AuthorizationConfig(
                max_elapsed_sec=5.0, wait_period_sec=0.5, max_attempts=40)))
        self.assertFalse(result.authorized)
        self.assertEqual(result.decision.action, TERMINAL)
        self.assertEqual(
            result.decision.reason, "DETECT_TARGET_ASSOCIATION_AMBIGUOUS")
        self.assertGreaterEqual(result.decision.elapsed_sec, 5.0)

    def test_measured_support_authorizes(self):
        self.assertTrue(pick_authorized(_box(
            height_valid=True,
            height_source=HEIGHT_SOURCE_MEASURED_SUPPORT)))

    def test_top_only_does_not_authorize(self):
        self.assertFalse(pick_authorized(_box(
            height_valid=False,
            height_source=HEIGHT_SOURCE_UNAVAILABLE)))

    def test_configured_never_authorizes(self):
        self.assertFalse(pick_authorized(_box(
            height_valid=True,
            height_source=HEIGHT_SOURCE_CONFIGURED_SUPPORT)))

    def test_catalog_never_authorizes(self):
        self.assertFalse(pick_authorized(_box(
            height_valid=True,
            height_source=HEIGHT_SOURCE_CATALOG_PRIOR)))

    def test_missing_top_does_not_authorize(self):
        self.assertFalse(pick_authorized(_box(
            top_surface_valid=False,
            height_valid=True,
            height_source=HEIGHT_SOURCE_MEASURED_SUPPORT)))

    def test_none_does_not_authorize(self):
        self.assertFalse(pick_authorized(None))

    def test_hold_track_fields_never_authorize(self):
        box = _box(
            height_valid=True,
            height_source=HEIGHT_SOURCE_MEASURED_SUPPORT,
            pca_source="hold_track",
            support_gate="hold_track",
            support_reason=HOLD_TRACK_SUPPORT)
        self.assertTrue(is_continuity_hold(box))
        self.assertFalse(pick_authorized(box))

    def test_hold_track_message_never_authorizes(self):
        box = _box(
            height_valid=True,
            height_source=HEIGHT_SOURCE_MEASURED_SUPPORT)
        self.assertTrue(is_continuity_hold(box, _hold_reason()))
        self.assertFalse(pick_authorized(box, _hold_reason()))


class TestPickAuthorizationPolicy(unittest.TestCase):

    def setUp(self):
        self.policy = PickAuthorizationPolicy(AuthorizationConfig(
            max_attempts=3, max_elapsed_sec=2.0, wait_period_sec=0.1))

    def test_full_3d_authorizes(self):
        decision = self.policy.evaluate(
            True, _box(height_valid=True,
                       height_source=HEIGHT_SOURCE_MEASURED_SUPPORT),
            elapsed_sec=0.0, attempts=1)
        self.assertEqual(decision.action, AUTHORIZE)
        self.assertEqual(decision.reason, "ok")

    def test_top_only_waits_within_budget(self):
        decision = self.policy.evaluate(
            True, _box(), elapsed_sec=0.1, attempts=1)
        self.assertEqual(decision.action, WAIT)
        self.assertEqual(decision.reason, WAIT_FOR_FULL_3D)

    def test_top_only_exhaust_emits_reobserve(self):
        decision = self.policy.evaluate(
            True, _box(), elapsed_sec=3.0, attempts=3)
        self.assertEqual(decision.action, REOBSERVE)
        self.assertEqual(decision.reason, DETECT_FULL_GEOMETRY_REQUIRED)

    def test_configured_is_terminal_without_waiting(self):
        decision = self.policy.evaluate(
            True, _box(height_valid=True,
                       height_source=HEIGHT_SOURCE_CONFIGURED_SUPPORT),
            elapsed_sec=0.0, attempts=1)
        self.assertEqual(decision.action, TERMINAL)
        self.assertEqual(decision.reason, DETECT_FULL_GEOMETRY_REQUIRED)

    def test_catalog_is_terminal_without_waiting(self):
        decision = self.policy.evaluate(
            True, _box(height_valid=True,
                       height_source=HEIGHT_SOURCE_CATALOG_PRIOR),
            elapsed_sec=0.0, attempts=1)
        self.assertEqual(decision.action, TERMINAL)
        self.assertEqual(decision.reason, DETECT_FULL_GEOMETRY_REQUIRED)

    def test_detect_fail_waits_then_terminals(self):
        waiting = self.policy.evaluate(
            False, None, elapsed_sec=0.0, attempts=1,
            detect_reason="DETECT_NO_CLOUD")
        self.assertEqual(waiting.action, WAIT)
        done = self.policy.evaluate(
            False, None, elapsed_sec=3.0, attempts=3,
            detect_reason="DETECT_NO_CLOUD")
        self.assertEqual(done.action, TERMINAL)
        self.assertEqual(done.reason, "DETECT_NO_CLOUD")

    def test_detect_fail_without_reason_uses_detect_failed(self):
        done = self.policy.evaluate(
            False, None, elapsed_sec=3.0, attempts=3)
        self.assertEqual(done.reason, DETECT_FAILED)


class TestAuthorizationLoop(unittest.TestCase):

    def test_second_attempt_authorizes(self):
        boxes = [
            _box(),
            _box(height_valid=True,
                 height_source=HEIGHT_SOURCE_MEASURED_SUPPORT),
        ]
        sleeps = []
        clock = {"t": 0.0}

        def detect():
            box = boxes.pop(0)
            return True, box, "perception estimate"

        def sleep(period):
            sleeps.append(period)
            clock["t"] += period

        policy = PickAuthorizationPolicy(AuthorizationConfig(
            max_attempts=4, max_elapsed_sec=5.0, wait_period_sec=0.25))
        result = run_authorization_loop(
            detect, sleep, lambda: clock["t"], policy)
        self.assertTrue(result.authorized)
        self.assertEqual(result.decision.action, AUTHORIZE)
        self.assertEqual(len(result.trace), 2)
        self.assertEqual(result.trace[0]["action"], WAIT)
        self.assertEqual(sleeps, [0.25])

    def test_exhaust_records_reobserve_and_does_not_authorize(self):
        clock = {"t": 0.0}

        def detect():
            return True, _box(), "perception estimate (height_valid=False)"

        def sleep(period):
            clock["t"] += period

        policy = PickAuthorizationPolicy(AuthorizationConfig(
            max_attempts=2, max_elapsed_sec=10.0, wait_period_sec=0.1))
        result = run_authorization_loop(
            detect, sleep, lambda: clock["t"], policy)
        self.assertFalse(result.authorized)
        self.assertEqual(result.decision.action, REOBSERVE)
        self.assertEqual(len(result.trace), 2)


class TestHoldTrackWait(unittest.TestCase):
    """Continuity frames wait; only a fresh measured FULL_3D authorizes."""

    def _run(self, samples, max_elapsed, period=0.5, max_attempts=11):
        pending = list(samples)
        clock = {"t": 0.0}

        def detect():
            return pending.pop(0)

        def sleep(wait):
            clock["t"] += wait

        policy = PickAuthorizationPolicy(AuthorizationConfig(
            max_attempts=max_attempts,
            max_elapsed_sec=max_elapsed,
            wait_period_sec=period))
        return run_authorization_loop(
            detect, sleep, lambda: clock["t"], policy)

    def test_hold_then_measure_authorizes(self):
        measured = _box(
            height_valid=True,
            height_source=HEIGHT_SOURCE_MEASURED_SUPPORT,
            pca_source="measure",
            support_reason="ok")
        result = self._run([
            (True, _box(pca_source="hold_track",
                        support_gate="hold_track",
                        support_reason=HOLD_TRACK_SUPPORT),
             _hold_reason()),
            (True, _box(pca_source="hold_track",
                        support_gate="hold_track",
                        support_reason=HOLD_TRACK_SUPPORT),
             _hold_reason()),
            (True, measured, _measure_ok_reason()),
        ], max_elapsed=5.0)
        self.assertTrue(result.authorized)
        self.assertEqual(result.decision.reason, "ok")
        self.assertEqual(result.hold_track_skipped, 2)
        self.assertEqual(result.attempts, 3)
        self.assertAlmostEqual(result.first_measure_latency_sec, 1.0)
        self.assertEqual(result.last_gate, "measure")
        self.assertAlmostEqual(result.last_measure_age_sec, 0.0)
        self.assertIs(result.box, measured)

    def test_hold_only_times_out_full_geometry(self):
        hold = (
            True,
            _box(pca_source="hold_track", support_gate="hold_track",
                 support_reason=HOLD_TRACK_SUPPORT),
            _hold_reason(),
        )
        result = self._run([hold] * 8, max_elapsed=1.0, period=0.5)
        self.assertFalse(result.authorized)
        self.assertEqual(
            result.decision.reason, DETECT_FULL_GEOMETRY_REQUIRED)
        self.assertGreaterEqual(result.hold_track_skipped, 1)
        self.assertIsNone(result.first_measure_latency_sec)
        self.assertEqual(result.last_gate, "hold_track")

    def test_unstable_measure_is_not_authorized(self):
        unstable = (
            True,
            _box(height_valid=False,
                 height_source=HEIGHT_SOURCE_UNAVAILABLE,
                 pca_source="measure",
                 support_reason="DETECT_SUPPORT_UNSTABLE"),
            _unstable_reason(),
        )
        result = self._run([unstable] * 8, max_elapsed=1.0, period=0.5)
        self.assertFalse(result.authorized)
        self.assertEqual(
            result.decision.reason, DETECT_FULL_GEOMETRY_REQUIRED)
        self.assertEqual(result.hold_track_skipped, 0)
        self.assertIsNotNone(result.first_measure_latency_sec)
        self.assertEqual(result.last_gate, "measure")


class TestAuthorizedPickBoundary(unittest.TestCase):
    def test_records_agreement_without_replacing_size(self):
        record = authorized_pick_boundary(
            {"stamp_sec": 130.285, "size_wdh": [1.256, 0.415, 0.277],
             "pca_source": ""},
            authorized=True,
            detect_reason="support=ok",
            observation_gate_name="measure",
            selected_frame={
                "stamp": 130.285, "pca_source": "measure", "selected": True,
                "size_wdh": [1.256, 0.415, 0.277],
            },
            scene_size=[1.256, 0.415, 0.277],
        )
        self.assertTrue(record["scene_size_matches_pick"])
        self.assertTrue(record["selected_frame_size_matches_pick"])
        self.assertTrue(record["selected_frame_stamp_matches_pick"])
        self.assertEqual(record["selected_frame"]["pca_source"], "measure")
        self.assertFalse(record["authorizing_message_has_pca_source"])
        self.assertEqual(record["pick"]["size_wdh"][0], 1.256)

    def test_names_a_different_frame_when_the_window_row_disagrees(self):
        record = authorized_pick_boundary(
            {"stamp_sec": 130.285, "size_wdh": [0.64, 0.40, 0.28],
             "pca_source": ""},
            authorized=True, detect_reason="", observation_gate_name="measure",
            selected_frame={
                "stamp": 129.691, "pca_source": "hold_track",
                "size_wdh": [1.256, 0.415, 0.277],
            },
            scene_size=[0.64, 0.40, 0.28],
        )
        self.assertFalse(record["selected_frame_stamp_matches_pick"])
        self.assertFalse(record["selected_frame_size_matches_pick"])
        self.assertTrue(record["scene_size_matches_pick"])
        self.assertEqual(record["selected_frame"]["pca_source"], "hold_track")


if __name__ == "__main__":
    unittest.main()
