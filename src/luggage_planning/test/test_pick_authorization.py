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
    REOBSERVE,
    TERMINAL,
    WAIT,
    WAIT_FOR_FULL_3D,
    AuthorizationConfig,
    PickAuthorizationPolicy,
    pick_authorized,
    run_authorization_loop,
)


def _box(**fields):
    return SimpleNamespace(
        top_surface_valid=fields.get("top_surface_valid", True),
        height_valid=fields.get("height_valid", False),
        height_source=fields.get(
            "height_source", HEIGHT_SOURCE_UNAVAILABLE),
    )


class TestPickAuthorized(unittest.TestCase):

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


if __name__ == "__main__":
    unittest.main()
