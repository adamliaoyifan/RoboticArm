#!/usr/bin/env python3
"""[ST09-I S5.7 lock 2026-10-03] verify's map admission never falls back.

The re-verification fetches the LATEST map revision itself: a failed or
absent GetCargoMapStats fetch is the named refusal
MAP_STATUS_UNAVAILABLE and NEVER degrades into (a) a fabricated
revision 0 masquerading as MAP_RESTART or (b) a stale selection-time
carry-over nobody can still read. The request that does go out must
carry exactly the freshly fetched revision.
"""

import os
import sys
import unittest
from types import SimpleNamespace

_SCRIPTS = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from motion_occ_eval_driver import MotionOccDriver  # noqa: E402


class _Client(object):
    def wait_for_service(self, timeout_sec=None):
        return True


class _Driver(object):
    """Only the collaborators `_verify_selected_route` touches."""

    def __init__(self, stats_response, verify_response=None, with_stats=True):
        self._verify_route_client = _Client()
        self._stats = _Client() if with_stats else None
        self._frame_scene_revision = "scene-A"
        self._last_joint_state = None
        self._stats_response = stats_response
        self._verify_response = verify_response
        self.verify_requests = []

    def call_srv(self, client, request, timeout=None):
        from luggage_msgs.srv import (GetCargoMapStats,
                                      ValidateSelectedRoute)
        if isinstance(request, GetCargoMapStats.Request):
            return self._stats_response
        assert isinstance(request, ValidateSelectedRoute.Request)
        self.verify_requests.append(request)
        return self._verify_response

    _verify_selected_route = MotionOccDriver._verify_selected_route


def _ok_verify_response():
    return SimpleNamespace(
        valid=True, reason_code="", leg_digests=[],
        start_state_delta_rad=0.0, certification_id="cert-1",
        validated_map_revision=7)


def _selection_response():
    return SimpleNamespace(path_hash="hash-1", motion_segments=[])


class TestMapAdmissionNoStaleFallback(unittest.TestCase):

    def test_stats_fetch_failure_refuses_without_stale_fallback(self):
        driver = _Driver(stats_response=None)
        ok, reason = driver._verify_selected_route(
            _selection_response(), "req-1", {})
        self.assertFalse(ok)
        self.assertEqual(reason, "MAP_STATUS_UNAVAILABLE")
        # Fail-closed BEFORE any verification request: the route was
        # never verified against a fabricated or stale revision.
        self.assertEqual(driver.verify_requests, [])

    def test_stats_success_false_refuses_without_stale_fallback(self):
        driver = _Driver(
            stats_response=SimpleNamespace(success=False, map_revision=9))
        ok, reason = driver._verify_selected_route(
            _selection_response(), "req-1", {})
        self.assertFalse(ok)
        self.assertEqual(reason, "MAP_STATUS_UNAVAILABLE")
        self.assertEqual(driver.verify_requests, [])

    def test_absent_stats_client_refuses(self):
        driver = _Driver(stats_response=None, with_stats=False)
        ok, reason = driver._verify_selected_route(
            _selection_response(), "req-1", {})
        self.assertFalse(ok)
        self.assertEqual(reason, "MAP_STATUS_UNAVAILABLE")
        self.assertEqual(driver.verify_requests, [])

    def test_verify_request_carries_fetched_latest_revision(self):
        driver = _Driver(
            stats_response=SimpleNamespace(success=True, map_revision=7),
            verify_response=_ok_verify_response())
        ok, reason = driver._verify_selected_route(
            _selection_response(), "req-1", {})
        self.assertTrue(ok)
        self.assertEqual(reason, "")
        self.assertEqual(len(driver.verify_requests), 1)
        # The revision on the wire is the one JUST fetched — never a
        # fabricated 0 nor the selection-time snapshot.
        self.assertEqual(
            driver.verify_requests[0].latest_map_revision, 7)


if __name__ == "__main__":
    unittest.main()
