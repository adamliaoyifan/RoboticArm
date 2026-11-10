#!/usr/bin/env python3
"""B3: route re-verification arms the per-leg execution admission.

`_verify_selected_admission` (wired into _typed_place_slot behind the
--verify-selected-route flag, default on for the pack driver) mirrors the
motion_occ reference: fail-closed stats fetch, digest match against the
legs this driver holds, and an admission record that handoff polling,
the release transition, and the executor's pre-submission gate read.
"""
import importlib.util
import os
import sys
import unittest
from types import SimpleNamespace

_SCRIPTS = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
_ROOT = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
for _entry in (
        _SCRIPTS,
        os.path.join(_ROOT, "src", "luggage_planning"),
        os.path.join(_ROOT, "src", "luggage_description"),
        os.path.join(_ROOT, "src", "luggage_perception")):
    if _entry not in sys.path:
        sys.path.append(_entry)


def _load_smoke_module():
    try:
        import rclpy  # noqa: F401
        import luggage_msgs.srv  # noqa: F401
    except ImportError:
        return None
    try:
        spec = importlib.util.spec_from_file_location(
            "place_smoke_under_test",
            os.path.join(_SCRIPTS, "place_smoke_driver.py"))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    except Exception:  # noqa: BLE001 - peers of the script may be absent
        return None


class _FakeClient(object):
    def __init__(self, result):
        self.result = result
        self.requests = []

    def wait_for_service(self, timeout_sec=None):
        return True

    def call_async(self, request):
        self.requests.append(request)
        return SimpleNamespace(
            done=lambda: True,
            result=lambda: self.result,
            add_done_callback=lambda cb: cb(None))


def _validate_result(valid=True, reason="", digests=("d1",),
                     states=("carrying",), cert="c1", revision=5):
    return SimpleNamespace(
        valid=valid, reason_code=reason, leg_digests=list(digests),
        leg_load_states=list(states), certification_id=cert,
        validated_map_revision=revision, start_state_delta_rad=0.0,
        certify_ms=1.0, verify_ms=2.0)


def _stats_result(success=True, revision=5):
    return SimpleNamespace(success=success, map_revision=revision,
                           map_source="measured")


@unittest.skipIf(_load_smoke_module() is None, "ROS driver deps unavailable")
class TestVerifySelectedAdmission(unittest.TestCase):

    def setUp(self):
        self.module = _load_smoke_module()
        self.driver = SimpleNamespace(
            _frame_scene_revision="scene-1",
            _last_joint_state={"position": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]},
            _execution_admission=None,
            _leg_trajectory_digest=lambda segment: "d1",
        )
        self.driver._call_srv_sync = (
            self.module.PlaceSmokeDriver._call_srv_sync.__get__(self.driver))

    def _wire(self, validate, stats):
        self.validate_client = _FakeClient(validate)
        self.stats_client = _FakeClient(stats)

        def create_client(_srv, name, callback_group=None):
            return (self.validate_client if "validate" in name
                    else self.stats_client)

        self.driver.create_client = create_client
        self.driver._verify_route_client = None
        self.driver._map_stats_client = None

    def _selection(self):
        return SimpleNamespace(
            path_hash="ph", motion_segments=[SimpleNamespace(name="s1")])

    def _verify(self):
        return self.module.PlaceSmokeDriver._verify_selected_admission(
            self.driver, self._selection(), "req-1")

    def test_success_arms_admission(self):
        self._wire(_validate_result(), _stats_result())
        ok, reason = self._verify()
        self.assertTrue(ok, reason)
        admission = self.driver._execution_admission
        self.assertEqual(admission["route_digest"], "ph")
        self.assertEqual(admission["certification_id"], "c1")
        self.assertEqual(admission["admission_map_revision"], 5)
        self.assertEqual(admission["leg_digests"], ["d1"])
        self.assertEqual(admission["leg_load_states"], ["carrying"])
        # The request pinned the LATEST map revision (fail-closed fetch).
        self.assertEqual(
            self.validate_client.requests[0].latest_map_revision, 5)

    def test_invalid_result_refuses_with_reason(self):
        self._wire(_validate_result(valid=False, reason="MAP_RESTART"),
                   _stats_result())
        ok, reason = self._verify()
        self.assertFalse(ok)
        self.assertEqual(reason, "MAP_RESTART")
        self.assertIsNone(self.driver._execution_admission)

    def test_stats_unavailable_never_falls_back(self):
        self._wire(_validate_result(), _stats_result(success=False))
        ok, reason = self._verify()
        self.assertFalse(ok)
        self.assertEqual(reason, "MAP_STATUS_UNAVAILABLE")
        self.assertEqual(self.validate_client.requests, [])

    def test_digest_mismatch_refuses(self):
        self._wire(_validate_result(digests=("other",)), _stats_result())
        ok, reason = self._verify()
        self.assertFalse(ok)
        self.assertEqual(reason, "LEG_DIGEST_MISMATCH")
        self.assertIsNone(self.driver._execution_admission)

    def test_per_leg_admission_resolves_scope(self):
        segments = [SimpleNamespace(name="s1"), SimpleNamespace(name="s2")]
        selection = SimpleNamespace(path_hash="ph",
                                    motion_segments=segments)
        self.driver._leg_trajectory_digest = (
            lambda segment: "d1" if segment is segments[0] else "d2")
        self._wire(_validate_result(
            digests=("d1", "d2"), states=("carrying", "unloaded")),
            _stats_result())
        ok, _ = self.module.PlaceSmokeDriver._verify_selected_admission(
            self.driver, selection, "req-1")
        self.assertTrue(ok)
        self.driver._frame_motion_segments = list(segments)
        self.driver._frame_motion_index = 1
        admission = self.module.PlaceSmokeDriver._leg_execution_admission(
            self.driver, segments[1])
        self.assertEqual(admission["leg_digest"], "d2")
        self.assertEqual(admission["load_state"], "unloaded")
        self.assertEqual(admission["certification_id"], "c1")
        carrying = self.module.PlaceSmokeDriver._leg_execution_admission(
            self.driver, segments[0])
        self.assertEqual(carrying["load_state"], "carrying")


def _load_pack_module():
    try:
        import rclpy  # noqa: F401
        import luggage_msgs.srv  # noqa: F401
    except ImportError:
        return None
    try:
        spec = importlib.util.spec_from_file_location(
            "pack_eval_under_test",
            os.path.join(_SCRIPTS, "pack_eval_driver.py"))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    except Exception:  # noqa: BLE001
        return None


@unittest.skipIf(_load_pack_module() is None, "ROS driver deps unavailable")
class TestAwaitMapConfirmed(unittest.TestCase):

    def setUp(self):
        self.module = _load_pack_module()

    def _driver(self, responses):
        calls = {"n": 0}
        driver = SimpleNamespace()

        class _Stats(object):
            def wait_for_service(self, timeout_sec=None):
                return True

        driver._map_stats = _Stats()

        def call(client, request, timeout=5.0):
            response = responses[min(calls["n"], len(responses) - 1)]
            calls["n"] += 1
            return response

        driver._call = call
        return driver, calls

    def test_confirmed_map_proceeds(self):
        driver, calls = self._driver([SimpleNamespace(
            success=True, occupancy_confirmed=True)])
        self.module.PackEvalDriver._await_map_confirmed(driver, timeout_s=1.0)
        self.assertEqual(
            self.module.PackEvalDriver._await_map_confirmed(
                driver, timeout_s=1.0), "")

    def test_unconfirmed_map_fails_closed_after_timeout(self):
        driver, calls = self._driver([SimpleNamespace(
            success=True, occupancy_confirmed=False)])
        self.assertEqual(
            self.module.PackEvalDriver._await_map_confirmed(
                driver, timeout_s=0.3), "OCCUPANCY_UNCONFIRMED")

    def test_missing_stats_service_skips(self):
        driver = SimpleNamespace()

        class _Absent(object):
            def wait_for_service(self, timeout_sec=None):
                return False

        driver._map_stats = _Absent()
        driver._call = lambda *a, **k: (_ for _ in ()).throw(
            AssertionError("no calls expected"))
        self.assertEqual(
            self.module.PackEvalDriver._await_map_confirmed(
                driver, timeout_s=0.5), "")


if __name__ == "__main__":
    unittest.main()
