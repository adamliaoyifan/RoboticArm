#!/usr/bin/env python3
"""Release-failure contract at the driver (offline behavior tests).

Vacuum-off notifies the map (fail-closed); failing verifies keep the
evidence (poses + direction + verdict) on the shared dump path; the pack
suite retains a landed box instead of ClearCurrentBox.
"""
import importlib.util
import math
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


def _load(name, path):
    try:
        import rclpy  # noqa: F401
        import luggage_msgs.srv  # noqa: F401
    except ImportError:
        return None
    try:
        spec = importlib.util.spec_from_file_location(name, path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    except Exception:  # noqa: BLE001
        return None


def _smoke_module():
    return _load("place_smoke_rfc", os.path.join(
        _SCRIPTS, "place_smoke_driver.py"))


def _pack_module():
    return _load("pack_eval_rfc", os.path.join(
        _SCRIPTS, "pack_eval_driver.py"))


class _Scene(object):
    """Minimal scene-config stand-in for the world->base converters."""
    @staticmethod
    def world_from_container():
        return {"translation": [0.0, 0.0, 0.0], "rotation_rpy": [0, 0, 0]}


@unittest.skipIf(_smoke_module() is None, "ROS driver deps unavailable")
class TestNotifyBoxReleased(unittest.TestCase):

    def setUp(self):
        self.module = _smoke_module()
        self.driver = SimpleNamespace(
            _frame_slot_record={
                "position": [0.2, -0.1, 0.14],
                "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
            _frame_box_wdh=[0.51, 0.33, 0.22],
            _note_event_client=None,
        )
        self.driver._call_srv_sync = (
            self.module.PlaceSmokeDriver._call_srv_sync.__get__(self.driver))

    def _wire(self, response):
        client = SimpleNamespace(
            calls=[],
            wait_for_service=lambda timeout_sec=None: True)

        def call_async(request):
            client.calls.append(request)
            return SimpleNamespace(
                done=lambda: True, result=lambda: response,
                add_done_callback=lambda cb: cb(None))

        client.call_async = call_async
        self.driver.create_client = lambda srv, name: client
        return client

    def test_notification_carries_commanded_footprint(self):
        client = self._wire(SimpleNamespace(
            status="accepted", reason="", epoch="e9" * 16,
            content_digest="d9", occupancy_confirmed=False))
        ok, message = self.module.PlaceSmokeDriver._notify_box_released(
            self.driver)
        self.assertTrue(ok, message)
        request = client.calls[0]
        self.assertEqual(request.event, "box_released")
        self.assertEqual(list(request.box_center_container),
                         [0.2, -0.1, 0.14])
        self.assertEqual(list(request.box_size_wdh), [0.51, 0.33, 0.22])
        self.assertEqual(request.box_yaw, 0.0)

    def test_service_unavailable_fails_closed(self):
        client = self._wire(None)
        client.wait_for_service = lambda timeout_sec=None: False
        ok, message = self.module.PlaceSmokeDriver._notify_box_released(
            self.driver)
        self.assertFalse(ok)

    def test_rejected_status_fails_closed(self):
        self._wire(SimpleNamespace(
            status="rejected", reason="EVIDENCE_MISSING",
            epoch="", content_digest="", occupancy_confirmed=True))
        ok, message = self.module.PlaceSmokeDriver._notify_box_released(
            self.driver)
        self.assertFalse(ok)
        self.assertIn("rejected", message)

    def test_missing_footprint_fails_closed(self):
        self.driver._frame_box_wdh = []
        ok, _ = self.module.PlaceSmokeDriver._notify_box_released(
            self.driver)
        self.assertFalse(ok)


@unittest.skipIf(_smoke_module() is None, "ROS driver deps unavailable")
class TestVerifyEvidenceBeforeGates(unittest.TestCase):

    def setUp(self):
        self.module = _smoke_module()
        self.driver = SimpleNamespace(
            _scene_config={
                "world_frame": "world",
                "static_transforms": [
                    {"parent": "world", "child": "elfin_base_link",
                     "translation": [0.0, 0.0, 0.0],
                     "rotation_rpy": [0.0, 0.0, 0.0]},
                    {"parent": "world", "child": "container_link",
                     "translation": [0.0, 0.0, 0.0],
                     "rotation_rpy": [0.0, 0.0, 0.0]},
                ],
                "robot": {"base_frame": "elfin_base_link"},
                "container": {"inner": {
                    "length": 2.0, "width": 1.5, "height": 1.5,
                    "floor_z": 0.0, "ceiling_z": 1.5}},
            })
        self.trial = SimpleNamespace(
            extras={"verify": {
                "box_pose_after_release": None,
                "box_pose_after_drift_wait": None,
                "box_pose_at_release": None,
                "verdict": "pending"}}, err_xy=None, err_z=None,
            drift=None, roll=None, pitch=None, inside_inner_box=None)
        self.evaluate = (
            self.module.PlaceSmokeDriver._evaluate_place_verify.__get__(
                self.driver))

    def test_failing_verify_keeps_full_evidence(self):
        """[witness r10 lesson] The failing path must persist exactly the
        evidence the passing path does: poses, direction, errors. The two
        poses are the post-release settle measurements (after_release,
        after_drift_wait) — near-identical once the box is static."""
        expected = (0.10, 0.10, 0.15)
        settled = [0.1428, 0.0827, 0.1503, 0.0, -0.000004, 1.5891]
        settled_later = [0.1428, 0.0827, 0.15031, 0.0, -0.000004, 1.5891]
        # Mirror the production caller: the poses land in the shared
        # evidence dict BEFORE the gate evaluation.
        self.trial.extras["verify"]["box_pose_after_release"] = settled
        self.trial.extras["verify"]["box_pose_after_drift_wait"] = settled_later
        gate = self.evaluate(
            self.trial, expected, settled, pose_release=settled,
            pose_drift=settled_later)
        self.assertEqual(gate, "PLACE_VERIFY_XY")
        evidence = self.trial.extras["verify"]
        self.assertAlmostEqual(
            evidence["err_xy"], math.hypot(0.0428, -0.0173), places=6)
        # The direction the witness could not recover, now always there.
        self.assertAlmostEqual(
            evidence["release_drift_direction_world"][0], 0.0428, places=6)
        self.assertAlmostEqual(
            evidence["release_drift_direction_world"][1], -0.0173, places=6)
        self.assertEqual(evidence["box_pose_after_drift_wait"], settled_later)
        self.assertEqual(evidence["box_pose_after_release"], settled)

    def test_passing_verify_returns_none_with_same_fields(self):
        expected = (0.10, 0.10, 0.15)
        settled = [0.1005, 0.1005, 0.1501, 0.0, 0.0, 1.5707]
        settled_later = [0.10051, 0.10051, 0.1501, 0.0, 0.0, 1.5707]
        gate = self.evaluate(
            self.trial, expected, settled, pose_release=settled,
            pose_drift=settled_later)
        self.assertIsNone(gate)
        self.assertAlmostEqual(
            self.trial.extras["verify"]["err_xy"], math.hypot(0.0005, 0.0005),
            places=6)


@unittest.skipIf(_pack_module() is None, "ROS driver deps unavailable")
class TestLandedBoxRetention(unittest.TestCase):

    def setUp(self):
        self.module = _pack_module()
        self.calls = []
        self.logs = []
        self.driver = SimpleNamespace(
            _frame_released=False,
            _clear=SimpleNamespace(),
            _clear_request=lambda: "CLEAR",
            _call=lambda client, request, timeout=None: self.calls.append(
                (request, timeout)),
            get_logger=lambda: SimpleNamespace(error=self.logs.append))

    def _cleanup(self):
        return self.module.PackEvalDriver._failure_cleanup(
            self.driver, 1, "PLACE_VERIFY_XY")

    def test_pre_release_failure_clears(self):
        self.assertEqual(self._cleanup(), "cleared")
        self.assertEqual(self.calls, [("CLEAR", 15.0)])

    def test_post_release_failure_retains(self):
        self.driver._frame_released = True
        self.assertEqual(self._cleanup(), "retained")
        self.assertEqual(self.calls, [])
        self.assertTrue(any("retained" in line for line in self.logs))


if __name__ == "__main__":
    unittest.main()


@unittest.skipIf(_smoke_module() is None, "ROS driver deps unavailable")
class TestReturnObserveEvidence(unittest.TestCase):
    def _host(self, error=None, ok=True):
        from copy import deepcopy
        host = SimpleNamespace(
            _place_state="VERIFIED", _args=SimpleNamespace(plan_only=False),
            _exit_yaw={}, dumps=[], _frame_runtime_handle=SimpleNamespace(snapshot=lambda: {"peer": "driver"}))
        def dump(trial, **kwargs):
            host.dumps.append(deepcopy(trial))
        def home():
            if error is not None:
                raise error
            return ok, "home verdict", None
        host._dump_place = dump
        host._set_state = lambda state: setattr(host, "_place_state", state)
        host._home_arm = home
        host.home_arm_code = lambda: "EXIT_FAILED"
        trial = SimpleNamespace(extras={"release_kinematic": {"verdict": "declared_xy_confirmed"},
                                       "verify": {"verdict": "ok"}}, fail_code="", place_state="")
        return host, trial

    def test_contract_exception_preserves_successful_upstream_and_failure(self):
        from luggage_description.frame_contracts import ContractError
        host, trial = self._host(ContractError("PEER_LEASE_EXPIRED", "peer", "fresh", "stale"))
        _smoke_module().PlaceSmokeDriver._return_observe_with_evidence(host, trial, {}, None)
        self.assertEqual(len(host.dumps), 2)
        self.assertEqual(host.dumps[0].place_state, "VERIFIED")
        self.assertEqual(host.dumps[0].extras["verify"]["verdict"], "ok")
        self.assertEqual(host.dumps[1].extras["return_exception"]["code"], "PEER_LEASE_EXPIRED")
        self.assertEqual(trial.fail_code, "RETURN_OBSERVE_EXCEPTION")

    def test_unexpected_exception_is_persisted_before_reraise(self):
        host, trial = self._host(RuntimeError("injected"))
        with self.assertRaisesRegex(RuntimeError, "injected"):
            _smoke_module().PlaceSmokeDriver._return_observe_with_evidence(host, trial, {}, None)
        self.assertEqual(len(host.dumps), 2)
        self.assertEqual(host.dumps[-1].extras["return_exception"]["type"], "RuntimeError")

    def test_return_verdict_and_scope_evidence_are_retained(self):
        for ok in (True, False):
            host, trial = self._host(ok=ok)
            _smoke_module().PlaceSmokeDriver._return_observe_with_evidence(host, trial, {}, None)
            self.assertEqual(trial.extras["return_observe"], "home verdict")
            self.assertIn("return_readiness_before", trial.extras)
            self.assertIn("return_readiness_after", trial.extras)
            self.assertEqual(trial.fail_code, "" if ok else "EXIT_FAILED")
