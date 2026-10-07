"""The M3 driver's release commit to the mapper ledger.

Run 2 of the 2026-10-07 two-box session spent a live slot on a NameError in
this hook; it is exercised here with fakes so the path runs offline.
"""
import importlib.util
import os
import sys
import types
import unittest

SCRIPTS = os.path.join(os.path.dirname(__file__), "..", "scripts")


def _load_driver_module():
    try:
        import rclpy  # noqa: F401
        import luggage_msgs.srv  # noqa: F401
    except ImportError:
        return None
    sys.path.insert(0, SCRIPTS)
    try:
        spec = importlib.util.spec_from_file_location(
            "motion_occ_eval_driver_under_test",
            os.path.join(SCRIPTS, "motion_occ_eval_driver.py"))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    except Exception:  # noqa: BLE001 - peers of the script may be absent
        return None
    finally:
        sys.path.remove(SCRIPTS)


class FakeTyped:
    def __init__(self, success=True, raises=None):
        self.calls = []
        self._success = success
        self._raises = raises

    def add_placed_box(self, **kwargs):
        self.calls.append(kwargs)
        if self._raises:
            raise self._raises
        return types.SimpleNamespace(success=self._success, message="committed 1 boxes",
                                     resulting_map_revision=5)


@unittest.skipIf(_load_driver_module() is None, "ROS driver deps unavailable")
class TestCommitToCargoMap(unittest.TestCase):
    def _driver(self, commit_mode=True, typed=None, stats_ok=True):
        module = _load_driver_module()
        driver = module.MotionOccDriver.__new__(module.MotionOccDriver)
        driver._commit_mode = commit_mode
        driver._typed = typed
        driver._frame_slot_record = {"request_id": "r1"}
        driver._frame_box_wdh = [0.5, 0.4, 0.3]
        driver._stats = object()
        stats = types.SimpleNamespace(
            success=stats_ok, map_revision=4, geometry_hash="gh")
        driver.call_srv = lambda client, request, timeout=5.0: stats
        return driver

    def _trial(self):
        return types.SimpleNamespace(extras={})

    def test_flag_off_commits_nothing(self):
        typed = FakeTyped()
        driver = self._driver(commit_mode=False, typed=typed)
        self.assertTrue(driver._commit_to_cargo_map(self._trial(), None, None))
        self.assertEqual(typed.calls, [])

    def test_commit_uses_current_revision_hash_and_declared_source(self):
        typed = FakeTyped()
        driver = self._driver(typed=typed)
        trial = self._trial()
        self.assertTrue(driver._commit_to_cargo_map(trial, None, None))
        call = typed.calls[0]
        self.assertEqual(call["expected_map_revision"], 4)
        self.assertEqual(call["geometry_hash"], "gh")
        self.assertEqual(call["measured_source"], "verify_placed_box")
        self.assertEqual(call["size_wdh"], [0.5, 0.4, 0.3])
        self.assertEqual(trial.extras["cargo_map_commit"], "committed 1 boxes")
        self.assertEqual(trial.extras["cargo_map_commit_revisions"],
                         {"expected": 4, "resulting": 5})

    def test_refused_commit_fails_the_case(self):
        driver = self._driver(typed=FakeTyped(success=False))
        self.assertFalse(driver._commit_to_cargo_map(self._trial(), None, None))

    def test_contract_error_is_a_result_not_a_crash(self):
        driver = self._driver(typed=FakeTyped(raises=ValueError("stale")))
        trial = self._trial()
        self.assertFalse(driver._commit_to_cargo_map(trial, None, None))
        self.assertIn("rejected", trial.extras["cargo_map_commit"])

    def test_unavailable_stats_fails_the_case(self):
        driver = self._driver(typed=FakeTyped(), stats_ok=False)
        self.assertFalse(driver._commit_to_cargo_map(self._trial(), None, None))


if __name__ == "__main__":
    unittest.main()
