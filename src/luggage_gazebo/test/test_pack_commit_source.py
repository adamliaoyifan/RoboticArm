#!/usr/bin/env python3
"""Commit-source fixes in pack_eval_driver (witness_02 root cause).

1. ``_place_slot_for`` must refresh ``_pending_placed`` when the Select
   winner differs from the pre-select compute preview — the cargo-map
   commit describes the EXECUTED slot, not a stale proposal (the
   0.65 m phantom footprint).
2. ``_frame_on_detected`` (compute preview) must NOT write
   ``_frame_slot_record``: the Select winner is its single writer.
3. ``_commit_anchor_kwargs`` ties the commit to the verify-measured pose
   (world -> container_link through the one shared transform).
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
        os.path.join(_ROOT, "src", "luggage_description")):
    if _entry not in sys.path:
        sys.path.append(_entry)

from luggage_planning.place_service import pose_message  # noqa: E402


def _load_pack_module():
    try:
        import rclpy  # noqa: F401
        import luggage_msgs.srv  # noqa: F401
    except ImportError:
        return None
    try:
        spec = importlib.util.spec_from_file_location(
            "pack_eval_driver_under_test",
            os.path.join(_SCRIPTS, "pack_eval_driver.py"))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    except Exception:  # noqa: BLE001 - peers of the script may be absent
        return None


_SCENE = "scene-1"
_REQUEST = "req-pack-1"


def _scene_config():
    return {
        "world_frame": "world",
        "static_transforms": [{
            "parent": "world",
            "child": "container_link",
            "translation": [1.0, 0.0, 0.5],
            "rotation_rpy": [0.0, 0.0, 0.0],
        }],
        "container": {"inner": {
            "length": 2.0, "width": 1.5, "height": 1.5,
            "floor_z": 0.0, "ceiling_z": 1.5,
        }},
        "robot": {"base_frame": "elfin_base_link"},
    }


def _slot_record():
    return dict(
        contract_version=2, frame_id="container_link", length_unit="m",
        stamp_ns=1_000_000_000, time_basis="scene",
        scene_revision=_SCENE, request_id=_REQUEST,
        reference="box_center", orientation_semantics="box_axis_pi",
        position=[0.20, -0.10, 0.14],
        quaternion_xyzw=[0.0, 0.0, 0.0, 1.0])


class _PreviewTyped(object):
    """compute_placement preview for _frame_on_detected."""

    def __init__(self, slot_message):
        self.slot_message = slot_message

    def compute_placement(self, **kwargs):
        return SimpleNamespace(
            success=True, message="ranked", reason_code="",
            slot=self.slot_message)


@unittest.skipIf(_load_pack_module() is None, "ROS driver deps unavailable")
class TestPackCommitSource(unittest.TestCase):

    def setUp(self):
        self.module = _load_pack_module()
        self.driver = self.module.PackEvalDriver.__new__(
            self.module.PackEvalDriver)
        self.driver._typed = object()
        self.driver._frame_scene_revision = _SCENE
        self.driver._surface_v2_msg = SimpleNamespace(
            map_revision=7, geometry_hash="gh-7")
        self.driver._surface_2d = {}
        self.driver._scene_config = _scene_config()

    # -- fix 1: select refreshes the pending commit record --------------

    def test_select_refreshes_pending_placed(self):
        stale = {"record": "preview-record", "size_wdh": [9.0, 9.0, 9.0],
                 "map_revision": 2, "geometry_hash": "old"}
        self.driver._pending_placed = stale
        select_record = {"request_id": "sel-1"}
        self.driver._frame_slot_record = select_record
        self.driver._frame_box_wdh = [0.51, 0.33, 0.22]
        slot = SimpleNamespace(width=1.0)
        meta = SimpleNamespace(pose_world={})
        self.driver._typed_place_slot = lambda pick, trial: (slot, meta)
        self.driver._pending_audit = {"preview": True}
        self.driver._pending_center_local = [9.0, 9.0, 9.0]
        self.driver._corridor_audit = (
            lambda s: ({"selected": True}, [0.2, -0.1, 0.14]))

        got_slot, _ = self.driver._place_slot_for(
            SimpleNamespace(), SimpleNamespace(),
            SimpleNamespace(extras={}, fail_code=""))

        self.assertIs(got_slot, slot)
        self.assertIs(self.driver._pending_placed["record"], select_record)
        self.assertEqual(self.driver._pending_placed["size_wdh"],
                         [0.51, 0.33, 0.22])
        self.assertEqual(self.driver._pending_placed["map_revision"], 7)
        self.assertEqual(self.driver._pending_placed["geometry_hash"],
                         "gh-7")
        # [local-ledger refresh] audit + container-local center follow the
        # Select winner too: _committed_ledger_boxes downstream must not
        # keep the preview slot's center.
        self.assertEqual(self.driver._pending_audit, {"selected": True})
        self.assertEqual(self.driver._pending_center_local,
                         [0.2, -0.1, 0.14])

    def test_no_selection_keeps_preview_record(self):
        """A refused Select leaves the provisional record untouched."""
        stale = {"record": "preview-record", "map_revision": 2}
        self.driver._pending_placed = stale
        self.driver._typed_place_slot = lambda pick, trial: (None, None)
        self.driver._place_slot_for(
            SimpleNamespace(), SimpleNamespace(),
            SimpleNamespace(extras={}, fail_code=""))
        self.assertIs(self.driver._pending_placed, stale)

    # -- fix 2: compute preview does not write the shared slot record ----

    def test_compute_preview_leaves_frame_slot_record_alone(self):
        self.driver._typed = _PreviewTyped(pose_message(_slot_record()))
        self.driver._frame_slot_record = None
        self.driver._frame_request_id = lambda phase: _REQUEST
        self.driver._corridor_audit = lambda slot: ({}, None)
        self.driver._slot_meta = lambda slot: {"pose_world": {}}
        trial = SimpleNamespace(extras={}, fail_code="")
        pick = SimpleNamespace(width=0.51, depth=0.33, height=0.22,
                               height_valid=True)

        reason = self.driver._frame_on_detected(pick, trial)

        self.assertEqual(reason, "")
        self.assertIsNone(self.driver._frame_slot_record)
        # The provisional commit record still exists (pre-select preview),
        # and it is a decoded container_link record.
        self.assertEqual(
            self.driver._pending_placed["record"]["position"],
            [0.20, -0.10, 0.14])
        self.assertEqual(self.driver._pending_placed["size_wdh"],
                         [0.51, 0.33, 0.22])

    # -- fix 3: commit anchor kwargs from the verify pose ----------------

    def test_commit_anchor_converts_verify_pose(self):
        record = SimpleNamespace(extras={"verify": {
            "box_pose_after_drift_wait": [1.2, -0.1, 0.64, 0.0, 0.0, 0.0]}})
        kwargs = self.driver._commit_anchor_kwargs(record)
        self.assertAlmostEqual(kwargs["verified_center_container"][0],
                               0.20, places=9)
        self.assertAlmostEqual(kwargs["verified_center_container"][1],
                               -0.10, places=9)
        self.assertAlmostEqual(kwargs["verified_center_container"][2],
                               0.14, places=9)
        self.assertEqual(kwargs["verify_tolerance_m"], 0.10)

    def test_commit_anchor_absent_without_verify(self):
        self.assertEqual(
            self.driver._commit_anchor_kwargs(SimpleNamespace(extras={})),
            {})
        self.assertEqual(
            self.driver._commit_anchor_kwargs(SimpleNamespace(
                extras={"verify": {"box_pose_after_drift_wait": None}})),
            {})
        self.assertEqual(
            self.driver._commit_anchor_kwargs(SimpleNamespace(
                extras={"verify": {"box_pose_after_drift_wait": [
                    float("nan"), 0.0, 0.5]}})),
            {})

    def test_witness02_geometry_is_caught(self):
        """The exact witness_02 geometry: asserted slot 0.65 m from the
        verified pose must exceed the commit tolerance."""
        from luggage_perception.cargo_volume_mapper import (
            verify_anchor_delta_m)
        delta = verify_anchor_delta_m(
            [0.53, -0.51, 0.639], [0.53, 0.14, 0.639])
        self.assertTrue(math.isclose(delta, 0.65, abs_tol=1e-6))
        self.assertGreater(delta, self.driver._COMMIT_VERIFY_TOLERANCE_M)


class _TypedAdd(object):
    """typed-adapter recorder for _typed_commit_add."""

    def __init__(self, response):
        self.response = response
        self.calls = []

    def add_placed_box(self, **kwargs):
        self.calls.append(kwargs)
        return self.response


@unittest.skipIf(_load_pack_module() is None, "ROS driver deps unavailable")
class TestTypedCommitAdd(unittest.TestCase):
    """[commit anchor + ledger convergence] Production refuses a commit
    without a verify anchor; the local placed ledger only mirrors ACCEPTED
    mapper verdicts."""

    def setUp(self):
        self.module = _load_pack_module()
        self.env = self.module.PackEvalDriver._COMMIT_ANCHOR_COMPAT_ENV
        self.driver = self.module.PackEvalDriver.__new__(
            self.module.PackEvalDriver)
        self.driver._typed = None
        self.driver._scene_config = _scene_config()
        self.pending = {"record": {"request_id": "sel-1"},
                        "size_wdh": [0.51, 0.33, 0.22],
                        "map_revision": 7, "geometry_hash": "gh-7"}

    def _env_ctx(self, compat):
        import unittest.mock as mock
        return mock.patch.dict(os.environ, {self.env: "1" if compat else ""},
                               clear=False)

    def test_missing_anchor_rejected_in_production(self):
        self.driver._typed = _TypedAdd(SimpleNamespace(
            success=True, message="committed"))
        record = SimpleNamespace(extras={})
        with self._env_ctx(compat=False):
            committed = self.driver._typed_commit_add(record, self.pending)
        self.assertFalse(committed)
        self.assertEqual(self.driver._typed.calls, [])
        self.assertTrue(record.extras["add_placed"].startswith(
            "COMMIT_ANCHOR_MISSING"))
        self.assertFalse(hasattr(self.driver, "_frame_placed_records"))

    def test_missing_anchor_compat_env_keeps_legacy(self):
        self.driver._typed = _TypedAdd(SimpleNamespace(
            success=True, message="committed"))
        record = SimpleNamespace(extras={})
        with self._env_ctx(compat=True):
            committed = self.driver._typed_commit_add(record, self.pending)
        self.assertTrue(committed)
        self.assertEqual(len(self.driver._typed.calls), 1)
        kwargs = self.driver._typed.calls[0]
        self.assertNotIn("verified_center_container", kwargs)
        self.assertNotIn("verify_tolerance_m", kwargs)
        self.assertEqual(self.driver._frame_placed_records, [{
            "record": self.pending["record"],
            "size_wdh": [0.51, 0.33, 0.22]}])

    def test_accepted_anchor_commit_passes_anchor(self):
        self.driver._typed = _TypedAdd(SimpleNamespace(
            success=True, message="committed"))
        record = SimpleNamespace(extras={"verify": {
            "box_pose_after_drift_wait": [1.2, -0.1, 0.64, 0, 0, 0]}})
        committed = self.driver._typed_commit_add(record, self.pending)
        self.assertTrue(committed)
        kwargs = self.driver._typed.calls[0]
        self.assertAlmostEqual(kwargs["verified_center_container"][0],
                               0.20, places=9)
        self.assertAlmostEqual(kwargs["verified_center_container"][1],
                               -0.10, places=9)
        self.assertEqual(kwargs["verify_tolerance_m"], 0.10)

    def test_rejected_service_response_leaves_local_ledger(self):
        """The mapper refused the commit (e.g. anchor gate): the driver's
        placed-record ledger must not gain the box."""
        self.driver._typed = _TypedAdd(SimpleNamespace(
            success=False,
            message="verify anchor mismatch: slot 0.650 m from measured "
                    "pose (tolerance 0.100 m)"))
        record = SimpleNamespace(extras={"verify": {
            "box_pose_after_drift_wait": [1.2, -0.1, 0.64, 0, 0, 0]}})
        committed = self.driver._typed_commit_add(record, self.pending)
        self.assertFalse(committed)
        self.assertIn("verify anchor mismatch",
                      record.extras["add_placed"])
        self.assertFalse(hasattr(self.driver, "_frame_placed_records"))


if __name__ == "__main__":
    unittest.main()
