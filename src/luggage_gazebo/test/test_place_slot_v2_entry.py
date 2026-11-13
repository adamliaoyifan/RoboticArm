#!/usr/bin/env python3
"""The typed place entry must stay the one ``_place_slot_for`` that runs.

A second definition on the same class used to shadow this method and send
the carry back through the v1 slot picker (``fixed_floor_center``, no
SelectPlacePathV2). This test calls the live method with a stubbed v2
client and does not construct a ROS node.
"""

import os
import sys
import unittest
from types import SimpleNamespace

_SCRIPTS = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
_ROOT = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
for entry in (
        _SCRIPTS,
        os.path.join(_ROOT, "src", "luggage_planning"),
        os.path.join(_ROOT, "src", "luggage_description")):
    if entry not in sys.path:
        sys.path.append(entry)

from motion_occ_eval_driver import MotionOccDriver  # noqa: E402
from luggage_planning.place.place_service import pose_message  # noqa: E402


_SCENE = "scene-1"
_REQUEST = "req-place-1"


def _slot_message():
    return pose_message(dict(
        contract_version=2, frame_id="container_link", length_unit="m",
        stamp_ns=1_000_000_000, time_basis="scene",
        scene_revision=_SCENE, request_id=_REQUEST,
        reference="box_center", orientation_semantics="box_axis_pi",
        position=[0.20, -0.10, 0.14],
        quaternion_xyzw=[0.0, 0.0, 0.0, 1.0]))


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


class _Typed(object):
    def __init__(self, selection):
        self.selection = selection
        self.compute_calls = []
        self.select_calls = []

    def compute_placement(self, **kwargs):
        self.compute_calls.append(kwargs)
        return SimpleNamespace(
            success=True, message="ranked", reason_code="",
            candidates=[_slot_message()], candidate_scores=[0.4],
            slot=_slot_message())

    def select_place_path(self, **kwargs):
        self.select_calls.append(kwargs)
        return self.selection


class _Driver(object):
    def __init__(self, fault=""):
        self._args = SimpleNamespace(
            place_probe="off", rehearsal_fault=fault, rehearsal_dir="")
        self._typed = _Typed(SimpleNamespace(
            success=True, reason_code="", slot_index=0,
            path_source="route-graph", selection_id="route-1",
            path_hash="abc", stage_timings_json="{}",
            rows_json="[]", waypoint_poses=[],
            motion_segments=[SimpleNamespace(name="descend")],
            carry_segments=[SimpleNamespace(name="descend")]))
        self._surface_v2_msg = SimpleNamespace(
            geometry_hash="geo-1", map_revision=3)
        self._scene_config = _scene_config()
        self._frame_scene_revision = _SCENE
        self._frame_grasp = {
            "box_id": "box-1", "generation": 1, "request_id": _REQUEST}
        self._stats = object()
        self._last_joint_state = None
        self._selected_carry_v2 = None
        self.service_requests = []
        self.refused = []
        self.marks = []

    def call_srv(self, client, request, timeout=None):
        self.service_requests.append(type(request).__name__)
        return SimpleNamespace(
            geometry_hash="geo-1", map_revision=3, success=True)

    def _frame_request_id(self, phase):
        return _REQUEST

    def _portal_map(self):
        return [0.0, 0.0, 1.0]

    def _rehearsal_mark(self, name):
        self.marks.append(name)

    def _rehearsal_refuse(self, trial, phase):
        self.refused.append(phase)
        trial.fail_code = "MISSING_PEER"

    def _dump_selector(self, *args, **kwargs):
        return None

    def _slot_meta(self, slot):
        return {"planning_frame": "container_link"}

    def _planning_request_document(self, *args, **kwargs):
        return {}

    def _stage_timings_from_selection(self, selection):
        return {}

    _place_slot_for = MotionOccDriver._place_slot_for
    _place_slot_resolved = MotionOccDriver._place_slot_resolved
    _frame_place_slot = MotionOccDriver._frame_place_slot
    _rows_from_selection = staticmethod(MotionOccDriver._rows_from_selection)
    _surface_from_selection = staticmethod(
        MotionOccDriver._surface_from_selection)


def _pick():
    return SimpleNamespace(
        height_valid=True, width=0.40, depth=0.30, height=0.20)


class TestPlaceSlotV2Entry(unittest.TestCase):
    def test_place_slot_calls_select_place_path_v2(self):
        driver = _Driver()
        trial = SimpleNamespace(fail_code="", extras={})
        slot, _meta = driver._place_slot_for(_pick(), None, trial)
        self.assertIsNotNone(slot)
        self.assertEqual(len(driver._typed.select_calls), 1)
        self.assertEqual(len(driver._typed.compute_calls), 1)
        joined = " ".join(driver.service_requests)
        self.assertNotIn("ComputePlacement", joined)
        self.assertIn("GetCargoMapStats", joined)
        self.assertIn("selector", trial.extras)
        slot_pick = trial.extras.get("slot_pick") or {}
        self.assertNotEqual(slot_pick.get("source"), "fixed_floor_center")
        self.assertEqual(len(driver._selected_carry_v2), 1)

    def test_pre_place_refuses_before_v2_services(self):
        driver = _Driver(fault="pre_place")
        trial = SimpleNamespace(fail_code="", extras={})
        slot, meta = driver._place_slot_for(_pick(), None, trial)
        self.assertIsNone(slot)
        self.assertIsNone(meta)
        self.assertEqual(driver.refused, ["pre_place"])
        self.assertEqual(driver._typed.compute_calls, [])
        self.assertEqual(driver._typed.select_calls, [])
        self.assertEqual(driver.service_requests, [])
        self.assertNotIn("placement_request", driver.marks)


if __name__ == "__main__":
    unittest.main()


def test_rotated_container_slot_view_and_measured_crop_agree():
    import math
    from cargo_map_integrate_eval_driver import CargoMapIntegrateDriver
    driver = _Driver()
    driver._scene_config['static_transforms'][0]['rotation_rpy'][2] = math.pi / 2
    trial = SimpleNamespace(fail_code='', extras={})
    slot, _ = driver._place_slot_for(_pick(), None, trial)
    assert slot is not None
    yaw_base = 2 * math.atan2(slot.place_pose.orientation.z, slot.place_pose.orientation.w)
    assert abs(yaw_base - math.pi / 2) < 1e-9
    driver._release_physics = {'command_size': [9., 8., 7.]}
    center, size, yaw = CargoMapIntegrateDriver._placed_box_container(driver, slot, _pick())
    assert center == [0.2, -0.1, 0.14]
    assert size == [.4, .3, .2]
    assert abs(yaw) < 1e-9
