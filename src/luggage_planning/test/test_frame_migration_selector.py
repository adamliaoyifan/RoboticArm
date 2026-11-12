#!/usr/bin/env python3
"""[ST09-I] frame-migration surface after the selection deletion.

The legacy in-process selection (trajectory_selector, path_candidates,
place_path_planner, place_selection_client, Section A of the graph
modules, build_place_legs_v2/select_place_v2) is gone: the unified
route producer behind SelectPlacePathV2 is the only place-motion
source. These tests pin the deletions and the surviving boundaries
(request admission, build refusals, pure transforms).
"""

from __future__ import division

import importlib
import math
import sys
import unittest
from pathlib import Path

TEST_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_DIR))
PKG_DIR = TEST_DIR.parent
SCRIPTS = PKG_DIR / "scripts"
for entry in (str(PKG_DIR), str(SCRIPTS)):
    if entry not in sys.path:
        sys.path.insert(0, entry)

from luggage_description.frame_contracts import ContractError  # noqa: E402
from luggage_planning import place_service  # noqa: E402
from luggage_planning import place_tcp  # noqa: E402

GRASP = {
    "contract_version": 2,
    "time_basis": "scene",
    "length_unit": "m",
    "reference": "suction_contact_frame",
    "orientation_semantics": "box",
    "position": [0.0, 0.0, 0.10],
    "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
    "scene_revision": "scene-1",
    "contract_version": 2,
    "request_id": "req-1",
    "frame_id": "suction_contact_frame",
    "stamp_ns": 1000,
}
SLOT = {
    "kind": "pose",
    "contract_version": 2,
    "time_basis": "scene",
    "frame_id": "container_link",
    "length_unit": "m",
    "request_id": "req-1",
    "scene_revision": "scene-1",
    "stamp_ns": 1000,
    "time_basis": "scene",
    "reference": "box_center",
    "orientation_semantics": "box_axis_pi",
    "position": [0.2, -0.1, 0.14],
    "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
}


class SelectionSurfaceRemoved(unittest.TestCase):
    def test_deleted_modules_are_gone(self):
        for name in ("luggage_planning.trajectory_selector",
                     "luggage_planning.path_candidates",
                     "luggage_planning.place_path_planner",
                     "luggage_planning.place_selection_client"):
            with self.assertRaises(ImportError):
                importlib.import_module(name)

    def test_deleted_entry_points_are_gone(self):
        import luggage_planning.place_tcp as tcp
        import luggage_planning.atlas.graph as graph
        import luggage_planning.routing.grid_search as search
        for module, names in (
                (place_service, ("fill_select_response", "handle_select_v2",
                                 "select_place_v2")),
                (tcp, ("select_place_v2", "build_place_legs_v2",
                       "build_motion_sequence_v2")),
                (graph, ("build_reachable_graph", "ReachableGraph",
                         "grid_disable_reason", "cell_center")),
                (search, ("search_paths", "PROFILES", "GridPath"))):
            for name in names:
                self.assertFalse(hasattr(module, name),
                                 "%s.%s" % (module.__name__, name))

    def test_route_fill_is_the_select_handler(self):
        self.assertTrue(hasattr(place_service, "fill_select_response_route"))


class BuildBoundaries(unittest.TestCase):
    def _build(self, phase):
        from luggage_msgs.msg import PoseContractV2

        slot_msg = place_service.pose_message(SLOT)
        grasp_msg = PoseContractV2()
        grasp_msg = place_service.pose_message(GRASP)
        request = type("Request", (), {
            "phase": phase, "request_id": "req-1",
            "scene_revision": "scene-1",
            "pick_pose": type("PickPose", (), {"identity": type(
                "Identity", (), {"contract_version": 0,
                                 "frame_id": ""})()})(),
            "pick": None, "slot": slot_msg, "grasp": grasp_msg,
            "box_wdh": [0.3, 0.2, 0.2],
            "container_source_frame": "container_link",
            "container_target_frame": "world",
            "container_translation": [0.0, 0.0, 0.0],
            "container_quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            "container_stamp_is_null": True, "container_stamp_ns": 0})()
        response = type("Response", (), {
            "motion_segments": ["x"], "segments": ["x"],
            "success": True, "message": "", "reason_code": "",
            "contact_pose": None})()
        return place_service.handle_build_v2(
            request, response, pick=None, pick_encoder=None)

    def test_place_build_refuses_named(self):
        response = self._build("place")
        self.assertFalse(response.success)
        self.assertEqual(response.reason_code, "PLANNING_ROUTE_REQUIRED")

    def test_pick_build_without_pick_refuses_named(self):
        # No pick observation at all is the earlier named refusal; the
        # encoder requirement fires once a measured pick exists.
        response = self._build("pick")
        self.assertFalse(response.success)
        self.assertEqual(response.reason_code, "MISSING_IDENTITY")


class PureTransforms(unittest.TestCase):
    def test_world_tcp_record_composes_grasp(self):
        transform = {
            "source_frame": "container_link", "target_frame": "world",
            "translation": [0.0, 0.0, 0.0],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            "stamp_ns": None, "scene_revision": "scene-1"}
        grasp = {"source_frame": "suction_contact_frame",
                 "target_frame": "box",
                 "translation": [0.0, 0.0, -0.10],
                 "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                 "scene_revision": "scene-1"}
        tcp = place_tcp.world_tcp_record(SLOT, grasp, transform)
        # grasp translation is T_box_tool: the tool sits 0.10 below the
        # box centre, so the world TCP pose composes exactly that offset.
        self.assertAlmostEqual(tcp["position"][2],
                                SLOT["position"][2] - 0.10, places=12)

    def test_lift_world_point_lifts_along_container_z(self):
        lifted = place_tcp.lift_world_point(
            (1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 0.10)
        self.assertAlmostEqual(lifted[0], 1.0, places=12)
        self.assertAlmostEqual(lifted[2], 0.10, places=12)


if __name__ == "__main__":
    unittest.main()
