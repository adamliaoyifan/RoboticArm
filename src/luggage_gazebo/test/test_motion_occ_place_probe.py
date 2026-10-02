#!/usr/bin/env python3

"""Selection-stage row probe in motion_occ_eval_driver.

``_place_probe_fn`` must mirror the traverse segment the winner executes
(cartesian, tool-down at the slot's place yaw, OMPL fallback allowed),
write back ``ik_ok``/``cartesian_fraction`` for PlacePathPlanner's gate,
reject rows on an unavailable service, and stop probing after the circuit
breaker trips. ``parse_args`` owns the ``--place-probe`` switch without
leaking it into the shared place parser.
"""

import os
import sys
import unittest

_SCRIPTS = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from geometry_msgs.msg import Quaternion  # noqa: E402
from luggage_msgs.msg import SlotSpec  # noqa: E402
from luggage_msgs.srv import ProbeMotionSegment  # noqa: E402
from luggage_planning.place_path_planner import PlacePathPlanner  # noqa: E402

import motion_occ_eval_driver as mod  # noqa: E402


def _slot(yaw_quat=(0.0, 0.0, 0.0, 1.0)):
    slot = SlotSpec()
    quat = Quaternion()
    quat.x, quat.y, quat.z, quat.w = yaw_quat
    slot.place_pose.orientation = quat
    return slot


class _FakeClient(object):
    def __init__(self, up=True):
        self.up = up

    def wait_for_service(self, timeout_sec=1.0):
        return self.up


def _driver(responses):
    """Driver shell exposing only what ``_place_probe_fn`` touches."""
    driver = mod.MotionOccDriver.__new__(mod.MotionOccDriver)
    driver._scene_config = {"scene": "unused"}
    driver._planner = PlacePathPlanner()
    driver._probe_place = _FakeClient()
    calls = []

    def call_srv(_client, request, timeout):
        calls.append(request)
        return responses.pop(0) if responses else None

    driver.call_srv = call_srv
    return driver, calls


class PlaceProbeFnTest(unittest.TestCase):
    def setUp(self):
        # Identity map->world keeps the assertions on wiring, not frame math.
        self._orig_map_to_world = mod._map_to_world
        mod._map_to_world = lambda _scene, xyz: list(xyz)
        self.addCleanup(setattr, mod, "_map_to_world", self._orig_map_to_world)

    def _row(self):
        return {
            "slot_index": 0,
            "method": "corridor",
            "waypoints": [[0.1, 0.2, 0.9], [0.3, 0.2, 0.9], [0.5, 0.2, 0.8]],
            "feasible": True,
        }

    def _response(self, ik_ok=True, fraction=1.0):
        response = ProbeMotionSegment.Response()
        response.ik_ok = bool(ik_ok)
        response.fraction = float(fraction)
        response.moveit_error_code = 0
        return response

    def test_segment_mirrors_traverse_semantics(self):
        driver, calls = _driver([self._response()])
        probe, stats = driver._place_probe_fn([_slot((1.0, 0.0, 0.0, 0.0))])
        probe(self._row())
        self.assertEqual(len(calls), 1)
        segment = calls[0].segment
        self.assertEqual(str(segment.name), "place_probe")
        self.assertEqual(str(segment.type), "cartesian")
        self.assertTrue(bool(segment.keep_tool_down))
        self.assertTrue(bool(segment.allow_ompl_fallback))
        # Target is the last waypoint; the rest stay intermediate, all with
        # the slot's place-yaw orientation.
        self.assertAlmostEqual(segment.target_pose.position.x, 0.5)
        self.assertAlmostEqual(segment.target_pose.position.z, 0.8)
        self.assertEqual(len(segment.waypoints), 2)
        self.assertAlmostEqual(segment.waypoints[0].position.x, 0.1)
        self.assertAlmostEqual(
            segment.target_pose.orientation.x, 1.0)
        self.assertAlmostEqual(
            segment.waypoints[1].orientation.y, 0.0)
        self.assertEqual(stats["mode"], "on")
        self.assertEqual(stats["probed"], 1)

    def test_probe_writes_gate_fields(self):
        driver, _calls = _driver([self._response(ik_ok=True, fraction=0.42)])
        probe, stats = driver._place_probe_fn([_slot()])
        row = self._row()
        probe(row)
        self.assertTrue(row["ik_ok"])
        self.assertAlmostEqual(row["cartesian_fraction"], 0.42)
        self.assertIn("probe_s", row)
        self.assertEqual(stats["ik_reject"], 0)
        self.assertEqual(stats["fraction_reject"], 1)

    def test_no_cartesian_solution_keeps_fraction_negative(self):
        driver, _calls = _driver([self._response(ik_ok=True, fraction=-1.0)])
        probe, stats = driver._place_probe_fn([_slot()])
        row = self._row()
        probe(row)
        # -1 must reach the plan() gate as a sub-threshold fraction, not be
        # blanked to None (which would pass the fraction gate).
        self.assertAlmostEqual(row["cartesian_fraction"], -1.0)
        self.assertEqual(stats["fraction_reject"], 1)

    def test_ik_failure_is_recorded_not_fraction(self):
        driver, _calls = _driver([self._response(ik_ok=False, fraction=-1.0)])
        probe, stats = driver._place_probe_fn([_slot()])
        row = self._row()
        probe(row)
        self.assertFalse(row["ik_ok"])
        self.assertEqual(stats["ik_reject"], 1)
        self.assertEqual(stats["fraction_reject"], 0)

    def test_unavailable_probe_rejects_row(self):
        driver, _calls = _driver([None])
        probe, stats = driver._place_probe_fn([_slot()])
        row = self._row()
        probe(row)
        self.assertFalse(row["ik_ok"])
        self.assertEqual(row["cartesian_fraction"], -1.0)
        self.assertNotIn("probe_s", row)
        self.assertEqual(stats["unavailable"], 1)
        self.assertEqual(stats["probed"], 0)

    def test_circuit_breaker_stops_the_sweep(self):
        driver, _calls = _driver([None, None, None, None])
        probe, stats = driver._place_probe_fn([_slot()])
        for _ in range(4):
            probe(self._row())
        # After PLACE_PROBE_MAX_UNAVAILABLE consecutive unavailable probes the
        # remaining rows are skipped without further service waits.
        self.assertEqual(stats["unavailable"], mod.PLACE_PROBE_MAX_UNAVAILABLE)
        # Breaker resets on a good response.
        driver2, _calls2 = _driver([None, self._response(), None, None])
        probe2, stats2 = driver2._place_probe_fn([_slot()])
        for _ in range(2):
            probe2(self._row())
        probe2(self._row())
        self.assertEqual(stats2["probed"], 1)
        self.assertEqual(stats2["unavailable"], 2)

    def test_service_down_at_selection_returns_rejecting_probe(self):
        driver, _calls = _driver([])
        driver._probe_place = _FakeClient(up=False)
        probe, stats = driver._place_probe_fn([_slot()])
        row = self._row()
        probe(row)
        self.assertFalse(row["feasible"])
        self.assertEqual(row["reason"], "PLACE_PROBE_UNAVAILABLE")
        self.assertEqual(stats["mode"], "service_unavailable")

    def test_empty_waypoints_never_calls_the_service(self):
        driver, calls = _driver([])
        probe, _stats = driver._place_probe_fn([_slot()])
        row = self._row()
        row["waypoints"] = []
        probe(row)
        self.assertEqual(calls, [])


class PlaceProbeSwitchTest(unittest.TestCase):
    def test_default_on(self):
        args = mod.parse_args(["--out", "/tmp/motion_occ_probe_test"])
        self.assertEqual(args.place_probe, "on")

    def test_off(self):
        args = mod.parse_args(
            ["--out", "/tmp/motion_occ_probe_test", "--place-probe", "off"])
        self.assertEqual(args.place_probe, "off")

    def test_equals_form_and_n_default(self):
        args = mod.parse_args(
            ["--out", "/tmp/motion_occ_probe_test",
             "--place-probe=off"])
        self.assertEqual(args.place_probe, "off")
        # --n defaults to 2 in one place (the wrapper); an explicit value
        # survives.
        self.assertEqual(args.n, 2)
        explicit = mod.parse_args(["--out", "/tmp/motion_occ_probe_test",
                                   "--n", "3"])
        self.assertEqual(explicit.n, 3)
        self.assertEqual(explicit.place_probe, "on")


class CarryFrameConvertTest(unittest.TestCase):
    def test_world_frame_skips_origin_shift(self):
        convert = mod.carry_frame_convert(
            {"waypoint_frame": "world", "waypoints": [[1.005, -0.535, 0.827]]},
            {"unused": True})
        self.assertIsNone(convert)

    def test_map_frame_keeps_origin_shift(self):
        orig = mod._map_to_world
        mod._map_to_world = lambda _scene, xyz: [xyz[0] + 1.5, xyz[1], xyz[2]]
        self.addCleanup(setattr, mod, "_map_to_world", orig)
        convert = mod.carry_frame_convert(
            {"waypoints": [[1.005, -0.535, 0.827]]}, {"unused": True})
        self.assertIsNotNone(convert)
        shifted = convert([1.005, -0.535, 0.827])
        self.assertAlmostEqual(shifted[0], 2.505)

    def test_apply_selected_carry_splices_world_v2(self):
        from types import SimpleNamespace
        from luggage_planning.typed_driver_adapter import (
            apply_selected_path_to_v2, pose_segment_message)
        from geometry_msgs.msg import Pose

        driver = mod.MotionOccDriver.__new__(mod.MotionOccDriver)
        driver._scene_config = {"unused": True}
        driver._typed = object()
        built = pose_segment_message(
            "transit", [2.505, -0.535, 0.827], [1.0, 0.0, 0.0, 0.0],
            "place-1", "scene-a", segment_type="pose_target",
            allow_ompl_fallback=False)
        selected = pose_segment_message(
            "transit", [0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0],
            "select-id", "other", segment_type="cartesian")
        apply_selected_path_to_v2(
            selected, [[0.86, -0.38, 1.5], [1.005, -0.535, 0.827]])
        driver._frame_motion_segments = [built]
        driver._frame_motion_index = 0
        driver._selected_carry_v2 = [selected]
        driver._selected_path = {
            "waypoint_frame": "world",
            "waypoints": [
                [0.86, -0.38, 1.5],
                [1.005, -0.535, 1.5],
                [1.005, -0.535, 0.827],
            ],
        }
        v1 = SimpleNamespace(
            name="transit", type="pose_target", allow_ompl_fallback=False,
            target_pose=Pose(), waypoints=[])
        driver._apply_selected_carry(v1)
        executed = driver._frame_motion_segments[0]
        self.assertEqual(executed.segment_type, "cartesian")
        self.assertTrue(executed.allow_ompl_fallback)
        self.assertEqual(executed.request_id, "place-1")
        self.assertAlmostEqual(executed.target_pose.pose.position.x, 0.86)
        self.assertEqual(list(executed.waypoints), [])
        self.assertEqual(v1.type, "cartesian")
        self.assertAlmostEqual(v1.target_pose.position.x, 0.86)

    def test_split_carry_waypoints(self):
        path = [[0.86, -0.38, 1.5], [1.005, -0.535, 1.5], [1.005, -0.535, 0.827]]
        kind, points = mod.split_carry_waypoints("transit", path)
        self.assertEqual(kind, "cartesian")
        self.assertEqual(points, [[0.86, -0.38, 1.5]])
        kind, points = mod.split_carry_waypoints("traverse", path)
        self.assertEqual(kind, "cartesian")
        self.assertEqual(len(points), 2)
        self.assertAlmostEqual(points[-1][2], 0.827)


if __name__ == "__main__":
    unittest.main()
