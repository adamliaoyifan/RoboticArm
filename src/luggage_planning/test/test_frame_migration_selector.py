#!/usr/bin/env python3
"""ST07 place TCP, occupancy sweep, and SelectPlacePathV2."""

from __future__ import division

import copy
import json
import math
import os
from contextlib import contextmanager
import sys
import time
import traceback
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as Rot

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(TEST_DIR, "..")))
sys.path.insert(0, os.path.normpath(os.path.join(TEST_DIR, "..", "scripts")))
sys.path.insert(0, os.path.normpath(os.path.join(TEST_DIR, "..", "..", "luggage_description")))

from luggage_description.container_geometry import cuboid_from_inner_size
from luggage_description.frame_contracts import ContractError
from luggage_description.frame_transforms import rotate_vector, transform_pose
from luggage_planning.occupancy_place_paths import (
    _aabb_distance,
    _footprint_cells,
    _legacy_view_of_surface_contract,
)
from luggage_planning.place_tcp import (
    FIXTURE_SELECTOR,
    build_place_legs_v2,
    lift_world_point,
    select_place_v2,
    world_slot_record,
    world_tcp_record,
)
from luggage_planning.place_selection_client import select_in_process_v2
from luggage_planning.pose import Point, Pose, Quaternion
from luggage_planning.reachability_atlas import ReachabilityAtlas
from luggage_planning.reachable_graph import build_reachable_graph
from luggage_planning.trajectory_selector import _path_hash, _sort_key
from luggage_planning.waypoint_generator import (
    build_sequence,
    insertion_clearance,
    slot_from_contract,
)
from rclpy.serialization import serialize_message
from trajectory_selector_node import _segments

DATA = Path(TEST_DIR) / "data" / "frame_migration"
INNER = (2.0, 2.0, 1.48)
FLOOR = 0.53
CEILING = 2.01
RES = 0.05
NX = NY = 40
PAYLOAD = (0.80, 0.50, 0.32)
PORTAL = (-0.55, -0.35, 1.10)
TARGET = (0.55, -0.35, 0.90)
SCENE = "st07-scene"
HEIGHT = PAYLOAD[2]


def json_safe(value):
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    if isinstance(value, float):
        return value
    return value


@contextmanager
def case(case_id, inputs):
    row = dict(case_id=case_id, input=copy.deepcopy(inputs), status="pass",
               observations={})
    started = time.monotonic()
    try:
        yield row["observations"]
    except BaseException:
        row.update(status="fail", traceback=traceback.format_exc())
        raise
    finally:
        row["elapsed_seconds"] = time.monotonic() - started
        output = os.environ.get("FRAME_MIGRATION_CASE_LOG")
        if output:
            with open(output, "a", encoding="utf-8") as handle:
                handle.write(json.dumps(json_safe(row), allow_nan=False,
                                        sort_keys=True) + "\n")
                handle.flush()
                os.fsync(handle.fileno())
        print("CASE", case_id, row["status"], flush=True)


def _quat_err(actual, expected):
    return min(max(abs(a - b) for a, b in zip(actual, expected)),
               max(abs(a + b) for a, b in zip(actual, expected)))


def _oracle(box_pos, box_q, grasp_t, grasp_q, origin, container_q):
    box = Rot.from_quat(box_q)
    tcp = box.apply(np.array(grasp_t, dtype=float)) + np.array(box_pos, dtype=float)
    tcp_q = (box * Rot.from_quat(grasp_q)).as_quat()
    world = Rot.from_quat(container_q)
    position = world.apply(tcp) + np.array(origin, dtype=float)
    quaternion = (world * Rot.from_quat(tcp_q)).as_quat()
    return position, quaternion


def _yaw_quat(yaw):
    return [0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)]


def _record(position, quaternion, reference, semantics, frame, request_id="r"):
    return {
        "contract_version": 2,
        "frame_id": frame,
        "length_unit": "m",
        "stamp_ns": 0,
        "time_basis": "scene",
        "scene_revision": SCENE,
        "request_id": request_id,
        "kind": "pose",
        "reference": reference,
        "orientation_semantics": semantics,
        "position": [float(v) for v in position],
        "quaternion_xyzw": [float(v) for v in quaternion],
    }


def _slot(position, yaw=0.0, request_id="slot"):
    return _record(position, _yaw_quat(yaw), "box_center", "box_axis_pi",
                   "container_link", request_id)


def _grasp(translation, quaternion=None):
    return {
        "reference": "suction_contact_frame",
        "orientation_semantics": "box",
        "position": [float(v) for v in translation],
        "quaternion_xyzw": list(quaternion or (1.0, 0.0, 0.0, 0.0)),
        "scene_revision": SCENE,
    }


def _transform(translation=(0, 0, 0), yaw=0.0):
    return {
        "source_frame": "container_link",
        "target_frame": "world",
        "translation": [float(v) for v in translation],
        "quaternion_xyzw": _yaw_quat(yaw),
        "stamp_ns": None,
        "scene_revision": SCENE,
    }


def _surface(kind, gap=None, short=False):
    hull = cuboid_from_inner_size(INNER, floor_z=FLOOR)
    desc = hull.descriptor()
    z = [[FLOOR for _ in range(NY)] for _ in range(NX)]
    state = [["free" for _ in range(NY)] for _ in range(NX)]
    conf = [["verified_geometry" for _ in range(NY)] for _ in range(NX)]
    state[1][1] = "unknown"
    z[1][1] = None
    conf[1][1] = "unknown"
    if kind != "open":
        blocked = set() if gap is None else set(gap)
        for iy in range(NY):
            if iy in blocked:
                continue
            if short:
                state[19][iy] = "occupied"
                z[19][iy] = FLOOR + 0.05
                conf[19][iy] = "sensor"
            elif iy == 8:
                state[19][iy] = "unknown"
                z[19][iy] = FLOOR + 0.30
                conf[19][iy] = "unknown"
            else:
                state[19][iy] = "occupied"
                z[19][iy] = CEILING
                conf[19][iy] = "sensor"
    record = {
        "kind": "surface",
        "frame_id": "container_link",
        "height_datum": "container_origin",
        "floor_z_container": FLOOR,
        "ceiling_z_container": CEILING,
        "origin_xy_container": [-1.0, -1.0],
        "resolution_m": RES,
        "map_revision": 3,
        "geometry_hash": desc["geometry_hash"],
        "surface_z_container": z,
        "state": state,
        "confidence": conf,
        "geometry_descriptor": desc,
    }
    return record


def _request(surface, portal, target_tcp, grasp_t, yaw=0.0, slots=None):
    slot_z = target_tcp[2] - grasp_t[2]
    slot = _slot((target_tcp[0], target_tcp[1], slot_z), yaw=yaw)
    if slots is None:
        slots = [slot]
    return {
        "scene_revision": SCENE,
        "box_wdh": list(PAYLOAD),
        "slots": slots,
        "slot_scores": [1.0 for _ in slots],
        "surface": surface,
        "inner_size": list(INNER),
        "geometry_hash": surface["geometry_hash"],
        "map_revision": 3,
        "start_tcp": _record(portal, (1, 0, 0, 0), "suction_contact_frame",
                             "tool_quaternion", "container_link", "start"),
        "start_joints": [],
        "grasp": _grasp(grasp_t),
        "container_transform": _transform(),
        "excluded_ids": [],
    }


def _walk(snapshot, points, yaw):
    bad = []
    for point in points:
        for ix, iy, local_fp in _footprint_cells(
                snapshot, point, PAYLOAD[0], PAYLOAD[1], yaw, 0.0):
            if not snapshot.obstacle[ix][iy]:
                continue
            if _aabb_distance(local_fp, snapshot.cell_aabb_xy(ix, iy)) > 1e-6:
                continue
            bad.append((ix, iy, float(snapshot.column_top[ix][iy])))
    return bad


class TcpMatrix(unittest.TestCase):
    def test_sixty_four_and_named_cases(self):
        yaws = (0.0, math.pi / 2, math.pi, math.atan2(4.0, 3.0))
        translations = ((0.0, 0.0, 0.0), (1.5, -0.4, 0.2))
        floors = (0.53, 0.71)
        supports = (0.0, 0.30)
        grasps = ((0.0, 0.0, 0.16), (0.10, -0.05, 0.16))
        seen = set()
        for yaw in yaws:
            for origin in translations:
                for floor in floors:
                    for support in supports:
                        for grasp_t in grasps:
                            slot_z = floor + support + 0.5 * HEIGHT
                            box_yaw = 0.3
                            slot = _slot((0.20, -0.10, slot_z), yaw=box_yaw)
                            grasp = {
                                "source_frame": "suction_contact_frame",
                                "target_frame": "box",
                                "translation": list(grasp_t),
                                "quaternion_xyzw": [1.0, 0.0, 0.0, 0.0],
                                "scene_revision": SCENE,
                            }
                            transform = _transform(origin, yaw)
                            case_id = "matrix/%g/%g/%g/%g/%g" % (
                                yaw, origin[0], floor, support, grasp_t[0])
                            if abs(grasp_t[0] - 0.10) < 1e-9 and abs(yaw) < 1e-12 and origin == (0, 0, 0):
                                case_id = "offcenter_grasp"
                            if grasp_t == (0, 0, 0.16) and abs(yaw) < 1e-12 and origin == (0, 0, 0) and floor == 0.53 and support == 0:
                                case_id = "center_grasp"
                            if abs(yaw - math.pi / 2) < 1e-12 and origin == (1.5, -0.4, 0.2) and floor == 0.53 and support == 0 and grasp_t == (0, 0, 0.16):
                                case_id = "translate_rotate90"
                            if abs(yaw - math.atan2(4, 3)) < 1e-12 and origin == (1.5, -0.4, 0.2) and grasp_t == (0, 0, 0.16) and floor == 0.53 and support == 0:
                                case_id = "translate_rotate53"
                            with case(case_id, {"yaw": yaw, "origin": origin}) as obs:
                                world = world_tcp_record(slot, grasp, transform)
                                expected_p, expected_q = _oracle(
                                    slot["position"], slot["quaternion_xyzw"],
                                    grasp_t, grasp["quaternion_xyzw"],
                                    origin, transform["quaternion_xyzw"])
                                pos_err = max(abs(a - b) for a, b in zip(world["position"], expected_p))
                                quat_err = _quat_err(world["quaternion_xyzw"], expected_q)
                                self.assertLessEqual(pos_err, 1e-9)
                                self.assertLessEqual(quat_err, 1e-9)
                                slot_pose = slot_from_contract(
                                    slot, (origin, transform["quaternion_xyzw"]))
                                slot_world = world_slot_record(slot, transform)
                                self.assertLessEqual(max(
                                    abs(slot_pose.position.x - slot_world["position"][0]),
                                    abs(slot_pose.position.y - slot_world["position"][1]),
                                    abs(slot_pose.position.z - slot_world["position"][2])), 1e-9)
                                self.assertLessEqual(_quat_err(
                                    (slot_pose.orientation.x, slot_pose.orientation.y,
                                     slot_pose.orientation.z, slot_pose.orientation.w),
                                    slot_world["quaternion_xyzw"]), 1e-9)
                                if case_id == "offcenter_grasp":
                                    top = rotate_vector(slot["quaternion_xyzw"], (0, 0, 0.16))
                                    tcp = rotate_vector(slot["quaternion_xyzw"], grasp_t)
                                    self.assertGreater(abs(tcp[0] - top[0]) + abs(tcp[1] - top[1]), 1e-6)
                                if case_id == "translate_rotate90":
                                    bad = [slot["position"][i] + origin[i] for i in range(3)]
                                    self.assertGreater(max(abs(a - b) for a, b in zip(bad, expected_p)), 1e-3)
                                obs.update(position_error=pos_err, quaternion_error=quat_err)
                            seen.add(case_id)
        self.assertIn("offcenter_grasp", seen)
        self.assertIn("center_grasp", seen)
        self.assertIn("translate_rotate90", seen)
        self.assertIn("translate_rotate53", seen)

    def test_normal_not_translated_and_tool_branches(self):
        slot = _slot((0.2, -0.1, 0.8), yaw=0.2)
        grasp = {
            "source_frame": "suction_contact_frame", "target_frame": "box",
            "translation": [0.0, 0.0, 0.16],
            "quaternion_xyzw": [1.0, 0.0, 0.0, 0.0],
            "scene_revision": SCENE,
        }
        with case("normal_not_translated", {}) as obs:
            a = world_tcp_record(slot, grasp, _transform((0, 0, 0), 0.4))
            b = world_tcp_record(slot, grasp, _transform((1.5, -0.4, 0.2), 0.4))
            lift_a = lift_world_point(a["position"], _yaw_quat(0.4), 0.15)
            lift_b = lift_world_point(b["position"], _yaw_quat(0.4), 0.15)
            delta_a = [lift_a[i] - a["position"][i] for i in range(3)]
            delta_b = [lift_b[i] - b["position"][i] for i in range(3)]
            self.assertLessEqual(max(abs(x - y) for x, y in zip(delta_a, delta_b)), 1e-12)
            pure = rotate_vector(_yaw_quat(0.4), (0, 0, 0.15))
            self.assertLessEqual(max(abs(x - y) for x, y in zip(delta_a, pure)), 1e-12)
            obs.update(lift=delta_a)
        with case("tool_pi_distinct", {}):
            q0 = world_tcp_record(slot, dict(grasp, quaternion_xyzw=[1, 0, 0, 0]), _transform())
            qpi = world_tcp_record(slot, dict(grasp, quaternion_xyzw=[0, 1, 0, 0]), _transform())
            self.assertGreater(_quat_err(q0["quaternion_xyzw"], qpi["quaternion_xyzw"]), 1e-6)
        with case("tool_sign_equivalent", {}):
            positive = world_tcp_record(slot, grasp, _transform(yaw=0.4))
            negative = [-v for v in positive["quaternion_xyzw"]]
            self.assertLessEqual(_quat_err(positive["quaternion_xyzw"], negative), 1e-12)

    def test_place_legs_use_tcp_and_distinct_lifts(self):
        slot = _slot((0.2, -0.1, 0.8))
        grasp = {
            "source_frame": "suction_contact_frame", "target_frame": "box",
            "translation": [0.10, -0.05, 0.16],
            "quaternion_xyzw": [1.0, 0.0, 0.0, 0.0],
            "scene_revision": SCENE,
        }
        segments, contact = build_place_legs_v2(
            slot, grasp, _transform(yaw=0.0), PAYLOAD)
        names = [seg.name for seg in segments]
        self.assertEqual(names, ["transit", "traverse", "insert", "descend", "retreat"])
        descend = segments[3].target_pose.position
        self.assertAlmostEqual(descend.x, contact["position"][0])
        self.assertAlmostEqual(descend.y, contact["position"][1])
        self.assertNotAlmostEqual(descend.x, 0.2, places=3)
        insert_z = segments[2].target_pose.position.z
        self.assertAlmostEqual(
            insert_z - descend.z, insertion_clearance(HEIGHT, 0.15), places=9)
        v1 = build_sequence(type("P", (), {"pose": Pose(position=Point(x=0, y=0, z=0), orientation=Quaternion(w=1)), "height": 0.32, "yaw_valid": True})(),
                            type("S", (), {"place_pose": Pose(position=Point(x=0.2, y=-0.1, z=0.8), orientation=Quaternion(w=1)), "height": 0.32})(),
                            "place")
        self.assertAlmostEqual(v1[3].target_pose.position.x, 0.2, places=6)


class MapFixtures(unittest.TestCase):
    def _run(self, surface, portal, target, yaw=0.0, grasp=(0, 0, 0.16)):
        calls = {"probe": 0, "moveit": 0}
        def probe(_row):
            calls["probe"] += 1
        def moveit():
            calls["moveit"] += 1
        before = copy.deepcopy(surface)
        result = select_in_process_v2(
            _request(surface, portal, target, grasp, yaw=yaw),
            atlas=None, config=dict(FIXTURE_SELECTOR),
            probe_fn=probe, moveit_client=moveit)
        self.assertEqual(surface, before)
        self.assertEqual(calls["probe"], 0)
        self.assertEqual(calls["moveit"], 0)
        self.assertNotIn("BIN_FULL", str(result["reason_code"]))
        return result

    def test_open_direct(self):
        surface = _surface("open")
        with case("open_direct", {}):
            result = self._run(surface, PORTAL, TARGET)
            self.assertTrue(result["success"])
            self.assertEqual(result["method"], "direct")
            self.assertEqual(result["probe_calls"], 0)

    def test_blocked_corridor_detour(self):
        surface = _surface("wall", gap=range(22, 34))
        with case("blocked_corridor_detour", {"gap": "iy22-33"}):
            result = self._run(surface, PORTAL, TARGET)
            self.assertEqual(result["method"], "gap_detour")
            legacy = _legacy_view_of_surface_contract(
                surface, list(INNER), surface["geometry_descriptor"])
            from luggage_planning.occupancy_place_paths import OccupancySnapshot
            snap = OccupancySnapshot.from_surface_2d(legacy, inflate_m=0.0)
            bad = _walk(snap, result["box_center_waypoints"], 0.0)
            self.assertEqual(bad, [])
            reasons = {row["reason"] for row in result["rows"] if not row["feasible"]}
            self.assertTrue(reasons <= {"occupancy_collision", "outside_hull"})

    def test_blocked_destination(self):
        surface = _surface("wall", gap=None)
        with case("blocked_destination", {}):
            result = self._run(surface, PORTAL, TARGET)
            self.assertFalse(result["success"])
            self.assertEqual(result["reason_code"], "PLACE_PATH_INFEASIBLE")
            self.assertTrue(all(row["reason"] in ("occupancy_collision", "outside_hull")
                                for row in result["rows"]))
            self.assertFalse(any(row["method"] == "gap_detour" and row["feasible"]
                                 for row in result["rows"]))

    def test_nonsquare_width_axis(self):
        narrow = _surface("wall", gap=range(22, 34))
        with case("nonsquare_width_axis", {}):
            turned = self._run(narrow, PORTAL, TARGET, yaw=math.pi / 2)
            self.assertNotEqual(turned["method"], "gap_detour")
            wide = _surface("wall", gap=range(20, 40))
            winner = self._run(wide, PORTAL, TARGET, yaw=math.pi / 2)
            self.assertEqual(winner["method"], "gap_detour")

    def test_short_wall_stays_raised(self):
        surface = _surface("wall", gap=None, short=True)
        low_portal = (PORTAL[0], PORTAL[1], 0.88)
        low_target = (TARGET[0], TARGET[1], 0.88)
        with case("short_wall", {}):
            result = self._run(surface, low_portal, low_target)
            self.assertIn(result["method"], ("clear_top", "clear_headroom", "corridor"))
            self.assertNotEqual(result["method"], "gap_detour")

    def test_checked_in_arrays(self):
        for name, surface in (
                ("open_direct", _surface("open")),
                ("blocked_corridor_detour", _surface("wall", gap=range(22, 34))),
                ("blocked_destination", _surface("wall", gap=None))):
            stored = json.loads((DATA / (name + ".json")).read_text(encoding="utf-8"))
            self.assertEqual(stored, json.loads(json.dumps(surface)))

    def test_offcenter_footprint_follows_box_center(self):
        surface = _surface("open")
        with case("offcenter_cells", {}):
            result = self._run(surface, PORTAL, TARGET, grasp=(0.10, -0.05, 0.16))
            tcp = result["container_waypoints"]
            centers = result["box_center_waypoints"]
            self.assertGreater(abs(tcp[0][0] - centers[0][0]) + abs(tcp[0][1] - centers[0][1]), 1e-6)

    def test_tie_break(self):
        with case("tie_break", {}):
            direct = {"cost": 0.1, "method": "direct", "slot_index": 1, "path_hash": "b"}
            detour = {"cost": 0.1, "method": "gap_detour", "slot_index": 0, "path_hash": "a"}
            self.assertEqual(min((direct, detour), key=_sort_key)["method"], "direct")
            low = {"cost": 0.1, "method": "direct", "slot_index": 0, "path_hash": "z"}
            high = {"cost": 0.1, "method": "direct", "slot_index": 1, "path_hash": "a"}
            self.assertEqual(min((high, low), key=_sort_key)["slot_index"], 0)
            left = {"cost": 0.1, "method": "direct", "slot_index": 0,
                    "waypoints": [(0.0, 0.0, 1.0), (0.2, 0.0, 1.0)]}
            right = {"cost": 0.1, "method": "direct", "slot_index": 0,
                     "waypoints": [(0.0, 0.0, 1.0), (0.2, 0.001, 1.0)]}
            left["path_hash"] = _path_hash(left)
            right["path_hash"] = _path_hash(right)
            self.assertNotEqual(left["path_hash"], right["path_hash"])
            winner = min((left, right), key=_sort_key)
            self.assertEqual(winner["path_hash"], min(left["path_hash"], right["path_hash"]))


class Negatives(unittest.TestCase):
    def _base(self):
        return _request(_surface("open"), PORTAL, TARGET, (0, 0, 0.16))

    def _reject(self, name, mutate, code):
        request = self._base()
        mutate(request)
        calls = {"probe": 0, "moveit": 0, "sweep_mutation": 0, "commit": 0}
        with case("negative/" + name, {}) as obs:
            result = select_place_v2(
                request, probe_fn=lambda _r: calls.__setitem__("probe", 1),
                moveit_client=lambda: calls.__setitem__("moveit", 1),
                counters=calls)
            self.assertFalse(result["success"])
            self.assertEqual(result["reason_code"], code)
            self.assertEqual(calls["probe"], 0)
            self.assertEqual(calls["moveit"], 0)
            self.assertEqual(result["sweep_mutations"], 0)
            self.assertEqual(result["commit_calls"], 0)
            obs.update(code=result["reason_code"])

    def test_codes(self):
        self._reject("empty-frame", lambda r: r["slots"][0].__setitem__("frame_id", ""), "MISSING_IDENTITY")
        self._reject("missing-grasp", lambda r: r.__setitem__("grasp", None), "MISSING_IDENTITY")
        self._reject("world-frame", lambda r: r["slots"][0].__setitem__("frame_id", "world"), "FRAME_CONTRACT_MISMATCH")
        self._reject("tcp-labelled-box", lambda r: r["slots"][0].__setitem__("reference", "suction_contact_frame"), "POSE_REFERENCE_MISMATCH")
        self._reject("tool-on-slot", lambda r: r["slots"][0].__setitem__("orientation_semantics", "tool_quaternion"), "ORIENTATION_SEMANTICS_MISMATCH")
        self._reject("version", lambda r: r["slots"][0].__setitem__("contract_version", 1), "CONTRACT_VERSION_MISMATCH")
        self._reject("nonfinite", lambda r: r["slots"][0].__setitem__("position", [math.nan, 0, 0]), "INVALID_NUMERIC_PAYLOAD")
        self._reject("zero-quat", lambda r: r["slots"][0].__setitem__("quaternion_xyzw", [0, 0, 0, 0]), "INVALID_QUATERNION")
        self._reject("stale-revision", lambda r: r.__setitem__("map_revision", 99), "MAP_REVISION_MISMATCH")
        request = self._base()
        request["geometry_hash"] = "deadbeef"
        result = select_place_v2(request, counters={"probe": 0, "moveit": 0, "sweep_mutation": 0, "commit": 0})
        self.assertIn("PLACE_PATH_INFEASIBLE", result["reason_code"])


class AtlasBins(unittest.TestCase):
    def _atlas(self, period=None):
        nx = ny = nz = 3
        nyaw = 2
        shape = (nx, ny, nz, nyaw)
        data = {
            "status": np.zeros(shape, np.uint8),
            "opening_connected": np.ones(shape, np.uint8),
            "contact_seeds": np.zeros(shape + (1, 6), np.float64),
            "transit_seeds": np.zeros(shape + (1, 6), np.float64),
            "contact_ik": np.ones(shape, np.uint8),
            "transit_ik": np.ones(shape, np.uint8),
            "solution_count": np.ones(shape, np.uint8),
            "neighbor_confidence": np.ones(shape, np.float32),
            "joint_margin": np.ones(shape, np.float32),
            "manipulability": np.ones(shape, np.float32),
        }
        meta = {
            "grid": {
                "resolution_xyz": 0.20,
                "origin": [0.0, 0.0, 0.0],
                "size": [nx, ny, nz],
                "yaw_bins": [0.0, math.pi],
            },
            "container": {"opening_side": "negative_x"},
        }
        if period is not None:
            meta["contract"] = {"yaw_period_rad": period}
        return ReachabilityAtlas(data, meta)

    def test_directed_bins_and_pi_fold(self):
        atlas = self._atlas()
        center = (0.1, 0.1, 0.1)
        with case("directed_bins", {}):
            zero = atlas.query(*center, yaw=0.0).indices[3]
            pi = atlas.query(*center, yaw=math.pi).indices[3]
            self.assertNotEqual(zero, pi)
            seen = []

            class Recording(object):
                def __init__(self, inner):
                    self._inner = inner
                def query(self, x, y, z, yaw=0.0, **kwargs):
                    seen.append(float(yaw))
                    return self._inner.query(x, y, z, yaw=yaw, **kwargs)
                def __getattr__(self, name):
                    return getattr(self._inner, name)

            build_reachable_graph(Recording(atlas), yaw=math.pi, blocked_fn=lambda _i, _c: "skip")
            self.assertTrue(seen)
            self.assertTrue(all(abs(abs(yaw) - math.pi) < 1e-8 or abs(yaw + math.pi) < 1e-8 for yaw in seen))

        class Folded(object):
            def __init__(self, inner):
                self._inner = inner
            def query(self, x, y, z, yaw=0.0, **kwargs):
                if abs(abs(yaw) - math.pi) < 1e-8:
                    yaw = 0.0
                return self._inner.query(x, y, z, yaw=yaw, **kwargs)
            def __getattr__(self, name):
                return getattr(self._inner, name)

        folded = Folded(atlas)
        with case("injected_pi_fold", {}):
            with self.assertRaises(AssertionError):
                self.assertNotEqual(
                    folded.query(*center, yaw=0.0).indices[3],
                    folded.query(*center, yaw=math.pi).indices[3])
        with case("atlas_pi_fold_rejected", {}):
            queries = []

            class Counting(object):
                def __init__(self, inner):
                    self._inner = inner
                def query(self, *args, **kwargs):
                    queries.append(args)
                    return self._inner.query(*args, **kwargs)
                def __getattr__(self, name):
                    return getattr(self._inner, name)

            bad = Counting(self._atlas(period=math.pi))
            with self.assertRaises(ContractError) as caught:
                build_reachable_graph(bad, yaw=0.0)
            self.assertEqual(caught.exception.code, "ATLAS_SEMANTICS_MISMATCH")
            self.assertEqual(queries, [])


class V1Bytes(unittest.TestCase):
    def test_v1_segment_bytes_unchanged(self):
        winner = {
            "slot_index": 0,
            "yaw": 0.3,
            "waypoints": [(-0.6, -0.4, 1.5), (0.08, -0.66, 1.2)],
            "segments": {
                "transit": {"type": "cartesian",
                            "waypoints": [(-0.6, -0.4, 1.5), (0.08, -0.66, 1.2)],
                            "joint_knots": [], "allow_ompl_fallback": True},
                "traverse": {"type": "cartesian",
                             "waypoints": [(-0.6, -0.4, 1.5), (0.08, -0.66, 1.2)],
                             "joint_knots": [], "allow_ompl_fallback": True},
            },
        }
        segments = _segments(winner, map_to_world=lambda xyz: [xyz[0] + 1.5, xyz[1], xyz[2]])
        self.assertEqual([seg.name for seg in segments], ["transit", "traverse"])
        pose = segments[0].target_pose
        self.assertAlmostEqual(pose.position.x, 0.08 + 1.5)
        self.assertAlmostEqual(pose.position.y, -0.66)
        self.assertAlmostEqual(pose.position.z, 1.2)
        half = 0.15
        self.assertAlmostEqual(pose.orientation.x, math.cos(half))
        self.assertAlmostEqual(pose.orientation.y, math.sin(half))
        self.assertAlmostEqual(pose.orientation.z, 0.0)
        self.assertAlmostEqual(pose.orientation.w, 0.0)
        # A second CDR pass is stable and preserves the yaw-only quaternion.
        raw = serialize_message(segments[0])
        from rclpy.serialization import deserialize_message
        from luggage_msgs.msg import MotionSegment
        back = deserialize_message(raw, MotionSegment)
        self.assertAlmostEqual(back.target_pose.orientation.w, 0.0)
        self.assertAlmostEqual(back.target_pose.orientation.z, 0.0)


class ServiceAgreement(unittest.TestCase):
    def test_service_matches_in_process_and_cdr(self):
        from luggage_planning.place_service import (
            fill_select_response, handle_build_v2, pose_message, request_dict)
        from luggage_msgs.srv import SelectPlacePathV2
        surfaces = {
            "open_direct": (_surface("open"), "direct"),
            "blocked_corridor_detour": (_surface("wall", gap=range(22, 34)), "gap_detour"),
            "blocked_destination": (_surface("wall", gap=None), None),
        }
        open_case = None
        for name, (surface, method) in surfaces.items():
            with case("service/" + name, {}):
                request = _request(surface, PORTAL, TARGET, (0, 0, 0.16))
                local = select_in_process_v2(request, atlas=None, config=dict(FIXTURE_SELECTOR))
                message = _to_message(request, surface)
                response = SelectPlacePathV2.Response()
                fill_select_response(message, response, atlas=None, config=dict(FIXTURE_SELECTOR))
                self.assertEqual(bool(response.success), local["success"])
                self.assertEqual(int(response.slot_index), int(local["slot_index"]))
                self.assertEqual(str(response.path_hash), local["path_hash"])
                if local["success"]:
                    self.assertEqual(local["method"], method or local["method"])
                    self.assertGreater(len(response.waypoint_poses), 0)
                    for pose, world in zip(response.waypoint_poses, local["world_waypoints"]):
                        got = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
                        self.assertLessEqual(max(abs(a - b) for a, b in zip(got, world["position"])), 1e-6)
                        raw = serialize_message(pose.pose)
                        from rclpy.serialization import deserialize_message
                        from geometry_msgs.msg import Pose
                        back = deserialize_message(raw, Pose)
                        self.assertLessEqual(abs(back.position.x - world["position"][0]), 1e-6)
                        self.assertNotEqual(
                            (pose.pose.orientation.x, pose.pose.orientation.y,
                             pose.pose.orientation.z, pose.pose.orientation.w),
                            (0.0, 0.0, 0.0, 0.0))
                if method:
                    self.assertEqual(local["method"], method)
                if name == "open_direct":
                    open_case = (request, surface, local["path_hash"])
        # Round-trip the decoded request dict through the same helper.
        again = request_dict(_to_message(request, surface))
        self.assertEqual(again["slots"][0]["position"], request["slots"][0]["position"])
        built = handle_build_v2(_build_message(request), type("R", (), {
            "segments": [], "success": False, "message": "", "reason_code": "",
            "contact_pose": None,
        })())
        self.assertTrue(built.success)
        self.assertIn("descend", [seg.name for seg in built.segments])
        self._live_open(*open_case)
        del pose_message

    def _live_open(self, request, surface, path_hash):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from luggage_msgs.srv import SelectPlacePathV2
        from luggage_planning.place_service import fill_select_response
        sys.path.insert(0, os.path.normpath(os.path.join(TEST_DIR, "..", "..", "..", "scripts")))
        from select_place_path_probe import call_select_v2
        rclpy.init()
        server = Node("st07_selector_server")
        client = Node("st07_selector_client")
        try:
            def callback(req, resp):
                return fill_select_response(
                    req, resp, atlas=None, config=dict(FIXTURE_SELECTOR))
            server.create_service(
                SelectPlacePathV2, "/trajectory_selector/select_place_path_v2", callback)
            executor = SingleThreadedExecutor()
            executor.add_node(server)
            import threading
            thread = threading.Thread(target=executor.spin, daemon=True)
            thread.start()
            response = call_select_v2(client, _to_message(request, surface), timeout=5.0)
            self.assertIsNotNone(response)
            self.assertTrue(response.success)
            self.assertEqual(str(response.path_hash), path_hash)
            executor.shutdown()
        finally:
            client.destroy_node()
            server.destroy_node()
            rclpy.shutdown()


def _to_message(request, surface):
    from luggage_msgs.srv import SelectPlacePathV2
    from luggage_planning.place_service import pose_message
    message = SelectPlacePathV2.Request()
    message.request_id = "svc"
    message.box_wdh = list(request["box_wdh"])
    message.slots = [pose_message(slot) for slot in request["slots"]]
    message.slot_scores = list(request["slot_scores"])
    message.inner_size = list(request["inner_size"])
    message.geometry_hash = request["geometry_hash"]
    message.map_revision = int(request["map_revision"])
    message.start_tcp = pose_message(request["start_tcp"])
    message.grasp = pose_message(_record(
        request["grasp"]["position"], request["grasp"]["quaternion_xyzw"],
        request["grasp"]["reference"], request["grasp"]["orientation_semantics"],
        "container_link", "grasp"))
    transform = request["container_transform"]
    message.container_source_frame = transform["source_frame"]
    message.container_target_frame = transform["target_frame"]
    message.container_translation = list(transform["translation"])
    message.container_quaternion_xyzw = list(transform["quaternion_xyzw"])
    message.container_stamp_is_null = True
    _fill_surface(message.surface, surface)
    return message


def _fill_surface(message, surface):
    from luggage_planning.place_service import _fill_identity
    rows = len(surface["surface_z_container"])
    cols = len(surface["surface_z_container"][0])
    message.rows = rows
    message.columns = cols
    message.height_datum = surface["height_datum"]
    message.floor_z_container = surface["floor_z_container"]
    message.ceiling_z_container = surface["ceiling_z_container"]
    message.origin_xy_container = list(surface["origin_xy_container"])
    message.resolution_m = surface["resolution_m"]
    message.geometry_hash = surface["geometry_hash"]
    message.map_revision = int(surface["map_revision"])
    identity = {
        "contract_version": 2, "frame_id": "container_link", "length_unit": "m",
        "stamp_ns": 0, "time_basis": "scene", "scene_revision": SCENE,
        "request_id": "surface",
    }
    _fill_identity(message.identity, identity)
    tops, states, confidence, valid = [], [], [], []
    floor = surface["floor_z_container"]
    for ix in range(rows):
        for iy in range(cols):
            top = surface["surface_z_container"][ix][iy]
            tops.append(floor if top is None else float(top))
            states.append(surface["state"][ix][iy])
            confidence.append(surface["confidence"][ix][iy])
            valid.append(True)
    message.surface_z_container = tops
    message.state = states
    message.confidence = confidence
    message.valid = valid


def _build_message(request):
    from luggage_msgs.srv import BuildMotionSequenceV2
    from luggage_planning.place_service import pose_message
    message = BuildMotionSequenceV2.Request()
    message.phase = "place"
    message.slot = pose_message(request["slots"][0])
    message.grasp = pose_message(_record(
        request["grasp"]["position"], request["grasp"]["quaternion_xyzw"],
        request["grasp"]["reference"], request["grasp"]["orientation_semantics"],
        "container_link", "grasp"))
    message.box_wdh = list(request["box_wdh"])
    transform = request["container_transform"]
    message.container_source_frame = transform["source_frame"]
    message.container_target_frame = transform["target_frame"]
    message.container_translation = list(transform["translation"])
    message.container_quaternion_xyzw = list(transform["quaternion_xyzw"])
    message.container_stamp_is_null = True
    return message


def dump_fixtures():
    DATA.mkdir(parents=True, exist_ok=True)
    specs = {
        "open_direct": _surface("open"),
        "blocked_corridor_detour": _surface("wall", gap=range(22, 34)),
        "blocked_destination": _surface("wall", gap=None),
    }
    for name, surface in specs.items():
        path = DATA / (name + ".json")
        path.write_text(json.dumps(surface) + "\n", encoding="utf-8")
        print("wrote", path)


if __name__ == "__main__":
    if "--dump" in sys.argv:
        dump_fixtures()
    else:
        unittest.main()
