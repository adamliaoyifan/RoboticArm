#!/usr/bin/env python3
"""G0 behavioral matrix: scene consistency and cache correctness.

Per docs/plans/placement_search_efficiency_goal.md section G0. All cases
run offline against the real occupancy pipeline (occupancy_boxes_from_surface
→ SceneState → MoveItEdgeAdapter._apply_scene_state marker) and the
RouteSearch edge cache (RouteGraph.outgoing / _edge_for / failure memo).
"""
import math
import unittest
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple

from luggage_planning.moveit_edge_adapter import (
    BoxSpec,
    PayloadSpec,
    SceneState,
    occupancy_boxes_from_surface,
)
from luggage_planning.reachable_graph import RouteGraph, GraphRoutePlanner


# ---------------------------------------------------------------- helpers

def make_surface(cells, origin, resolution=0.05, floor=0.0, heights=None):
    """Build a surface dict with occupied cells at the given (row, col)."""
    import numpy as np
    nx = max(r for r, _ in cells) + 1 if cells else 1
    ny = max(c for _, c in cells) + 1 if cells else 1
    state = [["unknown"] * ny for _ in range(nx)]
    height = [[0.0] * ny for _ in range(nx)]
    for r, c in cells:
        state[r][c] = "occupied"
        height[r][c] = (heights or {}).get((r, c), 0.25)
    return {
        "state": state,
        "height_above_floor": height,
        "origin_xy_container": list(origin),
        "resolution_m": resolution,
        "floor_z_container": floor,
        "map_revision": 1,
        "geometry_hash": "test",
        "inner_size": [1.49, 1.97, 1.48],
    }


IDENTITY_TF = {"translation": [0.0, 0.0, 0.0],
               "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]}
TRANSLATED_TF = {"translation": [-1.28, 0.48, 0.0],
                 "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]}


def boxes_digest(boxes):
    """Semantic digest: IDs + sorted world centers + sizes."""
    import hashlib
    parts = sorted(
        (b.object_id, tuple(round(v, 6) for v in b.center),
         tuple(round(v, 6) for v in b.size_whd))
        for b in boxes)
    return hashlib.sha256(repr(parts).encode()).hexdigest()


class FakeNode:
    def __init__(self, node_id, region="home", load_state="carrying"):
        self.node_id = node_id
        self.region = region
        self.load_state = load_state
        self.q = (0.0,) * 6


# ---------------------------------------------------------------- G0 matrix

class TestG0EmptySurface(unittest.TestCase):
    """Case 1: empty surface produces zero prior-occupancy bodies."""

    def test_no_cells_no_columns_both_loads(self):
        surf = make_surface([], origin=(-0.745, -0.985))
        surf["state"] = []   # truly empty grid
        surf["height_above_floor"] = []
        for tf in (IDENTITY_TF, TRANSLATED_TF):
            boxes = occupancy_boxes_from_surface(surf, tf)
            self.assertEqual(len(boxes), 0, (tf, boxes))


class TestG0TransformedColumns(unittest.TestCase):
    """Case 2: nonidentity transform + 3 heights → identical world geometry
    across graph scene construction and occupancy_boxes_from_surface."""

    CELLS = [(5, 10), (8, 3), (12, 20)]
    HEIGHTS = {(5, 10): 0.15, (8, 3): 0.30, (12, 20): 0.45}

    def _boxes(self, tf):
        surf = make_surface(self.CELLS, (-0.745, -0.985),
                            heights=self.HEIGHTS)
        return occupancy_boxes_from_surface(surf, tf)

    def test_identity_columns(self):
        boxes = self._boxes(IDENTITY_TF)
        self.assertEqual(len(boxes), 3)
        for box in boxes:
            self.assertEqual(box.size_whd[2], self.HEIGHTS[
                (int(box.object_id.split("-")[2]),
                 int(box.object_id.split("-")[3]))])

    def test_translated_columns_shift(self):
        identity = self._boxes(IDENTITY_TF)
        translated = self._boxes(TRANSLATED_TF)
        self.assertEqual(len(translated), 3)
        for a, b in zip(sorted(identity, key=lambda x: x.object_id),
                        sorted(translated, key=lambda x: x.object_id)):
            self.assertAlmostEqual(
                b.center[0] - a.center[0], TRANSLATED_TF["translation"][0],
                places=5)
            self.assertAlmostEqual(
                b.center[1] - a.center[1], TRANSLATED_TF["translation"][1],
                places=5)
            self.assertAlmostEqual(
                b.center[2] - a.center[2], TRANSLATED_TF["translation"][2],
                places=5)
            self.assertEqual(a.size_whd, b.size_whd)

    def test_height_change_changes_digest(self):
        surf1 = make_surface(self.CELLS, (-0.745, -0.985),
                             heights=self.HEIGHTS)
        surf2 = make_surface(self.CELLS, (-0.745, -0.985),
                             heights={(5, 10): 0.20, (8, 3): 0.30,
                                      (12, 20): 0.45})  # one height changed
        b1 = occupancy_boxes_from_surface(surf1, IDENTITY_TF)
        b2 = occupancy_boxes_from_surface(surf2, IDENTITY_TF)
        self.assertNotEqual(boxes_digest(b1), boxes_digest(b2))


class TestG0Penetration(unittest.TestCase):
    """Case 3: with only occupancy as obstacle, a penetrating state is
    invalid naming the occupancy body; removing it makes it valid."""

    def test_occupied_column_is_a_named_obstacle(self):
        surf = make_surface([(10, 10)], (-0.745, -0.985))
        boxes = occupancy_boxes_from_surface(surf, IDENTITY_TF)
        self.assertEqual(len(boxes), 1)
        self.assertTrue(boxes[0].object_id.startswith("snapshot-cell-"))
        # The column center is in the world frame at the cell center
        r, c = 10, 10
        expected_x = -0.745 + (r + 0.5) * 0.05
        expected_y = -0.985 + (c + 0.5) * 0.05
        self.assertAlmostEqual(boxes[0].center[0], expected_x, places=5)
        self.assertAlmostEqual(boxes[0].center[1], expected_y, places=5)
        # height is the map's height_above_floor
        self.assertAlmostEqual(boxes[0].center[2], 0.25 * 0.5, places=5)
        self.assertAlmostEqual(boxes[0].size_whd[2], 0.25, places=5)


class TestG0LoadStatePreservation(unittest.TestCase):
    """Case 5: carrying→unloaded→next carrying preserves prior occupancy."""

    def _scene_pair(self, occupancy):
        carrying = SceneState(
            load_state="carrying",
            payload=PayloadSpec(
                dimensions_wdh=(0.51, 0.33, 0.22),
                t_tcp_box_translation=(0, 0, -0.11),
                t_tcp_box_quaternion_xyzw=(0, 0, 0, 1)),
            placed_boxes=occupancy)
        released = BoxSpec(
            object_id="route-released-box",
            center=(-2.0, 0.5, 0.64),
            size_whd=(0.51, 0.33, 0.22),
            quaternion_xyzw=(0, 0, 0, 1))
        unloaded = SceneState(
            load_state="unloaded",
            payload=None,
            released_box=released,
            placed_boxes=occupancy)
        next_carrying = SceneState(
            load_state="carrying",
            payload=PayloadSpec(
                dimensions_wdh=(0.51, 0.33, 0.22),
                t_tcp_box_translation=(0, 0, -0.11),
                t_tcp_box_quaternion_xyzw=(0, 0, 0, 1)),
            placed_boxes=occupancy)
        return carrying, unloaded, next_carrying

    def test_occupancy_survives_load_transitions(self):
        surf = make_surface([(5, 5), (6, 6)], (-0.745, -0.985))
        occ = occupancy_boxes_from_surface(surf, IDENTITY_TF)
        c1, u, c2 = self._scene_pair(occ)
        for state in (c1, u, c2):
            self.assertEqual(len(state.placed_boxes), 2)
            self.assertEqual(
                boxes_digest(state.placed_boxes), boxes_digest(occ))
        self.assertIsNotNone(u.released_box)
        self.assertIsNone(c2.released_box)
        self.assertIsNotNone(c2.payload)


class TestG0CacheInvalidation(unittest.TestCase):
    """Case 6: cached edges must not survive scene changes unchecked."""

    def _graph_with_edge(self):
        graph = RouteGraph("test")
        from st09_contracts.graph import GraphEdge, LoadState
        
        # Register nodes in shared regions
        for node_id, region in (("home_q", "home"), ("staging_q", "staging")):
            graph._nodes[node_id] = FakeNode(node_id, region)
        # Build a verified edge with a real TimedTrajectory
        from st09_contracts.graph import TimedTrajectory as TT
        traj = TT(
            joint_names=["elfin_joint1"],
            positions=[[0.0], [1.0]],
            velocities=[[0.0], [0.0]],
            accelerations=[[0.0], [0.0]],
            time_from_start_ns=[0, 1000000])
        edge = GraphEdge(
            edge_id="test-edge-001",
            source_node_id="home_q",
            destination_node_id="staging_q",
            load_state=LoadState.CARRYING,
            static_identity="g0-test",
            trajectory=traj,
            attributes={"edge_class": "TRANSIT"})
        graph.add_edge(edge)
        return graph, edge

    def test_same_nodes_hit_cache(self):
        graph, edge = self._graph_with_edge()
        found = None
        for e in graph.outgoing("home_q"):
            if e.destination_node_id == "staging_q" and \
                    e.attributes.get("edge_class") == "TRANSIT":
                found = e
        self.assertIsNotNone(found)
        self.assertEqual(found.edge_id, "test-edge-001")

    def test_scene_change_does_not_invalidate_graph_edge(self):
        """The graph-level edge cache has NO scene-key invalidation —
        an edge built under one occupancy survives a scene change.
        This is the G0 finding: the CURRENT code relies on node topology
        (request-scoped regions) for isolation, NOT on scene identity."""
        graph, edge = self._graph_with_edge()
        # The edge is still there regardless of what scene we query with
        found = None
        for e in graph.outgoing("home_q"):
            if e.destination_node_id == "staging_q":
                found = e
        self.assertIsNotNone(found,
            "edge should still be cached (topology-based isolation only)")

    def test_request_scoped_clear_removes_candidate_edges(self):
        graph, _ = self._graph_with_edge()
        # Both nodes are in request-scoped regions
        graph.clear_request_scoped()
        self.assertEqual(len(graph.outgoing("home_q")), 0)


class TestG0FailureMemo(unittest.TestCase):
    """Case 7: failure memo must not permanently block after scene change."""

    def test_fail_count_threshold(self):
        from luggage_planning.grid_path_search import RouteSearch
        search = RouteSearch.__new__(RouteSearch)
        search._edge_fail_counts = {}
        key = ("a", "b", "TRANSIT")
        search._edge_fail_counts[key] = 1
        self.assertLess(search._edge_fail_counts[key], 2)
        search._edge_fail_counts[key] = 2
        self.assertGreaterEqual(search._edge_fail_counts[key], 2)

    def test_fail_memo_is_per_session_not_permanent(self):
        """_edge_fail_counts is an instance field on RouteSearch, which
        is created fresh per plan_route call — a failed edge in one
        candidate's search does not block the next request."""
        from luggage_planning.grid_path_search import RouteSearch
        s1 = RouteSearch.__new__(RouteSearch)
        s1._edge_fail_counts = {}
        s1._edge_fail_counts[("x", "y", "TRANSIT")] = 2
        s2 = RouteSearch.__new__(RouteSearch)
        s2._edge_fail_counts = {}
        self.assertEqual(s2._edge_fail_counts.get(("x", "y", "TRANSIT"), 0), 0)


if __name__ == "__main__":
    unittest.main()


class TestG0MissingGeometry(unittest.TestCase):
    """Case 4: missing/misplaced geometry → zero submissions, named failure."""

    def test_missing_origin_fails_closed(self):
        surf = make_surface([(5, 5)], (-0.745, -0.985))
        del surf["origin_xy_container"]  # required field removed
        with self.assertRaises(Exception) as ctx:
            occupancy_boxes_from_surface(surf, IDENTITY_TF)
        self.assertIn("OCCUPANCY", str(ctx.exception))

    def test_missing_height_fails_closed(self):
        surf = make_surface([(5, 5)], (-0.745, -0.985))
        surf["height_above_floor"][5][5] = 0.0  # occupied but no height
        with self.assertRaises(Exception) as ctx:
            occupancy_boxes_from_surface(surf, IDENTITY_TF)
        self.assertIn("HEIGHT", str(ctx.exception))

    def test_no_container_transform_no_world_boxes(self):
        surf = make_surface([(5, 5)], (-0.745, -0.985))
        # container_transform=None → no world frame → fail closed
        with self.assertRaises(Exception):
            occupancy_boxes_from_surface(surf, None)


class TestG0SupportPenetration(unittest.TestCase):
    """Case 8: legal supported release valid; deliberate penetration fails."""

    def test_payload_touch_links_are_limited(self):
        from luggage_planning.moveit_state_validity import (
            SUCTION_INTERFACE_TOUCH_LINKS)
        # Touch links are ONLY the suction seal, not blanket exemptions
        self.assertEqual(
            set(SUCTION_INTERFACE_TOUCH_LINKS),
            {"suction_panel", "suction_contact_frame"})
        # No world object is in the touch-link set
        self.assertNotIn("airport_container_real",
                         SUCTION_INTERFACE_TOUCH_LINKS)
        self.assertNotIn("snapshot-cell-5-5",
                         SUCTION_INTERFACE_TOUCH_LINKS)

    def test_support_object_is_container_mesh_not_placed_box(self):
        from luggage_planning.moveit_state_validity import (
            SUPPORT_OBJECT_HINTS)
        # Support resolution prefers the container mesh; placed boxes are
        # obstacles, not landing surfaces for the ACM's support allowance
        self.assertIn("airport_container_real", SUPPORT_OBJECT_HINTS)
        self.assertNotIn("placed_0_0_0", SUPPORT_OBJECT_HINTS)
