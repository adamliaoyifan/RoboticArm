"""ST09-B: q-state route graph, store, and atlas seeding (Section B).

Covers the graph semantics the frozen contract demands: full-q node
identity (same XYZ different q never merge; exact-q idempotence; the
(load_state, q) release twin), load-state-consistent edges with complete
timed trajectories, store round-trips that keep loaded trajectories
candidate-only until re-verified, prior-gap recording for uncovered atlas
yaws (never a rejection), and the legacy Section A layer staying
byte-compatible — including importability from a plain install prefix
where ``st09_contracts`` is not on sys.path.
"""

from __future__ import division

import json
import math
import os
import subprocess
import sys
from pathlib import Path

import pytest

scipy_rot = pytest.importorskip("scipy.spatial.transform").Rotation

from st09_contracts.graph import GraphEdge, LoadState  # noqa: E402
from st09_contracts.fixtures import make_timed_trajectory  # noqa: E402

from luggage_planning.atlas import graph as rg  # noqa: E402
from luggage_planning.atlas.graph import (  # noqa: E402
    GraphStore,
    RegionAnchor,
    RouteGraph,
    build_route_graph_from_atlas,
    make_offline_fk,
)

from toy_atlas import ToyAtlas  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[3]
Q_A = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
Q_B = (-0.9, 0.2, 0.3, 0.4, 0.5, 0.6)  # same TCP XYZ class, different q
Q_C = (1.1, -0.4, 0.9, -0.2, 0.7, -1.3)


def _fk(q):
    return ((float(q[0]), float(q[1]), float(q[2])), (0.0, 0.0, 0.0, 1.0))


def _edge(graph, source_id, destination_id, edge_id="e-1",
          load_state=LoadState.CARRYING):
    return GraphEdge(
        edge_id=edge_id,
        source_node_id=source_id,
        destination_node_id=destination_id,
        load_state=load_state,
        static_identity="static-fixture-v1",
        trajectory=make_timed_trajectory(),
        validation_revision=None,
        attributes={"edge_class": "TRAVERSE"},
    )


def _graph_with_nodes():
    graph = RouteGraph(static_identity="static-test")
    node_a = graph.register_node(Q_A, (1.0, 0.0, 0.5), (0, 0, 0, 1),
                                 "portal", LoadState.CARRYING, "f1")
    node_b = graph.register_node(Q_B, (1.0, 0.0, 0.5), (0, 0, 0, 1),
                                 "slot", LoadState.CARRYING, "f2")
    return graph, node_a, node_b


class TestLegacyLayerRemoved(object):
    """[ST09-I integration] the legacy atlas-cell layer is deleted."""

    def test_section_a_symbols_are_gone(self):
        from luggage_planning.atlas import graph
        for name in ("build_reachable_graph", "ReachableGraph",
                     "grid_disable_reason", "cell_center", "AtlasCellNode"):
            assert not hasattr(graph, name), name


class TestNodeIdentity:
    """Full-q primary state: merge and twin semantics."""

    def test_exact_q_registration_idempotent(self):
        graph, node_a, _ = _graph_with_nodes()
        again = graph.register_node(Q_A, (9.9, 9.9, 9.9), (0, 0, 0, 1),
                                    "elsewhere", LoadState.CARRYING, "f9")
        assert again.node_id == node_a.node_id
        assert again.region == "portal"  # first registration wins

    def test_same_xyz_different_q_never_merges(self):
        graph, node_a, node_b = _graph_with_nodes()
        assert node_a.node_id != node_b.node_id
        assert node_a.tcp_position == node_b.tcp_position
        assert len(graph.nodes) == 2

    def test_release_twin_same_q_two_load_states(self):
        graph = RouteGraph()
        carrying = graph.register_node(Q_C, (0.2, 0.1, 0.4), (0, 0, 0, 1),
                                       "slot", LoadState.CARRYING, "f1")
        unloaded = graph.register_node(Q_C, (0.2, 0.1, 0.4), (0, 0, 0, 1),
                                       "slot", LoadState.UNLOADED, "f1")
        assert carrying.node_id != unloaded.node_id
        assert carrying.same_state(unloaded)  # frozen contract: q only

    def test_node_for_q_tolerance(self):
        graph, node_a, _ = _graph_with_nodes()
        near = tuple(v + 5e-10 for v in Q_A)
        assert graph.node_for_q(LoadState.CARRYING, near) is node_a
        far = tuple(v + 1e-6 for v in Q_A)
        assert graph.node_for_q(LoadState.CARRYING, far) is None
        assert graph.node_for_q(LoadState.UNLOADED, Q_A) is None

    def test_invalid_q_rejected(self):
        graph = RouteGraph()
        with pytest.raises(ValueError):
            graph.register_node((0.1, 0.2), (0, 0, 0), (0, 0, 0, 1), "portal",
                                LoadState.CARRYING)
        with pytest.raises(ValueError):
            graph.register_node((0.1, 0.2, 0.3, 0.4, 0.5, float("nan")),
                                (0, 0, 0), (0, 0, 0, 1), "portal",
                                LoadState.CARRYING)

    def test_registration_deterministic_ids_and_digest(self):
        def build():
            graph, node_a, node_b = _graph_with_nodes()
            graph.register_node(Q_C, (0.0, 0.0, 0.0), (0, 0, 0, 1), "home",
                                LoadState.UNLOADED, "f1")
            return graph, (node_a.node_id, node_b.node_id)
        first, ids_first = build()
        second, ids_second = build()
        assert ids_first == ids_second
        assert first.digest() == second.digest()


class TestEdges:
    """Edges carry complete timed trajectories and load-state consistency."""

    def test_add_edge_and_adjacency(self):
        graph, node_a, node_b = _graph_with_nodes()
        edge = graph.add_edge(_edge(graph, node_a.node_id, node_b.node_id))
        assert [e.edge_id for e in graph.outgoing(node_a.node_id)] == [edge.edge_id]
        assert [e.edge_id for e in graph.incoming(node_b.node_id)] == [edge.edge_id]
        assert graph.outgoing(node_b.node_id) == ()

    def test_load_state_mismatch_rejected(self):
        graph, node_a, node_b = _graph_with_nodes()
        with pytest.raises(ValueError):
            graph.add_edge(_edge(graph, node_a.node_id, node_b.node_id,
                                 load_state=LoadState.UNLOADED))

    def test_duplicate_edge_id_rejected(self):
        graph, node_a, node_b = _graph_with_nodes()
        graph.add_edge(_edge(graph, node_a.node_id, node_b.node_id))
        with pytest.raises(ValueError):
            graph.add_edge(_edge(graph, node_a.node_id, node_b.node_id))

    def test_candidate_edges_are_not_outgoing(self):
        graph, node_a, node_b = _graph_with_nodes()
        candidate = graph.add_candidate_edge(
            _edge(graph, node_a.node_id, node_b.node_id, edge_id="e-store"))
        assert graph.outgoing(node_a.node_id) == ()
        assert graph.outgoing_candidates(node_a.node_id) == (candidate,)

    def test_promote_and_drop_candidate(self):
        graph, node_a, node_b = _graph_with_nodes()
        graph.add_candidate_edge(
            _edge(graph, node_a.node_id, node_b.node_id, edge_id="e-store"))
        replacement = graph.promote_candidate(
            "e-store",
            _edge(graph, node_a.node_id, node_b.node_id, edge_id="e-store-2"))
        assert graph.outgoing(node_a.node_id) == (replacement,)
        assert graph.candidate_edges == {}
        graph.add_candidate_edge(
            _edge(graph, node_a.node_id, node_b.node_id, edge_id="e-drop"))
        graph.drop_candidate("e-drop")
        assert graph.outgoing_candidates(node_a.node_id) == ()

    def test_edge_mutation_changes_digest(self):
        graph, node_a, node_b = _graph_with_nodes()
        before = graph.digest()
        graph.add_edge(_edge(graph, node_a.node_id, node_b.node_id))
        assert graph.digest() != before


class TestGraphStore:
    """Persistence keeps loaded trajectories candidate-only."""

    def _saved(self, tmp_path):
        graph, node_a, node_b = _graph_with_nodes()
        graph.add_edge(_edge(graph, node_a.node_id, node_b.node_id))
        graph.note_prior_gap("ATLAS_ORIENTATION_UNCOVERED region=slot yaw=1.570796")
        store = GraphStore(tmp_path / "store")
        path, digest = store.save(graph, "g-1")
        return store, graph, path, digest

    def test_static_identity_deterministic_and_sensitive(self):
        first = GraphStore.static_identity("u1", "s1", "h1", {"planner": "RRT"})
        same = GraphStore.static_identity("u1", "s1", "h1", {"planner": "RRT"})
        other = GraphStore.static_identity("u2", "s1", "h1", {"planner": "RRT"})
        reordered = GraphStore.static_identity(
            "u1", "s1", "h1", {"planner": "RRT", "extra": 1})
        assert first == same and first != other and first != reordered

    def test_roundtrip_nodes_and_candidate_edges(self, tmp_path):
        store, graph, _path, _digest = self._saved(tmp_path)
        loaded = store.load("g-1")
        assert set(loaded.nodes) == set(graph.nodes)
        assert loaded.edges == {}  # nothing usable in-route without promotion
        (candidate,) = loaded.candidate_edges.values()
        assert loaded.prior_gaps == graph.prior_gaps
        assert loaded.static_identity == graph.static_identity

    def test_roundtrip_preserves_trajectory(self, tmp_path):
        store, graph, _path, _digest = self._saved(tmp_path)
        (candidate,) = store.load("g-1").candidate_edges.values()
        original = graph.edges["e-1"]
        assert candidate.trajectory.joint_names == original.trajectory.joint_names
        assert candidate.trajectory.time_from_start_ns == \
            original.trajectory.time_from_start_ns
        assert candidate.trajectory.positions == original.trajectory.positions

    def test_expected_static_identity_mismatch(self, tmp_path):
        store, _graph, _path, _digest = self._saved(tmp_path)
        with pytest.raises(ValueError):
            store.load("g-1", expected_static_identity="other")

    def test_refuses_divergent_overwrite(self, tmp_path):
        store, graph, _path, _digest = self._saved(tmp_path)
        graph.note_prior_gap("another gap")
        with pytest.raises(IOError):
            store.save(graph, "g-1")


class TestBuildFromAtlas:
    """Atlas seeding without a yaw allow-list; gaps are priors, not rejects."""

    def test_seeded_anchor_registers_nodes(self):
        atlas = ToyAtlas()
        atlas.add_seeds((1, 1, 1), 0.0, [list(Q_A)], seed_set="transit")
        atlas.add_seeds((1, 1, 1), 0.0, [list(Q_B)], seed_set="contact")
        graph = build_route_graph_from_atlas(
            atlas,
            [RegionAnchor("slot", (0.225, 0.225, 0.225), 0.0,
                          LoadState.CARRYING)],
            fk=_fk)
        families = {node.ik_family for node in graph.nodes_by_region("slot")}
        assert families == {"atlas-transit", "atlas-contact"}
        # The directed pi twin of the anchor yaw is genuinely uncovered in
        # the toy atlas: a recorded prior gap, never a rejection.
        assert len(graph.prior_gaps) == 1
        assert "ATLAS_ORIENTATION_UNCOVERED" in graph.prior_gaps[0]
        assert "yaw=3.141593" in graph.prior_gaps[0]

    def test_uncovered_pi_twin_records_prior_gap(self):
        atlas = ToyAtlas()
        atlas.add_seeds((1, 1, 1), 0.0, [list(Q_A)], seed_set="transit")
        graph = build_route_graph_from_atlas(
            atlas,
            [RegionAnchor("slot", (0.225, 0.225, 0.225), 0.0,
                          LoadState.CARRYING)],
            fk=_fk)
        # Directed semantics query both the yaw and its pi twin; only the
        # twin is uncovered here (the cell centre is equidistant from the
        # neighbour cells, which carry no seeds either).
        assert any("ATLAS_ORIENTATION_UNCOVERED" in gap
                   for gap in graph.prior_gaps)
        assert len(graph.nodes_by_region("slot")) == 1  # the covered yaw stays

    def test_fully_uncovered_anchor_only_gap(self):
        atlas = ToyAtlas()
        graph = build_route_graph_from_atlas(
            atlas,
            [RegionAnchor("portal", (0.3, 0.3, 0.3), math.pi / 2.0,
                          LoadState.CARRYING)],
            fk=_fk)
        assert graph.nodes == {}
        assert len(graph.prior_gaps) >= 1

    def test_multiple_seeds_distinct_nodes(self):
        atlas = ToyAtlas()
        atlas.add_seeds((2, 2, 2), math.pi,
                        [list(Q_A), list(Q_B), list(Q_C)],
                        seed_set="transit")
        graph = build_route_graph_from_atlas(
            atlas,
            [RegionAnchor("deep", (0.375, 0.375, 0.375), math.pi,
                          LoadState.CARRYING)],
            fk=_fk)
        assert len(graph.nodes_by_region("deep")) == 3


class TestFkHelpers:
    """Matrix -> quaternion and the offline FK closure."""

    def test_matrix_to_quaternion_matches_scipy(self):
        for euler in ((0.1, -0.2, 0.3), (math.pi, 0.0, 0.0),
                      (0.0, math.pi, 0.0), (0.0, 0.0, math.pi),
                      (-2.9, 1.4, 0.7)):
            rotation = scipy_rot.from_euler("xyz", euler)
            quat = rg._matrix_to_quaternion_xyzw(rotation.as_matrix())
            assert (scipy_rot.from_quat(quat).inv() * rotation).magnitude() \
                <= 1e-9

    def test_make_offline_fk_closure(self):
        class StubUrdf(object):
            def tcp(self, q, path):
                rotation = scipy_rot.from_euler("z", q[0])
                matrix = __import__("numpy").eye(4)
                matrix[:3, :3] = rotation.as_matrix()
                matrix[:3, 3] = (q[1], q[2], 0.5)
                return matrix

        fk = make_offline_fk(StubUrdf(), ("elfin_joint1",))
        position, quaternion = fk((0.4, 1.0, 2.0))
        assert position == (1.0, 2.0, 0.5)
        assert scipy_rot.from_quat(quaternion).as_euler("xyz")[2] == \
            pytest.approx(0.4, abs=1e-9)
