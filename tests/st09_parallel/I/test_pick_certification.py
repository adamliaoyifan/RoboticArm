#!/usr/bin/env python3
"""[ST09-I S5.7-4 §4'] Pick-ladder certification.

The pick waypoint ladder stays the sanctioned design, but its legs now
leave the service only as EXPLICITLY-unloaded certified motion: an
absent attachment is never silently read as "unloaded" (the admission
is LoadState.UNLOADED bound to UNLOADED_ATTACHMENT_DIGEST), and a
failed certification is a named CERT_* refusal — never a degraded
execution.
"""
import os
import sys
from types import SimpleNamespace

import pytest

_ROOT = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
for entry in ("src/luggage_planning", "src/luggage_perception"):
    path = os.path.join(_ROOT, entry)
    if path not in sys.path:
        sys.path.append(path)

from luggage_planning.routing.producer import (  # noqa: E402
    certify_unloaded_legs,
    pick_leg_records,
)
from luggage_planning.motion.trajectory_certifier import (  # noqa: E402
    CertificationPolicy,
    TrajectoryCertifier,
)
from luggage_planning.planning_contract.attachment import UNLOADED_ATTACHMENT_DIGEST  # noqa: E402
from luggage_planning.planning_contract.fixtures import (  # noqa: E402
    make_attachment,
    make_graph_fixture,
    make_snapshot,
)
from luggage_planning.planning_contract.reasons import Reason  # noqa: E402


class _Traj(object):
    joint_names = tuple("elfin_joint%d" % i for i in range(1, 7))

    def __init__(self):
        self.positions = ((0.0,) * 6, (0.05,) * 6)
        self.velocities = ((0.0,) * 6, (0.01,) * 6)
        self.accelerations = ((0.0,) * 6, (0.01,) * 6)
        self.time_from_start_ns = (0, 200_000_000)


class _Edge(object):
    def __init__(self, name):
        self.edge_id = "pick-%s" % name
        self.trajectory = _Traj()
        self.attributes = {}

    def digest(self):
        return "digest-%s" % self.edge_id


class _Graph(object):
    def __init__(self):
        self.nodes = []

    def register_node(self, q, position, quaternion, name, load, mode):
        node = SimpleNamespace(
            node_id="%s-%d" % (name, len(self.nodes)), q=q)
        self.nodes.append(node)
        return node


_WAYPOINTS = (
    ("pre_grasp", (0.1, 0.2, 0.3), (0.0, 0.0, 0.0, 1.0)),
    ("approach", (0.1, 0.2, 0.25), (0.0, 0.0, 0.0, 1.0)),
)


def _pick_stack():
    graph = _Graph()
    edge_builder = lambda current, node, klass: _Edge(  # noqa: E731
        "%s-%s" % (node.node_id, klass))
    refine = lambda pos, quat, seed: (list(seed), {})  # noqa: E731
    fk = lambda q: ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))  # noqa: E731
    return graph, edge_builder, refine, fk


def test_pick_leg_records_returns_edges_for_certification():
    graph, edge_builder, refine, fk = _pick_stack()
    legs, edges = pick_leg_records(
        _WAYPOINTS, graph=graph, edge_builder=edge_builder,
        refine=refine, fk=fk, start_q=(0.0,) * 6,
        request_id="req-1", scene_revision="scene-A", stamp_ns=5)
    assert [leg["name"] for leg in legs] == ["pre_grasp", "approach"]
    assert len(edges) == len(_WAYPOINTS)
    assert all(edge.trajectory.positions for edge in edges)


def _fixture_edges():
    _, base_edges = make_graph_fixture(make_attachment("centered"))
    return tuple(base_edges)


def _certify(edges, adapter):
    snapshot = make_snapshot(map_revision=2, observed_free=[(1, 1)])
    result, route = certify_unloaded_legs(
        edges=edges, snapshot=snapshot, request_id="req-1",
        route_id="route-pick-1", purpose="pick",
        certifier=TrajectoryCertifier(), validity_adapter=adapter,
        policy=CertificationPolicy(), now_ns=2_000_000_000)
    return result, route


def test_certify_unloaded_legs_passes_and_binds_unloaded_digest():
    from luggage_planning.motion.moveit_validity_adapter import (
        StaticMoveItValidityAdapter,
        ValiditySample,
    )

    edges = _fixture_edges()
    samples = {}
    for edge in edges:
        for index in range(400):
            samples[(edge.edge_id, index)] = ValiditySample(
                edge_id=edge.edge_id, sample_index=index, valid=True,
                min_clearance_m=0.06, contacts=())
    result, route = _certify(edges, StaticMoveItValidityAdapter(samples))
    assert result.passed
    assert route.attachment_digest == UNLOADED_ATTACHMENT_DIGEST
    assert route.entry_edge_ids == tuple(
        edge.edge_id for edge in edges)


def test_certify_unloaded_legs_collision_is_named_refusal():
    from luggage_planning.motion.moveit_validity_adapter import (
        StaticMoveItValidityAdapter,
        ValiditySample,
    )

    edges = _fixture_edges()
    samples = {}
    for edge in edges:
        for index in range(400):
            samples[(edge.edge_id, index)] = ValiditySample(
                edge_id=edge.edge_id, sample_index=index, valid=False,
                min_clearance_m=-0.01,
                contacts=({"type": "mesh"},))
    result, _route = _certify(edges, StaticMoveItValidityAdapter(samples))
    assert not result.passed
    assert result.reason is Reason.CERT_COLLISION


def test_certify_unloaded_legs_requires_edges():
    snapshot = make_snapshot(map_revision=2)
    with pytest.raises(ValueError):
        certify_unloaded_legs(
            edges=(), snapshot=snapshot, request_id="req-1",
            route_id="route-pick-1", purpose="pick",
            certifier=TrajectoryCertifier(), validity_adapter=None,
            policy=CertificationPolicy(), now_ns=0)


if __name__ == "__main__":
    import unittest

    unittest.main()
