"""Behaviour tests for the carrying-anchor payload gate and its planner hooks."""

import types

import pytest

from luggage_planning.anchor_payload_gate import (
    REASON_COLLISION, REASON_NOT_EVALUATED, AnchorPayloadGate)
from st09_contracts.graph import LoadState
from luggage_planning.reachable_graph import GraphRoutePlanner, RouteGraph


def _node(node_id, q, region="portal"):
    return types.SimpleNamespace(
        node_id=node_id, q=tuple(q), region=region, load_state="carrying")


class _Checker(object):
    def __init__(self, bad_q=(), fail=False):
        self.bad = {tuple(q) for q in bad_q}
        self.fail = fail
        self.calls = 0

    def validate(self, items):
        self.calls += 1
        if self.fail:
            raise TimeoutError("checker timeout")
        return {item: item[0] not in self.bad for item in items}


def _gate(checker):
    return AnchorPayloadGate(lambda key: checker)


def test_colliding_anchor_rejected_with_name_and_valid_one_kept():
    checker = _Checker(bad_q=[(1.0,) * 6])
    nodes = [_node("a", (1.0,) * 6), _node("b", (2.0,) * 6)]
    result = _gate(checker).screen(nodes, "scene-1", slot_index=7)
    assert result.kept_node_ids == {"b"}
    verdicts = {row["node_id"]: row for row in result.rows}
    assert verdicts["a"]["reason"] == REASON_COLLISION
    assert verdicts["a"]["slot_index"] == 7
    assert verdicts["b"]["verdict"] == "accepted"


def test_repeated_target_is_not_resubmitted():
    checker = _Checker()
    gate = _gate(checker)
    gate.screen([_node("a", (1.0,) * 6)], "scene-1")
    gate.screen([_node("a2", (1.0,) * 6)], "scene-1")
    assert checker.calls == 1
    assert gate.cache_hits == 1


def test_scene_change_is_judged_independently():
    checker = _Checker(bad_q=[(1.0,) * 6])
    gate = _gate(checker)
    gate.screen([_node("a", (1.0,) * 6)], "scene-1")
    gate.screen([_node("a", (1.0,) * 6)], "scene-2")
    assert checker.calls == 2


def test_checker_failure_is_not_evaluated_not_collision():
    result = _gate(_Checker(fail=True)).screen(
        [_node("a", (1.0,) * 6)], "scene-1")
    row = result.rows[0]
    assert row["verdict"] == "not_evaluated"
    assert row["reason"] == REASON_NOT_EVALUATED
    assert "a" in result.kept_node_ids


def test_registration_screen_keeps_rejected_node_out_of_scope():
    graph = RouteGraph()
    planner = GraphRoutePlanner(
        graph, edge_builder=None, home_node_id="h", scope_node_ids=set(),
        payload_screen=lambda node: True)
    node = planner._register(
        (1.0,) * 6, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), "portal",
        LoadState.CARRYING, "ladder")
    assert node is None
    assert planner.scope == set()


def test_registration_screen_does_not_judge_unloaded_or_slot_nodes():
    graph = RouteGraph()
    seen = []
    planner = GraphRoutePlanner(
        graph, edge_builder=None, home_node_id="h", scope_node_ids=set(),
        payload_screen=lambda node: seen.append(node) or True)
    unloaded = planner._register(
        (1.0,) * 6, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), "portal",
        LoadState.UNLOADED, "frontier")
    slot = planner._register(
        (2.0,) * 6, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), "slot",
        LoadState.CARRYING, "frontier")
    assert unloaded is not None and slot is not None
    assert seen == []


# --- digest completeness (real moveit_msgs) -------------------------------

def _scene(vertex_x=0.0, default=False):
    from geometry_msgs.msg import Point
    from moveit_msgs.msg import CollisionObject, PlanningScene
    from shape_msgs.msg import Mesh, MeshTriangle

    scene = PlanningScene()
    obj = CollisionObject()
    obj.id = "container"
    mesh = Mesh()
    mesh.vertices = [Point(x=vertex_x, y=0.0, z=0.0), Point(x=1.0),
                     Point(y=1.0)]
    triangle = MeshTriangle()
    triangle.vertex_indices = [0, 1, 2]
    mesh.triangles = [triangle]
    obj.meshes = [mesh]
    scene.world.collision_objects = [obj]
    scene.allowed_collision_matrix.default_entry_names = ["a"]
    scene.allowed_collision_matrix.default_entry_values = [default]
    return scene


def test_digest_changes_with_mesh_vertex_and_acm_default():
    from luggage_planning.anchor_payload_gate import live_scene_digest

    base = live_scene_digest(_scene())
    assert live_scene_digest(_scene()) == base
    assert live_scene_digest(_scene(vertex_x=0.1)) != base
    assert live_scene_digest(_scene(default=True)) != base


# --- producer checker lifetime (production composition path) ---------------

class _LiveChecker(object):
    """Checker whose digest is the scene it was built against."""

    def __init__(self, scene_ref, log):
        self.scene_ref = scene_ref
        self.prepared_digest = None
        self.prepared_scene = None
        self.log = log

    def live_scene_digest(self):
        digest = self.scene_ref["digest"]
        if self.prepared_scene is None:
            self.prepared_scene = digest
            self.prepared_digest = digest
        return digest

    def validate(self, items):
        self.log.append(self.prepared_scene)
        bad = self.prepared_scene == "scene-B"
        return {item: not bad for item in items}


def _producer_double(scene_ref, log):
    from luggage_planning.moveit_edge_adapter import PayloadSpec, SceneState
    from luggage_planning.route_producer import RouteProducer

    state = SceneState(
        load_state="carrying",
        payload=PayloadSpec(
            dimensions_wdh=(0.1, 0.1, 0.1),
            t_tcp_box_translation=(0.0, 0.0, 0.0),
            t_tcp_box_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0)))
    graph = RouteGraph()
    node = graph.register_node(
        (1.0,) * 6, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), "portal",
        LoadState.CARRYING, ik_family="anchor")
    fake = types.SimpleNamespace(
        corridor_planner="graph", graph=graph, anchor_provenance=[],
        lattice_checker_factory=lambda **_k: _LiveChecker(scene_ref, log),
        edge_scene_states=lambda *a, **k: {"anchor-carrying": state},
        _anchor_checkers={}, _anchor_checkers_by_full={}, _anchor_gate=None,
        _payload_screen_fn=None, _log=lambda *_a: None)
    return RouteProducer._screen_carrying_anchors, fake, node


def test_new_live_scene_gets_a_new_checker_and_verdict():
    scene_ref = {"digest": "scene-A"}
    log = []
    screen, fake, node = _producer_double(scene_ref, log)
    kept, _rows = screen(fake, {node.node_id}, None, None, object(), None,
                         (0.0,) * 6, 1)
    assert node.node_id in kept
    scene_ref["digest"] = "scene-B"
    kept, rows = screen(fake, {node.node_id}, None, None, object(), None,
                        (0.0,) * 6, 2)
    assert node.node_id not in kept
    assert rows[0]["reason"] == REASON_COLLISION
    assert log == ["scene-A", "scene-B"]


# --- retreat twin semantics -------------------------------------------------

def _twin_planner(entry_ids, scope):
    graph = RouteGraph()
    planner = GraphRoutePlanner(
        graph, edge_builder=None, home_node_id="h", scope_node_ids=scope)
    planner.last_search = types.SimpleNamespace(entry_node_ids=entry_ids)
    return graph, planner


def _retreat(planner, frontier):
    hop = types.SimpleNamespace(to_region="portal", edge_class="RETREAT")
    return planner._retreat_twins(
        frontier, hop, (0.0, 0.0, 1.0), LoadState)


def _portal(graph, q0, quat):
    return graph.register_node(
        (q0,) * 6, (0.0, 0.0, 1.0), quat, "portal", LoadState.CARRYING,
        ik_family="ladder")


def test_retreat_twin_requires_same_orientation_and_entry_chain():
    graph, planner = _twin_planner((), set())
    same = _portal(graph, 1.0, (0.0, 0.0, 0.0, 1.0))
    flipped = _portal(graph, 2.0, (0.0, 0.0, 1.0, 0.0))
    foreign = _portal(graph, 3.0, (0.0, 0.0, 0.0, 1.0))
    frontier = types.SimpleNamespace(
        q=(1.0,) * 6, tcp_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0))
    planner.last_search.entry_node_ids = (
        same.node_id, flipped.node_id, foreign.node_id)
    planner.scope = {same.node_id, flipped.node_id}
    planner.last_search.entry_node_ids = (same.node_id, flipped.node_id)
    result = _retreat(planner, frontier)
    assert [node.q for node in result] == [same.q]
    assert all(str(node.load_state) == "unloaded" for node in result)


def test_retreat_twin_ignores_nodes_outside_entry_chain():
    graph, planner = _twin_planner((), None)
    _portal(graph, 1.0, (0.0, 0.0, 0.0, 1.0))
    frontier = types.SimpleNamespace(
        q=(1.0,) * 6, tcp_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0))
    assert _retreat(planner, frontier) == ()
