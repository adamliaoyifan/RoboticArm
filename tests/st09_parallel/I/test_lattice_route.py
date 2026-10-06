"""LatticeRoutePlanner: lattice chain -> PlannedRoute edges, no MoveIt."""

from __future__ import division

from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[3]
for _path in (ROOT, ROOT / "src" / "luggage_planning"):
    if str(_path) not in sys.path:
        sys.path.append(str(_path))

from st09_contracts.fixtures import make_attachment, make_snapshot  # noqa: E402
from st09_contracts.graph import GraphEdge, LoadState  # noqa: E402

from luggage_planning.corridor_lattice import (  # noqa: E402
    CorridorRequest,
    prepare_corridor,
    search_lazy,
)
from luggage_planning.grid_path_search import RoutePlanningError  # noqa: E402
from luggage_planning.lattice_route import (  # noqa: E402
    LatticeRoutePlanner,
    chain_segments,
    retime_positions,
)
from luggage_planning.reachable_graph import RouteGraph  # noqa: E402
from luggage_planning.route_producer import route_missing_parts  # noqa: E402

ANCHORS = {
    "portal": (0.00, 0.0, 0.10),
    "slot_approach": (0.02, 0.0, 0.10),
    "slot_upper": (0.04, 0.0, 0.08),
    "slot": (0.04, 0.0, 0.06),
}
LIMITS = tuple((-6.0, 6.0) for _ in range(6))
ORIENTATION = (0.0, 1.0, 0.0, 0.0)
HOME = (0.0, -0.2, 0.4, -1.0, 1.2, 0.3)


def _refine(position, _orientation, _seed):
    return [0.2 + position[2], -0.5 + position[0], 0.4, -1.0, 1.2, 0.3], {}


def _fk(q):
    return (float(q[0]), float(q[1]), float(q[2])), (0.0, 0.0, 0.0, 1.0)


class _Candidate(object):
    candidate_id = "cand-0"


class _Checker(object):
    def __init__(self, blocked=()):
        self.blocked = set(blocked)
        self.calls = []
        self.stats = {"service_calls": 0}

    def validate(self, states):
        self.calls.append(list(states))
        self.stats["service_calls"] += 1
        return {state: state not in self.blocked for state in states}


class _Builder(object):
    """Joint edges as two-state trajectories; optional refusals."""

    def __init__(self, refuse=(), refuse_q=()):
        self.refuse = set(refuse)
        self.refuse_q = set(refuse_q)
        self.calls = []

    def __call__(self, source, destination, edge_class, scene=None):
        self.calls.append((source.region, destination.region,
                           str(edge_class), scene))
        if (source.region, destination.region) in self.refuse \
                or source.q in self.refuse_q:
            raise RoutePlanningError("NO_FEASIBLE_ROUTE", {
                "kind": "moveit_error"})
        return GraphEdge(
            edge_id="e-%s-%s-%s" % (str(edge_class).lower(),
                                     source.node_id, destination.node_id),
            source_node_id=source.node_id,
            destination_node_id=destination.node_id,
            load_state=source.load_state,
            static_identity="fake-moveit",
            trajectory=retime_positions([source.q, destination.q], 0.3, 0.3),
            attributes={"edge_class": str(edge_class)})


def _planner(checker=None, builder=None, staging=(), staging_unloaded=()):
    checker = checker or _Checker()
    graph = RouteGraph(static_identity="test")
    planner = LatticeRoutePlanner(
        graph, builder or _Builder(),
        home_q=HOME, anchors=ANCHORS, orientation=ORIENTATION,
        joint_limits=LIMITS, refine=_refine, fk=_fk,
        checker_factory=lambda scene_states, start_q: checker,
        check_scene_states={"carrying": "C", "unloaded": "U"},
        build_scene_states={"carrying": "C", "unloaded": "U"},
        t_frame_container=(0.0, 0.0, 0.0),
        q_frame_container=(0.0, 0.0, 0.0, 1.0),
        staging_q=staging, staging_unloaded_q=staging_unloaded,
        candidate_id="cand-0")
    return planner, graph, checker


def _plan(planner):
    attachment = make_attachment("centered")
    return planner.plan_route(
        make_snapshot(map_revision=1), attachment, _Candidate(), [], int(15e9))


def _edges(graph, route):
    return [graph.edges[edge_id] for edge_id in
            route.entry_edge_ids + route.retreat_edge_ids]


def _reference_chain():
    request = CorridorRequest(
        anchors=ANCHORS, orientation=ORIENTATION, start_q=HOME,
        joint_limits=LIMITS)
    prepared = prepare_corridor(request, _refine)
    result, _ = search_lazy(
        prepared, lambda states: {state: True for state in states})
    return result


def test_route_classes_regions_and_loads_follow_the_corridor():
    planner, graph, _checker = _planner()
    route = _plan(planner)
    edges = _edges(graph, route)
    shape = [(edge.attributes["edge_class"],
              graph.node(edge.destination_node_id).region,
              str(edge.load_state)) for edge in edges]
    assert shape == [
        ("PORTAL_TRANSIT", "portal", "carrying"),
        ("TRAVERSE", "slot_approach", "carrying"),
        ("INSERT", "slot_upper", "carrying"),
        ("DESCEND", "slot", "carrying"),
        ("RETREAT", "slot_upper", "unloaded"),
        ("RETREAT", "slot_approach", "unloaded"),
        ("RETREAT", "portal", "unloaded"),
        ("RETURN", "home", "unloaded"),
    ]
    assert route_missing_parts(route, edges, graph) == []


def test_lattice_edges_carry_the_chain_states_in_order():
    planner, graph, _checker = _planner()
    route = _plan(planner)
    reference = _reference_chain()
    lattice = [edge for edge in _edges(graph, route)
               if edge.attributes.get("planner") == "corridor_lattice"]
    stitched = list(lattice[0].trajectory.positions)
    for edge in lattice[1:]:
        assert edge.trajectory.positions[0] == stitched[-1]
        stitched.extend(edge.trajectory.positions[1:])
    assert tuple(stitched) == tuple(
        tuple(float(v) for v in q) for q in reference.chain)
    for edge in lattice:
        times = edge.trajectory.time_from_start_ns
        assert all(b > a for a, b in zip(times, times[1:]))
        assert edge.trajectory.velocities[0] == (0.0,) * 6
        assert edge.trajectory.velocities[-1] == (0.0,) * 6


def test_joint_edges_join_home_and_the_lattice_ends():
    builder = _Builder()
    planner, graph, _checker = _planner(builder=builder)
    route = _plan(planner)
    edges = _edges(graph, route)
    assert edges[0].source_node_id == graph.node_for_q(
        LoadState.CARRYING, HOME).node_id
    assert edges[0].destination_node_id == edges[1].source_node_id
    assert edges[-1].destination_node_id == graph.node_for_q(
        LoadState.UNLOADED, HOME).node_id
    assert edges[-2].destination_node_id == edges[-1].source_node_id
    assert [(call[2], call[3]) for call in builder.calls] == [
        ("PORTAL_TRANSIT", "C"), ("RETURN", "U")]


def test_staging_falls_through_to_the_next_solution():
    staging_a = (0.1, -0.3, 0.3, -1.0, 1.2, 0.3)
    staging_b = (0.15, -0.3, 0.3, -1.0, 1.2, 0.3)
    builder = _Builder(refuse_q={staging_a})
    planner, graph, _checker = _planner(
        builder=builder, staging=(staging_a, staging_b),
        staging_unloaded=(staging_b,))
    route = _plan(planner)
    edges = _edges(graph, route)
    entry_staging = graph.node(edges[0].destination_node_id)
    assert entry_staging.region == "staging"
    assert entry_staging.q == staging_b
    classes = [edge.attributes["edge_class"] for edge in edges]
    assert classes[:2] == ["PORTAL_TRANSIT", "PORTAL_TRANSIT"]
    assert classes[-2:] == ["RETURN", "RETURN"]
    assert route_missing_parts(route, edges, graph) == []


def test_blocked_corridor_is_a_named_failure_without_moveit():
    reference = _reference_chain()
    builder = _Builder()
    slot_index = reference.layer_names.index("slot")
    blocked = {(reference.chain[slot_index], "carrying")}
    planner, _graph, _checker = _planner(
        checker=_Checker(blocked=blocked), builder=builder)
    with pytest.raises(RoutePlanningError) as caught:
        _plan(planner)
    assert caught.value.code == "NO_FEASIBLE_ROUTE"
    assert caught.value.detail["candidate_failure"] == "LATTICE_NO_CHAIN"
    assert builder.calls == []


def test_a_redraw_reuses_the_lattice_chain():
    builder = _Builder(refuse={("home", "portal")})
    planner, _graph, checker = _planner(builder=builder)
    with pytest.raises(RoutePlanningError):
        _plan(planner)
    calls = len(checker.calls)
    builder.refuse.clear()
    _plan(planner)
    assert len(checker.calls) == calls


def test_segments_drop_coincident_anchors():
    names = ("portal", "slot_approach", "slot_upper", "slot",
             "slot_upper_retreat", "slot_approach_retreat",
             "portal_retreat")
    a = (0.0,) * 6
    b = (0.1,) + (0.0,) * 5
    chain = (a, a, b, b, b, a, a)
    segments = chain_segments(names, chain)
    assert [(edge_class, region) for edge_class, region, *_ in segments] == [
        ("INSERT", "slot_upper"), ("RETREAT", "slot_approach")]
