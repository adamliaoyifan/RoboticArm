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


@pytest.mark.parametrize("limit", [2])
def test_retreat_twin_limit_is_bounded(limit):
    assert GraphRoutePlanner.RETREAT_TWIN_LIMIT == limit
