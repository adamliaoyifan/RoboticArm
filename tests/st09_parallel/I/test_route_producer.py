"""ST09-I S2 offline: the route producer composes the admitted route.

Fakes stand in for MoveIt (edge builder) and the validity reports; every
other collaborator is the real component: D's snapshot bridge and
measured attachment provider, B's graph/search/planner, C's certifier,
revalidator, and the frozen v2 segment admission the legs must survive.
"""

from __future__ import division

import dataclasses
import pytest
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
# APPEND, never insert at 0: a front src/luggage_planning entry flips
# this process's `luggage_planning` resolution from the install overlay
# to the source tree, and the whole-chain probe (running in-process)
# then reports src-path module provenance which every stack node's
# overlay gate rejects (live: all peers WRONG_OVERLAY(path), 180 s
# startup budget lost). The tail entry only serves a bare `pytest`
# without the documented PYTHONPATH.
if str(ROOT / "src" / "luggage_planning") not in sys.path:
    sys.path.append(str(ROOT / "src" / "luggage_planning"))

from st09_contracts._base import ContractViolation  # noqa: E402
from st09_contracts.fixtures import make_timed_trajectory  # noqa: E402
from st09_contracts.graph import GraphEdge, LoadState  # noqa: E402

from luggage_planning.attachment_provider import (  # noqa: E402
    MeasuredAttachmentProvider,
)
from luggage_planning.grid_path_search import RoutePlanningError  # noqa: E402
from luggage_planning.map_revalidation import (  # noqa: E402
    MapRevalidator,
    RevalidationPolicy,
)
from luggage_planning.moveit_state_validity import sample_states  # noqa: E402
from luggage_planning.moveit_validity_adapter import (  # noqa: E402
    StaticMoveItValidityAdapter,
    ValiditySample,
)
from luggage_planning.reachable_graph import RouteGraph  # noqa: E402
from luggage_planning.atlas_prior import (  # noqa: E402
    prior_for_targets,
    prior_from_statuses,
    replay_captured_candidates,
)
from luggage_planning.route_producer import (  # noqa: E402
    RouteProducer,
    _soft_sampled_unreachable,
    admit_attachment_for_route,
    attempt_tier,
    certification_exceeded,
    candidate_from_slot,
    leg_records,
    near_opening_cut,
    register_attach_from_request,
    route_missing_parts,
    surface_map_from_contract,
)
from luggage_planning.trajectory_certifier import (  # noqa: E402
    TrajectoryCertifier,
)

START_Q = (0.05, -0.15, 0.90, -0.40, 1.20, 0.00)


class FakeEdgeBuilder(object):
    """Deterministic offline edge builder through the B protocol."""

    def __init__(self, fail_dest=None, fail_all=False, fail_descend_x=None):
        self.fail_dest = set(fail_dest or ())
        self.fail_all = bool(fail_all)
        #: fail every slot-region DESCEND whose target TCP x is above
        #: this bound (candidate-discriminating across goal branches).
        self.fail_descend_x = fail_descend_x
        self.collision_calls = 0
        self.calls = []

    def __call__(self, from_node, to_node, edge_class, candidate=None,
                 scene=None):
        self.calls.append((from_node.node_id, to_node.node_id, edge_class))
        failed = self.fail_all or edge_class in getattr(
            self, "fail_classes", ()) or (to_node.node_id, edge_class) in \
            self.fail_dest
        if (not failed and self.fail_descend_x is not None
                and to_node.region == "slot" and edge_class == "DESCEND"
                and to_node.tcp_position[0] > self.fail_descend_x):
            failed = True
        if (not failed and getattr(self, "fail_collision_x", None) is not None
                and to_node.region == "slot" and edge_class == "DESCEND"
                and to_node.tcp_position[0] > self.fail_collision_x):
            self.collision_calls += 1
            raise RoutePlanningError("EDGE_BUILD_FAILED", {
                "kind": "collision",
                "edge_class": "DESCEND",
            })
        if failed:
            raise RoutePlanningError(
                "EDGE_BUILD_FAILED", {"pair": edge_class})
        return GraphEdge(
            edge_id="e-i-%03d" % len(self.calls),
            source_node_id=from_node.node_id,
            destination_node_id=to_node.node_id,
            load_state=from_node.load_state,
            static_identity="static-i",
            trajectory=make_timed_trajectory(),
            validation_revision=None,
            attributes={"edge_class": edge_class,
                        "duration_ns": 100_000_000,
                        "joint_travel_rad": 0.2})


class FakeClock(object):
    def __init__(self):
        self.now_ns = 1_000_000_000_000

    def __call__(self):
        return self.now_ns

    def advance_s(self, seconds):
        self.now_ns += int(seconds * 1e9)


def fake_refine(position, quaternion, seed):
    """Distinct deterministic q per anchor position (keeps nodes apart)."""
    q = [float(v) for v in seed]
    for index in range(3):
        q[index] += 0.02 * float(position[index % 3])
    q[4] += 0.01 * float(position[1])
    return q, {"pos_err_m": 0.0, "rot_err_rad": 0.0}


def fake_fk(q):
    return ((0.30 + 0.1 * q[0], 0.05 * q[1], 0.40 + 0.05 * q[4]),
            (0.0, 0.0, 0.0, 1.0))


def make_provider(clock):
    return MeasuredAttachmentProvider(
        "scene-1", "calib-1",
        tcp_frame="suction_contact_frame", box_frame="box")


def make_request(slots=None, stamp_s=1000.0):
    return {
        "request_id": "req-i-1",
        "scene_revision": "scene-1",
        "box_wdh": [0.30, 0.20, 0.20],
        "slots": slots if slots is not None else [{
            "position": [0.20, -0.10, 0.14],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            "request_id": "cand-0",
        }],
        "slot_scores": [1.0],
        "surface": {
            "frame_id": "container_link",
            "resolution_m": 0.05,
            "rows": 2,
            "columns": 2,
            "origin_xy_container": [0.0, 0.0],
            "state": [["free", "free"], ["free", "free"]],
            "stamp_ns": int(stamp_s * 1e9),
        },
        "inner_size": [0.6, 0.4, 0.4],
        "geometry_hash": "geo-1",
        "map_revision": 1,
        "start_tcp": {"stamp_ns": int(stamp_s * 1e9)},
        "start_joints": list(START_Q),
        "grasp": {
            "position": [0.0, 0.0, 0.10],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            "stamp_ns": int(stamp_s * 1e9),
        },
        "container_transform": {
            "translation": [0.0, 0.0, 0.0],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            "stamp_ns": None,
            "scene_revision": "scene-1",
        },
        "excluded_ids": [],
    }


class CountingStaticAdapter(StaticMoveItValidityAdapter):
    """Static reports plus a count of every certifier sample request."""

    def __init__(self, samples):
        super().__init__(samples)
        self.requested = []

    def validate_sample(self, edge_id, sample_index):
        self.requested.append((edge_id, sample_index))
        return super().validate_sample(edge_id, sample_index)


def static_factory_for(collector=None):
    """Factory producing static adapters with correct per-edge counts."""

    def factory(*, scene_states, edges):
        samples = {}
        for edge in edges:
            for index in range(len(sample_states(edge.trajectory))):
                samples[(str(edge.edge_id), index)] = ValiditySample(
                    edge_id=str(edge.edge_id), sample_index=index,
                    valid=True, min_clearance_m=0.05, contacts=())
        adapter = CountingStaticAdapter(samples)
        if collector is not None:
            collector.append(adapter)
        return adapter
    return factory


def build_producer(*, clock=None, fail_dest=None, revalidator="real",
                   validity_collector=None):
    clock = clock or FakeClock()
    graph = RouteGraph(static_identity="static-i")
    builder = FakeEdgeBuilder(fail_dest=fail_dest)
    producer = RouteProducer(
        graph=graph, edge_builder=builder,
        certifier=TrajectoryCertifier(),
        validity_adapter_factory=static_factory_for(validity_collector),
        attachment_provider=make_provider(clock),
        revalidator=MapRevalidator(
            RevalidationPolicy()) if revalidator == "real" else revalidator,
        fk=fake_fk, refine=fake_refine, clock_ns=clock)
    producer._builder = builder  # noqa: SLF001
    producer._graph = graph  # noqa: SLF001
    return producer, clock


class TestSurfaceBridge:

    def test_surface_map_shape(self):
        request = make_request()
        surface_map = surface_map_from_contract(
            request["surface"], request)
        assert surface_map["nx"] == 2 and surface_map["ny"] == 2
        assert surface_map["state"] == [["free", "free"], ["free", "free"]]
        assert surface_map["geometry_hash"] == "geo-1"
        assert surface_map["map_revision"] == 1

    def test_column_height_changes_are_a_different_digest(self):
        from luggage_perception.cargo_view_integration import (
            planning_snapshot_from_surface,
        )

        request = make_request()
        request["surface"]["state"] = [
            ["occupied", "free"], ["free", "free"]]
        request["surface"]["height"] = [[0.25, 0.0], [0.0, 0.0]]
        low = planning_snapshot_from_surface(
            surface_map_from_contract(request["surface"], request),
            scene_revision="scene-1", request_id="req-i-1")
        request["surface"]["height"] = [[0.30, 0.0], [0.0, 0.0]]
        high = planning_snapshot_from_surface(
            surface_map_from_contract(request["surface"], request),
            scene_revision="scene-1", request_id="req-i-1")
        assert low.column_height_digest
        assert low.column_height_digest != high.column_height_digest
        assert low.occupancy_digest == high.occupancy_digest
        assert low.geometry_hash == high.geometry_hash


class TestRegisterAttach:

    def test_freezes_inverse_of_measured_grasp(self):
        provider = make_provider(None)
        request = make_request()
        register_attach_from_request(
            provider, request, box_wdh=request["box_wdh"])
        record = provider.latest_record("req-i-1")
        # T_tcp_box = inverse(T_box_tool): translation mirrored
        assert record.t_tcp_box_translation == (0.0, 0.0, -0.10)
        assert record.tcp_frame == "suction_contact_frame"
        assert record.box_frame == "box"
        assert record.source.route_grade

    def test_duplicate_event_is_idempotent(self):
        provider = make_provider(None)
        request = make_request()
        register_attach_from_request(
            provider, request, box_wdh=request["box_wdh"])
        first = provider.latest_record("req-i-1")
        register_attach_from_request(
            provider, request, box_wdh=request["box_wdh"])
        again = provider.latest_record("req-i-1")
        assert again.digest() == first.digest()
        assert again.attachment_incarnation == first.attachment_incarnation


class TestAttachmentStampAdmission:

    def test_zero_stamp_is_not_a_live_attachment(self):
        producer, clock = build_producer()
        request = make_request()
        request["grasp"]["stamp_ns"] = 0
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"], box_id="pickup_box_0001",
            generation=3)
        result = producer.plan_place_route(request)
        assert not result["success"]
        assert result["reason_code"] == "STAMP_INVALID"
        row = result["rows"][0]
        assert row["record_stamp_ns"] == 0
        assert row["now_stamp_ns"] == clock()
        assert row["age_ns"] == clock()
        assert row["stale_window_ns"] == 60_000_000_000
        assert row["box_id"] == "pickup_box_0001"
        assert row["generation"] == 3
        assert row["attachment_incarnation"] == 0
        assert row["attachment_digest"]
        assert row["records_before"] == 1
        assert row["records_after"] == 0
        try:
            producer.attachment_provider.latest_record(request["request_id"])
            stayed_live = True
        except ContractViolation:
            stayed_live = False
        assert not stayed_live

    def test_measured_attach_stamp_is_not_expired_immediately(self):
        producer, clock = build_producer()
        request = make_request()
        request["grasp"]["stamp_ns"] = clock()
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"], box_id="pickup_box_0001",
            generation=3)
        record, reason, evidence = admit_attachment_for_route(
            producer.attachment_provider, request, clock())
        assert reason is None, evidence
        assert record is not None
        assert record.acquisition_stamp_ns == clock()
        assert evidence["age_ns"] == 0
        assert evidence["records_before"] == 1
        assert evidence["records_after"] == 1
        assert evidence["invalidation_reason"] == ""
        assert evidence["box_id"] == "pickup_box_0001"
        assert evidence["generation"] == 3

    def test_box_id_and_generation_mismatch_are_named(self):
        producer, _clock = build_producer()
        request = make_request()
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"], box_id="pickup_box_0001",
            generation=4)
        request["box_id"] = "pickup_box_0002"
        request["generation"] = 4
        mismatched_box = producer.plan_place_route(request)
        assert mismatched_box["reason_code"] == "ATTACHMENT_REQUEST_MISMATCH"
        box_row = mismatched_box["rows"][0]
        assert box_row["box_id"] == "pickup_box_0001"
        assert box_row["request_box_id"] == "pickup_box_0002"
        assert box_row["generation"] == 4
        assert box_row["records_before"] == 1
        assert box_row["records_after"] == 1

        request["box_id"] = "pickup_box_0001"
        request["generation"] = 9
        mismatched_generation = producer.plan_place_route(request)
        assert (mismatched_generation["reason_code"]
                == "ATTACHMENT_REQUEST_MISMATCH")
        assert mismatched_generation["rows"][0]["request_generation"] == 9
        assert mismatched_generation["rows"][0]["generation"] == 4


def test_cartesian_failure_names_the_unreached_waypoint():
    from luggage_planning.moveit_edge_adapter import cartesian_failure_detail
    detail = cartesian_failure_detail(
        [0.1], [0.2], [1.0, 0.0, 0.5], [1.2, 0.0, 0.5],
        0.286, 0.95, 8)
    assert detail["fraction"] == 0.286
    assert detail["failed_waypoint_index"] == 2
    assert detail["waypoint_count"] == 8
    assert detail["from_position"] == [1.0, 0.0, 0.5]


class TestGoalRefineEvidence:

    def test_failed_refine_names_the_seed_and_residual(self):
        producer, _clock = build_producer()

        def refuse(_position, _quaternion, _seed):
            return None, {"pos_err_m": 0.52, "rot_err_rad": 0.2, "ok": False}

        producer.refine = refuse
        seeds = producer._goal_seed_fn(
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), atlas_seeds=[])
        target = type("Target", (), {})()
        target.position = (-2.055, 1.035, 0.828)
        target.quaternion_xyzw = (0.7071, -0.7071, 0.0, 0.0)
        assert seeds(target) == []
        record = producer.last_goal_refine[-1]
        assert record["atlas_seed_count"] == 0
        assert record["target_world"][0][0] == -2.055
        labels = [item["seed"] for item in record["attempts"]]
        assert labels == [
            "tool_down", "tool_down_j1_minus_pi", "start"]
        assert all(item["ok"] is False for item in record["attempts"])
        assert record["attempts"][0]["pos_err_m"] == 0.52

    def test_joint1_goal_records_that_seed_not_the_fallback(self):
        from luggage_planning.route_producer import tool_down_branch_seeds

        producer, _clock = build_producer()
        shifted = tool_down_branch_seeds()[1][1]

        def refine(_position, _quaternion, seed):
            key = tuple(float(v) for v in seed)
            if key == shifted:
                return list(shifted), {"pos_err_m": 0.0, "rot_err_rad": 0.0}
            return None, {"pos_err_m": 0.2, "rot_err_rad": 0.1}

        producer.refine = refine
        seeds = producer._goal_seed_fn(
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), atlas_seeds=[])
        target = type("Target", (), {})()
        target.position = (1.2, -0.2, 0.8)
        target.quaternion_xyzw = (0.0, 1.0, 0.0, 0.0)
        goals = seeds(target)
        record = producer.last_goal_refine[-1]
        joint1 = next(item for item in record["attempts"]
                      if item["seed"] == "tool_down_j1_minus_pi")
        assert joint1["ok"] is True
        assert joint1["solved_by"] == "requested"
        assert any(goal.ik_family == "refine-tool_down_j1_minus_pi"
                   for goal in goals)
        producer, _clock = build_producer()
        requested = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)

        def refuse(_position, _quaternion, seed):
            err = 0.008 if tuple(seed) == requested else 0.24
            return None, {"pos_err_m": err, "rot_err_rad": 0.0}

        producer.refine = refuse
        q, info = producer._refine_to(
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), requested)
        assert q is None
        assert info["pos_err_m"] == 0.008
        assert [item["pos_err_m"] for item in info["fallback"]] == [0.24]

    def test_anchor_seed_precedes_atlas_and_does_not_fall_back(self):
        producer, _clock = build_producer()
        anchor = (0.2, -0.2, 0.2, -0.2, 0.2, -0.2)
        atlas = (0.4, -0.4, 0.4, -0.4, 0.4, -0.4)
        calls = []

        def refine(_position, _quaternion, seed):
            key = tuple(float(v) for v in seed)
            calls.append(key)
            if key == anchor:
                return None, {"pos_err_m": 0.05, "rot_err_rad": 0.0}
            return key, {"pos_err_m": 0.0, "rot_err_rad": 0.0}

        producer.refine = refine
        seeds = producer._goal_seed_fn(
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            anchor_qs=[anchor], atlas_seeds=[atlas])
        target = type("Target", (), {})()
        target.position = (0.0, 0.0, 0.5)
        target.quaternion_xyzw = (0.0, 1.0, 0.0, 0.0)
        seeds(target)
        record = producer.last_goal_refine[-1]
        labels = [item["seed"] for item in record["attempts"]]
        assert labels[:3] == ["anchor", "atlas", "tool_down"]
        assert record["attempts"][0]["ok"] is False
        assert record["attempts"][0]["pos_err_m"] == 0.05
        assert calls[0] == anchor
        # The anchor attempt is one call. Tool-down is not substituted for it.
        assert calls.count(anchor) == 1


MISS_SEED = (0.9, -0.9, 0.9, -0.9, 0.9, -0.9)
GOOD_SEED = tuple(START_Q)


def _two_slot_request():
    request = make_request(slots=[
        {"position": [0.20, -0.10, 0.14],
         "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0], "request_id": "cand-0"},
        {"position": [0.30, -0.10, 0.14],
         "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0], "request_id": "cand-1"},
    ])
    request["slot_scores"] = [1.0, 1.0]
    return request


def _seed_miss_producer(slot1_seeds):
    producer, _clock = build_producer()

    def atlas_prior(targets, _request):
        first_slot = float(targets[0].position[0]) < 0.25
        seeds = [MISS_SEED, MISS_SEED] if first_slot else list(slot1_seeds)
        return {"verdict": "reachable", "seeds": seeds}

    def refine(position, quaternion, seed):
        if tuple(float(v) for v in seed) == MISS_SEED:
            return None, {"pos_err_m": 0.008, "rot_err_rad": 0.0}
        return fake_refine(position, quaternion, seed)

    producer.atlas_prior = atlas_prior
    producer.refine = refine
    return producer


class TestAtlasCapture:
    def test_unreachable_rows_keep_slot_pose_and_targets(self):
        producer, _clock = build_producer()

        def atlas_prior(_targets, _request):
            return {"verdict": "unreachable", "seeds": []}

        producer.atlas_prior = atlas_prior
        request = _two_slot_request()
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        result = producer.plan_place_route(request)
        assert result["success"] is False
        rows = [row for row in result["rows"]
                if row.get("reason") == "ATLAS_UNREACHABLE"]
        assert len(rows) == 2
        for row in rows:
            assert row["candidate_id"]
            assert len(row["slot_pose"]["position"]) == 3
            assert row["targets"]
            assert len(row["targets"][0][0]) == 3
            assert len(row["targets"][0][1]) == 4
            assert row["reachability"] == "unreachable"
            assert row["seed_sources"] == []


class TestAtlasSeedCheck:

    def test_seed_miss_slot_is_deferred_behind_verified_slot(self):
        producer = _seed_miss_producer([GOOD_SEED])
        request = _two_slot_request()
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        result = producer.plan_place_route(request)
        assert result["success"], result["rows"]
        assert result["slot_index"] == 1
        miss = [row for row in result["rows"]
                if row.get("reason") == "ATLAS_SEED_MISS"]
        assert [row["slot_index"] for row in miss] == [0]
        assert miss[0]["seed_check"]["best_pos_err_m"] == 0.008
        assert miss[0]["seed_check"]["attempts"] >= 2

    def test_seed_miss_slot_is_retried_seedless_not_dropped(self):
        producer = _seed_miss_producer([])
        request = _two_slot_request()
        request["slots"] = request["slots"][:1]
        request["slot_scores"] = [1.0]
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        result = producer.plan_place_route(request)
        assert result["success"], result["rows"]
        assert result["slot_index"] == 0
        assert result["rows"][0]["reason"] == "ATLAS_SEED_MISS"


class _FirstSlotDescendFails(FakeEdgeBuilder):
    """Every DESCEND onto the first slot position it sees fails."""

    def __init__(self):
        super(_FirstSlotDescendFails, self).__init__()
        self.first_slot_xy = None
        self.nodes = {}

    def __call__(self, from_node, to_node, edge_class, candidate=None,
                 scene=None):
        self.nodes[from_node.node_id] = from_node
        self.nodes[to_node.node_id] = to_node
        if edge_class == "DESCEND":
            xy = tuple(round(v, 6) for v in to_node.tcp_position[:2])
            if self.first_slot_xy is None:
                self.first_slot_xy = xy
            if xy == self.first_slot_xy:
                self.calls.append((from_node.node_id, to_node.node_id,
                                   edge_class))
                raise RoutePlanningError("EDGE_BUILD_FAILED",
                                         {"pair": edge_class})
        edge = super(_FirstSlotDescendFails, self).__call__(
            from_node, to_node, edge_class, candidate=candidate, scene=scene)
        # End on the destination q: a shared canned endpoint would merge
        # every candidate's chain into one re-anchored node.
        base = edge.trajectory
        trajectory = type(base)(
            joint_names=base.joint_names,
            positions=(tuple(from_node.q), tuple(to_node.q)),
            velocities=tuple(base.velocities[:2]),
            accelerations=tuple(base.accelerations[:2]),
            time_from_start_ns=tuple(base.time_from_start_ns[:2]))
        return dataclasses.replace(edge, trajectory=trajectory)


class TestCandidateScope:

    def test_second_candidate_never_uses_first_candidates_hover(self):
        producer, _clock = build_producer()
        builder = _FirstSlotDescendFails()
        producer.edge_builder = builder
        request = _two_slot_request()
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        result = producer.plan_place_route(request)
        assert result["success"], result["rows"]
        first_xy = builder.first_slot_xy
        assert first_xy is not None
        second = [call for call in builder.calls
                  if call[2] in ("INSERT", "DESCEND")
                  and tuple(round(v, 6) for v in builder.nodes[call[1]]
                            .tcp_position[:2]) != first_xy]
        assert second, "the second candidate was never planned"
        start = builder.calls.index(second[0])
        for source, dest, klass in builder.calls[start:]:
            if klass not in ("INSERT", "DESCEND"):
                continue
            for node_id in (source, dest):
                xy = tuple(round(v, 6) for v in
                           builder.nodes[node_id].tcp_position[:2])
                assert xy != first_xy, (klass, node_id)


class TestMapStaleRefresh:

    def _rev(self, decisions):
        from st09_contracts.certification import (
            MapDecision, MapRevalidationDecision)
        from st09_contracts.reasons import Reason

        state = {"n": 0}

        class Rev(object):
            policy = type("P", (), {"max_snapshot_age_ns": 10_000_000_000})()

            def revalidate(self, request):
                del request
                kind = decisions[min(state["n"], len(decisions) - 1)]
                state["n"] += 1
                stale = kind == "stale"
                return MapRevalidationDecision(
                    decision=(MapDecision.FAULT if stale
                              else MapDecision.CONTINUE),
                    reason=Reason.MAP_STALE if stale else Reason.OK,
                    map_revision=4 if stale else 5,
                    validation_epoch=1,
                    affected_edge_ids=(),
                    detail={})

        return Rev(), state

    def test_one_stale_check_refreshes_the_same_route(self):
        producer, _clock = build_producer()
        rev, state = self._rev(["stale", "fresh"])
        producer.revalidator = rev
        request = make_request()
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        result = producer.plan_place_route(request)
        assert result["success"], result["rows"]
        assert state["n"] == 2
        assert result["rows"][0]["map_refresh"] == "fresh_snapshot_accepted"
        refusal = result["rows"][0]["map_refusal"]
        assert refusal["route_id"]
        assert refusal["latest_map_revision"] == 4
        assert refusal["planned_map_revision"] == request["map_revision"]
        assert refusal["legs"]
        assert refusal["stale_window_ns"] == 10_000_000_000

    def test_second_stale_check_names_the_repeat(self):
        producer, _clock = build_producer()
        rev, state = self._rev(["stale", "stale"])
        producer.revalidator = rev
        request = make_request()
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        result = producer.plan_place_route(request)
        assert not result["success"]
        assert result["reason_code"] == "MAP_STALE_REPEATED"
        assert state["n"] == 2
        row = result["rows"][0]
        assert row["map_refusal_after_refresh"]["latest_map_revision"] == 4
        assert row["elapsed_ms"] >= 0
        assert result["rows"][-1]["reason"] == "ATTEMPT_SUMMARY"
        assert result["rows"][-1]["tried_slot_indices"] == [0]

    def test_repeated_stale_does_not_try_the_next_slot(self):
        producer, _clock = build_producer()
        rev, state = self._rev(["stale", "stale"])
        producer.revalidator = rev
        request = make_request(slots=[
            {"position": [0.20, -0.10, 0.14],
             "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
             "request_id": "cand-0"},
            {"position": [0.30, -0.10, 0.14],
             "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
             "request_id": "cand-1"},
        ])
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        result = producer.plan_place_route(request)
        assert result["reason_code"] == "MAP_STALE_REPEATED"
        assert state["n"] == 2
        summary = result["rows"][-1]
        assert summary["n_tried"] == 1
        assert summary["tried_slot_indices"] == [0]


def _live_snapshot(request, stamp_ns, source, placed=0, observed=False):
    import dataclasses

    from luggage_perception.cargo_view_integration import (
        planning_snapshot_from_surface)

    base = planning_snapshot_from_surface(
        surface_map_from_contract(request["surface"], request),
        scene_revision=str(request["scene_revision"]),
        request_id=str(request["request_id"]))
    return dataclasses.replace(
        base, acquisition_stamp_ns=int(stamp_ns),
        map_source=str(source), placed_count=int(placed),
        cargo_observed=bool(observed))


class TestAcquisitionStampAdmission:

    def _producer(self, snapshots):
        producer, clock = build_producer()
        clock.now_ns = 38_350_000_000
        producer.revalidator = MapRevalidator(
            RevalidationPolicy(max_snapshot_age_ns=10_000_000_000))
        state = {"n": 0}

        def latest():
            snap = snapshots[min(state["n"], len(snapshots) - 1)]
            state["n"] += 1
            return snap

        producer.latest_snapshot_fn = latest
        return producer, clock, state

    def test_stamp_zero_stays_refused_after_refresh(self):
        request = make_request(stamp_s=38.35)
        stale = _live_snapshot(
            request, 0, "release_integrated", placed=1)
        producer, _clock, state = self._producer((stale, stale))
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        result = producer.plan_place_route(request)
        assert not result["success"]
        assert result["reason_code"] == "MAP_STALE_REPEATED"
        assert state["n"] == 2
        refusal = result["rows"][0]["map_refusal_after_refresh"]
        assert refusal["map_source"] == "release_integrated"
        assert refusal["placed_count"] == 1
        assert refusal["last_acquisition_stamp_ns"] == 0
        assert refusal["freshness_policy"] == "acquisition_stamp"
        assert refusal["why_allowed"] == ""
        assert refusal["why_refused"] == "acquisition_stamp_required"
        assert refusal["legs"]

    def test_refresh_with_a_new_stamp_admits_the_same_route(self):
        request = make_request(stamp_s=38.35)
        stale = _live_snapshot(
            request, 0, "release_integrated", placed=1)
        fresh = _live_snapshot(
            request, 38_350_000_000, "release_integrated", placed=1)
        producer, _clock, state = self._producer((stale, fresh))
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        result = producer.plan_place_route(request)
        assert result["success"], result["rows"]
        assert state["n"] == 2
        assert result["rows"][0]["map_refresh"] == "fresh_snapshot_accepted"
        admission = result["rows"][0]["map_admission"]
        assert admission["why_allowed"] == "confirmed_occupancy"
        assert admission["placed_count"] == 1

    def test_static_empty_first_box_is_admitted(self):
        request = make_request(stamp_s=38.35)
        initial = _live_snapshot(request, 0, "static_empty")
        producer, _clock, state = self._producer((initial,))
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        result = producer.plan_place_route(request)
        assert result["success"], result["rows"]
        assert state["n"] == 1
        admission = result["rows"][0]["map_admission"]
        assert admission["map_source"] == "static_empty"
        assert admission["placed_count"] == 0
        assert admission["last_acquisition_stamp_ns"] == 0
        assert admission["freshness_policy"] == "static_empty"
        assert admission["why_allowed"] == "static_empty_unplaced_unobserved"

    def test_static_empty_with_a_placed_box_does_not_execute(self):
        request = make_request(stamp_s=38.35)
        stale = _live_snapshot(request, 0, "static_empty", placed=1)
        producer, _clock, state = self._producer((stale, stale))
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        result = producer.plan_place_route(request)
        assert not result["success"]
        assert result["reason_code"] == "MAP_STALE_REPEATED"
        refusal = result["rows"][0]["map_admission"]
        assert refusal["why_refused"] == "static_empty_after_place"
        assert refusal["placed_count"] == 1

    def test_stamp_inside_the_window_admits_without_refresh(self):
        request = make_request(stamp_s=38.35)
        fresh = _live_snapshot(
            request, 30_000_000_000, "measured", observed=True)
        producer, _clock, state = self._producer((fresh,))
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        result = producer.plan_place_route(request)
        assert result["success"], result["rows"]
        assert state["n"] == 1
        assert result["rows"][0]["map_admission"]["why_allowed"] == \
            "confirmed_occupancy"


class TestPlanPlaceRoute:

    def test_happy_path_produces_certified_legs(self):
        producer, _clock = build_producer()
        request = make_request()
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        result = producer.plan_place_route(request)
        assert result["success"], result["rows"]
        assert result["path_source"] == "route-graph"
        assert result["slot_index"] == 0
        assert len(result["legs"]) == len(result["edges"]) > 0
        assert result["certification"].passed
        from st09_contracts.certification import MapDecision
        assert result["revalidation"].decision is MapDecision.CONTINUE
        names = [leg["name"] for leg in result["legs"]]
        # No opening datum in this fixture, so the portal hop is dropped.
        assert names[0] == "traverse"
        assert names[-1] == "return"

    def test_legs_admit_as_typed_v2_segments(self):
        from luggage_description.frame_diagnostics import BoundaryContext
        from luggage_planning.motion_frame_adapters import (
            message_from_admitted,
        )
        from luggage_planning.motion_frame_contracts import (
            TARGET_LINK, ExecutionContext,
        )
        producer, _clock = build_producer()
        request = make_request()
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        result = producer.plan_place_route(request)
        assert result["success"]
        assert result["success"]
        boundary = BoundaryContext(
            interface_id="F15", producer="route_producer",
            consumer="motion_executor",
            request_id="req-i-1", scene_revision="scene-1",
            frame_id="world", reference="suction_contact_frame",
            time_basis="robot_state")
        context = ExecutionContext(boundary, TARGET_LINK, "world")
        messages = [message_from_admitted(leg, context)
                    for leg in result["legs"]]
        assert all(m.segment_type == "joint_path" for m in messages)
        assert all(not m.allow_ompl_fallback for m in messages)
        assert all(len(m.joint_trajectory.trajectory.points) >= 2
                   for m in messages)

    def test_missing_attachment_is_named_refusal(self):
        producer, _clock = build_producer()
        result = producer.plan_place_route(make_request())
        assert not result["success"]
        assert result["reason_code"] == "ATTACHMENT_MISSING"
        assert result["slot_index"] == -1

    def test_expired_attachment_is_named_refusal(self):
        producer, clock = build_producer()
        request = make_request(stamp_s=1000.0)
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        clock.advance_s(60.0)  # exactly the placement window: still valid
        clock.now_ns += 1  # one nanosecond past it: expired
        result = producer.plan_place_route(request)
        assert not result["success"]
        assert result["reason_code"] == "ATTACHMENT_MISSING"

    def test_age_after_integration_does_not_block_search(self):
        producer, clock = build_producer()
        request = make_request(stamp_s=1000.0)
        request["surface"]["map_source"] = "measured"
        request["surface"]["cargo_observed"] = True
        request["surface"]["occupancy_confirmed"] = True
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        clock.advance_s(30.0)
        result = producer.plan_place_route(request)
        assert result["success"], result.get("reason_code")
        assert result["rows"][0]["map_admission"]["why_allowed"] == \
            "confirmed_occupancy"
        assert result["rows"][0]["map_admission"]["age_exceeds_window"] is True
        assert result["rows"][0]["map_admission"][
            "last_acquisition_stamp_ns"] == 1_000_000_000_000

    def test_entry_does_not_judge_identity_the_request_surface_lacks(self):
        """The request surface has no map source or stamp of its own.

        Whole-chain regression: the entry gate once judged the message
        snapshot (source "", stamp 0) and refused a map the authoritative
        snapshot admits.
        """
        from luggage_perception.cargo_view_integration import (
            planning_snapshot_from_surface,
        )

        producer, _clock = build_producer()
        request = make_request(stamp_s=1000.0)
        request["surface"]["stamp_ns"] = 0
        request["surface"].pop("map_source", None)
        authoritative = planning_snapshot_from_surface(
            surface_map_from_contract(dict(
                request["surface"], stamp_ns=1_000_000_000_000,
                map_source="measured", cargo_observed=True), request),
            scene_revision=request["scene_revision"],
            request_id=request["request_id"])
        producer.latest_snapshot_fn = lambda: authoritative
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        result = producer.plan_place_route(request)
        assert result["success"], result.get("reason_code")

    def test_change_event_blocks_search_until_reconfirmed(self):
        producer, _clock = build_producer()
        request = make_request(stamp_s=1000.0)
        request["surface"]["map_source"] = "measured"
        request["surface"]["cargo_observed"] = True
        request["surface"]["occupancy_confirmed"] = False
        request["surface"]["change_event"] = "box_placed"
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        result = producer.plan_place_route(request)
        assert result["reason_code"] == "MAP_CONFIRMATION_REQUIRED"
        assert producer.edge_builder.calls == []
        assert result["rows"][0]["map_admission"]["change_event"] == \
            "box_placed"

    def test_entry_gate_follows_the_mappers_published_pending_state(self):
        """The refusal reads the real mapper surface fields end to end."""
        from luggage_perception.occupancy_confirmation import (
            PendingConfirmation,
        )

        pending = PendingConfirmation(True)
        change_ns = 1_000_000_000_000
        pending.record("box_placed", change_ns, (
            [0.0, 0.0, 0.1], [0.2, 0.2, 0.2], 0.0))

        def request_with(fields):
            request = make_request(stamp_s=1000.0)
            request["surface"]["map_source"] = "measured"
            request["surface"]["cargo_observed"] = True
            request["surface"].update(fields)
            return request

        producer, clock = build_producer()
        request = request_with(pending.surface_fields())
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        refused = producer.plan_place_route(request)
        assert refused["reason_code"] == "MAP_CONFIRMATION_REQUIRED"
        assert producer.edge_builder.calls == []

        # A view that does not observe the box leaves the published state
        # pending; the refusal stands.
        elsewhere = [[0.4 + 0.02 * i, 0.3, 0.1] for i in range(6)]
        assert pending.settle(
            pending.begin(), change_ns + 1, elsewhere, True) == []
        assert request_with(pending.surface_fields())["surface"][
            "occupancy_confirmed"] is False

        # A settled post-change view with points on the box confirms it,
        # and the map stays valid however long planning then takes.
        on_box = [[0.02 * i - 0.05, 0.0, 0.18] for i in range(6)]
        assert pending.settle(
            pending.begin(), change_ns + 2, on_box, True)
        producer, clock = build_producer()
        request = request_with(pending.surface_fields())
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        clock.advance_s(30.0)
        result = producer.plan_place_route(request)
        assert result["success"], result.get("reason_code")

    def test_search_time_does_not_age_the_admitted_snapshot(self):
        from luggage_perception.cargo_view_integration import (
            planning_snapshot_from_surface,
        )

        producer, clock = build_producer()
        producer.revalidator = MapRevalidator(
            RevalidationPolicy(max_snapshot_age_ns=10_000_000_000))
        request = make_request(stamp_s=1000.0)
        request["surface"]["map_source"] = "measured"
        request["surface"]["cargo_observed"] = True
        snapshot = planning_snapshot_from_surface(
            surface_map_from_contract(request["surface"], request),
            scene_revision=request["scene_revision"],
            request_id=request["request_id"])

        def latest():
            clock.advance_s(30.0)
            return snapshot

        producer.latest_snapshot_fn = latest
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        result = producer.plan_place_route(request)
        assert result["success"], result.get("reason_code")

    def test_budget_exhaustion_never_bin_full(self):
        producer, clock = build_producer()
        producer.budget_ns = 1  # already exhausted
        request = make_request()
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        result = producer.plan_place_route(request)
        assert result["reason_code"] == "PLANNING_BUDGET_EXHAUSTED"
        assert result["reason_code"] != "BIN_FULL"

    def test_first_candidate_failure_falls_to_second(self):
        producer, _clock = build_producer()
        request = make_request(slots=[
            {"position": [0.20, -0.10, 0.14],
             "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
             "request_id": "cand-0"},
            {"position": [0.05, 0.10, 0.14],
             "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
             "request_id": "cand-1"},
        ])
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        # Pass 1 fails every build so BOTH candidates' goals register and
        # nothing is cached; pass 2 fails only candidate 0's descend.
        producer._builder.fail_all = True
        assert not producer.plan_place_route(request)["success"]
        producer._builder.fail_all = False
        goal_nodes = [node.node_id for node in producer._graph.nodes.values()
                      if node.region == "slot"
                      and node.load_state is LoadState.CARRYING]
        assert len(goal_nodes) >= 2
        # candidate 0 sits at container x 0.20, candidate 1 at 0.05: fail
        # every goal-branch DESCEND of candidate 0 (the pure-synthesis
        # target x composes through the identity container transform).
        producer._builder.fail_descend_x = 0.10
        result = producer.plan_place_route(request)
        assert result["success"], [r["reason"] for r in result["rows"]]
        assert result["slot_index"] == 1
        # B's taxonomy: one candidate's aggregated build failures surface
        # as NO_FEASIBLE_ROUTE with the failures in the row detail.
        assert result["rows"][0]["reason"] == "NO_FEASIBLE_ROUTE"
        assert result["rows"][0]["detail"]["edge_failures"]
        assert result["rows"][1]["reason"] == "OK"

    def test_colliding_candidate_yields_to_the_next(self):
        producer, _clock = build_producer()
        request = make_request(slots=[
            {"position": [0.20, -0.10, 0.14],
             "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
             "request_id": "blocked"},
            {"position": [0.05, 0.10, 0.14],
             "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
             "request_id": "open"},
        ])
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        # Candidate A sits at container x 0.20 and collides with a placed
        # box. The search must not keep sampling that slot.
        producer._builder.fail_collision_x = 0.10
        result = producer.plan_place_route(request)
        assert result["success"], [row.get("reason") for row in result["rows"]]
        assert result["slot_index"] == 1
        assert result["rows"][0]["reason"] == "CANDIDATE_COLLISION"
        assert producer._builder.collision_calls == 1

    def test_budget_exhaustion_records_untried_candidates(self):
        producer, _clock = build_producer()
        producer.budget_ns = 1
        request = make_request(slots=[
            {"position": [0.20, -0.10, 0.14],
             "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
             "request_id": "a"},
            {"position": [0.05, 0.10, 0.14],
             "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
             "request_id": "b"},
        ])
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        result = producer.plan_place_route(request)
        assert result["reason_code"] == "PLANNING_BUDGET_EXHAUSTED"
        assert result["n_tried"] == 0
        assert result["n_untried"] == 2

    def test_revalidation_stop_is_named(self):
        from luggage_planning.map_revalidation import (
            MapDecision, MapRevalidationDecision,
        )
        from st09_contracts.reasons import Reason

        class Blocked(object):
            def revalidate(self, _request):
                return MapRevalidationDecision(
                    decision=MapDecision.STOP,
                    reason=Reason.MAP_FULLY_BLOCKED,
                    map_revision=2, validation_epoch=1,
                    affected_edge_ids=(), detail={})

        producer, _clock = build_producer(revalidator=Blocked())
        request = make_request()
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        result = producer.plan_place_route(request)
        assert not result["success"]
        assert result["reason_code"] == "MAP_FULLY_BLOCKED"


class TestLegRecords:

    def test_leg_record_contract_shape(self):
        producer, _clock = build_producer()
        request = make_request()
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        result = producer.plan_place_route(request)
        assert result["success"]
        for leg in result["legs"]:
            joint = leg["joint_trajectory"]
            assert joint["kind"] == "joint_trajectory"
            assert joint["angle_unit"] == "rad"
            assert joint["time_basis"] == "robot_state"
            times = [p["time_from_start_ns"] for p in joint["points"]]
            assert all(b > a for a, b in zip(times, times[1:]))
            assert len(joint["joint_names"]) == 6
            for point in joint["points"]:
                assert len(point["positions"]) == 6
                assert len(point["velocities"]) == 6


def _hop_pairs(producer, *, opening, transit_clearance_z):
    """Run one plan and return the entry and retreat hops."""
    import luggage_planning.reachable_graph as reachable

    producer.opening_info = opening
    producer.transit_clearance_z = transit_clearance_z
    captured = {}
    real = reachable.GraphRoutePlanner

    class Capture(real):
        def __init__(self, *args, **kwargs):
            captured["entry_hops"] = kwargs.get("entry_hops")
            captured["retreat_hops"] = kwargs.get("retreat_hops")
            super().__init__(*args, **kwargs)

    reachable.GraphRoutePlanner = Capture
    try:
        request = make_request()
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        result = producer.plan_place_route(request)
    finally:
        reachable.GraphRoutePlanner = real
    assert result["success"], result.get("rows")

    def pairs(hops):
        assert hops is not None
        return [(hop.to_region, hop.edge_class, hop.load_state) for hop in hops]

    return pairs(captured["entry_hops"]), pairs(captured["retreat_hops"])


class TestReverseEgressTable:

    def test_staging_portal_is_joint_and_return_starts_outside(self):
        producer, _clock = build_producer()
        entry, retreat = _hop_pairs(
            producer,
            opening={"point": (1.0, 0.0, 0.4), "normal": (-1.0, 0.0, 0.0)},
            transit_clearance_z=5.0)
        assert entry == [
            ("staging", "PORTAL_TRANSIT", "carrying"),
            ("portal", "PORTAL_TRANSIT", "carrying"),
            ("slot_approach", "TRAVERSE", "carrying"),
            ("slot_upper", "INSERT", "carrying"),
            ("slot", "DESCEND", "carrying"),
        ]
        assert retreat == [
            ("slot_upper", "RETREAT", "unloaded"),
            ("slot_approach", "RETREAT", "unloaded"),
            ("portal", "RETREAT", "unloaded"),
            ("staging", "RETURN", "unloaded"),
            ("home", "RETURN", "unloaded"),
        ]
        assert "exit" not in [region for region, _cls, _load in retreat]

    def test_opening_without_staging_returns_from_the_portal(self):
        producer, _clock = build_producer()
        entry, retreat = _hop_pairs(
            producer,
            opening={"point": (1.0, 0.0, 0.4), "normal": (-1.0, 0.0, 0.0)},
            transit_clearance_z=None)
        assert entry == [
            ("portal", "PORTAL_TRANSIT", "carrying"),
            ("slot_approach", "TRAVERSE", "carrying"),
            ("slot_upper", "INSERT", "carrying"),
            ("slot", "DESCEND", "carrying"),
        ]
        assert retreat == [
            ("slot_upper", "RETREAT", "unloaded"),
            ("slot_approach", "RETREAT", "unloaded"),
            ("portal", "RETREAT", "unloaded"),
            ("home", "RETURN", "unloaded"),
        ]
        assert "exit" not in [region for region, _cls, _load in retreat]

    def test_failed_reverse_hop_yields_and_does_not_resample(self):
        producer, _clock = build_producer()
        producer._builder.fail_classes = ("RETREAT",)
        request = make_request()
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        result = producer.plan_place_route(request)
        assert not result["success"]
        assert result["rows"][0]["reason"] == "EGRESS_INFEASIBLE"
        descends = [call for call in producer._builder.calls
                    if call[2] == "DESCEND"]
        retreats = [call for call in producer._builder.calls
                    if call[2] == "RETREAT"]
        assert len(descends) == 1
        assert len(retreats) == 1


class _Query:
    def __init__(self, status, hard=False, cell=(1, 2, 3, 0), seeds=()):
        self.status = status
        self.hard_reject_safe = hard
        self.indices = cell
        self.contact_seeds = list(seeds)


class TestAtlasAdmission:
    def test_only_hard_reject_is_unreachable_and_marginal_is_separate(self):
        assert prior_from_statuses([1, 1], [], [True, True])["verdict"] == \
            "unreachable"
        low = prior_from_statuses([1, 1], [], [True, False])
        assert low["verdict"] == "unknown"
        assert prior_from_statuses([2], [(0.1,) * 6])["verdict"] == "marginal"
        assert prior_from_statuses([3, 2], [(0.2,) * 6])["verdict"] == \
            "reachable"
        assert prior_from_statuses([0], [])["verdict"] == "unknown"

    def test_replay_recomputes_the_same_cell(self):
        seen = []

        def query_fn(x, y, z, yaw):
            seen.append((x, y, z, yaw))
            return _Query(3, cell=(4, 5, 6, 1))

        document = {
            "container_transform": {
                "translation": [0.0, 0.0, 0.0],
                "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            },
            "candidates": [{
                "candidate_id": "c",
                "slot_index": 0,
                "reachability": "reachable",
                "targets": [[[0.2, -0.1, 0.3], [0.0, 0.0, 0.0, 1.0]]],
                "seed_sources": [],
            }],
        }
        first = replay_captured_candidates(document, query_fn)
        document["candidates"][0]["atlas_queries"] = first[0]["queries"]
        second = replay_captured_candidates(document, query_fn)
        assert second[0]["query_match"] is True
        assert second[0]["queries"][0]["cell"] == [4, 5, 6, 1]
        assert prior_for_targets(
            document["candidates"][0]["targets"],
            document["container_transform"], query_fn)["verdict"] == "reachable"


def _tier_slots():
    specs = [
        (0.05, "A"),
        (0.35, "D"),
        (0.45, "E"),
        (0.25, "C"),
        (0.15, "B"),
    ]
    return [{
        "position": [x, -0.10, 0.14],
        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
        "request_id": name,
    } for x, name in specs]


def _tier_prior(targets, _request):
    x = float(targets[0].position[0])
    if x < 0.10:
        verdict, seeds = "unreachable", []
    elif x < 0.20:
        verdict, seeds = "reachable", [GOOD_SEED]
    elif x < 0.30:
        verdict, seeds = "reachable", [GOOD_SEED]
    elif x < 0.40:
        verdict, seeds = "marginal", []
    else:
        verdict, seeds = "unknown", []
    return {
        "verdict": verdict, "seeds": list(seeds), "admission": "strict",
        "queries": [{
            "query_xyz": [round(x, 6), -0.1, 0.24],
            "query_yaw": 0.0, "status": 3, "hard_reject_safe": False,
            "cell": [0, 0, 0, 0],
        }],
    }


class TestTieredRouteAdmission:
    def test_unreachable_is_not_built_and_reachable_failure_falls_through(self):
        producer, _clock = build_producer()
        producer.atlas_prior = _tier_prior
        producer.edge_builder = _FirstSlotDescendFails()
        request = make_request(slots=_tier_slots())
        request["slot_scores"] = [0.1, 0.2, 0.3, 0.9, 0.4]
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        result = producer.plan_place_route(request)
        assert result["success"], result.get("rows")
        assert result["elapsed_s"] < 10.0
        assert result["selection_id"]
        assert result["legs"]
        names = [leg["name"] for leg in result["legs"]]
        assert "descend" in names and "return" in names
        assert all(leg["joint_trajectory"]["points"] for leg in result["legs"])
        selected = result["selected"]
        assert selected["candidate_id"] == "C"
        assert selected["atlas_status"] == "reachable"
        assert selected["route_id"] == result["selection_id"]
        assert selected["route_leg_count"] == len(result["legs"])
        rows = result["rows"]
        unreachable = [row for row in rows if row.get("candidate_id") == "A"]
        assert len(unreachable) == 1
        assert unreachable[0]["reason"] == "ATLAS_UNREACHABLE"
        assert unreachable[0]["route_build_count"] == 0
        failed = [row for row in rows if row.get("candidate_id") == "B"]
        assert failed and failed[0]["reason"]
        assert failed[0]["reason"] != "OK"
        assert failed[0]["route_build_count"] >= 1
        for name in ("D", "E"):
            built = [
                row for row in rows
                if row.get("candidate_id") == name
                and int(row.get("route_build_count") or 0) > 0]
            assert built == []
        assert "staging" not in names

    def test_unknown_is_planned_even_when_hash_policy_is_strict(self):
        producer, _clock = build_producer()

        def only_unknown(_targets, _request):
            return {
                "verdict": "unknown", "seeds": [], "admission": "strict",
                "queries": [{
                    "status": 0, "hard_reject_safe": False,
                    "query_xyz": [0.2, -0.1, 0.24], "query_yaw": 0.0,
                    "cell": [0, 0, 0, 0],
                }],
            }

        producer.atlas_prior = only_unknown
        request = make_request()
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        calls_before = len(producer._builder.calls)
        result = producer.plan_place_route(request)
        assert result["success"], result.get("rows")
        assert result["rows"][0]["reason"] != "ATLAS_UNKNOWN_STRICT"
        assert int(result["rows"][0]["route_build_count"]) >= 1
        assert len(producer._builder.calls) > calls_before

    def test_near_opening_status1_precedes_deep_unknown(self):
        producer, _clock = build_producer()
        # opening_info.normal points outward. Negating it makes depth
        # increase with +x, away from an opening on the negative-x face.
        producer.opening_info = {
            "point": [0.0, 0.0, 0.0], "normal": [-1.0, 0.0, 0.0]}
        producer.edge_builder.fail_all = True
        # Depth is slot x. Shallowest reachable is 0.80, so the near cut
        # is 0.70. 0.40 and 0.55 are near; 0.90 and 1.10 are not.
        specs = [
            (0.05, "hard", "unreachable", 1, True),
            (0.80, "reachable", "reachable", 3, False),
            (0.20, "marginal", "marginal", 2, False),
            (0.40, "near1", "unknown", 1, False),
            (0.55, "near0", "unknown", 0, False),
            (0.90, "far1", "unknown", 1, False),
            (1.10, "far0", "unknown", 0, False),
        ]

        def prior(targets, _request):
            x = float(targets[0].position[0])
            row = min(specs, key=lambda item: abs(item[0] - x))
            _x, _name, verdict, status, hard = row
            seeds = [GOOD_SEED] if verdict == "reachable" else []
            return {
                "verdict": verdict,
                "seeds": seeds,
                "admission": "strict",
                "queries": [{
                    "status": status,
                    "hard_reject_safe": hard,
                    "query_xyz": [round(x, 6), -0.1, 0.24],
                    "query_yaw": 0.0,
                    "cell": [0, 0, 0, 0],
                }],
            }

        producer.atlas_prior = prior
        slots = [{
            "position": [x, -0.10, 0.14],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            "request_id": name,
        } for x, name, _verdict, _status, _hard in specs]
        request = make_request(slots=slots)
        request["slot_scores"] = [0.0] * len(slots)
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        result = producer.plan_place_route(request)
        assert result["success"] is False
        dropped = [
            row["candidate_id"] for row in result["rows"]
            if row.get("reason") == "ATLAS_UNREACHABLE"]
        assert dropped == ["hard"]
        built = [
            row["candidate_id"] for row in result["rows"]
            if int(row.get("route_build_count") or 0) > 0]
        assert built == ["reachable", "marginal", "near1", "near0", "far1", "far0"]
        assert abs(near_opening_cut({"reachable": [0.80]}) - 0.70) < 1e-9
        assert attempt_tier("unknown", 0.40, 0.70, True, False) == 3
        assert attempt_tier("unknown", 1.10, 0.70, False, False) == 6

    def test_frozen_221_and_704_stay_near_opening_status1(self):
        # Frozen atlas queries from g3_online_n3_carryon3 box_00.
        # 221 is status (1, 0); 704 is status (1, 1). Neither query is
        # hard_reject_safe. A reachable cell at the 656 depth puts the
        # near cut on these opening-side depths.
        frozen = {
            "s221": [
                {"status": 1, "hard_reject_safe": False,
                 "query_xyz": [-0.095, -0.235, 0.827199],
                 "query_yaw": 0.0, "cell": [4, 4, 2, 0]},
                {"status": 0, "hard_reject_safe": False,
                 "query_xyz": [-0.095, -0.235, 0.827199],
                 "query_yaw": 3.141593, "cell": [4, 5, 2, 2]},
            ],
            "s704": [
                {"status": 1, "hard_reject_safe": False,
                 "query_xyz": [-0.095, -0.285, 0.827199],
                 "query_yaw": 1.570796, "cell": [4, 4, 2, 1]},
                {"status": 1, "hard_reject_safe": False,
                 "query_xyz": [-0.095, -0.285, 0.827199],
                 "query_yaw": -1.570796, "cell": [4, 4, 2, 3]},
            ],
        }
        priors = {}
        for name, queries in frozen.items():
            prior = prior_from_statuses(
                [item["status"] for item in queries], [],
                [item["hard_reject_safe"] for item in queries])
            prior["queries"] = queries
            prior["admission"] = "strict"
            priors[name] = prior
            assert prior["verdict"] != "unreachable"
            assert _soft_sampled_unreachable(prior)
        near_cut = near_opening_cut({"reachable": [0.005]})
        assert abs(near_cut - (-0.095)) < 1e-9
        assert attempt_tier(
            priors["s221"]["verdict"], -0.095, near_cut, True, False) == 3
        assert attempt_tier(
            priors["s704"]["verdict"], -0.095, near_cut, True, False) == 3

        producer, _clock = build_producer()
        producer.opening_info = {
            "point": [0.0, 0.0, 0.0], "normal": [-1.0, 0.0, 0.0]}
        producer.edge_builder.fail_all = True

        def atlas_prior(targets, _request):
            y = float(targets[0].position[1])
            x = float(targets[0].position[0])
            if abs(y - 0.015) < 0.005:
                return {
                    "verdict": "reachable", "seeds": [GOOD_SEED],
                    "admission": "strict",
                    "queries": [{
                        "status": 3, "hard_reject_safe": False,
                        "query_xyz": [0.005, 0.015, 0.827199],
                        "query_yaw": 1.570796, "cell": [5, 6, 2, 1],
                    }],
                }
            if abs(y - (-0.235)) < 0.005:
                return priors["s221"]
            if abs(y - (-0.285)) < 0.005:
                return priors["s704"]
            if x > 0.2:
                return {
                    "verdict": "unknown", "seeds": [], "admission": "strict",
                    "queries": [{
                        "status": 0, "hard_reject_safe": False,
                        "query_xyz": [0.40, 0.0, 0.827199],
                        "query_yaw": 0.0, "cell": [0, 0, 0, 0],
                    }],
                }
            return {
                "verdict": "unreachable", "seeds": [], "admission": "strict",
                "queries": [{
                    "status": 1, "hard_reject_safe": True,
                    "query_xyz": [x, 0.0, 0.827199],
                    "query_yaw": 0.0, "cell": [0, 0, 0, 0],
                }],
            }

        producer.atlas_prior = atlas_prior
        slots = [
            {"position": [-0.095, -0.235, 0.654],
             "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
             "request_id": "s221"},
            {"position": [0.02, -0.10, 0.654],
             "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
             "request_id": "hard"},
            {"position": [-0.095, -0.285, 0.654],
             "quaternion_xyzw": [0.0, 0.0, 0.7071067811865476,
                                 0.7071067811865476],
             "request_id": "s704"},
            {"position": [0.005, 0.015, 0.654],
             "quaternion_xyzw": [0.0, 0.0, 0.7071067811865476,
                                 0.7071067811865476],
             "request_id": "reachable"},
            {"position": [0.40, 0.0, 0.654],
             "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
             "request_id": "deep"},
        ]
        request = make_request(slots=slots)
        request["slot_scores"] = [0.0] * len(slots)
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        result = producer.plan_place_route(request)
        assert result["success"] is False
        dropped = [
            row["candidate_id"] for row in result["rows"]
            if row.get("reason") == "ATLAS_UNREACHABLE"]
        assert dropped == ["hard"]
        built = [
            row["candidate_id"] for row in result["rows"]
            if int(row.get("route_build_count") or 0) > 0]
        assert built[0] == "reachable"
        assert built.index("s221") < built.index("deep")
        assert built.index("s704") < built.index("deep")
        assert "hard" not in built

    def test_cert_budget_is_service_time_when_the_service_reports_it(self):
        assert certification_exceeded(0.53, 2.4) is False
        assert certification_exceeded(2.1, 2.1) is True
        assert certification_exceeded(None, 2.4) is True
        assert certification_exceeded(None, 0.4) is False

    def test_route_completeness_names_the_missing_part(self):
        producer, _clock = build_producer()
        request = make_request()
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        result = producer.plan_place_route(request)
        assert result["success"], result.get("rows")
        route, edges, legs = result["route"], result["edges"], result["legs"]
        graph = producer.graph
        assert route_missing_parts(route, edges, graph, legs) == []
        regions = [
            graph.node(edge.destination_node_id).region for edge in edges]
        assert "staging" not in regions

        no_return_ids = tuple(
            edge.edge_id for edge in edges
            if edge.edge_id in route.retreat_edge_ids
            and edge.attributes.get("edge_class") != "RETURN")
        no_return = dataclasses.replace(
            route, retreat_edge_ids=no_return_ids)
        assert "return_home" in route_missing_parts(
            no_return, edges, graph, legs)

        no_retreat_ids = tuple(
            edge.edge_id for edge in edges
            if edge.edge_id in route.retreat_edge_ids
            and edge.attributes.get("edge_class") != "RETREAT")
        no_retreat = dataclasses.replace(
            route, retreat_edge_ids=no_retreat_ids)
        assert "unloaded_retreat" in route_missing_parts(
            no_retreat, edges, graph, legs)

        bad = []
        flipped = False
        for edge in edges:
            if not flipped and edge.attributes.get("edge_class") == "DESCEND":
                bad.append(dataclasses.replace(
                    edge, attributes=dict(edge.attributes, certified=False)))
                flipped = True
            else:
                bad.append(edge)
        assert "typed_trajectory" in route_missing_parts(
            route, bad, graph, legs)


def _service_message(stamp_ns):
    from luggage_msgs.srv import SelectPlacePathV2
    from luggage_description.ros_frame_bindings.surfaces import (
        surface_to_message,
    )
    from luggage_planning.place_service import boundary_context, pose_message

    scene = "scene-1"
    request_id = "req-i-1"

    def identity():
        return dict(
            contract_version=2, frame_id="container_link", length_unit="m",
            stamp_ns=stamp_ns, time_basis="scene",
            scene_revision=scene, request_id=request_id)

    def slot(x, name):
        record = identity()
        record.update(
            reference="box_center", orientation_semantics="box_axis_pi",
            position=[x, -0.10, 0.14],
            quaternion_xyzw=[0.0, 0.0, 0.0, 1.0])
        record["request_id"] = request_id
        message = pose_message(record)
        message.identity.request_id = request_id
        return message

    message = SelectPlacePathV2.Request()
    message.request_id = request_id
    for x, _name in (
            (0.05, "A"), (0.35, "D"), (0.45, "E"), (0.25, "C"), (0.15, "B")):
        message.slots.append(slot(x, _name))
    message.slot_scores = [0.1, 0.2, 0.3, 0.9, 0.4]
    surface = dict(
        contract_version=2, kind="surface", frame_id="container_link",
        length_unit="m", stamp_ns=stamp_ns, time_basis="scene",
        scene_revision=scene, request_id="req-map-1",
        geometry_hash="geo-1", map_revision=1,
        height_datum="container_origin",
        floor_z_container=0.0, ceiling_z_container=1.0,
        resolution_m=0.05, origin_xy_container=[0.0, 0.0],
        surface_z_container=[[0.0, 0.0], [0.0, 0.0]],
        state=[["free", "free"], ["free", "free"]],
        confidence=[["sensor", "sensor"], ["sensor", "sensor"]])
    message.surface = surface_to_message(
        surface, boundary_context(
            "req-map-1", scene, "container_link", "",
            geometry_hash="geo-1", map_revision=1))
    message.box_wdh = [0.30, 0.20, 0.20]
    message.inner_size = [0.6, 0.4, 0.4]
    message.geometry_hash = "geo-1"
    message.map_revision = 1
    start = identity()
    start.update(
        reference="suction_contact_frame",
        orientation_semantics="tool_quaternion",
        position=[0.0, 0.0, 0.8],
        quaternion_xyzw=[0.0, 0.0, 0.0, 1.0])
    message.start_tcp = pose_message(start)
    message.start_joints = list(START_Q)
    grasp = identity()
    grasp.update(
        reference="box", orientation_semantics="suction_contact_frame",
        position=[0.0, 0.0, 0.10],
        quaternion_xyzw=[0.0, 0.0, 0.0, 1.0])
    message.grasp = pose_message(grasp)
    message.container_source_frame = "container_link"
    message.container_target_frame = "world"
    message.container_translation = [0.0, 0.0, 0.0]
    message.container_quaternion_xyzw = [0.0, 0.0, 0.0, 1.0]
    message.container_stamp_is_null = True
    message.box_id = "box-1"
    message.generation = 1
    return message


class TestSelectPlacePathV2Admission:
    def test_service_selects_the_reachable_route(self):
        import json
        from luggage_msgs.srv import SelectPlacePathV2
        from luggage_planning.place_service import fill_select_response_route

        producer, clock = build_producer()
        producer.atlas_prior = _tier_prior
        producer.edge_builder = _FirstSlotDescendFails()
        message = _service_message(clock.now_ns)
        response = SelectPlacePathV2.Response()
        fill_select_response_route(message, response, producer)
        assert response.success, response.reason_code
        assert response.slot_index == 3
        assert response.motion_segments
        rows = json.loads(response.rows_json)
        selected = [row.get("selected") for row in rows if row.get("selected")]
        assert selected
        assert selected[-1]["atlas_status"] == "reachable"
        assert selected[-1]["route_id"]
        assert selected[-1]["route_leg_count"] == len(response.motion_segments)
        by_index = {row.get("slot_index"): row for row in rows}
        assert by_index[0]["reason"] == "ATLAS_UNREACHABLE"
        assert by_index[0]["route_build_count"] == 0
        assert by_index[4]["reason"] not in ("", "OK")
        assert by_index[4]["route_build_count"] >= 1


class TestCandidateBoundaryCapture:
    def test_cartesian_rows_do_not_share_failures(self):
        producer, _clock = build_producer()

        class Log(object):
            def __init__(self):
                self.failures = []
                self.dumps = []
                self.scenes = []

            def edge_failure_log(self):
                return list(self.failures)

            def request_dumps(self):
                return list(self.dumps)

            def scene_dumps(self):
                return list(self.scenes)

            def planner_parameters(self):
                return {
                    "planner_id": "RRTConnect",
                    "ompl_rng_seed": "unfixed",
                }

        log = Log()
        producer.edge_builder = log
        log.failures.append({
            "reason": "fraction_below_min", "edge_class": "TRAVERSE",
            "from_q": [0.0], "to_q": [1.0]})
        log.dumps.append({
            "edge_class": "TRAVERSE", "from_q": [0.0], "to_q": [1.0],
            "fraction": 0.6})
        first = {"_failure_at": 0, "_dump_at": 0, "_scene_at": 0}
        producer._close_row([], first)
        log.failures.append({
            "reason": "fraction_below_min", "edge_class": "TRAVERSE",
            "from_q": [2.0], "to_q": [3.0]})
        log.dumps.append({
            "edge_class": "TRAVERSE", "from_q": [2.0], "to_q": [3.0],
            "fraction": 0.2})
        second = {"_failure_at": 1, "_dump_at": 1, "_scene_at": 0}
        producer._close_row([], second)
        assert first["cartesian"][0]["from_q"] == [0.0]
        assert second["cartesian"][0]["from_q"] == [2.0]
        assert second["cartesian"][0]["to_q"] != first["cartesian"][0]["to_q"]
        assert first["first_edge"]["to_q"] == [1.0]
        assert second["first_edge"]["from_q"] == [2.0]
        assert first["planner_parameters"]["ompl_rng_seed"] == "unfixed"

    def test_refine_attempt_records_the_joint_solution(self):
        producer, _clock = build_producer()
        seeds = producer._goal_seed_fn(
            (0.1, 0.2, 0.3, 0.4, 0.5, 0.6),
            atlas_seeds=[(0.2, 0.2, 0.2, 0.2, 0.2, 0.2)])

        class Target(object):
            position = (0.4, 0.0, 0.5)
            quaternion_xyzw = (0.0, 0.0, 0.0, 1.0)

        assert seeds(Target())
        attempt = producer.last_goal_refine[0]["attempts"][0]
        assert attempt["seed"] == "atlas"
        assert attempt["q"] is not None and len(attempt["q"]) == 6
        assert attempt["seed_q"] == [0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
        assert attempt["wrapped_delta"] is not None
        assert producer.last_goal_refine[0]["atlas_seeds"] == [
            [0.2, 0.2, 0.2, 0.2, 0.2, 0.2]]


class _LatticeUrdf(object):
    def joint_limits(self):
        return [(-6.0, 6.0)] * 6


class _ClearChecker(object):
    def __init__(self):
        self.states = []
        self.stats = {"service_calls": 0}

    def validate(self, states):
        self.states.extend(states)
        self.stats["service_calls"] += 1
        return {state: True for state in states}


def _lattice_producer(checker):
    clock = FakeClock()
    graph = RouteGraph(static_identity="static-i")
    builder = FakeEdgeBuilder()
    producer = RouteProducer(
        graph=graph, edge_builder=builder,
        certifier=TrajectoryCertifier(),
        validity_adapter_factory=static_factory_for(),
        attachment_provider=make_provider(clock),
        revalidator=MapRevalidator(RevalidationPolicy()),
        fk=fake_fk, refine=fake_refine, urdf=_LatticeUrdf(),
        clock_ns=clock, corridor_planner="lattice",
        lattice_checker_factory=lambda scene_states, start_q: checker)
    return producer, builder, clock


class TestLatticeCorridorPlanner:

    def test_lattice_route_is_certified_and_complete(self):
        checker = _ClearChecker()
        producer, builder, _clock = _lattice_producer(checker)
        request = make_request()
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        result = producer.plan_place_route(request)
        assert result["success"], result["rows"]
        classes = [edge.attributes["edge_class"] for edge in result["edges"]]
        assert classes == ["PORTAL_TRANSIT", "TRAVERSE", "INSERT", "DESCEND",
                           "RETREAT", "RETREAT", "RETREAT", "RETURN"]
        planners = [edge.attributes.get("planner")
                    for edge in result["edges"]]
        assert planners[1:-1] == ["corridor_lattice"] * 6
        assert [call[2] for call in builder.calls] == [
            "PORTAL_TRANSIT", "RETURN"]
        assert result["certification"].passed
        assert len(result["legs"]) == len(result["edges"])
        assert checker.states
        selected = [row for row in result["rows"] if row.get("reason") == "OK"]
        assert selected[0]["lattice"]["service_calls"] >= 1

    def test_lattice_without_a_chain_names_the_row(self):
        class _Blocked(_ClearChecker):
            def validate(self, states):
                super().validate(states)
                return {state: state[1] != "carrying" for state in states}

        producer, builder, _clock = _lattice_producer(_Blocked())
        request = make_request()
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        result = producer.plan_place_route(request)
        assert not result["success"]
        assert result["reason_code"] == "NO_FEASIBLE_ROUTE"
        reasons = [row.get("reason") for row in result["rows"]]
        assert "LATTICE_NO_CHAIN" in reasons
        assert builder.calls == []

    def test_lattice_requires_its_collaborators(self):
        import pytest

        with pytest.raises(ValueError):
            RouteProducer(
                graph=RouteGraph(static_identity="static-i"),
                edge_builder=FakeEdgeBuilder(),
                certifier=TrajectoryCertifier(),
                validity_adapter_factory=static_factory_for(),
                attachment_provider=make_provider(None),
                fk=fake_fk, refine=fake_refine,
                corridor_planner="lattice")


@pytest.mark.parametrize("available", [True, False, None])
def test_staging_collision_can_use_alternate_transit_position(available):
    """Real graph/route pipeline must escape a rejected staging centerline."""
    producer, _clock = build_producer()
    producer.opening_info = {
        "point": (1.0, 0.0, 0.4), "normal": (-1.0, 0.0, 0.0)}
    producer.transit_clearance_z = 5.0
    original = producer.edge_builder
    rejected, accepted = [], []

    class RejectCenterline:
        def __getattr__(self, name):
            return getattr(original, name)

        def __call__(self, source, destination, edge_class, **kwargs):
            if destination.region == "staging":
                if abs(destination.tcp_position[1]) < 0.01:
                    rejected.append(destination.tcp_position)
                    raise RoutePlanningError("EDGE_BUILD_FAILED", {
                        "kind": "collision", "edge_class": edge_class})
                accepted.append(destination.tcp_position)
            return original(source, destination, edge_class, **kwargs)

    producer.edge_builder = RejectCenterline()
    checked = []

    class EndpointChecker:
        def validate(self, states, expired):
            verdicts = {}
            for q, load in states:
                node = next(n for n in producer.graph.nodes_by_region("staging")
                            if tuple(n.q) == q and str(n.load_state) == load)
                verdicts[(q, load)] = (abs(node.tcp_position[1]) >= 0.01
                                       if available is True else available)
                checked.append(verdicts[(q, load)])
            return verdicts

    producer.lattice_checker_factory = lambda **kwargs: EndpointChecker()
    request = make_request()
    register_attach_from_request(
        producer.attachment_provider, request, box_wdh=request["box_wdh"])
    result = producer.plan_place_route(request)
    if available is not True:
        assert not result["success"]
        assert not rejected and not accepted
        assert any(r.get("detail", {}).get("stage") == "staging_endpoint"
                   for r in result.get("rows", []))
        return
    assert result["success"], [(r.get("reason"), r.get("failure"), r.get("detail")) for r in result.get("rows", [])]
    assert False in checked and True in checked
    assert not rejected  # Invalid endpoints never spend a connection attempt.
    assert accepted
    assert all(abs(p[1]) >= 0.01 for p in accepted)
