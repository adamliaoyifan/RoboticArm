"""ST09-I S2 offline: the route producer composes the admitted route.

Fakes stand in for MoveIt (edge builder) and the validity reports; every
other collaborator is the real component: D's snapshot bridge and
measured attachment provider, B's graph/search/planner, C's certifier,
revalidator, and the frozen v2 segment admission the legs must survive.
"""

from __future__ import division

import dataclasses
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
from luggage_planning.route_producer import (  # noqa: E402
    RouteProducer,
    admit_attachment_for_route,
    candidate_from_slot,
    leg_records,
    register_attach_from_request,
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
        self.calls = []

    def __call__(self, from_node, to_node, edge_class, candidate=None,
                 scene=None):
        self.calls.append((from_node.node_id, to_node.node_id, edge_class))
        failed = self.fail_all or (to_node.node_id, edge_class) in \
            self.fail_dest
        if (not failed and self.fail_descend_x is not None
                and to_node.region == "slot" and edge_class == "DESCEND"
                and to_node.tcp_position[0] > self.fail_descend_x):
            failed = True
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
        assert row["stale_window_ns"] == 10_000_000_000
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
        assert labels[:2] == ["tool_down", "start"]
        assert all(item["ok"] is False for item in record["attempts"])
        assert record["attempts"][0]["pos_err_m"] == 0.52


    def test_refine_failure_reports_requested_seed_not_fallback(self):
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
        assert refusal["freshness_policy"] == "acquisition_age"
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
        assert admission["why_allowed"] == "release_integrated_within_window"
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
            "measured_within_window"


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
        clock.advance_s(60.0)  # far beyond the frozen 10 s window
        result = producer.plan_place_route(request)
        assert not result["success"]
        assert result["reason_code"] == "ATTACHMENT_MISSING"

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
