"""ST09-I S2 offline: the route producer composes the admitted route.

Fakes stand in for MoveIt (edge builder) and the validity reports; every
other collaborator is the real component: D's snapshot bridge and
measured attachment provider, B's graph/search/planner, C's certifier,
revalidator, and the frozen v2 segment admission the legs must survive.
"""

from __future__ import division

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
