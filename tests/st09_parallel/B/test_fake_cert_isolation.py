"""ST09-B acceptance (g): no fake certifier enters real output.

Until ST09-C lands, tests may use explicitly fake certification results;
production B code must be structurally incapable of emitting one. This is
asserted three ways: an AST scan proving the four production modules
never import st09_contracts.fixtures and never construct a
CertificationResult; a structural check that PlannedRoute has no
certification field to smuggle one through; and an end-to-end offline
plan_route whose output digest is byte-identical with and without a fake
certifier attached.
"""

from __future__ import division

import ast
import dataclasses
from pathlib import Path

import pytest

from st09_contracts.graph import GraphEdge, LoadState, PlannedRoute  # noqa: E402
from st09_contracts.fixtures import (  # noqa: E402
    make_attachment,
    make_snapshot,
    make_timed_trajectory,
)
from st09_contracts.planning import (  # noqa: E402
    GeometryVerdict,
    PlacementCandidate,
)
from st09_contracts.attachment import PlaceTarget  # noqa: E402
from st09_contracts.certification import CertificationResult  # noqa: E402
from st09_contracts.reasons import Reason  # noqa: E402

from luggage_planning.place_targets import GoalSeed  # noqa: E402

from luggage_planning.routing.grid_search import (  # noqa: E402
    RoutePlanningError,
    RouteSearch,
    RouteBudget,
)
from luggage_planning.atlas.graph import (  # noqa: E402
    GraphRoutePlanner,
    RouteGraph,
)

REPO_ROOT = Path(__file__).resolve().parents[3]
PRODUCTION_MODULES = [
    "place_targets.py",
    "reachable_graph.py",
    "grid_path_search.py",
    "moveit_edge_adapter.py",
]

BANNED_IMPORTS = ("st09_contracts.fixtures",)
BANNED_CONSTRUCTIONS = {"CertificationResult", "make_certification"}


class TestProductionAstHygiene:

    @pytest.mark.parametrize("module_name", PRODUCTION_MODULES)
    def test_no_fixture_imports_or_cert_constructions(self, module_name):
        module_path = (REPO_ROOT / "src" / "luggage_planning" / "luggage_planning"
                       / module_name)
        tree = ast.parse(module_path.read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom):
                assert ".".join(
                    filter(None, [node.module or "", node.level * "."])) \
                    not in BANNED_IMPORTS, module_name
                assert node.module != "st09_contracts.fixtures", module_name
            if isinstance(node, ast.Import):
                for alias in node.names:
                    assert alias.name not in BANNED_IMPORTS, module_name
            if isinstance(node, ast.Call):
                func = node.func
                name = getattr(func, "id", None) or getattr(func, "attr", None)
                assert name not in BANNED_CONSTRUCTIONS, \
                    "%s constructs %s" % (module_name, name)

    def test_planned_route_has_no_certification_field(self):
        field_names = {f.name for f in dataclasses.fields(PlannedRoute)}
        assert "certification" not in field_names
        assert "certifier" not in field_names
        assert "validation" not in field_names


class FakeCertifier(object):
    """Explicitly fake: fixture ids, fixture method note, pass verdict."""

    def __init__(self, route_ids=None):
        self.route_ids = list(route_ids or [])
        self.calls = 0

    def certify(self, route):
        self.calls += 1
        self.route_ids.append(route.route_id)
        return CertificationResult(
            certification_id="cert-fixture-%d" % self.calls,
            route_id=route.route_id,
            passed=True,
            reason=Reason.OK,
            checks=("fk",),
            validation_revision=1,
            method_note="fixture: fake certifier until ST09-C lands")


class OfflineChain(object):
    """A complete offline plan_route harness (fake edges, real planner)."""

    def __init__(self):
        self.graph = RouteGraph(static_identity="static-offline")
        self._register_regions()
        self.builder = _OfflineEdgeBuilder(self.graph)
        self.attachment = make_attachment("centered", symmetry="none")
        self.candidate = PlacementCandidate(
            candidate_id="cand-b-iso",
            footprint_class=0,
            heading_theta=0.0,
            position_container=(0.20, -0.10, 0.14),
            quaternion_xyzw_container=(0.0, 0.0, 0.0, 1.0),
            support_layer_z=0.0,
            depth_m=0.6,
            verdict=GeometryVerdict.LEGAL,
            score=1.0,
            detail={"route_clearance_m": 0.05},
        )
        self.target = PlaceTarget(
            candidate_id=self.candidate.candidate_id,
            heading_theta=0.0,
            tcp_frame=self.attachment.tcp_frame,
            frame_id="world",
            position=(0.6, 0.1, 0.4),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            attachment_digest=self.attachment.digest())
        self.goal_q = (0.3, -0.6, 0.85, -0.3, 1.4, 0.1)

    def _register_regions(self):
        carrying = LoadState.CARRYING
        unloaded = LoadState.UNLOADED
        self.graph.register_node((0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                                 (0.0, 0.0, 0.0), (0, 0, 0, 1), "home", carrying)
        self.graph.register_node((0.1, -0.2, 0.9, -0.3, 1.2, 0.0),
                                 (0.4, 0.0, 0.2), (0, 0, 0, 1), "portal", carrying)
        self.graph.register_node((0.2, -0.4, 0.9, -0.3, 1.3, 0.05),
                                 (0.5, 0.05, 0.35), (0, 0, 0, 1),
                                 "slot_approach", carrying)
        self.graph.register_node((0.25, -0.5, 0.88, -0.3, 1.35, 0.08),
                                 (0.55, 0.08, 0.4), (0, 0, 0, 1),
                                 "slot_upper", carrying)
        self.graph.register_node((0.1, -0.2, 0.9, -0.3, 1.2, 0.0),
                                 (0.4, 0.0, 0.2), (0, 0, 0, 1),
                                 "exit", unloaded)
        self.graph.register_node((0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                                 (0.0, 0.0, 0.0), (0, 0, 0, 1),
                                 "home", unloaded)

    def planner(self, certifier=None):
        return GraphRoutePlanner(
            self.graph, self.builder,
            home_node_id=[n for n in self.graph.nodes.values()
                          if n.region == "home"
                          and n.load_state is LoadState.CARRYING][0].node_id,
            goal_seeds=lambda _target: [GoalSeed(
                self.goal_q, "f1", "offline-seed")],
            certifier=certifier,
            refine=lambda position, quaternion, seed: (
                list(self.goal_q), {"pos_err_m": 0.0, "rot_err_rad": 0.0}))

    def plan(self, certifier=None):
        planner = self.planner(certifier)
        return planner.plan_route(
            make_snapshot(1), self.attachment, self.candidate,
            (self.target,), budget_ns=10_000_000_000)


class _OfflineEdgeBuilder(object):
    """Fake edges through the same builder protocol the search invokes."""

    def __init__(self, graph):
        self.graph = graph
        self.count = 0

    def __call__(self, from_node, to_node, edge_class, candidate=None,
                 scene=None):
        self.count += 1
        return GraphEdge(
            edge_id="e-off-%03d" % self.count,
            source_node_id=from_node.node_id,
            destination_node_id=to_node.node_id,
            load_state=from_node.load_state,
            static_identity="static-offline",
            trajectory=make_timed_trajectory(),
            validation_revision=None,
            attributes={"edge_class": edge_class,
                        "duration_ns": 100, "joint_travel_rad": 0.1})


class TestFakeCertifierIsolation:

    def test_route_identical_with_and_without_fake_certifier(self):
        chain = OfflineChain()
        plain = chain.plan(certifier=None)
        fake = FakeCertifier()
        certified = chain.plan(certifier=fake)
        assert fake.calls == 1
        assert certified.digest() == plain.digest()
        assert certified.route_id == plain.route_id
        assert certified.edge_digests == plain.edge_digests

    def test_advisory_result_is_recorded_not_embedded(self):
        chain = OfflineChain()
        fake = FakeCertifier()
        planner = chain.planner(certifier=fake)
        route = planner.plan_route(
            make_snapshot(1), chain.attachment, chain.candidate,
            (chain.target,), budget_ns=10_000_000_000)
        assert fake.route_ids == [route.route_id]
        assert planner.last_advisory_cert.method_note.startswith("fixture:")
        assert planner.last_advisory_cert.certification_id.startswith(
            "cert-fixture-")
        # Nothing from the fake leaked into the route's identity.
        assert "cert-fixture" not in route.digest()
        assert route.digest() == OfflineChain().plan().digest()

    def test_fake_certificate_cannot_enter_any_route_field(self):
        route = OfflineChain().plan()
        for field in dataclasses.fields(PlannedRoute):
            value = getattr(route, field.name)
            text = str(value)
            assert "cert-fixture" not in text
            assert "fixture:" not in text

    def test_budget_exhaustion_never_bin_full(self):
        chain = OfflineChain()
        planner = chain.planner()
        with pytest.raises(RoutePlanningError) as err:
            planner.plan_route(
                make_snapshot(1), chain.attachment, chain.candidate,
                (chain.target,), budget_ns=0)
        assert err.value.code == "PLANNING_BUDGET_EXHAUSTED"
        assert err.value.code != "BIN_FULL"
