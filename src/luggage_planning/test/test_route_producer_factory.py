"""Production route-producer factory matches the selector's collaborators.

The test stubs MoveIt clients. It still loads the resolved planning
contract, the deploy scene, and the offline URDF, and checks that the
selector caches one factory result.
"""

from __future__ import division

import sys
import unittest
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
SCRIPTS = PKG / "scripts"
ROOT = PKG.parents[1]
for entry in (str(PKG), str(SCRIPTS), str(ROOT)):
    if entry not in sys.path:
        sys.path.insert(0, entry)

SCENE = str(
    ROOT / "src" / "luggage_description" / "config" / "scene_tf_deploy.yaml")


class _Param(object):
    def __init__(self, value):
        self.value = value


class _Clock(object):
    def now(self):
        return type("Stamp", (), {"nanoseconds": 1_000_000_000})()


class _Logger(object):
    def info(self, *_args, **_kwargs):
        return None

    def error(self, *_args, **_kwargs):
        return None


class _Identity(object):
    ready = False
    hash = ""

    def poll(self):
        return None


class _Node(object):
    def __init__(self):
        self._scene_tf_hash = "factory-test"
        self._atlas_cfg = {"enabled": False, "query_offset": [0.0, 0.0, 0.05]}
        self._urdf_identity = _Identity()
        self._params = {
            "box_count": 3,
            "corridor_planner": "lattice",
            "frame_scene_revision": "scene-runtime",
            "diagnostic_route_budget_s": 0.0,
        }
        self._vacuum_state = None

    def get_parameter(self, name):
        return _Param(self._params[name])

    def get_logger(self):
        return _Logger()

    def get_clock(self):
        return _Clock()

    def create_client(self, *_args, **_kwargs):
        return object()

    def _runtime_atlas(self):
        return None, {"loaded": False, "reason": "disabled"}

    def _site_poses(self):
        return None


class FactoryCollaborators(unittest.TestCase):
    def setUp(self):
        import luggage_planning.routing.moveit_edge_adapter as adapter

        self._adapter = adapter
        self._original = adapter.MoveItEdgeAdapter

        class _Edge(object):
            def __init__(self, node, scene_client=None, **_kwargs):
                self.node = node
                self.scene_client = scene_client

        adapter.MoveItEdgeAdapter = _Edge

    def tearDown(self):
        self._adapter.MoveItEdgeAdapter = self._original

    def test_factory_uses_contract_policies_and_production_types(self):
        from luggage_description.scene_tf_config_utils import (
            load_scene_tf_config,
        )
        from luggage_planning.attachment_provider import (
            MeasuredAttachmentProvider,
        )
        from luggage_planning.routing.grid_search import route_budget_ns
        from luggage_planning.map_revalidation import MapRevalidator
        from luggage_planning.planning_scene_client import PlanningSceneClient
        from luggage_planning.routing.producer import (
            RouteProducer,
            certification_policy_from_contract,
            revalidation_policy_from_contract,
        )
        from luggage_planning.routing.factory import (
            build_production_route_producer,
        )
        from luggage_planning.trajectory_certifier import TrajectoryCertifier
        import json
        from ament_index_python.packages import get_package_share_directory
        import os

        contract_path = os.path.join(
            get_package_share_directory("st09_contracts"),
            "resolved_planning_contract.json")
        with open(contract_path, encoding="utf-8") as handle:
            document = json.load(handle)
        node = _Node()
        node._scene = load_scene_tf_config(SCENE)
        calls = {"stats": 0}

        def surface():
            return {}

        def stats():
            calls["stats"] += 1
            return None

        producer = build_production_route_producer(
            node, SCENE, surface, stats)
        self.assertIsInstance(producer, RouteProducer)
        self.assertIsInstance(producer.certifier, TrajectoryCertifier)
        self.assertIsInstance(producer.revalidator, MapRevalidator)
        self.assertIsInstance(
            producer.attachment_provider, MeasuredAttachmentProvider)
        self.assertIsInstance(
            producer.edge_builder.scene_client, PlanningSceneClient)
        self.assertEqual(
            producer.cert_policy,
            certification_policy_from_contract(document))
        self.assertEqual(
            producer.revalidator.policy,
            revalidation_policy_from_contract(document))
        self.assertEqual(producer.budget_ns, route_budget_ns(3))
        self.assertEqual(producer.corridor_planner, "lattice")
        self.assertTrue(callable(producer.lattice_checker_factory))
        self.assertTrue(callable(producer.atlas_prior))
        self.assertIsNone(producer.latest_snapshot_fn())
        self.assertEqual(calls["stats"], 0)
        self.assertIsNone(producer.atlas_prior([], {}))

    def test_diagnostic_budget_override_reaches_searcher(self):
        """[Functional diagnostic] override > 0 replaces the frozen
        n*60s budget IN THE SEARCHER (producer.budget_ns); the default
        (0.0) leaves the frozen contract untouched."""
        from luggage_planning.routing.factory import (
            build_production_route_producer,
        )
        from luggage_description.scene_tf_config_utils import (
            load_scene_tf_config,
        )

        node = _Node()
        node._scene = load_scene_tf_config(SCENE)
        producer = build_production_route_producer(
            node, SCENE, lambda: {}, lambda: None,
            route_budget_override_s=180.0)
        self.assertEqual(producer.budget_ns, int(180.0 * 1e9))
        # Non-positive override keeps the frozen formula (box_count=3).
        producer_default = build_production_route_producer(
            node, SCENE, lambda: {}, lambda: None,
            route_budget_override_s=0.0)
        from luggage_planning.routing.grid_search import route_budget_ns
        self.assertEqual(producer_default.budget_ns, route_budget_ns(3))
        # Node-param path: the factory reads the declared param when the
        # kwarg is absent (production wiring).
        node._params["diagnostic_route_budget_s"] = 180.0
        producer_param = build_production_route_producer(
            node, SCENE, lambda: {}, lambda: None)
        self.assertEqual(producer_param.budget_ns, int(180.0 * 1e9))

    def test_selector_caches_one_factory_result(self):
        import trajectory_selector_node as selector
        from luggage_planning.routing import factory

        node = selector.TrajectorySelectorNode.__new__(
            selector.TrajectorySelectorNode)
        node._route_producer = None
        node._scene_tf_path = SCENE
        node._surface = {"height": [1]}
        node._fetch_cargo_map_stats = lambda: {"success": True}
        calls = []

        def build(host, path, surface_provider, stats_provider):
            calls.append((
                host, path, surface_provider(), stats_provider()))
            return "producer"

        original = factory.build_production_route_producer
        factory.build_production_route_producer = build
        try:
            # The node imports the name inside the method, so patch the
            # module attribute before the call. Re-import binding is the
            # function's local import, which reads this attribute.
            self.assertEqual(node._route_producer_or_none(), "producer")
            self.assertEqual(node._route_producer_or_none(), "producer")
        finally:
            factory.build_production_route_producer = original
        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0][1], SCENE)
        self.assertEqual(calls[0][2], {"height": [1]})
        self.assertEqual(calls[0][3], {"success": True})


class StageTimingsTests(unittest.TestCase):
    """[Functional diagnostic, 2026-10-22] prep is its own stage; the
    search residue never borrows prep/certify/revalidate time."""

    def test_prep_is_separate_from_search_residue(self):
        from luggage_planning.routing.producer import RouteProducer

        producer = RouteProducer.__new__(RouteProducer)
        producer.budget_ns = 60_000_000_000
        producer._clock_ns = lambda: 30_000_000_000
        producer._stage_ns = {
            "prep": 2_000_000_000,
            "certify": 3_000_000_000,
            "revalidate": 1_000_000_000,
        }
        rows = [
            {"reason": "IK_SEED_MISS", "elapsed_ms": 4000.0},
            {"reason": "ATTEMPT_SUMMARY", "elapsed_ms": 99.0},
        ]
        timings = producer._stage_timings(0, rows)
        self.assertEqual(timings["prep_ms"], 2000.0)
        self.assertEqual(timings["certify_ms"], 3000.0)
        self.assertEqual(timings["revalidate_ms"], 1000.0)
        # 30 s total - 2 prep - 3 certify - 1 revalidate = 24 s search.
        self.assertEqual(timings["search_ms"], 24000.0)
        self.assertEqual(timings["total_ms"], 30000.0)
        self.assertEqual(timings["by_reason_ms"], {"IK_SEED_MISS": 4000.0})

    def test_prep_missing_counts_as_zero(self):
        from luggage_planning.routing.producer import RouteProducer

        producer = RouteProducer.__new__(RouteProducer)
        producer.budget_ns = 60_000_000_000
        producer._clock_ns = lambda: 10_000_000_000
        producer._stage_ns = {"certify": 0, "revalidate": 0}
        timings = producer._stage_timings(0, [])
        self.assertEqual(timings["prep_ms"], 0.0)
        self.assertEqual(timings["search_ms"], 10000.0)


if __name__ == "__main__":
    unittest.main()
