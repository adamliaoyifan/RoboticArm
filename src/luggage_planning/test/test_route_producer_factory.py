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
        import luggage_planning.moveit_edge_adapter as adapter

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
        from luggage_planning.grid_path_search import route_budget_ns
        from luggage_planning.map_revalidation import MapRevalidator
        from luggage_planning.planning_scene_client import PlanningSceneClient
        from luggage_planning.route_producer import (
            RouteProducer,
            certification_policy_from_contract,
            revalidation_policy_from_contract,
        )
        from luggage_planning.route_producer_factory import (
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

    def test_selector_caches_one_factory_result(self):
        import trajectory_selector_node as selector
        from luggage_planning import route_producer_factory as factory

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


if __name__ == "__main__":
    unittest.main()
