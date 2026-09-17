#!/usr/bin/env python3
"""Live cargo_volume_mapper + placement_planner geometry identity (GEO-8).

Launch-free: both production nodes run in one executor, no Gazebo and no
MoveIt. The negative case gives the mapper a different container hull and
requires ComputePlacement to fail closed instead of answering from its own
floor prior.
"""

import copy
import importlib.util
import os
import tempfile
import time
import unittest

import yaml

from luggage_description._share import ENV_SCENE_TF
from luggage_description.scene_tf_config_utils import (
    container_inner_geometry_descriptor,
    default_scene_tf_config_path,
    load_scene_tf_config,
)

_HERE = os.path.dirname(os.path.abspath(__file__))
_WS_SRC = os.path.normpath(os.path.join(_HERE, "..", ".."))


def _load_node_module(name, package, relative):
    path = os.path.join(_WS_SRC, package, "scripts", relative)
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _mutated_scene(path):
    """Same container with a different floor chamfer -> different hash."""
    scene = copy.deepcopy(load_scene_tf_config(path))
    chamfer = scene["container"]["inner"]["chamfer"]
    chamfer["floor_y"] = round(float(chamfer["floor_y"]) - 0.10, 4)
    handle = tempfile.NamedTemporaryFile(
        "w", suffix="_scene_tf.yaml", delete=False, encoding="utf-8")
    yaml.safe_dump(scene, handle)
    handle.close()
    return handle.name


class TestGeometryIdentityLive(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        try:
            import rclpy  # noqa: F401
            from luggage_msgs.srv import ComputePlacement  # noqa: F401
        except ImportError as exc:
            raise unittest.SkipTest("rclpy/luggage_msgs unavailable: %s" % exc)
        cls.scene_path = default_scene_tf_config_path()
        cls.mutated_path = _mutated_scene(cls.scene_path)
        cls.mapper_mod = _load_node_module(
            "cargo_volume_mapper_node", "luggage_perception",
            "cargo_volume_mapper_node.py")
        cls.planner_mod = _load_node_module(
            "placement_planner_node", "luggage_packing",
            "placement_planner_node.py")

    @classmethod
    def tearDownClass(cls):
        if getattr(cls, "mutated_path", None):
            os.unlink(cls.mutated_path)

    def setUp(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor

        self.rclpy = rclpy
        rclpy.init()
        self.executor = SingleThreadedExecutor()
        self.nodes = []
        self.addCleanup(self._shutdown)

    def _shutdown(self):
        for node in self.nodes:
            self.executor.remove_node(node)
            node.destroy_node()
        self.executor.shutdown()
        self.rclpy.shutdown()

    def _start(self, mapper_scene, planner_scene):
        # Each node resolves its own hull from its own scene config, which is
        # exactly the split GEO-8 has to detect.
        mapper = _construct(
            self.mapper_mod.CargoVolumeMapperNode, mapper_scene)
        planner = _construct(
            self.planner_mod.PlacementPlannerNode, planner_scene)
        for node in (mapper, planner):
            self.nodes.append(node)
            self.executor.add_node(node)
        return mapper, planner

    def _spin(self, seconds=1.0):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)

    def _spin_until(self, predicate, timeout_sec=10.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)
            if predicate():
                return True
        return False

    def _request(self, geometry_hash=""):
        from luggage_msgs.msg import DetectedLuggage
        from luggage_msgs.srv import ComputePlacement

        request = ComputePlacement.Request()
        box = DetectedLuggage()
        box.width, box.depth, box.height = 0.55, 0.40, 0.25
        box.height_valid = True
        box.height_source = 2
        box.pose.orientation.w = 1.0
        request.box = box
        request.geometry_hash = geometry_hash
        return request

    def _compute(self, planner, request):
        from luggage_msgs.srv import ComputePlacement

        client = planner.create_client(
            ComputePlacement, "/placement_planner/compute_placement")
        self.assertTrue(
            self._spin_until(client.service_is_ready, 10.0),
            "compute_placement service never came up")
        future = client.call_async(request)
        self.assertTrue(
            self._spin_until(future.done, 30.0), "ComputePlacement timed out")
        return future.result()

    # ------------------------------------------------------------------

    def test_matching_geometry_answers_with_its_hull_hash(self):
        expected = str(container_inner_geometry_descriptor(
            load_scene_tf_config(self.scene_path))["geometry_hash"])
        mapper, planner = self._start(self.scene_path, self.scene_path)
        self.assertEqual(mapper._geometry_hash, expected)
        self.assertTrue(
            self._spin_until(lambda: planner._surface is not None, 10.0),
            "planner never accepted the cargo map")
        self.assertEqual(
            str(planner._surface.get("geometry_hash")), expected)

        response = self._compute(planner, self._request(expected))
        self.assertTrue(response.success, response.message)
        self.assertEqual(response.geometry_hash, expected)
        self.assertEqual(response.reason_code, "")
        self.assertEqual(planner._rejected_maps, 0)

    def test_foreign_cargo_map_fails_closed_without_floor_prior(self):
        mapper, planner = self._start(self.mutated_path, self.scene_path)
        self.assertNotEqual(mapper._geometry_hash, planner._geometry_hash)
        self.assertTrue(
            self._spin_until(lambda: planner._rejected_maps > 0, 10.0),
            "planner accepted a map built for another hull")
        self.assertIsNone(planner._surface)

        response = self._compute(planner, self._request())
        self.assertFalse(response.success)
        self.assertEqual(response.reason_code, "CARGO_MAP_GEOMETRY_MISMATCH")
        self.assertEqual(response.geometry_hash, planner._geometry_hash)
        # Fail closed means no slot at all, not a floor-prior guess.
        self.assertEqual(
            (response.slot.width, response.slot.depth, response.slot.height),
            (0.0, 0.0, 0.0))

    def test_request_hash_pin_is_enforced(self):
        _mapper, planner = self._start(self.scene_path, self.scene_path)
        self._spin(0.5)
        response = self._compute(planner, self._request("not-this-hull"))
        self.assertFalse(response.success)
        self.assertEqual(response.reason_code, "CARGO_MAP_GEOMETRY_MISMATCH")


def _construct(node_class, scene_path):
    """Instantiate a production node against one scene config."""
    previous = os.environ.get(ENV_SCENE_TF)
    os.environ[ENV_SCENE_TF] = scene_path
    try:
        return node_class()
    finally:
        if previous is None:
            os.environ.pop(ENV_SCENE_TF, None)
        else:
            os.environ[ENV_SCENE_TF] = previous


if __name__ == "__main__":
    unittest.main()
