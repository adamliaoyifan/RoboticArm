"""B0 calls for the master pack baseline enablement.

Each gate is a real function call: one passing input and one related
rejection. Source text is not the assertion.
"""

import hashlib
import importlib.util
import os
import sys
import tempfile
import unittest

from luggage_description.box_catalog_utils import (
    box_catalog_entries,
    load_box_catalog,
)
from luggage_description.runtime_identity import scene_revision_from_path
from luggage_description.suitcase_visual import (
    size_tier_name,
    sized_model_name,
)
from luggage_planning.site_pose_schema import (
    SIM_SITE_POSE_SCHEMA,
    load_site_poses,
    validate_site_poses,
)
from rclpy.qos import DurabilityPolicy

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DESC = os.path.normpath(os.path.join(PKG, "..", "luggage_description"))
DRIVER = os.path.join(PKG, "scripts", "pack_eval_driver.py")


def _driver():
    scripts = os.path.join(PKG, "scripts")
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    spec = importlib.util.spec_from_file_location("pack_eval_driver_b0", DRIVER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class _QosHost(object):
    def __init__(self, typed):
        self._typed = typed
        self._group = None
        self.subscriptions = []

    def _on_surface_v2(self, _msg):
        return None

    def create_subscription(self, *args, **kwargs):
        self.subscriptions.append((args, kwargs))


class TestDriverCli(unittest.TestCase):
    def test_frame_runtime_flags_reach_the_parent(self):
        driver = _driver()
        out = tempfile.mkdtemp(prefix="b0-cli-")
        args = driver.merge_args([
            "--out", out,
            "--frame-runtime",
            "--frame-peer", "pack_eval_driver",
            "--stop-on-first-failure",
            "--max-boxes", "2",
            "--sequence-ids", "inch20",
            "--observe-pose", "pickup_observe_sim",
            "--seed", "0",
        ])
        self.assertTrue(args.frame_runtime)
        self.assertEqual(args.frame_peer, "pack_eval_driver")
        self.assertTrue(args.stop_on_first_failure)
        self.assertEqual(args.max_boxes, 2)
        self.assertEqual(args.sequence_ids, "inch20")
        self.assertEqual(args.observe_pose, "pickup_observe_sim")

    def test_missing_out_is_rejected(self):
        driver = _driver()
        with self.assertRaises(SystemExit):
            driver.merge_args(["--frame-runtime"])

    def test_slot_decode_uses_the_planner_request_id(self):
        from luggage_description.frame_diagnostics import (
            BoundaryContext, validate_context)
        from luggage_description.frame_contracts import ContractError
        driver = _driver()
        stamped = driver.placement_slot_context_request_id(
            "pack-1", "place-v2-3")
        self.assertEqual(stamped, "place-v2-3")
        record = {
            "request_id": stamped,
            "scene_revision": "abc",
            "time_basis": "scene",
            "frame_id": "container_link",
            "reference": "box_center",
        }
        context = BoundaryContext(
            "F07", "typed_driver_adapter", "pack_eval_driver", stamped,
            "abc", "container_link", "box_center", "scene")
        validate_context(record, context)
        foreign = BoundaryContext(
            "F07", "typed_driver_adapter", "pack_eval_driver", stamped,
            "other-scene", "container_link", "box_center", "scene")
        with self.assertRaises(ContractError):
            validate_context(record, foreign)

    def test_exit_code_requires_every_requested_box(self):
        driver = _driver()
        self.assertEqual(driver.process_exit_code("MAX_BOXES", 2, 2), 0)
        self.assertEqual(driver.process_exit_code("ABORT", 0, 2), 1)
        self.assertEqual(driver.process_exit_code("MAX_BOXES", 1, 2), 1)

    def test_default_does_not_adopt_frame_runtime(self):
        driver = _driver()
        out = tempfile.mkdtemp(prefix="b0-cli-off-")
        args = driver.merge_args(["--out", out])
        self.assertFalse(args.frame_runtime)
        self.assertFalse(args.stop_on_first_failure)


class TestDriverConstruction(unittest.TestCase):
    def test_typed_chain_builds_a_transient_local_subscription(self):
        driver = _driver()
        host = _QosHost(typed=object())
        driver.PackEvalDriver._subscribe_typed_surface(host)
        self.assertEqual(len(host.subscriptions), 1)
        qos = host.subscriptions[0][0][3]
        self.assertEqual(qos.durability, DurabilityPolicy.TRANSIENT_LOCAL)

    def test_without_typed_chain_there_is_no_v2_subscription(self):
        driver = _driver()
        host = _QosHost(typed=None)
        driver.PackEvalDriver._subscribe_typed_surface(host)
        self.assertEqual(host.subscriptions, [])


class TestInstallIdentity(unittest.TestCase):
    def test_source_hash_matches_the_file_that_was_loaded(self):
        driver = _driver()
        loaded = driver.driver_source_sha256()
        digest = hashlib.sha256()
        with open(DRIVER, "rb") as handle:
            digest.update(handle.read())
        self.assertEqual(loaded, digest.hexdigest())

    def test_foreign_bytes_are_not_this_driver(self):
        driver = _driver()
        loaded = driver.driver_source_sha256()
        foreign = tempfile.NamedTemporaryFile(delete=False)
        foreign.write(b"not the pack eval driver")
        foreign.close()
        try:
            digest = hashlib.sha256(open(foreign.name, "rb").read()).hexdigest()
        finally:
            os.unlink(foreign.name)
        self.assertNotEqual(loaded, digest)


class TestPoseIdentity(unittest.TestCase):
    def setUp(self):
        self.scene = os.path.join(DESC, "config", "scene_tf_deploy_update.yaml")
        self.poses = os.path.join(DESC, "config", "robot_poses.baseline_update.yaml")
        self.revision = scene_revision_from_path(self.scene)

    def test_baseline_poses_match_the_deploy_update_scene(self):
        report = validate_site_poses(
            load_site_poses(self.poses),
            expected_scene_revision=self.revision,
            schema=SIM_SITE_POSE_SCHEMA)
        self.assertTrue(report["ok"], report["issues"])
        self.assertEqual(report["scene_revision"], self.revision)
        self.assertTrue(report["poses"]["pickup_observe_sim"]["valid"])

    def test_wrong_scene_revision_is_rejected(self):
        report = validate_site_poses(
            load_site_poses(self.poses),
            expected_scene_revision="0" * 32,
            schema=SIM_SITE_POSE_SCHEMA)
        self.assertFalse(report["ok"])
        self.assertTrue(any("wrong scene" in item for item in report["issues"]))


class TestInch20Catalog(unittest.TestCase):
    def test_inch20_resolves_to_its_model_directory(self):
        catalog = load_box_catalog(os.path.join(
            DESC, "config", "box_catalog.yaml.example"))
        by_id = {entry["id"]: entry for entry in box_catalog_entries(catalog)}
        entry = by_id["inch20"]
        self.assertEqual(size_tier_name(entry["size"]), "inch20")
        model = sized_model_name(entry["visual"], "inch20")
        sdf = os.path.join(PKG, "models", model, "model.sdf")
        self.assertTrue(os.path.isfile(sdf), sdf)

    def test_unknown_sequence_id_is_rejected(self):
        scripts = os.path.join(PKG, "scripts")
        if scripts not in sys.path:
            sys.path.insert(0, scripts)
        from pickup_box_spawner_node import PickupBoxSpawner

        catalog = load_box_catalog(os.path.join(
            DESC, "config", "box_catalog.yaml.example"))
        host = type("Host", (), {})()
        host._sequence = 0
        host._sequence_ids = ["not-a-box"]
        host._entries_by_id = {
            entry["id"]: entry for entry in box_catalog_entries(catalog)}

        def _missing(_name):
            raise RuntimeError("parameter unset")

        host.get_parameter = _missing
        with self.assertRaises(RuntimeError) as caught:
            PickupBoxSpawner._choose_entry(host)
        self.assertIn("unknown sequence box id", str(caught.exception))
