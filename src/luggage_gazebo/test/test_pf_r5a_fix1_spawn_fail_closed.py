#!/usr/bin/env python3
"""PF-R5A-FIX1: spawner-level fail-closed behavior (handler spies).

docs/agents/reviews/2026-09-05_1550_pf-r5a-closure-review.md findings
1-4: the tier-omitting cache key, unknown-visual normalization, and
clear-before-validate ordering are not covered by pure-resolver tests.
These tests drive ``handle_spawn_next`` itself with stubbed services and
publishers and prove a reference failure mutates nothing and publishes
nothing.
"""

import importlib.util
import os
import random
import tempfile
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.normpath(os.path.join(_HERE, ".."))  # src/luggage_gazebo
_MODELS = os.path.join(_PKG_ROOT, "models")

_spec = importlib.util.spec_from_file_location(
    "pickup_box_spawner_node",
    os.path.join(_PKG_ROOT, "scripts", "pickup_box_spawner_node.py"))
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)


class _Logger(object):
    def error(self, *_a, **_k):
        pass

    def warning(self, *_a, **_k):
        pass

    def info(self, *_a, **_k):
        pass


class _Pub(object):
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


def _make_spawner(models_root):
    node = object.__new__(_mod.PickupBoxSpawner)
    node._visual_kind = "mesh"
    node._model_prefix = "pickup_box"
    node._size_mode = "catalog"
    node._generation = 3
    node._published_id = "pickup_box_0041_prior"
    node._finalized_models = []
    node._current_yaw = 0.0
    node._current_mass = 8.0
    node._models_root_dir = models_root
    node._observable_cache = {}
    node._rng = random.Random(7)
    node._sequence = 41
    node._current_box = "PREEXISTING_BOX_STATE"
    node._current_model = "pickup_box_0041_prior"
    node._current_ref = {"version": "v", "stl_sha256": "prior"}
    node._visual_settle_sec = 0.0
    node._replace_settle_sec = 0.0
    node._place_persist_sec = 0.0
    node._place_read_timeout_sec = 0.0
    node._box_pub = _Pub()
    node._size_eval_pub = _Pub()
    node._finalized_pub = _Pub()
    calls = {"clear": 0, "spawn": 0, "delete": 0}

    def _sample_box():
        return ({"id": "carryon"}, [0.55, 0.40, 0.25], 8.0, False,
                "carryon")

    def _entry_pose(entry, size=None):
        from geometry_msgs.msg import Pose
        return Pose(), 0.0

    def _clear(_req, _resp):
        calls["clear"] += 1
        class _R(object):
            success = True
            message = ""
        return _R()

    def _spawn_model(*_a, **_k):
        calls["spawn"] += 1
        return None

    def _delete_model(*_a, **_k):
        calls["delete"] += 1
        return None

    def _enforce_intended_pose(*_a, **_k):
        return None

    node._sample_box = _sample_box
    node._entry_pose = _entry_pose
    node.handle_clear = _clear
    node._spawn_model = _spawn_model
    node._delete_model = _delete_model
    node._enforce_intended_pose = _enforce_intended_pose
    node.get_logger = lambda: _Logger()
    return node, calls


class TestSpawnFailClosedHandler(unittest.TestCase):

    def test_reference_failure_mutates_and_publishes_nothing(self):
        with tempfile.TemporaryDirectory() as empty_root:
            node, calls = _make_spawner(empty_root)
            rng_before = node._rng.getstate()
            response = _mod.SpawnNextBox.Response()
            out = node.handle_spawn_next(None, response)
            self.assertFalse(out.success)
            self.assertTrue(out.message.startswith(
                "MESH_REFERENCE_UNAVAILABLE"), out.message)
            # No clear/delete/create of the existing instance.
            self.assertEqual(calls["clear"], 0)
            self.assertEqual(calls["spawn"], 0)
            self.assertEqual(calls["delete"], 0)
            # No state, sequence, or publication changes.
            self.assertEqual(node._sequence, 41)
            self.assertEqual(node._current_box, "PREEXISTING_BOX_STATE")
            self.assertEqual(node._current_model, "pickup_box_0041_prior")
            self.assertEqual(node._current_ref["stl_sha256"], "prior")
            self.assertEqual(node._box_pub.published, [])
            self.assertEqual(node._size_eval_pub.published, [])
            self.assertEqual(node._finalized_pub.published, [])
            # PF-R5A-FIX2: exact RNG-state rollback on failure.
            self.assertEqual(node._rng.getstate(), rng_before)

    def test_repeated_failures_select_the_same_candidate(self):
        """Deterministic rollback: every retry samples the SAME invalid
        candidate (same failure message), and the RNG never advances
        across failures."""
        with tempfile.TemporaryDirectory() as empty_root:
            node, calls = _make_spawner(empty_root)
            rng_before = node._rng.getstate()
            messages = []
            for _ in range(3):
                response = _mod.SpawnNextBox.Response()
                out = node.handle_spawn_next(None, response)
                self.assertFalse(out.success)
                messages.append(out.message)
                self.assertEqual(node._rng.getstate(), rng_before)
            self.assertEqual(len(set(messages)), 1,
                             "retry selected a different candidate")
            self.assertEqual(calls["clear"], 0)
            self.assertEqual(calls["spawn"], 0)

    def test_valid_reference_clears_and_spawns(self):
        node, calls = _make_spawner(_MODELS)
        response = _mod.SpawnNextBox.Response()
        out = node.handle_spawn_next(None, response)
        self.assertTrue(out.success, out.message)
        self.assertEqual(calls["clear"], 1)
        self.assertEqual(calls["spawn"], 1)
        # Sequence advanced exactly once and state was replaced.
        self.assertEqual(node._sequence, 42)
        self.assertIsNot(node._current_box, "PREEXISTING_BOX_STATE")


class TestCacheTierSeparation(unittest.TestCase):

    def test_same_visual_cross_tier_cache_misses(self):
        """Finding 1: the cache key must carry the resolved tier — a
        small-then-large sequence must return differing dimensions and
        hashes, not the first tier's object."""
        with tempfile.TemporaryDirectory() as tmp:
            node, _calls = _make_spawner(tmp)
            node._models_root_dir = _MODELS
            small = node._observable_reference(
                (0.55, 0.40, 0.25), "suitcase_loafbrr")
            large = node._observable_reference(
                (0.80, 0.50, 0.32), "suitcase_loafbrr")
            self.assertIsNot(small, large)
            self.assertAlmostEqual(small["width"], 0.5159, places=3)
            self.assertAlmostEqual(large["width"], 0.7504, places=3)
            self.assertNotEqual(small["stl_sha256"], large["stl_sha256"])
            self.assertEqual(small["tier"], "small")
            self.assertEqual(large["tier"], "large")
            # Cached re-resolution returns the same object per tier.
            self.assertIs(
                node._observable_reference(
                    (0.55, 0.40, 0.25), "suitcase_loafbrr"), small)
            self.assertIs(
                node._observable_reference(
                    (0.80, 0.50, 0.32), "suitcase_loafbrr"), large)

    def test_unknown_visual_rejected_independently_of_tier(self):
        """Finding 2: unknown visual ids must be rejected before path
        normalization (no silent loafbrr fallback), even with a valid
        catalog tier."""
        from luggage_description.suitcase_visual import (
            MeshReferenceError, resolve_observable_reference)
        with self.assertRaises(MeshReferenceError) as ctx:
            resolve_observable_reference(
                _MODELS, (0.55, 0.40, 0.25), "unknown_visual")
        self.assertIn("unknown visual id", str(ctx.exception))
        self.assertNotIn("unknown size tier", str(ctx.exception))
        with self.assertRaises(MeshReferenceError) as ctx2:
            resolve_observable_reference(
                _MODELS, (0.61, 0.44, 0.27), "suitcase_loafbrr")
        self.assertIn("unknown size tier", str(ctx2.exception))


if __name__ == "__main__":
    unittest.main()
