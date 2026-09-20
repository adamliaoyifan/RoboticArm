#!/usr/bin/env python3
"""SegmenterIngest assembles RGB stamp + optional task epoch (no ROS)."""

from types import SimpleNamespace
import unittest

from luggage_perception.segmenter_ingest import SegmenterIngest


class TestSegmenterIngest(unittest.TestCase):

    def test_assemble_without_epoch_is_generation_zero(self):
        ingest = SegmenterIngest(maxlen=8, horizon_sec=1.0)
        obs = ingest.assemble(SimpleNamespace(stamp=1.5, frame_id="optical"))
        self.assertEqual(obs.generation, 0)
        self.assertEqual(obs.instance_id, "")
        self.assertEqual(obs.stamp, 1.5)
        self.assertEqual(obs.frame_id, "optical")

    def test_next_assemble_picks_up_note_epoch(self):
        ingest = SegmenterIngest(maxlen=8, horizon_sec=1.0)
        first = ingest.assemble(SimpleNamespace(stamp=1.0, frame_id="optical"))
        ingest.note_epoch("pickup_box_0001_carryon", 2)
        second = ingest.assemble(SimpleNamespace(stamp=1.1, frame_id="optical"))
        self.assertEqual(first.generation, 0)
        self.assertEqual(first.instance_id, "")
        self.assertEqual(second.generation, 2)
        self.assertEqual(second.instance_id, "pickup_box_0001_carryon")

    def test_note_epoch_does_not_rewrite_prior_observation(self):
        ingest = SegmenterIngest(maxlen=8, horizon_sec=1.0)
        ingest.note_epoch("box_a", 1)
        held = ingest.assemble(SimpleNamespace(stamp=2.0, frame_id="optical"))
        ingest.note_epoch("box_b", 2)
        self.assertEqual(held.generation, 1)
        self.assertEqual(held.instance_id, "box_a")

    def test_ring_eviction_does_not_mutate_held_observation(self):
        ingest = SegmenterIngest(maxlen=2, horizon_sec=10.0)
        ingest.note_epoch("box_a", 1)
        held = ingest.assemble(SimpleNamespace(stamp=1.0, frame_id="optical"))
        ingest.assemble(SimpleNamespace(stamp=1.1, frame_id="optical"))
        ingest.assemble(SimpleNamespace(stamp=1.2, frame_id="optical"))
        self.assertEqual(held.generation, 1)
        self.assertEqual(held.instance_id, "box_a")
        self.assertEqual(held.stamp, 1.0)

    def test_note_epoch_returns_false_when_unchanged(self):
        ingest = SegmenterIngest()
        self.assertTrue(ingest.note_epoch("box_a", 3))
        self.assertFalse(ingest.note_epoch("box_a", 3))


if __name__ == "__main__":
    unittest.main()
