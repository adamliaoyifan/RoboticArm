"""Unbound multi-detection cargo is not a pick, and its bundle is one stamp."""

import json
import unittest
from pathlib import Path

import numpy as np

from luggage_perception.ambiguity_capture import FrameCapture
from luggage_perception.target_association import (
    admit_pick_frame, association_inputs_aligned,
    target_association_ambiguous)


class TestAssociationRule(unittest.TestCase):
    def test_two_detections_without_an_instance_are_ambiguous(self):
        self.assertTrue(target_association_ambiguous(2, 0))

    def test_two_detections_bound_to_an_instance_are_usable(self):
        self.assertFalse(target_association_ambiguous(2, 32045))

    def test_one_detection_is_not_this_refusal(self):
        self.assertFalse(target_association_ambiguous(1, 0))

    def test_a_missing_mask_with_two_detections_is_unbound(self):
        self.assertTrue(target_association_ambiguous(2, None))

    def test_one_detection_without_a_mask_is_still_one_target(self):
        self.assertFalse(target_association_ambiguous(1, None))

    def test_a_count_from_another_stamp_is_not_association(self):
        # 135.432 has one detection. It must not clear 135.367.
        cloud = (135, 367000000)
        later = (135, 432000000)
        self.assertFalse(association_inputs_aligned(later, cloud, cloud))
        self.assertTrue(target_association_ambiguous(
            1, 32045, stamps_aligned=False))
        self.assertTrue(association_inputs_aligned(cloud, cloud, cloud))
        self.assertTrue(target_association_ambiguous(2, 0, stamps_aligned=True))
        self.assertTrue(target_association_ambiguous(
            2, None, stamps_aligned=True))

    def test_a_mask_from_another_stamp_is_not_this_frame(self):
        cloud = (135, 367000000)
        other = (135, 432000000)
        self.assertFalse(association_inputs_aligned(
            cloud, cloud, cloud, instance_stamp=other))
        self.assertTrue(association_inputs_aligned(
            cloud, cloud, cloud, instance_stamp=None))

    def test_a_missing_count_is_not_association(self):
        self.assertTrue(target_association_ambiguous(None, 32045))

    def test_the_failed_window_refuses_only_the_unbound_frames(self):
        # 2026-10-08 n3 trial 1, same-stamp detector rows. 135.432 is a
        # later one-detection frame; it is not used to bless 135.367.
        rows = [
            (134.838, 1, 32045, False),
            (134.904, 2, 0, True),
            (135.367, 2, 0, True),
            (135.432, 1, 32045, False),
        ]
        for _stamp, n_det, pixels, expect in rows:
            self.assertEqual(
                target_association_ambiguous(n_det, pixels), expect)

    def test_an_older_frame_is_not_a_substitute(self):
        older = {"size": 0.630, "support": False}
        self.assertIsNone(admit_pick_frame(True, {"size": 1.246}))
        self.assertIs(admit_pick_frame(False, older), older)


class TestCaptureLossBoundaries(unittest.TestCase):
    def setUp(self):
        import tempfile
        self.root = tempfile.mkdtemp()
        self.epoch = {
            "generation": 4, "instance_id": "pickup_box_0002_standard"}

    def _cap(self, **kwargs):
        kwargs.setdefault("max_pending", 8)
        kwargs.setdefault("timeout_sec", 1.0)
        return FrameCapture(self.root, **kwargs)

    def _meta(self, cap, stamp, role):
        cap.flush()
        found = list(Path(self.root).rglob("*%d*" % stamp))
        dirs = [p for p in found if p.is_dir()]
        self.assertTrue(dirs, stamp)
        return json.loads((dirs[0] / "frame.json").read_text())

    def test_omitted_component_is_written_incomplete(self):
        cap = self._cap()
        cap.consume(
            131934000000, self.epoch, "anomaly",
            {"detections": [{"bbox": [0, 0, 10, 10]}]},
            {"instance_mask": "not_in_buffer_at_consume"},
            now=0.0)
        cap.flush()
        meta = self._meta(cap, 131934000000, "anomaly")
        self.assertFalse(meta["evidence_complete"])
        self.assertFalse(meta["components"]["instance_mask"]["present"])
        self.assertEqual(
            meta["components"]["instance_mask"]["reason"],
            "not_in_buffer_at_consume")
        self.assertEqual(cap.counts["written"], 1)

    def test_delayed_component_completes_before_timeout(self):
        cap = self._cap()
        stamp = 132000000000
        cap.receive(stamp, self.epoch, "detections", [{"bbox": [1, 1, 2, 2]}], 0.0)
        cap.receive(
            stamp, self.epoch, "instance_mask",
            np.zeros((2, 2), dtype=np.uint16), 0.4)
        self.assertEqual(cap.expire(0.4), [])
        cap.seal(stamp, "anomaly", 0.4)
        cap.flush()
        meta = self._meta(cap, stamp, "anomaly")
        self.assertTrue(meta["evidence_complete"])
        self.assertEqual(cap.counts["expired"], 0)
        self.assertEqual(cap.counts["matched"], 1)

    def test_timeout_writes_the_partial_and_keeps_it(self):
        cap = self._cap()
        stamp = 131934000000
        cap.receive(stamp, self.epoch, "detections", [{"bbox": [0, 0, 4, 4]}], 0.0)
        cap.expire(1.0)
        cap.flush()
        meta = self._meta(cap, stamp, "anomaly")
        self.assertEqual(meta["closed"], "timeout")
        self.assertFalse(meta["components"]["timeout"]["present"])
        self.assertEqual(cap.counts["expired"], 1)
        self.assertTrue((Path(self.root) / "status.json").is_file())

    def test_pending_overflow_writes_the_evicted_stamp(self):
        cap = self._cap(max_pending=8)
        for i in range(9):
            cap.receive(1000 + i, self.epoch, "detections", [{"i": i}], float(i))
        cap.flush()
        meta = self._meta(cap, 1000, "record")
        self.assertEqual(meta["closed"], "overflow_before_seal")
        self.assertEqual(
            meta["components"]["pending"]["reason"], "overflow_before_seal")
        self.assertEqual(cap.counts["overflowed"], 1)
        self.assertGreaterEqual(cap.counts["written"], 1)

    def test_later_events_do_not_replace_the_pinned_pair(self):
        cap = self._cap()
        anomaly = np.array([[1.25, 0.0, 1.0]], dtype=np.float32)
        control = np.array([[0.63, 0.0, 1.0]], dtype=np.float32)
        cap.consume(131934000000, self.epoch, "anomaly", {"cloud": anomaly}, {}, 0.0)
        cap.consume(132067000000, self.epoch, "control", {"cloud": control}, {}, 1.0)
        cap.flush()
        for i in range(10):
            cap.consume(
                140000000000 + i, self.epoch, "anomaly",
                {"cloud": np.array([[9.0, 0.0, 1.0]], dtype=np.float32)},
                {}, 2.0 + i)
        cap.flush()
        meta = self._meta(cap, 131934000000, "anomaly")
        self.assertEqual(meta["stamp_ns"], 131934000000)
        arrays = np.load(list(Path(self.root).rglob("anomaly_131934000000"))[0] / "arrays.npz")
        self.assertAlmostEqual(float(arrays["cloud"][0, 0]), 1.25)
        self.assertEqual(cap.counts["repeats"], 10)
        self.assertTrue(list(Path(self.root).rglob("control_132067000000")))

    def test_write_failure_is_counted_and_leaves_a_stub(self):
        def boom(_job):
            raise OSError("disk")

        cap = self._cap(writer=boom)
        cap.consume(131934000000, self.epoch, "anomaly", {"detections": []}, {}, 0.0)
        cap.flush()
        self.assertEqual(cap.counts["write_failures"], 1)
        meta = self._meta(cap, 131934000000, "anomaly")
        self.assertEqual(meta["closed"], "write_failed")
        self.assertIn("disk", meta["error"])


if __name__ == "__main__":
    unittest.main()
