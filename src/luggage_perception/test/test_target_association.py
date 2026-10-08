"""Unbound multi-detection cargo is not a pick, and its bundle is one stamp."""

import json
import unittest

import numpy as np

from luggage_perception.ambiguity_capture import (
    AmbiguityCapture, selected_detection_index)
from luggage_perception.target_association import (
    admit_pick_frame, target_association_ambiguous)


class TestAssociationRule(unittest.TestCase):
    def test_two_detections_without_an_instance_are_ambiguous(self):
        self.assertTrue(target_association_ambiguous(2, 0))

    def test_two_detections_bound_to_an_instance_are_usable(self):
        self.assertFalse(target_association_ambiguous(2, 32045))

    def test_one_detection_is_not_this_refusal(self):
        self.assertFalse(target_association_ambiguous(1, 0))

    def test_a_missing_mask_is_not_treated_as_unbound(self):
        self.assertFalse(target_association_ambiguous(2, None))

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


class TestAmbiguityBundle(unittest.TestCase):
    def _measurement(self):
        return {
            "depth": np.zeros((2, 2), dtype=np.uint16),
            "label_map": np.array([[2, 0], [2, 2]], dtype=np.uint8),
            "instance_map": np.zeros((2, 2), dtype=np.uint16),
            "cargo_before": np.array([[0.0, 0.0, 1.0]], dtype=np.float32),
            "cargo_after": np.array([[0.0, 0.0, 1.0], [1.2, 0.0, 1.0]],
                                    dtype=np.float32),
            "epoch": {"generation": 4, "instance_id": "pickup_box_0002_standard"},
            "tf": {"translation": [0.0, 0.0, 0.0]},
            "frames": {"camera": "camera_depth_optical_frame", "world": "world"},
        }

    def test_one_detection_does_not_write(self):
        import tempfile
        cap = AmbiguityCapture(tempfile.mkdtemp())
        stamp = 135367000000
        cap.note_measurement(stamp, self._measurement())
        self.assertIsNone(cap.note_detections(stamp, [{
            "label": 2, "prompt": "suitcase", "confidence": 0.9,
            "bbox": [0, 0, 10, 10],
        }], 0))
        self.assertEqual(list(cap.root.iterdir()), [])

    def test_same_stamp_two_detections_write_both_clouds(self):
        import tempfile
        cap = AmbiguityCapture(tempfile.mkdtemp())
        stamp = 135367000000
        cap.note_measurement(stamp, self._measurement())
        dets = [
            {"label": 2, "prompt": "suitcase", "confidence": 0.4,
             "bbox": [0, 0, 10, 10]},
            {"label": 2, "prompt": "suitcase", "confidence": 0.8,
             "bbox": [20, 0, 40, 30]},
        ]
        path = cap.note_detections(stamp, dets, selected_detection_index(dets))
        self.assertIsNotNone(path)
        meta = json.loads(open(path + "/frame.json").read())
        self.assertEqual(meta["stamp_ns"], stamp)
        self.assertEqual(meta["epoch"]["generation"], 4)
        self.assertFalse(meta["selection_used"])
        self.assertEqual(meta["selected_index"], 1)
        arrays = np.load(path + "/arrays.npz")
        self.assertEqual(int(arrays["cargo_before"].shape[0]), 1)
        self.assertEqual(int(arrays["cargo_after"].shape[0]), 2)
        self.assertEqual(int(arrays["instance_map"].sum()), 0)


if __name__ == "__main__":
    unittest.main()
