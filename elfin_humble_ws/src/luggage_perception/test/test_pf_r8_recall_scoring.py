"""Unit tests for the PF-R8 A4 recall scorer (offline, no ROS)."""

import importlib.util
import os
import unittest

_SPEC = importlib.util.spec_from_file_location(
    "pf_r8_recall_capture",
    os.path.join(os.path.dirname(__file__), "pf_r8_recall_capture.py"))
_MOD = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MOD)
score_rows = _MOD.score_rows


def _row(i, instance="box-1", accepted=True, gen=1):
    return {
        "stamp": float(i),
        "generation": gen,
        "instance_id": instance,
        "image_width": 640,
        "image_height": 480,
        "detections": ([{"label": 2, "conf": 0.25,
                         "bbox": [178, 144, 405, 297],
                         "accepted": accepted}] if accepted else []),
    }


class TestScoring(unittest.TestCase):

    def test_settled_warmup_excluded(self):
        rows = [_row(i) for i in range(10)]
        s = score_rows(rows, warmup=5)
        # rows 0-4 are warmup; 5 settled frames, all accepted.
        self.assertEqual(5, s["settled_frames"])
        self.assertEqual(1.0, s["recall_raw"])

    def test_miss_run_and_bridged_recall(self):
        rows = [_row(i) for i in range(10)]
        # two consecutive misses among settled frames (rows 6, 7): with
        # window 5 the ratios are [P,P,P,P,M]=0.8 then [P,P,P,M,M]=0.6,
        # both >= 0.5, so the gate bridges both (A4-1's k<=2).
        rows[6] = _row(6, accepted=False)
        rows[7] = _row(7, accepted=False)
        s = score_rows(rows, warmup=5)
        self.assertEqual(2, s["max_miss_run"])
        self.assertEqual(0.6, round(s["recall_raw"], 1))
        self.assertEqual(1.0, s["recall_gate_bridged"])

    def test_third_consecutive_miss_not_bridged(self):
        rows = [_row(i) for i in range(10)]
        rows[6] = _row(6, accepted=False)
        rows[7] = _row(7, accepted=False)
        rows[8] = _row(8, accepted=False)
        s = score_rows(rows, warmup=5)
        self.assertEqual(3, s["max_miss_run"])
        # settled rows 5..9 = P,M,M,M,P: holds fire at 6 (0.8) and 7 (0.6)
        # but not 8 ([P,P,M,M,M] = 0.4 < 0.5): 4 of 5 bridged.
        self.assertEqual(0.8, round(s["recall_gate_bridged"], 1))

    def test_epoch_change_resets_gate(self):
        rows = [_row(i, instance="a") for i in range(7)]
        rows += [_row(i + 7, instance="b", gen=2) for i in range(7)]
        rows[10] = _row(10, instance="b", gen=2, accepted=False)
        s = score_rows(rows, warmup=5)
        # settled = instance a rows 5,6 + instance b rows 12,13 = 4; the
        # miss at row 10 is instance-b warmup. The gate reset at the epoch
        # boundary keeps instance b's window free of instance a's samples.
        self.assertEqual(4, s["settled_frames"])
        self.assertEqual(1.0, s["recall_gate_bridged"])


if __name__ == "__main__":
    unittest.main()
