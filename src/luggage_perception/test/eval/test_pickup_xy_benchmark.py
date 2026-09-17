import unittest

from luggage_perception.eval.pickup_xy_benchmark import (
    Candidate,
    Label,
    acceptance,
    score_candidates,
)


class PickupXyBenchmarkTest(unittest.TestCase):
    def test_scores_acceptance_against_pca_baseline(self):
        labels = {
            "f1": Label("f1", "bag1", 1.0, (0.0, 0.0)),
            "f2": Label("f2", "bag1", 2.0, (0.0, 0.0)),
            "f3": Label("f3", "bag2", 3.0, (0.0, 0.0)),
            "f4": Label("f4", "bag2", 4.0, (0.0, 0.0)),
            "f5": Label("f5", "bag3", 5.0, (0.0, 0.0)),
        }
        candidates = []
        for frame_id in labels:
            candidates.append(Candidate(
                frame_id, "bag", 0.0, "pca_center", (0.09, 0.0)))
            candidates.append(Candidate(
                frame_id, "bag", 0.0, "robust_blended_center", (0.02, 0.0)))

        summary = score_candidates(labels, candidates)
        result = acceptance(summary, "robust_blended_center")

        self.assertEqual(result["outcome"], "pass")
        self.assertEqual(result["sample_count"], 5)
        self.assertAlmostEqual(result["median_error_m"], 0.02)
        self.assertAlmostEqual(result["p95_error_m"], 0.02)
        self.assertAlmostEqual(result["improved_fraction_vs_pca"], 1.0)

    def test_missing_comparison_is_not_evaluated(self):
        labels = {"f1": Label("f1", "bag", 1.0, (0.0, 0.0))}
        candidates = [
            Candidate("f1", "bag", 1.0, "robust_blended_center", (0.01, 0.0)),
        ]

        summary = score_candidates(labels, candidates)
        result = acceptance(summary, "robust_blended_center")

        self.assertEqual(result["outcome"], "not_evaluated")


if __name__ == "__main__":
    unittest.main()
