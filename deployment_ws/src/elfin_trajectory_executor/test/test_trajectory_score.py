"""实机代码 (site / real-cell). Not part of the Gazebo simulation stack."""

import json
import tempfile
import unittest
from pathlib import Path

from elfin_trajectory_executor.trajectory_score import (
    internal_zero_speed_gaps,
    score_run,
)


def _q(j1: float):
    return [j1, 0.0, 0.0, 0.0, 0.0, 0.0]


class TrajectoryScoreTest(unittest.TestCase):
    def test_180ms_internal_stop_fails_continuity(self):
        samples = []
        t = 0.0
        pos = 0.0
        # move
        for _ in range(10):
            samples.append({"t": t, "q_deg": _q(pos)})
            pos += 0.5
            t += 0.01
        # 180 ms stop
        for _ in range(18):
            samples.append({"t": t, "q_deg": _q(pos)})
            t += 0.01
        # move again
        for _ in range(10):
            samples.append({"t": t, "q_deg": _q(pos)})
            pos += 0.5
            t += 0.01
        score = score_run(
            {
                "joint_samples": samples,
                "commands": [{"vel_deg": 20.0, "accel_deg": 60.0, "code": 0}],
                "events": [{"event": "succeeded"}],
            }
        )
        self.assertGreaterEqual(score["gap_max_s"], 0.17)
        self.assertFalse(score["pass_continuity"])
        self.assertTrue(score["pass_40083_fix"])

    def test_clean_20ms_grid_jitter(self):
        times = [i * 0.02 for i in range(50)]
        score = score_run(
            {
                "joint_samples": [
                    {"t": t, "q_deg": _q(t * 10.0)} for t in times
                ],
                "commands": [{"vel_deg": 20.0, "accel_deg": 60.0, "code": 0}],
                "command_times": times,
                "events": [{"event": "succeeded"}],
            }
        )
        self.assertTrue(score["checks"]["jitter_p99_ok"])
        self.assertTrue(score["checks"]["continuity_no_gap_80ms"])

    def test_40083_log_fails_fix_gate(self):
        score = score_run(
            {
                "joint_samples": [
                    {"t": 0.0, "q_deg": _q(0.0)},
                    {"t": 0.01, "q_deg": _q(0.2)},
                ],
                "commands": [{"vel_deg": 54.0, "accel_deg": 108.0, "code": 40083}],
                "events": [{"event": "aborted"}],
            }
        )
        self.assertFalse(score["pass_40083_fix"])
        self.assertFalse(score["checks"]["zero_40083"])
        self.assertFalse(score["checks"]["accel_le_cap"])

    def test_internal_gaps_ignore_leading_trailing(self):
        rows = [
            {"t": 0.00, "speed_deg_s": 0.0},
            {"t": 0.10, "speed_deg_s": 0.0},
            {"t": 0.20, "speed_deg_s": 10.0},
            {"t": 0.30, "speed_deg_s": 10.0},
            {"t": 0.40, "speed_deg_s": 0.0},
            {"t": 0.50, "speed_deg_s": 0.0},
        ]
        gaps = internal_zero_speed_gaps(rows)
        self.assertEqual(gaps, [])

    def test_score_files_roundtrip(self):
        payload = {
            "joint_samples": [
                {"t": 0.0, "q_deg": _q(0.0)},
                {"t": 0.01, "q_deg": _q(0.3)},
            ],
            "commands": [{"vel_deg": 20.0, "accel_deg": 60.0, "code": 0}],
        }
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "run.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            from elfin_trajectory_executor.trajectory_score import score_files
            score = score_files(payload_json=path)
            self.assertTrue(score["pass_40083_fix"])


if __name__ == "__main__":
    unittest.main()
