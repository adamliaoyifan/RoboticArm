#!/usr/bin/env python3
"""Unit tests for scripts/backfill_pickup_labels.py (click -> world XY)."""
import importlib.util
import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "fixtures"))
from make_tiny_replay_bag import (  # noqa: E402
    BASE_NS,
    FRAME_DT_NS,
    build_fixture,
)


def _load_script():
    path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "..", "scripts", "backfill_pickup_labels.py")
    spec = importlib.util.spec_from_file_location(
        "backfill_pickup_labels", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class TestPerLabelTfMode(unittest.TestCase):
    def test_tf_mode_reflects_each_labels_own_lookup(self):
        # Dynamic base->optical samples at t0 and t0+40 ms bracket the
        # t1 depth frame (interpolated) but not the t2+5 ms one (nearest
        # fallback). Each label's tf_mode must describe its OWN lookup —
        # not the cumulative run state, which used to stamp every label
        # after the first interpolation as "interpolated".
        mod = _load_script()
        t0 = BASE_NS
        t1 = t0 + FRAME_DT_NS                       # 33 ms: bracketed
        t2b = t0 + 2 * FRAME_DT_NS + 5_000_000      # 71 ms: unbracketed
        with tempfile.TemporaryDirectory() as tmp:
            bag = build_fixture(os.path.join(tmp, "tiny.mcap"),
                                camera_tf_stamps=(t0, t0 + 40_000_000))
            clicks = os.path.join(tmp, "clicks.json")
            with open(clicks, "w", encoding="utf-8") as handle:
                json.dump({"labels": [
                    {"frame_id": "f_interp", "stamp": t1 / 1e9,
                     "suction_safe_lid_center_pixel": [8, 6]},
                    {"frame_id": "f_nearest", "stamp": t2b / 1e9,
                     "suction_safe_lid_center_pixel": [8, 6]},
                ]}, handle)
            out = os.path.join(tmp, "labels.json")
            rc = mod.main(["--bag", bag, "--clicks", clicks,
                           "--out", out, "--tf-interpolate"])
            self.assertEqual(rc, 0, out)
            with open(out, encoding="utf-8") as handle:
                payload = json.load(handle)
            self.assertEqual(payload["unmatched_stamp_count"], 0)
            modes = {row["frame_id"]: row["tf_mode"]
                     for row in payload["labels"]}
            self.assertEqual(modes["f_interp"], "interpolated")
            self.assertEqual(modes["f_nearest"], "nearest")
            for row in payload["labels"]:
                self.assertEqual(row["depth_mm"], 1500.0)
                self.assertIsNotNone(
                    row["suction_safe_lid_center_world_xy"])


if __name__ == "__main__":
    unittest.main()
