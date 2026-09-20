#!/usr/bin/env python3
"""Unit tests for the per-bag replay index sidecar."""
import json
import os
import sys
import tempfile
import time
import unittest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "fixtures"))
from make_tiny_replay_bag import build_fixture  # noqa: E402

from luggage_perception.eval.bag_replay_index import (  # noqa: E402
    bag_identity,
    camera_info_frame_from_payload,
    camera_info_payload,
    count_jsonl_rows,
    default_cache_dir,
    load_index,
    write_index,
)
from luggage_perception.eval.bag_mcap_source import (  # noqa: E402
    scan_bag,
    select_image_topics,
)

COLOR = "/camera/d555/color/image_raw"
DEPTH = "/camera/d555/aligned_depth_to_color/image_raw"


def _base_index():
    return {
        "color_topic": COLOR, "depth_topic": DEPTH,
        "color_entries": [[10, 11], [20, 21]],
        "color_duplicates": [[15, 2]],
        "depth_entries": [[10, 11], [20, 21]],
        "depth_duplicates": [],
        "joint_payloads": [[10, {"stamp": 1e-8}]],
        "tcp_payloads": [],
        "camera_info": {},
        "camera_k_variants": {},
        "tf_static_message_count": 1,
        "lidar_stamps": [10],
        "tf_edges_file": None,
    }


class TestBagReplayIndex(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._tmp = tempfile.TemporaryDirectory()
        cls.bag = build_fixture(os.path.join(cls._tmp.name, "tiny.mcap"))
        cls.cache = os.path.join(cls._tmp.name, "cache")

    @classmethod
    def tearDownClass(cls):
        cls._tmp.cleanup()

    def _scan(self):
        scan = scan_bag(self.bag)
        color, depth = select_image_topics(scan)
        return scan, color, depth

    def test_key_stable_and_identity_sensitive(self):
        key_a, identity_a = bag_identity(self.bag)
        key_b, _identity = bag_identity(self.bag)
        self.assertEqual(key_a, key_b)
        self.assertTrue(key_a.startswith("v1:"))
        # An mtime bump alone changes the identity (copy/rewrite guard).
        mcap = self.bag
        real = os.path.realpath(mcap)
        past = time.time() - 5000
        stamp = os.stat(real)
        os.utime(real, (past, past))
        try:
            key_c, _identity_c = bag_identity(self.bag)
            self.assertNotEqual(key_a, key_c)
        finally:
            os.utime(real, ns=(stamp.st_atime_ns, stamp.st_mtime_ns))

    def test_write_then_load_round_trip(self):
        scan, color, depth = self._scan()
        # Entries must be ascending AND add up to the scan counts.
        n_color = scan.topics[color]["message_count"]
        n_depth = scan.topics[depth]["message_count"]
        index = _base_index()
        index["color_entries"] = [[i, i] for i in range(n_color)]
        index["color_duplicates"] = []
        index["depth_entries"] = [[i, i] for i in range(n_depth)]
        index["depth_duplicates"] = []
        self.assertIsNotNone(
            write_index(self.bag, index, self.cache))
        loaded = load_index(self.bag, scan, color, depth, self.cache)
        self.assertIsNotNone(loaded)
        self.assertEqual(loaded["color_topic"], color)
        # No tmp leftovers anywhere in the cache: writes are atomic.
        for base, _dirs, files in os.walk(self.cache):
            for name in files:
                self.assertFalse(name.endswith(".tmp"), name)

    def test_missing_sidecar_is_miss(self):
        scan, color, depth = self._scan()
        self.assertIsNone(load_index(
            self.bag, scan, color, depth,
            os.path.join(self._tmp.name, "empty-cache")))

    def test_count_mismatch_against_scan_is_miss(self):
        scan, color, depth = self._scan()
        index = _base_index()  # 2+2 color entries vs 4 real messages
        write_index(self.bag, index, self.cache)
        self.assertIsNone(load_index(self.bag, scan, color, depth,
                                     self.cache))

    def test_truncated_json_is_miss(self):
        scan, color, depth = self._scan()
        index = _base_index()
        n_color = scan.topics[color]["message_count"]
        n_depth = scan.topics[depth]["message_count"]
        index["color_entries"] = [[i, i] for i in range(n_color)]
        index["color_duplicates"] = []
        index["depth_entries"] = [[i, i] for i in range(n_depth)]
        index["depth_duplicates"] = []
        write_index(self.bag, index, self.cache)
        key = bag_identity(self.bag)[0]
        hexid = key.split(":", 1)[1]
        path = os.path.join(default_cache_dir(self.cache),
                            hexid[:2], hexid, "index.json")
        with open(path, "r+", encoding="utf-8") as handle:
            body = handle.read()
            handle.seek(0)
            handle.truncate(len(body) // 2)
        self.assertIsNone(load_index(self.bag, scan, color, depth,
                                     self.cache))

    def test_wrong_schema_version_is_miss(self):
        scan, color, depth = self._scan()
        index = _base_index()
        index["schema_version"] = 99
        key = bag_identity(self.bag)[0]
        hexid = key.split(":", 1)[1]
        sidecar = os.path.join(default_cache_dir(self.cache),
                               hexid[:2], hexid)
        os.makedirs(sidecar, exist_ok=True)
        with open(os.path.join(sidecar, "index.json"), "w",
                  encoding="utf-8") as handle:
            json.dump(index, handle)
        self.assertIsNone(load_index(self.bag, scan, color, depth,
                                     self.cache))

    def test_topic_mismatch_is_miss(self):
        scan, color, depth = self._scan()
        index = _base_index()
        n_color = scan.topics[color]["message_count"]
        n_depth = scan.topics[depth]["message_count"]
        index["color_entries"] = [[i, i] for i in range(n_color)]
        index["color_duplicates"] = []
        index["depth_entries"] = [[i, i] for i in range(n_depth)]
        index["depth_duplicates"] = []
        index["color_topic"] = "/some/other/topic"
        write_index(self.bag, index, self.cache)
        self.assertIsNone(load_index(self.bag, scan, color, depth,
                                     self.cache))

    def test_producer_mismatch_is_miss(self):
        # The two replay tools cache different fact sets for one bag
        # (site_pick records no lidar stamps / K variants); a sidecar
        # written by the other tool must miss, never serve short fields.
        scan, color, depth = self._scan()
        index = _base_index()
        n_color = scan.topics[color]["message_count"]
        n_depth = scan.topics[depth]["message_count"]
        index["color_entries"] = [[i, i] for i in range(n_color)]
        index["color_duplicates"] = []
        index["depth_entries"] = [[i, i] for i in range(n_depth)]
        index["depth_duplicates"] = []
        write_index(self.bag, index, self.cache,
                    producer="site_pick_replay")
        self.assertIsNone(load_index(
            self.bag, scan, color, depth, self.cache,
            producer="replay_evaluate"))
        self.assertIsNotNone(load_index(
            self.bag, scan, color, depth, self.cache,
            producer="site_pick_replay"))
        # Any other consumer name (including the pre-gate default) misses.
        self.assertIsNone(load_index(
            self.bag, scan, color, depth, self.cache, producer=""))

    def test_default_cache_dir_respects_override_and_xdg(self):
        explicit = "/tmp/somewhere"
        self.assertEqual(default_cache_dir(explicit), explicit)
        os.environ["XDG_CACHE_HOME"] = os.path.join(
            self._tmp.name, "xdg")
        try:
            self.assertEqual(
                default_cache_dir(""),
                os.path.join(self._tmp.name, "xdg", "luggage_perception",
                             "replay_index"))
        finally:
            del os.environ["XDG_CACHE_HOME"]

    def test_camera_info_payload_round_trip(self):
        from luggage_perception.sensor_types import CameraInfoFrame

        frame = CameraInfoFrame(
            stamp=1.5, frame_id="optical", width=16, height=12,
            fx=200.0, fy=201.0, cx=8.0, cy=6.0,
            distortion_model="plumb_bob",
            distortion_coeffs=(0.1, 0.2, 0.0, 0.0, 0.0),
            rectification=(1.0,) * 9,
            projection=tuple([200.0, 0.0, 8.0, 0.0, 0.0, 201.0, 6.0, 0.0,
                              0.0, 0.0, 1.0, 0.0]),
            binning_x=0, binning_y=0)
        rebuilt = camera_info_frame_from_payload(
            camera_info_payload(frame))
        self.assertEqual(rebuilt.fx, 200.0)
        self.assertEqual(rebuilt.distortion_coeffs, (0.1, 0.2, 0.0, 0.0,
                                                     0.0))
        self.assertEqual(rebuilt.projection, frame.projection)
        self.assertEqual(rebuilt.frame_id, "optical")

    def test_count_jsonl_rows(self):
        path = os.path.join(self._tmp.name, "rows.jsonl")
        with open(path, "w", encoding="utf-8") as handle:
            handle.write('{"a": 1}\n{"a": 2}\n\n')
        self.assertEqual(count_jsonl_rows(path), 2)
        self.assertIsNone(count_jsonl_rows(path + ".missing"))


if __name__ == "__main__":
    unittest.main()
