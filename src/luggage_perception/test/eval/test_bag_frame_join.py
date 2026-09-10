#!/usr/bin/env python3
"""Unit tests for the pure bag frame join. No ROS, no I/O."""
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "fixtures"))
from make_tiny_replay_bag import BASE_NS, FRAME_DT_NS  # noqa: E402

from luggage_perception.eval.bag_frame_join import (  # noqa: E402
    dedupe_stamped_entries,
    frame_dir_name,
    frame_dir_name_to_ns,
    nearest_stamp,
    plan_frame_join,
)

MS = 1_000_000


class TestNearestStamp(unittest.TestCase):
    def setUp(self):
        self.stamps = [1000, 2000, 3000]

    def test_hit_exact(self):
        self.assertEqual(nearest_stamp(self.stamps, 2000, 100), (1, 0))

    def test_hit_within_tolerance(self):
        self.assertEqual(nearest_stamp(self.stamps, 2040, 100), (1, -40))
        self.assertEqual(nearest_stamp(self.stamps, 1960, 100), (1, 40))

    def test_tie_prefers_earlier(self):
        # 2500 is equidistant to 2000 and 3000 -> earlier wins.
        self.assertEqual(nearest_stamp(self.stamps, 2500, 600), (1, -500))

    def test_out_of_tolerance_none(self):
        self.assertIsNone(nearest_stamp(self.stamps, 2750, 200))
        self.assertIsNone(nearest_stamp(self.stamps, 500, 400))

    def test_empty(self):
        self.assertIsNone(nearest_stamp([], 1, 100))


class TestDedupe(unittest.TestCase):
    def test_keeps_newest_log_time(self):
        entries = [(100, 5, "old"), (100, 9, "new"), (50, 1, "a")]
        ordered, duplicates = dedupe_stamped_entries(entries)
        self.assertEqual([e[0] for e in ordered], [50, 100])
        kept = {stamp: payload for stamp, _log, payload in ordered}
        self.assertEqual(kept[100], "new")
        self.assertEqual(duplicates, {100: 1})

    def test_no_duplicates(self):
        ordered, duplicates = dedupe_stamped_entries(
            [(3, 1, None), (1, 2, None)])
        self.assertEqual(duplicates, {})
        self.assertEqual([e[0] for e in ordered], [1, 3])


class TestPlanFrameJoin(unittest.TestCase):
    def test_fixture_join_plan(self):
        # The tiny fixture: f0/f1 exact, duplicate color at f1, f2 color
        # orphan (nearest depth 5 ms > 1 ms tolerance), f3 depth orphan.
        t0 = BASE_NS
        t1 = BASE_NS + FRAME_DT_NS
        t2 = BASE_NS + 2 * FRAME_DT_NS
        t3 = BASE_NS + 3 * FRAME_DT_NS
        color = [t0, t1, t1, t2]
        depth = [t0, t1, t2 + 5 * MS, t3]
        plan = plan_frame_join(color, depth, tolerance_ns=MS)
        self.assertEqual(plan.stats["n_pairs"], 2)
        self.assertEqual(plan.stats["n_exact"], 2)
        self.assertEqual(plan.stats["n_tolerance"], 0)
        self.assertEqual(plan.color_orphans, [t2])
        self.assertEqual(plan.depth_orphans, [t2 + 5 * MS, t3])

    def test_tolerance_pairing_rescues_near_match(self):
        t0 = BASE_NS
        plan = plan_frame_join(
            [t0], [t0 + 2 * MS], tolerance_ns=5 * MS)
        self.assertEqual(plan.stats["n_tolerance"], 1)
        self.assertEqual(plan.pairs[0].dt_ns, 2 * MS)

    def test_one_to_one_greedy(self):
        # Two colors compete for one depth inside tolerance: the earlier
        # color wins the pair, the later color becomes an orphan.
        base = BASE_NS
        plan = plan_frame_join(
            [base, base + 2 * MS], [base + 1 * MS], tolerance_ns=2 * MS)
        self.assertEqual(plan.stats["n_pairs"], 1)
        self.assertEqual(plan.pairs[0].stamp_ns, base)
        self.assertEqual(plan.color_orphans, [base + 2 * MS])

    def test_bag1_shape_452_vs_475(self):
        # Reproduce the record_site shape: 452 color frames at ~22.6 Hz
        # where 452 of 475 depth stamps coincide exactly and 23 depths
        # have no colour counterpart.
        stamps = [BASE_NS + int(i * 44.25e6) for i in range(452)]
        depth = sorted(
            stamps + [BASE_NS + 20_000_000_000 + i * 87e6
                      for i in range(23)])
        plan = plan_frame_join(stamps, depth, tolerance_ns=MS)
        self.assertEqual(plan.stats["n_pairs"], 452)
        self.assertEqual(plan.stats["n_exact"], 452)
        self.assertEqual(plan.stats["n_depth_orphans"], 23)
        self.assertEqual(plan.stats["n_color_orphans"], 0)

    def test_randomized_exact_and_orphans(self):
        rng = random.Random(7)
        color = sorted(stamp * MS for stamp in
                       rng.sample(range(100_000), 200))
        dropped = set(rng.sample(color, 25))
        extra = [10_500 * MS + i * MS for i in range(5)]
        depth = sorted((set(color) - dropped) | set(extra))
        plan = plan_frame_join(color, depth, tolerance_ns=MS)
        self.assertEqual(plan.stats["n_exact"], 175)
        self.assertEqual(plan.stats["n_tolerance"], 0)
        self.assertEqual(plan.stats["n_color_orphans"], len(dropped))
        self.assertEqual(plan.stats["n_depth_orphans"], len(extra))
        for pair in plan.pairs:
            self.assertEqual(pair.stamp_ns, pair.depth_stamp_ns)


class TestFrameDirName(unittest.TestCase):
    def test_roundtrip(self):
        for stamp in (0, 1, BASE_NS, BASE_NS + 999_999_999):
            self.assertEqual(frame_dir_name_to_ns(frame_dir_name(stamp)),
                             stamp)

    def test_sortable(self):
        a = frame_dir_name(BASE_NS)
        b = frame_dir_name(BASE_NS + 1)
        self.assertLess(a, b)


if __name__ == "__main__":
    unittest.main()
