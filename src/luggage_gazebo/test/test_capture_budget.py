"""The continuous bag stays under 1 GiB for a 20 minute session."""

import os
import sys
import unittest

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO, "src", "luggage_gazebo"))

from luggage_gazebo.capture_budget import (  # noqa: E402
    FAT_TOPICS, LIGHT_TOPICS, RUN_BUDGET_BYTES, bag_topics, worst_case_bytes,
)


class CaptureBudget(unittest.TestCase):
    def test_worst_session_is_under_one_gibibyte(self):
        total = worst_case_bytes()
        self.assertLess(total, RUN_BUDGET_BYTES)

    def test_bag_does_not_record_full_rate_fat_topics(self):
        recorded = set(bag_topics())
        for source, output, _type_name in FAT_TOPICS:
            self.assertNotIn(source, recorded)
            self.assertIn(output, recorded)
        for topic in LIGHT_TOPICS:
            self.assertIn(topic, recorded)


if __name__ == "__main__":
    unittest.main()
