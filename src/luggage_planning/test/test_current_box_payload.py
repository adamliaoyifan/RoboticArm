#!/usr/bin/env python3
"""Parse /luggage/current_box JSON the spawner actually publishes (schema 2).

Identity + measured only; GT spawn fields are behind the GetCurrentBox
pull service (docs/architecture/privilege_boundary.md).
"""

import json
import unittest

from luggage_planning.current_box_payload import (
    identity_from_current_box_json,
    identity_from_current_box_payload,
    measured_from_current_box_json,
    measured_from_current_box_payload,
)

# Captured shape of pickup_box_spawner_node._publish_box_state (schema 2)
# after sync_detected_pickup_box backfilled a perception measurement.
SYNCED_PAYLOAD = {
    "schema": 2,
    "id": "suitcase_standard_vintage_0003",
    "generation": 7,
    "measured": {
        "width": 0.68,
        "depth": 0.44,
        "height": 0.26,
        "height_source": 1,
        "yaw_valid": True,
        "stamp_sec": 1789980000.5,
    },
}

UNSYNCED_PAYLOAD = {"schema": 2, "id": "suitcase_a_0004", "generation": 8}
CLEARED_PAYLOAD = {"schema": 2, "id": "", "generation": 9}


class TestIdentity(unittest.TestCase):

    def test_synced_record(self):
        self.assertEqual(
            identity_from_current_box_payload(SYNCED_PAYLOAD),
            ("suitcase_standard_vintage_0003", 7))

    def test_cleared_and_malformed(self):
        self.assertEqual(
            identity_from_current_box_payload(CLEARED_PAYLOAD), ("", 9))
        self.assertEqual(identity_from_current_box_payload({}), ("", 0))
        self.assertEqual(identity_from_current_box_payload(None), ("", 0))
        self.assertEqual(identity_from_current_box_payload("junk"), ("", 0))

    def test_json_string(self):
        box_id, generation = identity_from_current_box_json(
            json.dumps(SYNCED_PAYLOAD))
        self.assertEqual(box_id, "suitcase_standard_vintage_0003")
        self.assertEqual(generation, 7)


class TestMeasured(unittest.TestCase):

    def test_synced_record(self):
        measured = measured_from_current_box_payload(SYNCED_PAYLOAD)
        self.assertEqual(
            [measured["width"], measured["depth"], measured["height"]],
            [0.68, 0.44, 0.26])
        self.assertEqual(measured["height_source"], 1)
        self.assertTrue(measured["yaw_valid"])
        self.assertEqual(measured["generation"], 7)

    def test_unsynced_and_cleared_are_none(self):
        self.assertIsNone(measured_from_current_box_payload(UNSYNCED_PAYLOAD))
        self.assertIsNone(measured_from_current_box_payload(CLEARED_PAYLOAD))
        self.assertIsNone(measured_from_current_box_payload(None))

    def test_malformed_measured_block_is_none(self):
        self.assertIsNone(measured_from_current_box_payload({
            "id": "x", "generation": 1, "measured": {"width": 0.5}}))
        self.assertIsNone(measured_from_current_box_payload({
            "id": "x", "generation": 1, "measured": "junk"}))

    def test_json_string(self):
        measured = measured_from_current_box_json(
            json.dumps(SYNCED_PAYLOAD))
        self.assertAlmostEqual(measured["height"], 0.26)
        self.assertIsNone(measured_from_current_box_json("not-json"))


if __name__ == "__main__":
    unittest.main()
