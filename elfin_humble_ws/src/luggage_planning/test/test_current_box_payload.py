#!/usr/bin/env python3
"""Parse /luggage/current_box JSON the spawner actually publishes."""

import unittest

from luggage_planning.current_box_payload import (
    box_from_current_box_payload,
    parse_current_box_json,
)

# Captured shape of pickup_box_spawner_node._box_to_record (plus generation).
SPAWNER_PAYLOAD = {
    "id": "suitcase_standard_vintage_0003",
    "width": 0.70,
    "depth": 0.45,
    "height": 0.28,
    "yaw": 0.0,
    "mass_kg": 12.5,
    "size_mode": "catalog",
    "visual_kind": "mesh",
    "model_name": "suitcase_vintage_standard",
    "generation": 7,
    "pose": {
        "position": {"x": -1.0, "y": 0.0, "z": 1.01},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    },
}


class TestCurrentBoxPayload(unittest.TestCase):

    def test_spawner_record_has_size_list(self):
        box = box_from_current_box_payload(SPAWNER_PAYLOAD)
        self.assertIsNotNone(box)
        self.assertEqual(box["size"], [0.70, 0.45, 0.28])
        self.assertEqual(box["model_name"], "suitcase_vintage_standard")
        self.assertEqual(box["xyz"], [-1.0, 0.0, 1.01])
        self.assertAlmostEqual(box["mass_kg"], 12.5)
        self.assertEqual(box["generation"], 7)

    def test_nested_size_key_is_not_required(self):
        self.assertIsNone(box_from_current_box_payload({
            "model_name": "x", "id": "x", "size": [0.7, 0.45, 0.28],
        }))

    def test_clear_payload_is_none(self):
        self.assertIsNone(box_from_current_box_payload(
            {"id": "", "generation": 8}))
        self.assertIsNone(box_from_current_box_payload({}))
        self.assertIsNone(box_from_current_box_payload(None))

    def test_parse_json_string(self):
        import json
        box = parse_current_box_json(json.dumps(SPAWNER_PAYLOAD, sort_keys=True))
        self.assertEqual(box["size"][0], 0.70)
        self.assertIsNone(parse_current_box_json("not-json"))


if __name__ == "__main__":
    unittest.main()
