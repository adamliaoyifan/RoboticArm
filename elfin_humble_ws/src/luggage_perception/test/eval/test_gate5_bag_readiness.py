#!/usr/bin/env python3
"""PF-A2 Gate 5 readiness checker: synthetic fixtures, no real bag."""

from __future__ import division

import os
import unittest

from luggage_perception.eval.gate5_bag_readiness import (
    BAG_DURATION_SHORT,
    BAG_EMPTY_STREAM,
    BAG_GATE5_ACCURACY_NOT_CLAIMED,
    BAG_MISSING_TOPIC,
    BAG_MISSING_FRAME,
    BAG_REFERENCE_LEAK,
    BAG_SEGMENTATION_INPUT_MISSING,
    BAG_STAMP_NON_MONOTONIC,
    BAG_TF_GAP,
    BAG_WINDOWS_NO_OVERLAP,
    BAG_WRONG_TYPE,
    check_manifest,
    mutate_fixture,
    valid_fixture_manifest,
)


class TestGate5ReadyFixture(unittest.TestCase):
    def test_valid_fixture_is_ready_and_does_not_claim_accuracy(self):
        report = check_manifest(valid_fixture_manifest())
        self.assertTrue(report["ready"], report["issues"])
        self.assertEqual(report["gate5_accuracy"], "not_claimed")
        self.assertEqual(report["reasons"], [])

    def test_claim_accuracy_always_fails(self):
        report = check_manifest(
            valid_fixture_manifest(), claim_accuracy=True)
        self.assertFalse(report["ready"])
        self.assertIn(BAG_GATE5_ACCURACY_NOT_CLAIMED, report["reasons"])
        self.assertEqual(report["gate5_accuracy"], "not_claimed")


class TestGate5Defects(unittest.TestCase):
    def _codes(self, kind):
        return check_manifest(mutate_fixture(kind))["reasons"]

    def test_missing_topic(self):
        self.assertIn(BAG_MISSING_TOPIC, self._codes("missing_topic"))

    def test_wrong_type(self):
        self.assertIn(BAG_WRONG_TYPE, self._codes("wrong_type"))

    def test_empty_stream(self):
        self.assertIn(BAG_EMPTY_STREAM, self._codes("empty_stream"))

    def test_non_overlap(self):
        self.assertIn(BAG_WINDOWS_NO_OVERLAP, self._codes("non_overlap"))

    def test_missing_tf(self):
        self.assertIn(BAG_TF_GAP, self._codes("missing_tf"))

    def test_reference_leak(self):
        self.assertIn(BAG_REFERENCE_LEAK, self._codes("reference_leak"))

    def test_non_monotonic(self):
        self.assertIn(BAG_STAMP_NON_MONOTONIC, self._codes("non_monotonic"))

    def test_claim_accuracy_fixture(self):
        self.assertIn(
            BAG_GATE5_ACCURACY_NOT_CLAIMED, self._codes("claim_accuracy"))

    def test_missing_segmentation(self):
        manifest = valid_fixture_manifest()
        manifest["topics"] = [
            t for t in manifest["topics"]
            if t["name"] != "/luggage/semantic/mask"]
        report = check_manifest(manifest)
        self.assertIn(BAG_SEGMENTATION_INPUT_MISSING, report["reasons"])

    def test_missing_frame(self):
        manifest = valid_fixture_manifest()
        manifest["frame_ids"] = ["elfin_base_link"]
        for topic in manifest["topics"]:
            topic["frame_id"] = ""
        report = check_manifest(manifest)
        self.assertIn(BAG_MISSING_FRAME, report["reasons"])

    def test_short_duration(self):
        report = check_manifest(
            valid_fixture_manifest(), min_duration_sec=100.0)
        self.assertIn(BAG_DURATION_SHORT, report["reasons"])

    def test_offline_label_on_online_topic_name(self):
        manifest = valid_fixture_manifest()
        for topic in manifest["topics"]:
            if topic["name"] == "/luggage/preprocessed/status":
                topic["role"] = "reference"
        report = check_manifest(manifest)
        self.assertIn(BAG_REFERENCE_LEAK, report["reasons"])

    def test_current_box_as_online_geometry_is_leak(self):
        manifest = valid_fixture_manifest()
        for topic in manifest["topics"]:
            if topic["name"] == "/luggage/current_box":
                topic["role"] = "online"
        report = check_manifest(manifest)
        self.assertIn(BAG_REFERENCE_LEAK, report["reasons"])

    def test_defects_are_deterministic(self):
        first = check_manifest(mutate_fixture("wrong_type"))
        second = check_manifest(mutate_fixture("wrong_type"))
        self.assertEqual(first, second)


class TestGate5FixtureFiles(unittest.TestCase):
    def test_on_disk_fixtures_match_kinds(self):
        here = os.path.dirname(os.path.abspath(__file__))
        fixture_dir = os.path.join(here, "fixtures", "gate5")
        if not os.path.isdir(fixture_dir):
            self.skipTest("fixture dir not written yet")
        import json
        path = os.path.join(fixture_dir, "valid.json")
        if not os.path.isfile(path):
            self.skipTest("valid.json not present")
        with open(path) as handle:
            manifest = json.load(handle)
        self.assertTrue(check_manifest(manifest)["ready"])


if __name__ == "__main__":
    unittest.main()
