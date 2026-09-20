#!/usr/bin/env python3
"""YOLO_NOT_READY dumps freeze the gated stats stamp (no Gazebo)."""

from __future__ import division

import json
import os
import shutil
import tempfile
import unittest
from types import SimpleNamespace

from luggage_gazebo.eval_dump_identity import (
    dump_seg_stats,
    dump_tf_stamp_from_color,
    freeze_dump_bundle,
)


def _key(sec, nsec=0):
    return (int(sec), int(nsec))


class _Stamp(object):
    def __init__(self, sec, nsec=0):
        self.sec = int(sec)
        self.nanosec = int(nsec)


class _Header(object):
    def __init__(self, sec, frame_id="optical"):
        self.stamp = _Stamp(sec)
        self.frame_id = frame_id


class _Image(object):
    def __init__(self, sec, frame_id="optical"):
        self.header = _Header(sec, frame_id)


def _write_trial_json(dump_dir, fields, extras):
    dest = os.path.join(
        dump_dir, "trial_%02d_YOLO_NOT_READY" % int(fields.get("index") or 0))
    os.makedirs(dest, exist_ok=True)
    payload = dict(fields)
    payload["frame_meta"] = extras
    with open(os.path.join(dest, "trial.json"), "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")
    return dest


class TestYoloNotReadyDumpIdentity(unittest.TestCase):

    def test_dump_keeps_decision_stamp_not_later_overlay(self):
        decision = {"generation": 0, "stamp": 10.0, "raw_cargo": True}
        later = _Image(12)
        snaps = {
            "color": {_key(12): later},
            "overlay": {_key(12): later},
        }
        freeze = freeze_dump_bundle(decision, snaps)
        self.assertIn("stamp_joined_frames", freeze["missing"])
        self.assertFalse(freeze["capture_complete"])
        self.assertEqual(dump_seg_stats(freeze, {"stamp": 12.0})["stamp"], 10.0)

    def test_matching_color_frame_marks_capture_complete(self):
        decision = {
            "generation": 2, "instance_id": "pickup_box_0001_carryon",
            "stamp": 10.0, "raw_cargo": True,
        }
        color = _Image(10)
        snaps = {"color": {_key(10): color}, "overlay": {_key(10): color}}
        freeze = freeze_dump_bundle(decision, snaps)
        self.assertTrue(freeze["capture_complete"])
        self.assertEqual(freeze["join_key"], _key(10))
        self.assertEqual(freeze["join_dt"], 0.0)
        self.assertIs(freeze["snapshot"]["color"], color)
        self.assertIs(freeze["snapshot"]["overlay"], color)

    def test_freeze_does_not_copy_newer_latch_over_decision(self):
        decision = {"generation": 2, "stamp": 10.0}
        freeze = freeze_dump_bundle(decision, {"color": {_key(10): _Image(10)}})
        live = {"generation": 2, "stamp": 12.5, "raw_cargo": True}
        self.assertEqual(dump_seg_stats(freeze, live)["stamp"], 10.0)
        self.assertIn("overlay", freeze["missing"])
        self.assertFalse(freeze["capture_complete"])

    def test_tf_stamp_does_not_fall_back_to_latest(self):
        self.assertIsNone(dump_tf_stamp_from_color(None))
        color = _Image(10)
        self.assertEqual(dump_tf_stamp_from_color(color), (10, 0))
        self.assertIsNone(dump_tf_stamp_from_color(SimpleNamespace()))

    def test_write_trial_json_keeps_decision_stats(self):
        decision = {"generation": 0, "stamp": 10.0, "raw_cargo": True}
        freeze = freeze_dump_bundle(
            decision, {"color": {_key(12): _Image(12)}})
        extras = {
            "seg_stats": dump_seg_stats(freeze, {"stamp": 12.0}),
            "join_stamp": freeze.get("stamp"),
            "join_dt": freeze.get("join_dt"),
            "capture_complete": freeze["capture_complete"],
            "missing": freeze["missing"],
        }
        fields = {
            "index": 0,
            "fail_code": "YOLO_NOT_READY",
            "extras": {
                "seg_stats": extras["seg_stats"],
                "spawn_generation": 2,
                "decision_record": decision,
            },
        }
        tmp = tempfile.mkdtemp()
        try:
            dest = _write_trial_json(tmp, fields, extras)
            with open(os.path.join(dest, "trial.json"), encoding="utf-8") as handle:
                payload = json.load(handle)
            self.assertEqual(payload["extras"]["seg_stats"]["stamp"], 10.0)
            self.assertNotEqual(payload["extras"]["seg_stats"]["stamp"], 12.0)
            self.assertFalse(payload["frame_meta"]["capture_complete"])
            self.assertEqual(payload["frame_meta"]["join_stamp"], 10.0)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
