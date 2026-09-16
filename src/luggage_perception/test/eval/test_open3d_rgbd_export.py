#!/usr/bin/env python3
"""Unit tests for the eval-only Open3D RGB-D dump."""
import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "fixtures"))
from make_tiny_replay_bag import build_fixture  # noqa: E402

from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

from luggage_perception.eval.bag_mcap_source import (  # noqa: E402
    COLOR_INFO_TOPIC,
    COLOR_TOPIC,
    DEPTH_INFO_TOPIC,
    DEPTH_TOPIC,
    TF_STATIC_TOPIC,
    TF_TOPIC,
    BagMessage,
    iter_bag_messages,
)
from luggage_perception.eval.open3d_rgbd_export import (  # noqa: E402
    export_open3d_rgbd,
)


def _optical_tf_static():
    msg = TFMessage()
    tf = TransformStamped()
    tf.header.frame_id = "world"
    tf.child_frame_id = "d555_color_optical_frame"
    tf.transform.translation.x = 0.1
    tf.transform.translation.y = -0.2
    tf.transform.translation.z = 0.8
    tf.transform.rotation.w = 1.0
    msg.transforms = [tf]
    return msg


class TestOpen3dRgbdExport(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._tmp = tempfile.TemporaryDirectory()
        cls.bag = build_fixture(os.path.join(cls._tmp.name, "tiny.mcap"))

    @classmethod
    def tearDownClass(cls):
        cls._tmp.cleanup()

    def test_skips_when_optical_tf_missing(self):
        out = os.path.join(self._tmp.name, "no_tf")
        summary = export_open3d_rgbd(
            self.bag, out, stride=1, max_frames=10)
        self.assertEqual(summary["n_emitted"], 0)
        self.assertGreaterEqual(summary["n_skipped"], 1)
        self.assertEqual(summary["skipped"][0]["reason"], "missing_tf")
        self.assertTrue(os.path.isfile(os.path.join(out, "manifest.json")))
        self.assertTrue(os.path.isfile(os.path.join(out, "intrinsic.json")))

    def test_emits_when_optical_chain_injected(self):
        records = list(iter_bag_messages(self.bag, topics=[
            COLOR_TOPIC, DEPTH_TOPIC, COLOR_INFO_TOPIC, DEPTH_INFO_TOPIC,
            TF_TOPIC, TF_STATIC_TOPIC,
        ]))
        extra = BagMessage(
            topic=TF_STATIC_TOPIC,
            header_stamp_ns=0,
            log_time_ns=0,
            message=_optical_tf_static(),
        )

        def source(topics):
            wanted = set(topics)
            for rec in [extra] + records:
                if rec.topic in wanted:
                    yield rec

        out = os.path.join(self._tmp.name, "with_tf")
        summary = export_open3d_rgbd(
            self.bag, out, stride=1, max_frames=10, source_iter=source)
        self.assertGreaterEqual(summary["n_emitted"], 1)
        idx0 = os.path.join(out, "color", "000000.jpg")
        depth0 = os.path.join(out, "depth", "000000.png")
        self.assertTrue(os.path.isfile(idx0))
        self.assertTrue(os.path.isfile(depth0))
        traj = open(os.path.join(out, "trajectory.log"),
                    encoding="utf-8").read().strip().splitlines()
        self.assertEqual(traj[0], "0 0 1")
        self.assertEqual(len(traj), 5 * summary["n_emitted"])
        # trajectory.log is Open3D extrinsic (world-to-camera).
        tx = float(traj[1].split()[3])
        self.assertAlmostEqual(tx, -0.1, places=6)
        intrinsic = json.load(open(os.path.join(out, "intrinsic.json")))
        self.assertEqual(intrinsic["width"], 16)
        self.assertEqual(intrinsic["height"], 12)


if __name__ == "__main__":
    unittest.main()
