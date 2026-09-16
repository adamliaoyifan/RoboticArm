#!/usr/bin/env python3
"""Unit tests for isolated-domain site pick replay (no live ROS graph)."""
import json
import os
import sys
import tempfile
import unittest
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "fixtures"))
from make_tiny_replay_bag import BASE_NS, build_fixture  # noqa: E402

from luggage_perception.eval.bag_mcap_source import (  # noqa: E402
    COLOR_COMPRESSED_TOPIC,
    COLOR_TOPIC,
    DEPTH_COMPRESSED_TOPIC,
    DEPTH_TOPIC,
    decode_compressed_color_rgb,
    decode_compressed_depth_mm,
    select_image_topics,
)
from luggage_perception.eval.bag_tf import BagTfBuffer  # noqa: E402
from luggage_perception.eval.isolated_domain import (  # noqa: E402
    DEFAULT_REPLAY_DOMAIN_ID,
    IsolatedDomainError,
    LIVE_SIM_DOMAIN_ID,
    assert_isolated_domain,
    isolated_replay_env,
)
from luggage_perception.eval.site_pick_replay import (  # noqa: E402
    SitePickConfig,
    pick_from_pipeline_result,
    pick_waypoints,
    replay_site_pick,
    workspace_for_cloud,
)
from luggage_perception.eval.site_pick_viz import write_viz_html  # noqa: E402
from luggage_perception.top_support_estimator import (  # noqa: E402
    BoxGeometryEstimate,
    TopSurfaceEstimate,
)
from luggage_perception.platform_free_pipeline import PipelineResult  # noqa: E402


class TestIsolatedDomain(unittest.TestCase):
    def test_refuses_live_sim_domain(self):
        with self.assertRaises(IsolatedDomainError) as ctx:
            assert_isolated_domain(LIVE_SIM_DOMAIN_ID)
        self.assertIn("7", str(ctx.exception))

    def test_accepts_replay_domain(self):
        self.assertEqual(
            assert_isolated_domain(DEFAULT_REPLAY_DOMAIN_ID), 42)

    def test_env_overwrites_inherited_sim_domain(self):
        env = isolated_replay_env(
            42, environ={"ROS_DOMAIN_ID": "7", "ROS_LOCALHOST_ONLY": "0"})
        self.assertEqual(env["ROS_DOMAIN_ID"], "42")
        self.assertEqual(env["ROS_LOCALHOST_ONLY"], "1")

    def test_string_seven_refused(self):
        with self.assertRaises(IsolatedDomainError):
            assert_isolated_domain("7")


class TestBagTf(unittest.TestCase):
    def _tf(self, parent, child, xyz, stamp_ns=0):
        from geometry_msgs.msg import TransformStamped
        msg = TransformStamped()
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.header.stamp.sec = int(stamp_ns // 1_000_000_000)
        msg.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
        msg.transform.translation.x, msg.transform.translation.y, \
            msg.transform.translation.z = (float(v) for v in xyz)
        msg.transform.rotation.w = 1.0
        return msg

    def test_static_chain_world_from_optical(self):
        buf = BagTfBuffer()
        buf.add_transform(self._tf("world", "base", (1.0, 0.0, 0.0)),
                          static=True)
        buf.add_transform(self._tf("base", "optical", (0.0, 0.0, 0.5)),
                          static=True)
        pts = buf.transform_points(
            np.zeros((1, 3)), "world", "optical", 0)
        self.assertIsNotNone(pts)
        np.testing.assert_allclose(pts[0], (1.0, 0.0, 0.5), atol=1e-9)

    def test_inverse_lookup(self):
        buf = BagTfBuffer()
        buf.add_transform(self._tf("world", "optical", (0.2, -0.1, 0.8)),
                          static=True)
        pts = buf.transform_points(
            np.array([[0.2, -0.1, 0.8]]), "optical", "world", 0)
        np.testing.assert_allclose(pts[0], (0.0, 0.0, 0.0), atol=1e-9)

    def test_stamped_dynamic_within_window(self):
        buf = BagTfBuffer(max_dt_ns=20_000_000)
        buf.add_transform(self._tf("world", "ee", (0.0, 0.0, 0.0), 100),
                          static=False)
        buf.add_transform(self._tf("world", "ee", (1.0, 0.0, 0.0),
                                   100 + 10_000_000),
                          static=False)
        pts = buf.transform_points(
            np.zeros((1, 3)), "world", "ee", 100 + 10_000_000)
        np.testing.assert_allclose(pts[0], (1.0, 0.0, 0.0), atol=1e-9)

    def test_missing_link_fails_closed(self):
        buf = BagTfBuffer()
        buf.add_transform(self._tf("world", "base", (0, 0, 0)), static=True)
        self.assertIsNone(buf.lookup_matrix("world", "camera", 0))


class TestCompressedDecode(unittest.TestCase):
    def test_jpeg_roundtrip(self):
        import cv2
        rgb = np.zeros((8, 12, 3), dtype=np.uint8)
        rgb[2, 3] = (10, 20, 30)
        ok, buf = cv2.imencode(".jpg", rgb[:, :, ::-1],
                               [int(cv2.IMWRITE_JPEG_QUALITY), 95])
        self.assertTrue(ok)
        msg = SimpleNamespace(format="jpeg", data=bytes(buf))
        out = decode_compressed_color_rgb(msg)
        self.assertEqual(out.shape, (8, 12, 3))

    def test_png_depth_roundtrip(self):
        import cv2
        depth = np.zeros((6, 7), dtype=np.uint16)
        depth[1, 2] = 1500
        ok, buf = cv2.imencode(".png", depth)
        self.assertTrue(ok)
        msg = SimpleNamespace(format="png", data=bytes(buf))
        out = decode_compressed_depth_mm(msg)
        self.assertEqual(int(out[1, 2]), 1500)


class TestSelectImageTopics(unittest.TestCase):
    def test_prefers_raw(self):
        scan = SimpleNamespace(topics={
            COLOR_TOPIC: {}, DEPTH_TOPIC: {},
            COLOR_COMPRESSED_TOPIC: {}, DEPTH_COMPRESSED_TOPIC: {},
        })
        self.assertEqual(select_image_topics(scan),
                         (COLOR_TOPIC, DEPTH_TOPIC))

    def test_falls_back_to_compressed(self):
        scan = SimpleNamespace(topics={
            COLOR_COMPRESSED_TOPIC: {}, DEPTH_COMPRESSED_TOPIC: {},
        })
        self.assertEqual(
            select_image_topics(scan),
            (COLOR_COMPRESSED_TOPIC, DEPTH_COMPRESSED_TOPIC))

    def test_missing_pair_raises(self):
        with self.assertRaises(ValueError):
            select_image_topics(SimpleNamespace(topics={COLOR_TOPIC: {}}))


class TestWorkspaceForCloud(unittest.TestCase):
    def test_keeps_scene_when_cargo_inside(self):
        pts = np.array([[-1.0, 0.0, 0.8], [-0.9, 0.1, 0.8]] * 40)
        xy, half, src = workspace_for_cloud(pts, [-1.0, 0.0], [0.5, 0.5])
        self.assertEqual(src, "scene")
        self.assertEqual(xy, [-1.0, 0.0])

    def test_recenters_when_site_cloud_outside_sim_crop(self):
        rng = np.random.RandomState(0)
        pts = np.column_stack([
            rng.normal(-0.07, 0.1, 200),
            rng.normal(1.29, 0.1, 200),
            rng.normal(0.2, 0.02, 200)])
        xy, half, src = workspace_for_cloud(pts, [-1.0, 0.0], [0.5, 0.5])
        self.assertEqual(src, "cloud")
        self.assertAlmostEqual(xy[0], -0.07, delta=0.05)
        self.assertAlmostEqual(xy[1], 1.29, delta=0.05)
        self.assertGreaterEqual(half[0], 0.6)


class TestPickWaypoints(unittest.TestCase):
    def test_attach_on_top_surface(self):
        pick = SimpleNamespace(
            x=0.1, y=-0.2, z=0.9, top_z=1.0, yaw=0.0, yaw_valid=True)
        wps = pick_waypoints(pick)
        names = [w["name"] for w in wps]
        self.assertEqual(
            names, ["pre_grasp", "approach", "attach", "pick_retreat"])
        self.assertAlmostEqual(wps[2]["xyz"][2], 1.0)
        self.assertAlmostEqual(wps[0]["xyz"][2], 1.30)
        self.assertEqual(wps[0]["xyz"][:2], [0.1, -0.2])

    def test_pipeline_result_to_pick(self):
        top = TopSurfaceEstimate(
            center_xy=np.array([0.4, 0.1]), top_z=0.8, yaw=0.2,
            width=0.5, depth=0.3, confidence=0.9)
        result = PipelineResult(
            top_valid=True, top_reason="ok", support=None,
            height_valid=False, height_source=0,
            box=BoxGeometryEstimate(
                top=top, support=None, height_valid=False,
                height_source=0, width=0.5, depth=0.3, height=0.0))
        pick = pick_from_pipeline_result(result)
        self.assertTrue(pick.top_surface_valid)
        self.assertFalse(pick.height_valid)
        self.assertAlmostEqual(pick.top_z, 0.8)


class TestVizHtml(unittest.TestCase):
    def test_embeds_payload(self):
        tmp = tempfile.TemporaryDirectory()
        path = os.path.join(tmp.name, "replay.html")
        write_viz_html({"bag": "x", "frames": [], "domain": {"reserved": 7}},
                       path)
        html = open(path, encoding="utf-8").read()
        self.assertIn("Plotly", html)
        self.assertIn("\"reserved\":7", html)
        tmp.cleanup()


class TestReplayFixture(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._tmp = tempfile.TemporaryDirectory()
        cls.bag = build_fixture(os.path.join(cls._tmp.name, "tiny.mcap"))
        cls.out = os.path.join(cls._tmp.name, "out")
        cfg = SitePickConfig(
            backend="stub", require_backend=False, device="cpu",
            stride=1, max_frames=2, plan_moveit=False)
        cls.summary = replay_site_pick(cls.bag, cls.out, cfg)

    @classmethod
    def tearDownClass(cls):
        cls._tmp.cleanup()

    def test_offline_summary(self):
        self.assertGreaterEqual(self.summary["frames_processed"], 1)
        self.assertIsNone(self.summary["ros_domain_id"])
        self.assertTrue(os.path.isfile(self.summary["html"]))
        payload = json.load(open(os.path.join(self.out, "replay.json")))
        self.assertEqual(payload["domain"]["perception"], "offline-mcap")
        self.assertEqual(payload["domain"]["reserved"], 7)
        self.assertIsNone(payload["domain"]["moveit"])
        self.assertGreaterEqual(len(payload["joints"]["q"]), 1)
        self.assertGreaterEqual(len(payload["tcp"]["xyz"]), 1)

    def test_plan_moveit_refuses_domain_seven(self):
        with self.assertRaises(IsolatedDomainError):
            replay_site_pick(
                self.bag, os.path.join(self._tmp.name, "nope"),
                SitePickConfig(backend="stub", require_backend=False,
                               plan_moveit=True, ros_domain_id=7))

    def test_injected_moveit_stays_off_graph(self):
        called = {}

        def _fake(**kwargs):
            called.update(kwargs)
            return {"message": "fake-plan", "t_sec": [0.0, 1.0],
                    "q": [[0.0] * 6, [0.1] * 6], "xyz": []}

        out = os.path.join(self._tmp.name, "moveit")
        summary = replay_site_pick(
            self.bag, out,
            SitePickConfig(backend="stub", require_backend=False,
                           stride=1, max_frames=2, plan_moveit=True,
                           ros_domain_id=42),
            moveit_plan_fn=_fake)
        self.assertEqual(summary["ros_domain_id"], 42)
        self.assertIn("domain_id", called)
        self.assertEqual(called["domain_id"], 42)
        payload = json.load(open(os.path.join(out, "replay.json")))
        self.assertEqual(payload["planned"]["message"], "fake-plan")


class TestCliDomainGuard(unittest.TestCase):
    def test_main_refuses_seven(self):
        import importlib.util
        path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "..", "..", "scripts", "site_pick_replay_viz.py")
        spec = importlib.util.spec_from_file_location(
            "site_pick_replay_viz_cli", path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        rc = mod.main(["--bag", "/tmp/nope", "--ros-domain-id", "7"])
        self.assertEqual(rc, 2)


if __name__ == "__main__":
    unittest.main()
