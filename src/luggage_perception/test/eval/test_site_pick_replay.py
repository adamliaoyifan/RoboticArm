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
from luggage_perception.eval.bag_replay_index import sidecar_path  # noqa: E402
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

    @staticmethod
    def _golden_parent_of(dynamic, static, max_dt_ns, child, stamp_ns):
        """Pre-change linear-scan _parent_of, verbatim semantics."""
        hist = dynamic.get(child)
        if hist:
            best = None
            for row_stamp, parent, mat, *_rest in sorted(
                    hist, key=lambda row: row[0]):
                dt = int(row_stamp) - int(stamp_ns)
                adt = abs(dt)
                if adt > max_dt_ns:
                    if row_stamp > stamp_ns:
                        break
                    continue
                cand = (adt, int(row_stamp), parent, mat)
                if (best is None or cand[0] < best[0]
                        or (cand[0] == best[0] and cand[1] < best[1])):
                    best = cand
            if best is not None:
                return best[2], best[3]
        return static.get(child)

    def _golden_lookup(self, buf, target, source, stamp_ns):
        def root_of(frame):
            chain, cur = [], str(frame)
            seen = set()
            while cur and cur not in seen:
                seen.add(cur)
                edge = self._golden_parent_of(
                    buf._dynamic, buf._static, buf.max_dt_ns, cur, stamp_ns)
                if edge is None:
                    break
                parent, mat = edge
                chain.append(mat)
                cur = parent
            mat = np.eye(4)
            for edge in chain:
                mat = edge.dot(mat)
            return mat, cur
        from luggage_perception.eval.bag_tf import invert_matrix
        src_mat, src_root = root_of(source)
        tgt_mat, tgt_root = root_of(target)
        if not src_root or not tgt_root or src_root != tgt_root:
            return None
        return invert_matrix(tgt_mat).dot(src_mat)

    def test_array_cache_matches_linear_scan_reference(self):
        rng = np.random.RandomState(11)
        buf = BagTfBuffer(max_dt_ns=30_000_000)
        stamps = sorted(rng.choice(
            range(0, 500_000_000, 10_000_000), size=40, replace=False))
        for i, stamp in enumerate(stamps):
            buf.add_transform(self._tf(
                "world", "ee", (0.001 * i, -0.002 * i, 0.003 * i),
                int(stamp)), static=False)
            buf.add_transform(self._tf(
                "ee", "optical", (0.0, 0.0, 0.01 * (i % 7)),
                int(stamp)), static=False)
        buf.set_static("root", "world", np.eye(4))
        for probe in list(stamps[::3]) + [0, 250_000_000, 499_999_999,
                                          600_000_000]:
            for target, source in (("world", "optical"),
                                   ("optical", "world"),
                                   ("world", "nonexistent"),
                                   ("root", "optical")):
                got = buf.lookup_matrix(target, source, int(probe))
                want = self._golden_lookup(buf, target, source, int(probe))
                if want is None:
                    self.assertIsNone(got, (target, source, probe))
                else:
                    self.assertIsNotNone(got, (target, source, probe))
                    np.testing.assert_array_equal(got, want)

    def test_memo_invalidated_by_add_transform(self):
        buf = BagTfBuffer()
        buf.add_transform(self._tf("world", "base", (1.0, 0, 0), 100),
                          static=False)
        first = buf.lookup_matrix("world", "base", 100)
        np.testing.assert_allclose(first[0, 3], 1.0)
        buf.add_transform(self._tf("world", "base", (2.0, 0, 0),
                                   100 + 1_000_000), static=False)
        second = buf.lookup_matrix("world", "base", 100 + 1_000_000)
        np.testing.assert_allclose(second[0, 3], 2.0)

    def test_npz_round_trip_preserves_lookups(self):
        buf = BagTfBuffer(max_dt_ns=50_000_000)
        for i, stamp in enumerate(range(0, 200_000_000, 20_000_000)):
            buf.add_transform(self._tf("world", "ee", (0.01 * i, 0, 0),
                                       stamp), static=False)
        buf.set_static("anchor", "world", np.eye(4))
        pts = np.array([[0.5, -0.2, 0.3]])
        before = buf.transform_points(pts, "world", "ee", 90_000_000)
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "tf_edges.npz")
            buf.save_edges_npz(path)
            restored = BagTfBuffer(max_dt_ns=50_000_000)
            restored.load_edges_npz(path)
            after = restored.transform_points(pts, "world", "ee",
                                              90_000_000)
            self.assertEqual(restored.frames(), buf.frames())
        np.testing.assert_array_equal(before, after)
        # The restored buffer still accepts appends and re-finalizes.
        restored.add_transform(self._tf("world", "ee", (9.0, 0, 0),
                                        300_000_000), static=False)
        late = restored.lookup_matrix("world", "ee", 300_000_000)
        np.testing.assert_allclose(late[0, 3], 9.0)

    def test_npz_escape_round_trips_slash_and_percent(self):
        # v2 escaping is injective: "/"-bearing, "%"-bearing and
        # literal "%2F"/"%25" frame ids all survive the npz key round
        # trip as distinct names.
        names = ["plain", "slashed/name", "pct%25", "literal%2Fname",
                 "mix%2Fed/name"]
        buf = BagTfBuffer()
        for i, name in enumerate(names):
            buf.add_transform(self._tf("world", name, (0.1 * i, 0, 0)),
                              static=False)
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "tf_edges.npz")
            buf.save_edges_npz(path)
            restored = BagTfBuffer()
            restored.load_edges_npz(path)
            self.assertEqual(restored.frames(), buf.frames())
            for i, name in enumerate(names):
                mat = restored.lookup_matrix("world", name, 0)
                self.assertIsNotNone(mat, name)
                np.testing.assert_allclose(mat[0, 3], 0.1 * i)

    def test_npz_v1_file_loads_with_v1_rules(self):
        # Files written before the escape_version marker escaped only
        # "/" and must keep decoding exactly as they always did.
        child = "slashed/name"
        escaped = "slashed%2Fname"
        payload = {
            "dynamic_children": np.asarray([escaped], dtype=np.str_),
            "dyn_%s__stamps" % escaped: np.asarray([0], dtype=np.int64),
            "dyn_%s__parents" % escaped: np.asarray(
                ["world"], dtype=np.str_),
            "dyn_%s__translations" % escaped: np.zeros((1, 3)),
            "dyn_%s__quats" % escaped: np.asarray([[0, 0, 0, 1.0]]),
            "dyn_%s__mats" % escaped: np.eye(4).reshape(1, 4, 4),
            "static_children": np.asarray([], dtype=np.str_),
        }
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "v1_edges.npz")
            with open(path, "wb") as handle:
                np.savez(handle, **payload)
            restored = BagTfBuffer()
            restored.load_edges_npz(path)
            self.assertIn(child, restored.frames())
            self.assertIsNotNone(restored.lookup_matrix(
                "world", child, 0))


class TestTfInterpolation(unittest.TestCase):
    def _tf(self, parent, child, xyz, quat=(0.0, 0.0, 0.0, 1.0),
            stamp_ns=0):
        from geometry_msgs.msg import TransformStamped
        msg = TransformStamped()
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.header.stamp.sec = int(stamp_ns // 1_000_000_000)
        msg.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
        (msg.transform.translation.x, msg.transform.translation.y,
         msg.transform.translation.z) = (float(v) for v in xyz)
        (msg.transform.rotation.x, msg.transform.rotation.y,
         msg.transform.rotation.z, msg.transform.rotation.w) = (
            float(v) for v in quat)
        return msg

    def test_midpoint_translation_and_quaternion(self):
        import math
        buf = BagTfBuffer(max_dt_ns=100_000_000)
        quarter = math.pi / 4.0
        # -90° about z at t=0, identity at t=40 ms: the 20 ms midpoint
        # must be a -45° rotation with the translation lerped to 0.5.
        buf.add_transform(self._tf(
            "world", "ee", (0.0, 0.0, 0.0),
            quat=(0.0, 0.0, -math.sin(quarter),
                  math.cos(quarter)), stamp_ns=0))
        buf.add_transform(self._tf(
            "world", "ee", (1.0, 0.0, 0.0), quat=(0.0, 0.0, 0.0, 1.0),
            stamp_ns=40_000_000))
        mid = buf.lookup_matrix("world", "ee", 20_000_000,
                                interpolate=True, max_gap_ns=50_000_000)
        np.testing.assert_allclose(mid[0, 3], 0.5, atol=1e-12)
        expected = np.array([
            [math.cos(-quarter), -math.sin(-quarter), 0.0],
            [math.sin(-quarter), math.cos(-quarter), 0.0],
            [0.0, 0.0, 1.0]])
        np.testing.assert_allclose(mid[:3, :3], expected, atol=1e-9)
        self.assertEqual(buf.stats["interpolated_edges"], 1)
        self.assertAlmostEqual(buf.stats["max_edge_gap_ns"],
                               40_000_000)
        self.assertEqual(buf.interpolation_log[-1]["child"], "ee")

    def test_exact_stamp_interpolation_returns_stored_matrix(self):
        buf = BagTfBuffer()
        buf.add_transform(self._tf("world", "ee", (1.0, 2.0, 3.0),
                                   stamp_ns=10_000_000))
        near = buf.lookup_matrix("world", "ee", 10_000_000,
                                 interpolate=True)
        exact = buf.lookup_matrix("world", "ee", 10_000_000)
        np.testing.assert_array_equal(near, exact)
        self.assertEqual(buf.stats["interpolated_edges"], 0)

    def test_wide_gap_falls_back_to_nearest(self):
        # Bracket spans 200 ms > max_gap 50 ms: interpolation is refused
        # and the lookup degrades to the nearest sample (never extrapol
        # ates, never removes a previously-working lookup).
        buf = BagTfBuffer(max_dt_ns=100_000_000)
        buf.add_transform(self._tf("world", "ee", (0.0, 0.0, 0.0),
                                   stamp_ns=0))
        buf.add_transform(self._tf("world", "ee", (2.0, 0.0, 0.0),
                                   stamp_ns=200_000_000))
        interp = buf.lookup_matrix("world", "ee", 100_000_000,
                                   interpolate=True,
                                   max_gap_ns=50_000_000)
        nearest = buf.lookup_matrix("world", "ee", 100_000_000)
        np.testing.assert_array_equal(interp, nearest)
        self.assertIn(interp[0, 3], (0.0, 2.0))
        self.assertEqual(buf.stats["interpolated_edges"], 0)

    def test_default_off_is_bit_identical(self):
        buf = BagTfBuffer()
        buf.add_transform(self._tf("world", "ee", (0.3, 0.0, 0.0),
                                   stamp_ns=10_000_000))
        buf.add_transform(self._tf("world", "ee", (0.5, 0.0, 0.0),
                                   stamp_ns=30_000_000))
        default = buf.lookup_matrix("world", "ee", 20_000_000)
        explicit = buf.lookup_matrix("world", "ee", 20_000_000,
                                     interpolate=False)
        np.testing.assert_array_equal(default, explicit)
        # Nearest-sample semantics: the 20 ms query picks whichever
        # sample is nearer, unchanged by the interpolation feature.
        self.assertIn(default[0, 3], (0.3, 0.5))


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

    def test_moveit_scene_objects_unit(self):
        from luggage_perception.eval.site_pick_replay import (
            _moveit_scene_objects,
        )
        pick = {"xyz": [0.4, -0.2, 0.9], "top_z": 0.9,
                "width": 0.5, "depth": 0.35, "height": 0.7,
                "height_valid": True}
        objects = _moveit_scene_objects("cargo", pick)
        self.assertEqual(len(objects), 1)
        self.assertEqual(objects[0]["id"], "replay_cargo")
        self.assertEqual(objects[0]["dimensions"], [0.5, 0.35, 0.7])
        # The box hangs BELOW the top surface with a 1 cm modelling
        # margin: the attach goal sits exactly at top_z, and a goal on
        # the collision surface is numerically in collision.
        self.assertAlmostEqual(objects[0]["xyz"][0], 0.4)
        self.assertAlmostEqual(objects[0]["xyz"][2], 0.9 - 0.01 - 0.35)
        ground = _moveit_scene_objects("cargo_ground", pick)
        self.assertEqual(len(ground), 2)
        self.assertEqual(ground[1]["id"], "replay_ground")
        self.assertEqual(_moveit_scene_objects("none", pick), [])
        self.assertEqual(_moveit_scene_objects("cargo", None), [])
        invalid = dict(pick, height_valid=False)
        fell_back = _moveit_scene_objects("cargo", invalid)
        self.assertAlmostEqual(fell_back[0]["dimensions"][2], 0.30)

    def test_moveit_scene_plumbing_reaches_planner(self):
        # The stub backend yields no detections on the fixture bag, so
        # the replay has no pick and no cargo object — but the mode is
        # still plumbed through and recorded verbatim.
        called = {}

        def _fake(**kwargs):
            called.update(kwargs)
            return {"message": "fake-plan", "t_sec": [], "q": [], "xyz": []}

        out = os.path.join(self._tmp.name, "moveit_scene")
        replay_site_pick(
            self.bag, out,
            SitePickConfig(backend="stub", require_backend=False,
                           stride=1, max_frames=2, plan_moveit=True,
                           ros_domain_id=42, moveit_scene="cargo"),
            moveit_plan_fn=_fake)
        self.assertIn("collision_objects", called)
        payload = json.load(open(os.path.join(out, "replay.json")))
        self.assertEqual(payload["moveit_scene"]["mode"], "cargo")
        self.assertEqual(payload["moveit_scene"]["objects"],
                         called["collision_objects"])

    def test_tf_interpolate_config_reaches_rows_and_summary(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = os.path.join(tmp, "interp")
            cfg = SitePickConfig(backend="stub", require_backend=False,
                                 device="cpu", stride=1, max_frames=2,
                                 tf_interpolate=True)
            summary = replay_site_pick(self.bag, out, cfg)
            self.assertTrue(summary["tf_interpolate"])
            payload = json.load(open(os.path.join(out, "replay.json")))
            self.assertGreaterEqual(len(payload["frames"]), 1)
            for row in payload["frames"]:
                self.assertIn(row["tf_mode"],
                              ("interpolated", "nearest", "none"))
                self.assertIn("tf_gap_ms", row)
            # Default-off run reports the old semantics explicitly.
            out0 = os.path.join(tmp, "plain")
            summary0 = replay_site_pick(self.bag, out0, SitePickConfig(
                backend="stub", require_backend=False, device="cpu",
                stride=1, max_frames=2))
            self.assertFalse(summary0["tf_interpolate"])
            self.assertEqual(summary0["n_frames_with_interpolated_tf"], 0)
            self.assertIsNone(summary0["max_tf_gap_ms"])

    def test_cached_replay_matches_cold(self):
        # The TF-edge sidecar must be lookup-neutral: a warm site-pick
        # replay (facts + tf_edges.npz from cache, no /tf stream) writes
        # the same replay.json payload as the cold run.
        with tempfile.TemporaryDirectory() as tmp:
            cache = os.path.join(tmp, "cache")
            cfg = dict(backend="stub", require_backend=False,
                       device="cpu", stride=1, max_frames=2,
                       index_cache_dir=cache)
            replay_site_pick(self.bag, os.path.join(tmp, "cold"),
                             SitePickConfig(**cfg))
            warm = replay_site_pick(self.bag, os.path.join(tmp, "warm"),
                                    SitePickConfig(**cfg))
            self.assertGreaterEqual(warm["frames_processed"], 1)
            cold_payload = json.load(open(
                os.path.join(tmp, "cold", "replay.json")))
            warm_payload = json.load(open(
                os.path.join(tmp, "warm", "replay.json")))
            self.assertEqual(cold_payload["frames"], warm_payload["frames"])
            self.assertEqual(cold_payload["tcp"], warm_payload["tcp"])
            self.assertEqual(cold_payload["joints"], warm_payload["joints"])
            self.assertEqual(cold_payload["tf_frames"],
                             warm_payload["tf_frames"])
            self.assertTrue(os.path.isfile(sidecar_path(
                self.bag, "tf_edges.npz", cache)))

    def test_upgrade_path_rewrites_index_pointer(self):
        # tf_edges.npz deleted after a cold run: the warm run re-streams
        # /tf, re-saves the npz, AND index.json must say so — the
        # tf_edges_file pointer has to match what is on disk.
        with tempfile.TemporaryDirectory() as tmp:
            cache = os.path.join(tmp, "cache")
            cfg = dict(backend="stub", require_backend=False,
                       device="cpu", stride=1, max_frames=2,
                       index_cache_dir=cache)
            replay_site_pick(self.bag, os.path.join(tmp, "cold"),
                             SitePickConfig(**cfg))
            tf_npz = sidecar_path(self.bag, "tf_edges.npz", cache)
            self.assertTrue(os.path.isfile(tf_npz))
            os.remove(tf_npz)
            replay_site_pick(self.bag, os.path.join(tmp, "warm"),
                             SitePickConfig(**cfg))
            self.assertTrue(os.path.isfile(tf_npz))
            with open(os.path.join(os.path.dirname(tf_npz), "index.json"),
                      encoding="utf-8") as handle:
                index = json.load(handle)
            self.assertEqual(index["tf_edges_file"], "tf_edges.npz")


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
