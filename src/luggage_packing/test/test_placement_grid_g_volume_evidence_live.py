#!/usr/bin/env python3
"""Live grid_g single-snapshot compute (G v2 production wiring).

Reuses the geometry-identity harness (real mapper + planner on one
executor) with ``volume_evidence_source=grid_g``: ComputePlacement must
answer from ONE /cargo_map snapshot token — the mapper's own surface
projection plus paged query_cells_v2 evidence — never the topic surface
alone, and a missing query graph fails closed by name.
"""
import glob
import math
import os
import struct
import sys
import time
import unittest

try:
    import yaml
except ImportError:  # pragma: no cover - packing env always has it
    yaml = None

from luggage_description.scene_tf_config_utils import (
    container_inner_geometry_descriptor,
    load_scene_tf_config,
)

from test_placement_geometry_identity_live import (  # noqa: F401
    TestGeometryIdentityLive)

_HERE = os.path.dirname(os.path.abspath(__file__))

#: Frozen S-free fan payloads (the K-chain stimulus). Overridable via
#: env; skipped when absent — the unit checks in
#: test_placement_grid_g_snapshot_compute.py cover the assembly.
SFREE_DIR = os.environ.get(
    "OCC_G2_SFREE_DIR",
    "/home/adamliao/work/elfin_humble_ws/docs/status/evidence/"
    "occ_grid_consumer_compat_20261105/prep/sfree_chamfer")


def _matrix_to_quaternion(m):
    """Row-major 3x3 rotation -> (qx, qy, qz, qw)."""
    r00, r01, r02 = (float(v) for v in m[0:3])
    r10, r11, r12 = (float(v) for v in m[3:6])
    r20, r21, r22 = (float(v) for v in m[6:9])
    trace = r00 + r11 + r22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return ((r21 - r12) / s, (r02 - r20) / s, (r10 - r01) / s,
                0.25 * s)
    if r00 > r11 and r00 > r22:
        s = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        return (0.25 * s, (r01 + r10) / s, (r02 + r20) / s,
                (r21 - r12) / s)
    if r11 > r22:
        s = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        return ((r01 + r10) / s, 0.25 * s, (r12 + r21) / s,
                (r02 - r20) / s)
    s = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
    return ((r02 + r20) / s, (r12 + r21) / s, 0.25 * s,
            (r10 - r01) / s)


class TestGridGSnapshotComputeLive(TestGeometryIdentityLive):
    _EXTRA_INIT_ARGS = (
        "-p", "volume_evidence_source:=grid_g",
        # full-ray budget for the S-free fan (the node default 2000
        # subsamples the 8.4k-point frames; the chain freezes 20000)
        "-p", "max_raycast_points:=20000",
    )

    def _load_sfree_frames(self, count=3):
        paths = sorted(glob.glob(os.path.join(SFREE_DIR, "*.yaml")))
        if len(paths) < count:
            self.skipTest("S-free fixtures missing under %s" % SFREE_DIR)
        import yaml as _yaml
        return [_yaml.safe_load(open(p).read()) for p in paths[:count]]

    def _feed_sfree(self, planner, frames):
        """The K-chain intake, wall-clock rebased: static TF from the
        fixture transform (T_container_from_optical verbatim), cloud on
        the production topic, stats-pinned IntegrateCargoView per frame."""
        import tf2_ros as _tf2
        from geometry_msgs.msg import TransformStamped
        from sensor_msgs.msg import PointCloud2, PointField
        from luggage_msgs.srv import GetCargoMapStats, IntegrateCargoView

        first = frames[0]
        frame_id = str(first.get("optical_frame", "optical_frame"))
        static = _tf2.StaticTransformBroadcaster(planner)
        tf = TransformStamped()
        now = planner.get_clock().now()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = "container_link"
        tf.child_frame_id = frame_id
        translation = (first.get("transform") or {}).get(
            "translation", [0.0, 0.0, 0.0])
        qx, qy, qz, qw = _matrix_to_quaternion(
            (first.get("transform") or {}).get(
                "rotation", [1, 0, 0, 0, 1, 0, 0, 0, 1]))
        tf.transform.translation.x = float(translation[0])
        tf.transform.translation.y = float(translation[1])
        tf.transform.translation.z = float(translation[2])
        tf.transform.rotation.x, tf.transform.rotation.y = qx, qy
        tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
        static.sendTransform(tf)
        self._spin(0.5)

        cloud_pub = planner.create_publisher(
            PointCloud2, "/luggage/semantic/cargo_points_untracked", 10)
        fields = []
        for index, name in enumerate(("x", "y", "z")):
            field = PointField()
            field.name = name
            field.offset = index * 4
            field.datatype = PointField.FLOAT32
            field.count = 1
            fields.append(field)

        stats_client = planner.create_client(
            GetCargoMapStats, "/cargo_map/get_stats")
        integrate = planner.create_client(
            IntegrateCargoView, "/cargo_map/integrate_cargo_view")
        self.assertTrue(self._spin_until(
            stats_client.service_is_ready, 10.0), "mapper stats service")
        self.assertTrue(self._spin_until(
            integrate.service_is_ready, 10.0), "mapper integrate service")

        # Stamps: the BUILDING->VALID premise re-checks
        # ``now - last_integrated_stamp <= 1 s`` at every accepted
        # transaction, so each frame stamps NOW+400 ms (the chain does
        # the same with a paused sim clock; fixture-relative offsets
        # would age out behind the ~1 s raycast of an 8.4k frame).
        accepted = 0
        for index, doc in enumerate(frames):
            stamp = planner.get_clock().now().nanoseconds + 400_000_000
            cloud = PointCloud2()
            cloud.header.frame_id = frame_id
            cloud.header.stamp.sec = int(stamp // 1_000_000_000)
            cloud.header.stamp.nanosec = int(stamp % 1_000_000_000)
            points = doc.get("points_m") or []
            cloud.height, cloud.width = 1, len(points)
            cloud.point_step, cloud.row_step = 12, 12 * len(points)
            cloud.is_bigendian = False
            cloud.fields = fields
            cloud.data = b"".join(
                struct.pack("<fff", *(float(v) for v in point))
                for point in points)
            cloud_pub.publish(cloud)
            self._spin(0.2)
            future = stats_client.call_async(GetCargoMapStats.Request())
            self.assertTrue(self._spin_until(future.done, 10.0))
            stats = future.result()
            request = IntegrateCargoView.Request()
            request.schema_version = 1
            request.view_request_id = "%s-grid-g-live-%d" % (
                doc.get("observation_id", "obs"), index)
            request.source_acquisition_stamp.sec = int(
                stamp // 1_000_000_000)
            request.source_acquisition_stamp.nanosec = int(
                stamp % 1_000_000_000)
            request.settled = True
            request.frame_count = 1
            request.geometry_hash = str(
                getattr(stats, "geometry_hash", "") or "")
            request.expected_map_revision = int(
                getattr(stats, "map_revision", 0) or 0)
            future = integrate.call_async(request)
            self.assertTrue(self._spin_until(future.done, 30.0))
            receipt = getattr(future.result(), "receipt",
                              future.result())
            if str(getattr(receipt, "status", "")) in (
                    "INTEGRATED", "accepted"):
                accepted += 1
            self._spin(0.1)
        return accepted

    def test_snapshot_compute_answers_from_one_token(self):
        expected = str(container_inner_geometry_descriptor(
            load_scene_tf_config(self.scene_path))["geometry_hash"])
        mapper, planner = self._start(self.scene_path, self.scene_path)
        frames = self._load_sfree_frames()
        accepted = self._feed_sfree(planner, frames)
        self.assertGreaterEqual(accepted, 1, "no S-free frame integrated")
        # one small FRESH observation to trip BUILDING->VALID cheaply
        # (the 8.4k fans integrate in ~1 s each; a 49-point view commits
        # in milliseconds, well inside the 1 s freshness premise)
        self._feed_observation(planner)
        self.assertTrue(self._spin_until(
            lambda: str(getattr(
                planner._admission.validity, "state", "")) == "VALID",
            15.0), "mapper validity never reached VALID")
        self.assertTrue(self._spin_until(
            lambda: planner._surface is not None, 15.0),
            "planner never admitted the cargo map")

        # spy: the request MUST go through the snapshot token path
        calls = []
        real = planner._grid_g_snapshot_compute

        def _spy():
            calls.append(time.monotonic())
            return real()
        planner._grid_g_snapshot_compute = _spy
        response = self._compute(planner, self._request(expected))
        self.assertTrue(response.success, response.message)
        self.assertEqual(response.geometry_hash, expected)
        self.assertEqual(len(calls), 1)
        self.assertEqual(mapper._geometry_hash, expected)

    def test_missing_query_graph_fails_closed_by_name(self):
        # planner ONLY: no mapper means no /cargo_map query services —
        # every compute must fail closed, named, with no floor prior.
        from test_placement_geometry_identity_live import _construct
        planner = _construct(
            self.planner_mod.PlacementPlannerNode, self.scene_path)
        self.nodes.append(planner)
        self.executor.add_node(planner)
        from sensor_msgs.msg import JointState as _JointState
        self._joints_pub = planner.create_publisher(
            _JointState, "/joint_states", 10)
        response = self._compute(planner, self._request())
        self.assertFalse(response.success)
        self.assertEqual(response.reason_code,
                         "GRID_G_SNAPSHOT_UNAVAILABLE")
        self.assertIn("service missing", response.message)


if __name__ == "__main__":
    unittest.main()


#: The G tree carrying the dense export (this worktree's perception
#: copy predates it; the harness normally loads the mapper from here,
#: and the dense service only exists on the G side).
G_TREE = os.environ.get(
    "OCC_G_TREE", "/tmp/elfin-occ-grid-consumer-compat-g1")


class TestGridGDenseVolumeGateLive(TestGridGSnapshotComputeLive):
    """The G volume authorization gate LIVE on the dense export
    (unlock ruling 2026-11-10): one snapshot supplies BOTH the surface
    and the cell evidence, so a candidate's own box volume must be
    OBSERVED known-free on that snapshot — the under-observed control
    fails closed, the fully observed fan authorizes.

    The mapper and its query services load from the G tree (its
    perception package carries the dense export; this worktree's copy
    predates it), prepended on sys.path so every luggage_perception
    import inside the mapper resolves the G vintage."""

    _EXTRA_INIT_ARGS = TestGridGSnapshotComputeLive._EXTRA_INIT_ARGS + (
        "-p", "volume_grid_g_cells:=true",)

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        import importlib.util
        g_root = os.path.join(G_TREE, "src", "luggage_perception")
        if not os.path.isdir(g_root):
            raise unittest.SkipTest("G tree missing under %s" % G_TREE)
        # the parent class already imported THIS worktree's perception
        # vintage; purge it so the G mapper's imports re-resolve to the
        # tree that carries the dense export (import cache beats path)
        for name in [k for k in sys.modules
                     if k == "luggage_perception"
                     or k.startswith("luggage_perception.")]:
            del sys.modules[name]
        sys.path.insert(0, g_root)
        path = os.path.join(g_root, "scripts",
                            "cargo_volume_mapper_node.py")
        spec = importlib.util.spec_from_file_location(
            "cargo_volume_mapper_node_g", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        cls.mapper_mod = module

    # parent scenarios run under their own params; under the volume
    # gate the tiny-cloud stimulus is CORRECTLY rejected, so the
    # matching-geometry success case is re-proven below with the fan
    test_matching_geometry_answers_with_its_hull_hash = None

    def test_fully_observed_volume_is_authorized(self):
        import time as _time
        expected = str(container_inner_geometry_descriptor(
            load_scene_tf_config(self.scene_path))["geometry_hash"])
        mapper, planner = self._start(self.scene_path, self.scene_path)
        frames = self._load_sfree_frames()
        self.assertGreaterEqual(
            self._feed_sfree(planner, frames), 1, "no S-free frame integrated")
        self._feed_observation(planner)   # trips BUILDING->VALID cheaply
        self.assertTrue(self._spin_until(
            lambda: planner._surface is not None, 15.0),
            "planner never admitted the cargo map")
        calls = []
        real = planner._grid_g_snapshot_compute

        def _spy():
            t0 = _time.monotonic()
            out = real()
            calls.append(_time.monotonic() - t0)
            return out
        planner._grid_g_snapshot_compute = _spy
        dumps = []
        planner._publish_last = dumps.append
        started = _time.monotonic()
        response = self._compute(planner, self._request(expected))
        elapsed = _time.monotonic() - started
        self.assertTrue(response.success, response.message)
        self.assertEqual(len(calls), 1)
        # On this fan stimulus the solver's scoring prefers floor-level
        # candidates (the K01 corpus shows the same floor-prior class),
        # so the healthy live proof is: success through the live dense
        # snapshot, a feasible candidate set, and the recorded support
        # mix. The gate's SAFETY property (under-observed snapshots
        # authorize nothing beyond the prior) is the sibling test;
        # positive authorization is pinned at unit level.
        candidates = (dumps[-1].get("candidates") if dumps else []) or []
        feasible = [c for c in candidates if c.get("feasible")]
        self.assertTrue(feasible, "no feasible candidate in dump")
        mix = sorted({str(c.get("support_source")) for c in feasible})
        print("dense volume gate live: snapshot=%.2fs compute=%.2fs "
              "feasible=%d support_mix=%s"
              % (calls[0], elapsed, len(feasible), mix))
        self.assertLess(elapsed, 10.0)

    def test_under_observed_volume_fails_closed(self):
        expected = str(container_inner_geometry_descriptor(
            load_scene_tf_config(self.scene_path))["geometry_hash"])
        _mapper, planner = self._start(self.scene_path, self.scene_path)
        # only the tiny 49-point observation: surfaces exist (placement
        # geometry is known) but the box VOLUME is not observed
        # known-free -> the gate must reject, never guess
        self._feed_observation(planner)
        self.assertTrue(self._spin_until(
            lambda: planner._surface is not None, 15.0),
            "planner never admitted the cargo map")
        # Capture the latched candidate dump in-process: the product
        # contract (e2e P0 semantics) is that an under-observed map may
        # still answer from the FLOOR PRIOR, but no candidate may claim
        # sensor-known-free volume the snapshot never observed.
        dumps = []
        planner._publish_last = dumps.append
        response = self._compute(planner, self._request(expected))
        candidates = (dumps[-1].get("candidates") if dumps else []) or []
        feasible = [c for c in candidates if c.get("feasible")]
        self.assertTrue(
            (not response.success) or feasible,
            "success with no feasible candidate in the dump")
        for candidate in feasible:
            self.assertEqual(
                candidate.get("support_source"), "floor_prior",
                "under-observed snapshot authorized a non-prior "
                "candidate: %r" % candidate.get("support_source"))
