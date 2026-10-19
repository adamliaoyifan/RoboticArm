#!/usr/bin/env python3
"""r7 offline gates: release pose stream (5 input classes), frozen-pose
release order, and five-body contact attribution.

The release boundary reads ONLY the continuous pose/info cache with
frozen freshness thresholds (sim age <=100 ms, wall age <=500 ms,
future/old-epoch rejected, wait <=2 s wall). A refusal is a named stop
BEFORE the static delete: the entity and the pending obstacle survive,
no fallback read runs, no released box is deleted.
"""
import os
import sys
import time
import types
import unittest

_SCRIPTS = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

try:
    import rclpy  # noqa: F401
    import luggage_msgs.srv  # noqa: F401
    import place_smoke_driver
except ImportError:
    place_smoke_driver = None


def _msg(sec, model="pickup_box_0001_inch20", x=1.0, recv_wall=None):
    return {
        "stamp_ns": int(sec * 1e9),
        "pose7": [x, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0],
        "recv_wall": recv_wall,
        "model": model,
    }


def _stream_with(entries):
    """A started-shaped stream whose cache is preloaded per entry."""
    stream = place_smoke_driver.ReleasePoseStream(ign_bin="/bin/true")
    # The floor mirrors the live cache: set by the FIRST legitimate
    # message the stream ever observed; later entries are just that.
    stream._epoch_floor_ns = (
        entries[0]["stamp_ns"]
        - int(place_smoke_driver.ReleasePoseStream.EPOCH_TOL_S * 1e9)
        if entries else None)
    stream._latest = {
        e["model"]: (e["stamp_ns"], e["pose7"], e["recv_wall"])
        for e in entries
    }
    return stream


class _WallClock:
    def __init__(self, t0):
        self.t = float(t0)

    def __call__(self):
        return self.t


@unittest.skipIf(place_smoke_driver is None, "driver peers unavailable")
class ReleasePoseStreamTest(unittest.TestCase):

    def setUp(self):
        self.now = 100.0
        self.wall = _WallClock(50.0)
        # Patch the poll sleep so refusals return immediately offline.
        self._orig_sleep = time.sleep
        time.sleep = lambda _s: None

    def tearDown(self):
        time.sleep = self._orig_sleep

    def _read(self, stream):
        return stream.read_release_pose(
            "pickup_box_0001_inch20", self.now,
            timeout_s=0.05, wall_now=self.wall)

    def test_fresh_sample_accepted(self):
        stream = _stream_with([_msg(99.96, recv_wall=49.999)])
        pose, record = self._read(stream)
        self.assertIsNotNone(pose)
        self.assertEqual(record["verdict"], "fresh")
        self.assertLessEqual(record["sim_age_ms"], 100.0)
        self.assertLessEqual(record["wall_age_ms"], 500.0)
        # quaternion form converts to the historical rpy6 shape.
        rpy6 = place_smoke_driver._quat7_to_rpy6(pose)
        self.assertAlmostEqual(rpy6[5], 0.0, places=6)

    def test_stale_sim_age_refused(self):
        stream = _stream_with([_msg(99.5, recv_wall=49.999)])
        pose, record = self._read(stream)
        self.assertIsNone(pose)
        self.assertEqual(record["verdict"], "refused")
        self.assertEqual(record["reason"], "stale_sim_age")

    def test_stale_wall_receive_age_refused(self):
        stream = _stream_with([_msg(99.96, recv_wall=49.0)])
        pose, record = self._read(stream)
        self.assertIsNone(pose)
        self.assertEqual(record["reason"], "stale_wall_age")

    def test_future_stamp_refused(self):
        stream = _stream_with([_msg(100.5, recv_wall=49.999)])
        pose, record = self._read(stream)
        self.assertIsNone(pose)
        self.assertEqual(record["reason"], "future_stamp")

    def test_old_epoch_stamp_refused(self):
        # Epoch floor set by an earlier legitimate message at 99.96; a
        # jump-back sample (previous instance) must never be served.
        stream = _stream_with([
            _msg(99.96, recv_wall=49.999),
            _msg(3.0, x=9.9, recv_wall=49.9995),
        ])
        pose, record = self._read(stream)
        self.assertIsNone(pose)
        self.assertEqual(record["reason"], "old_epoch_stamp")

    def test_no_message_times_out_within_budget(self):
        stream = _stream_with([])
        t0 = time.monotonic()
        pose, record = self._read(stream)
        self.assertIsNone(pose)
        self.assertEqual(record["reason"], "no_message")
        self.assertLessEqual(time.monotonic() - t0, 2.5)

    def test_wrong_instance_not_matched(self):
        stream = _stream_with([
            _msg(99.96, model="pickup_box_0002_inch20", recv_wall=49.999)])
        pose, record = self._read(stream)
        self.assertIsNone(pose)
        self.assertEqual(record["reason"], "no_message")

    def test_parser_extracts_model_in_order(self):
        text = """
header {
  stamp {
    sec: 12
    nsec: 500000000
  }
}
pose {
  name: "S20"
  position {
    x: 0.0
    y: 0.0
    z: 0.0
  }
  orientation {
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
  }
}
pose {
  name: "pickup_box_0001_inch20"
  position {
    x: -2.03
    y: 0.46
    z: 0.69
  }
  orientation {
    x: 0.0
    y: 0.0
    z: 0.7071
    w: 0.7071
  }
}
"""
        out = place_smoke_driver.parse_pose_stream_sample(
            text, "pickup_box_0001_inch20")
        self.assertEqual(len(out), 1)
        stamp, pose7 = out[0]
        self.assertEqual(stamp, 12_500_000_000)
        self.assertAlmostEqual(pose7[0], -2.03, places=6)

    def test_ingest_line_populates_cache_and_epoch(self):
        stream = place_smoke_driver.ReleasePoseStream(ign_bin="/bin/true")
        for line in ("header {", "  stamp {", "    sec: 7",
                     "    nsec: 250000000", "  }", "}", "pose {",
                     '  name: "b1"', "  position {", "    x: 1.5",
                     "    y: 0.0", "    z: 0.1", "  }",
                     "  orientation {", "    x: 0", "    y: 0",
                     "    z: 0", "    w: 1", "  }", "}"):
            stream._ingest_line(line)
        entry = stream.latest("b1")
        self.assertIsNotNone(entry)
        self.assertEqual(entry[0], 7_250_000_000)


@unittest.skipIf(place_smoke_driver is None, "driver peers unavailable")
class ReleaseOrderAndRefusalTest(unittest.TestCase):
    """The refusal path never reaches the delete; the frozen pose is
    the pre-delete value; spawn failure keeps the entity."""

    def _driver_stub(self, read_result):
        class _Stub:
            _release_physics = {
                "command_size": [0.328, 0.517, 0.209],
                "visual_id": "suitcase_loafbrr",
                "mass_kg": 5.67,
                "model_name": "pickup_box_0001_inch20"}
            _box_model = "pickup_box_0001_inch20"
            _placed_manifest = {
                "suitcase_loafbrr_inch20": {
                    "primitive_collision_size": [0.35, 0.53, 0.22],
                    "lid_offset": 0.011033}}
            _placed_models_root = "/nonexistent"
            calls = []

            def ros_now_sec(self):
                return 100.0

            def _release_pose_stream(self):
                class _S:
                    def read_release_pose(_self, model, now, **_k):
                        _Stub.calls.append("pose_read")
                        return read_result
                return _S()

            def _placed_primitive_size(self, visual, size):
                return [0.35, 0.53, 0.22]

            def _placed_lid_offset(self, visual, size):
                return 0.011033

            def _gz_delete(self):
                raise AssertionError("delete service must not be touched")

            def _release_capture_event(self, event, **_k):
                pass  # prior-g1 provenance hook (env-gated no-op)

            def call_srv(self, cli, req, timeout=0.0):
                _Stub.calls.append(("call_srv", cli))
                return None

        return _Stub()

    def test_refusal_stops_before_delete(self):
        stub = self._driver_stub((None, {"reason": "stale_sim_age"}))
        ok, msg, pose = place_smoke_driver.PlaceSmokeDriver \
            ._release_as_dynamic(stub)
        self.assertFalse(ok)
        self.assertIn("release pose refused", msg)
        self.assertIn("stale_sim_age", msg)
        self.assertEqual(stub.calls, ["pose_read"])
        # No delete/create service call happened: the entity survives.

    def test_frozen_pose_used_after_delete(self):
        pose7 = [-2.03, 0.46, 0.6926, 0.0, 0.0, 0.7071, 0.7071]
        stub = self._driver_stub((pose7, {"verdict": "fresh"}))
        # The gz create/delete clients get stubbed at call_srv level;
        # SDF build needs the sized manifest import path only.
        stub._gz_create = object()
        stub._gz_delete = object()

        created = {"count": 0}

        def fake_call_srv(cli, req, timeout=0.0):
            created["count"] += 1
            if created["count"] == 1:          # delete
                return types.SimpleNamespace(success=True)
            return types.SimpleNamespace(success=True)  # create

        stub.call_srv = fake_call_srv
        ok, msg, pose = place_smoke_driver.PlaceSmokeDriver \
            ._release_as_dynamic(stub)
        self.assertTrue(ok, msg)
        # The frozen pose (pre-delete read) is what the spawn carried:
        # the returned record keeps the rpy6 conversion of pose7.
        self.assertAlmostEqual(pose[0], -2.03, places=6)
        self.assertAlmostEqual(pose[5], 1.5707, places=3)
        self.assertEqual(created["count"], 2)

    def test_spawn_failure_keeps_diagnostics(self):
        pose7 = [-2.03, 0.46, 0.6926, 0.0, 0.0, 0.7071, 0.7071]
        stub = self._driver_stub((pose7, {"verdict": "fresh"}))
        stub._gz_create = object()
        stub._gz_delete = object()
        stub._first_dynamic_state = lambda m, **k: {
            "available": False, "reason": "offline"}
        n = {"count": 0}

        def fake_call_srv(cli, req, timeout=0.0):
            n["count"] += 1
            if n["count"] == 1:
                return types.SimpleNamespace(success=True)
            return None                    # create times out

        stub.call_srv = fake_call_srv
        ok, msg, _pose = place_smoke_driver.PlaceSmokeDriver \
            ._release_as_dynamic(stub)
        self.assertFalse(ok)
        self.assertEqual(msg, "create failed")
        timing = getattr(stub, "_release_dynamic_timing", {})
        self.assertIn("t_delete_req_sim", timing)


G3_SCRIPTS = os.path.normpath(os.path.join(
    _SCRIPTS, "..", "..", "..", "scripts", "g3"))
if G3_SCRIPTS not in sys.path:
    sys.path.insert(0, G3_SCRIPTS)


class FiveBodyContactIdentityTest(unittest.TestCase):
    """The offline parser attributes each contact to the exact body."""

    def test_parser_names_each_recess_body(self):
        import tempfile
        bodies = ["body_collision", "rim_collision_px", "rim_collision_nx",
                  "rim_collision_py", "rim_collision_ny"]
        seen = set()
        with tempfile.NamedTemporaryFile(
                "w", suffix=".txt", delete=False) as handle:
            for i, body in enumerate(bodies):
                handle.write(
                    "sec: %d\nnsec: %d\ncontact {\n"
                    "  collision1 {\n    id: 1\n"
                    "    name: \"pickup_box_0001_inch20::suitcase_link::%s\"\n  }\n"
                    "  collision2 {\n    id: 2\n"
                    "    name: \"airport_container_real_aprime::container_link::collision_mesh\"\n  }\n"
                    "  position { x: 0.0 y: 0.1 z: 0.2 }\n}\n"
                    % (10 + i, i, body))
                seen.add("pickup_box_0001_inch20::suitcase_link::%s"
                         % body)
            path = handle.name
        sys.path.insert(0, G3_SCRIPTS)
        from analyze_release_window import parse_contacts
        events = parse_contacts(path)
        os.unlink(path)
        names = set()
        for _t, bodies_ in events:
            names.update(bodies_)
        self.assertTrue(seen <= names,
                        "missing %s" % (seen - names))
        self.assertEqual(len(events), 5)


if __name__ == "__main__":
    unittest.main()
