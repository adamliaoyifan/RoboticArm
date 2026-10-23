#!/usr/bin/env python3
"""Planning-verification (kinematic release) offline gates + follow grace.

Kinematic mode keeps the carried body as a STATIC obstacle at the
DECLARED release end-state: one set_pose transition, fresh-only
presence re-read, NO delete/create, NO dynamic spawn. Refusals are
named stops before any retreat — the entity never leaves the scene.
Follow grace: a single 50 ms set_pose transport blip must not publish
a payload-loss reason; three consecutive failures must.
"""
import importlib.util
import os
import sys
import types
import unittest

_SCRIPTS = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)
_G3 = os.path.normpath(os.path.join(
    _SCRIPTS, "..", "..", "..", "scripts", "g3"))
if _G3 not in sys.path:
    sys.path.insert(0, _G3)

try:
    import rclpy  # noqa: F401
    import luggage_msgs.srv  # noqa: F401
    import place_smoke_driver
except ImportError:
    place_smoke_driver = None

SLOT_META = {"pose_world": {
    "position": [-2.03, 0.46, 0.6388], "yaw": 1.5708}}


class _KinHost:
    """Stub driver for the unbound kinematic release call."""

    def __init__(self, set_pose_ok=True, presence=None, mismatch=0.0):
        self._set_pose_ok = set_pose_ok
        self._presence = presence          # pose7 list or None
        self._mismatch = mismatch
        self.calls = []

    def ros_now_sec(self):
        return 100.0

    def _set_model_pose_declared(self, model, pose_rpy6):
        self.calls.append(("set_pose", model, list(pose_rpy6)))
        if not self._set_pose_ok:
            return False, "set_pose service unavailable"
        return True, "ok"

    def _release_pose_stream(self):
        host = self

        class _S:
            def read_release_pose(_self, model, now, **_k):
                host.calls.append(("presence_read", model))
                if host._presence is None:
                    return None, {"verdict": "refused",
                                  "reason": "stale_sim_age"}
                pose = list(host._presence)
                if host._mismatch:
                    pose[0] += host._mismatch
                return pose, {"verdict": "fresh"}
        return _S()


@unittest.skipIf(place_smoke_driver is None, "driver peers unavailable")
class KinematicReleaseTest(unittest.TestCase):

    def _run(self, host):
        return place_smoke_driver.PlaceSmokeDriver._release_kinematic(
            host, "pickup_box_0001_inch20", SLOT_META)

    def test_happy_path_declared_state_confirmed(self):
        pose7 = [-2.03, 0.46, 0.6388, 0.0, 0.0, 0.7071, 0.7071]
        host = _KinHost(presence=pose7)
        ok, msg, rec = self._run(host)
        self.assertTrue(ok, msg)
        self.assertEqual(rec["verdict"], "declared_xy_confirmed")
        # One declared transition; presence re-read fresh-only.
        self.assertEqual(
            [c[0] for c in host.calls], ["set_pose", "presence_read"])
        declared = rec["declared_pose_rpy6"]
        self.assertAlmostEqual(declared[2], 0.6388, places=6)
        self.assertAlmostEqual(declared[5], 1.5708, places=6)
        self.assertLessEqual(rec["declared_vs_world_m"], 0.005)

    def test_no_delete_no_dynamic_spawn_anywhere_in_mode(self):
        # The kinematic path only ever calls set_pose + presence; the
        # gz delete/create services are not referenced at all.
        pose7 = [-2.03, 0.46, 0.6388, 0.0, 0.0, 0.7071, 0.7071]
        host = _KinHost(presence=pose7)
        self._run(host)
        for call in host.calls:
            self.assertNotIn(call[0], ("delete", "create", "spawn"))

    def test_set_pose_refusal_is_named_stop(self):
        host = _KinHost(set_pose_ok=False)
        ok, msg, rec = self._run(host)
        self.assertFalse(ok)
        self.assertIn("kinematic set_pose refused", msg)
        self.assertEqual([c[0] for c in host.calls], ["set_pose"])

    def test_presence_refusal_keeps_entity(self):
        host = _KinHost(presence=None)
        ok, msg, rec = self._run(host)
        self.assertFalse(ok)
        self.assertIn("kinematic presence refused", msg)
        self.assertIn("stale_sim_age", msg)
        # No delete happened: the obstacle stays in the scene.

    def test_world_mismatch_above_5mm_refused(self):
        pose7 = [-2.03, 0.46, 0.6388, 0.0, 0.0, 0.7071, 0.7071]
        host = _KinHost(presence=pose7, mismatch=0.02)
        ok, msg, rec = self._run(host)
        self.assertFalse(ok)
        self.assertIn("kinematic xy mismatch", msg)
        self.assertAlmostEqual(rec["declared_vs_world_m"], 0.02, places=6)

    def test_large_positive_and_negative_z_are_observation_only(self):
        for dz in (0.052, 0.15, -0.15):
            pose7 = [-2.027, 0.462, 0.6388 + dz, 0, 0, 0, 1]
            ok, msg, record = self._run(_KinHost(presence=pose7))
            self.assertTrue(ok, msg)
            self.assertEqual(record["position_acceptance_axes"], "xy")
            self.assertAlmostEqual(record["declared_vs_world_z_m"], dz)
            self.assertAlmostEqual(record["declared_vs_world_xy_m"],
                                   (0.003**2 + 0.002**2)**0.5)
            self.assertEqual(record["observed_pose_xyzw7"], pose7)
            self.assertIn("set_pose_request", record)
            self.assertIn("set_pose_response", record)

    def test_diagonal_xy_over_5mm_rejected_even_with_large_z(self):
        # Each component is below5mm, but planar distance exceeds5mm.
        pose7 = [-2.026, 0.464, 0.7388, 0, 0, 0, 1]
        ok, msg, record = self._run(_KinHost(presence=pose7))
        self.assertFalse(ok)
        self.assertIn("kinematic xy mismatch", msg)
        self.assertGreater(record["declared_vs_world_xy_m"], 0.005)
        self.assertAlmostEqual(record["declared_vs_world_z_m"], 0.1)

    def test_release_mode_default_is_kinematic(self):
        text = open(os.path.join(_SCRIPTS, "place_smoke_driver.py"),
                    encoding="utf-8").read()
        self.assertIn('"--release-mode", choices=("kinematic", "dynamic")',
                      text)


def _load_vacuum_node():
    path = os.path.normpath(os.path.join(
        _SCRIPTS, "..", "..", "luggage_planning", "scripts",
        "vacuum_controller_node.py"))
    if not os.path.isfile(path):
        return None
    try:
        spec = importlib.util.spec_from_file_location(
            "vacuum_node_under_test", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    except Exception:  # noqa: BLE001 - heavy ROS imports may be absent
        return None


class FollowGraceTest(unittest.TestCase):
    """r7 lesson: one 50ms set_pose blip latched a payload-loss reason."""

    def setUp(self):
        self.module = _load_vacuum_node()
        if self.module is None:
            self.skipTest("vacuum node peers unavailable")
        self._orig_sleep = sys.modules.get("time").sleep
        sys.modules["time"].sleep = lambda _s: None

    def tearDown(self):
        sys.modules["time"].sleep = self._orig_sleep

    def _host(self, outcomes):
        """ outcomes: list of (ok, message) per follow_step call. """
        seq = list(outcomes)
        state = {"fail_reason": "", "streak": 0}

        class _Backend:
            name = "sim"

            def is_attached(_self):
                return True

            def follow_step(_self, x, y):
                return seq.pop(0) if seq else (True, "")

        class _Host:
            _backend = _Backend()
            _last_fail_reason = ""
            _follow_fail_streak = 0
            _follow_fail_msg = None
            _vacuum_on = True

            def _panel_pose(_self):
                return ([0.0, 0.0, 1.0], [0.0, 0.0, 0.0, 1.0])

            def _publish_state(_self):
                state["fail_reason"] = _self._last_fail_reason
                state["streak"] = _self._follow_fail_streak

            _follow_lock = types.SimpleNamespace(
                acquire=lambda self, blocking=False: True,
                release=lambda self: None)

        import threading
        _Host._follow_lock = threading.Lock()
        return _Host(), state

    def _tick(self, host):
        self.module.VacuumControllerNode._follow_tick(host)

    def test_single_blip_does_not_publish_loss(self):
        host, state = self._host([
            (True, ""), (False, "set_pose timeout"), (True, "")])
        self._tick(host)
        self._tick(host)
        self._tick(host)
        self.assertEqual(state["fail_reason"], "")

    def test_three_consecutive_failures_publish(self):
        host, state = self._host([(False, "set_pose timeout")] * 4)
        for _ in range(3):
            self._tick(host)
        self.assertEqual(state["fail_reason"], "set_pose timeout")

    def test_success_clears_a_previously_published_follow_reason(self):
        host, state = self._host(
            [(False, "set_pose timeout")] * 3 + [(True, "")])
        for _ in range(3):
            self._tick(host)
        self.assertEqual(state["fail_reason"], "set_pose timeout")
        self._tick(host)
        self.assertEqual(state["fail_reason"], "")


if __name__ == "__main__":
    unittest.main()
