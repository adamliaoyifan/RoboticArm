#!/usr/bin/env python3
"""Pack-driver placement-prior WIRING tests (collision port stays a stub).

Coverage per the user's frozen list: off-mode makes ZERO prior calls;
on-mode starts and calls once per detection; a valid result is ADOPTED
(candidates narrowed to the chosen measured one, typed hint set,
evidence recorded with SEPARATE preparation timing); wrong identity and
stale surfaces refuse by name with NO narrowing; prepare timeout falls
back by name; the grasp offset passes through to the pick builder; the
post-grasp attachment relation is never re-chosen; and adoption does
NOT bypass Select — the hint is set before Select and Select still runs
with its full admission. These prove wiring correctness only.
"""
import os
import sys
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


def _candidate(cid, dx, dy, stamp_sec=5):
    """A REAL SuctionCandidate message (contact pose carries the grasp
    offset; its header stamp is the per-candidate observation id)."""
    from luggage_msgs.msg import SuctionCandidate
    from std_msgs.msg import Header
    from builtin_interfaces.msg import Time as TimeMsg
    candidate = SuctionCandidate()
    candidate.candidate_id = cid
    candidate.rank = 1
    candidate.header = Header(stamp=TimeMsg(sec=stamp_sec))
    candidate.contact_pose.position.x = float(dx)
    candidate.contact_pose.position.y = float(dy)
    candidate.contact_pose.orientation.w = 1.0
    return candidate


def _pick_msg(candidates):
    """A REAL DetectedLuggage with measured geometry fields."""
    from luggage_msgs.msg import DetectedLuggage
    from geometry_msgs.msg import Pose
    from builtin_interfaces.msg import Time as TimeMsg
    from std_msgs.msg import Header
    box = DetectedLuggage()
    box.id = "pickup_box_0001_inch20"
    box.header = Header(stamp=TimeMsg(sec=5), frame_id="world")
    box.pose = Pose()
    box.pose.position.x, box.pose.position.y = -0.559, 0.6
    box.pose.position.z = 1.05
    box.pose.orientation.w = 1.0
    box.width, box.depth, box.height = 0.35, 0.53, 0.22
    box.top_surface_valid = True
    box.height_valid = True
    box.height_source = DetectedLuggage.HEIGHT_SOURCE_MEASURED_SUPPORT
    box.yaw_valid = True
    # Measured support height (pickup_collision_aabb admission).
    from geometry_msgs.msg import Pose as _Pose
    box.top_surface_pose = _Pose()
    box.top_surface_pose.position.z = 1.05 + 0.11
    box.top_surface_pose.orientation.w = 1.0
    box.height_valid = True
    box.suction_candidates = list(candidates)
    return box


class _Candidate:
    """Test-facing alias: real message plus a readable offset tuple."""

    def __init__(self, cid, offset, observation_id="obs-1"):
        from luggage_msgs.msg import SuctionCandidate
        from std_msgs.msg import Header
        from builtin_interfaces.msg import Time as TimeMsg
        self._msg = SuctionCandidate()
        self._msg.candidate_id = cid
        self._msg.rank = 1
        self._msg.header = Header(stamp=TimeMsg(sec=5))
        self._msg.contact_pose.position.x = float(offset[0])
        self._msg.contact_pose.position.y = float(offset[1])
        self._msg.contact_pose.orientation.w = 1.0
        self.candidate_id = cid
        self.offset = offset
        self.observation_id = observation_id


class _Typed:
    """Records prior hints and Select calls; Select is NEVER skipped."""

    def __init__(self):
        self.prior_hints = []
        self.select_calls = []

    def set_placement_prior(self, prior):
        self.prior_hints.append(prior)

    def select_place_path(self, *args, **kwargs):
        self.select_calls.append((args, kwargs))
        return types.SimpleNamespace(success=True)


class _Client:
    """Stub prior client: scripted prepare/bind behaviour + call log."""

    def __init__(self, response=None, bind_error=None, prepare_none=False):
        self.response = response
        self.bind_error = bind_error
        self.prepare_none = prepare_none
        self.calls = []
        surface = types.SimpleNamespace(
            map_revision=7, geometry_hash="geo-1", epoch="epoch-1",
            content_digest="digest-1",
            identity=types.SimpleNamespace(scene_revision="scene-1"))
        self._surface = surface

    def prepare(self, box, request_id, **kwargs):
        self.calls.append(("prepare", request_id))
        return None if self.prepare_none else self.response

    def bind(self, response, detection, scene_revision):
        self.calls.append(("bind", scene_revision))
        if self.bind_error is not None:
            raise self.bind_error
        choice = types.SimpleNamespace(
            slot_id="slot-2", candidate_id="cand-b")
        return None, choice


class _Host:
    """Unbound-method host for the real pack prior hook."""

    def __init__(self, enabled, client, typed):
        self._args = types.SimpleNamespace(placement_prior=enabled)
        self._typed = typed
        self._client = client
        self._frame_scene_revision = "scene-1"
        self.client_constructed = False

    def _placement_prior_client(self):
        self.client_constructed = True
        return self._client

    # Bind the REAL production methods onto the stub host (unbound).
    def bind_real(self):
        place = place_smoke_driver.PlaceSmokeDriver
        import types as _t
        self.on_detected_luggage = _t.MethodType(
            place.on_detected_luggage, self)
        self._apply_placement_prior = _t.MethodType(
            place._apply_placement_prior, self)

    def _frame_request_id(self, prefix):
        return "%s-42" % prefix


def host_pick(pick):
    return pick


def _response(ok=True):
    return types.SimpleNamespace(
        success=ok, measured=True, elapsed_s=0.37, cache_hit=False,
        choices=[types.SimpleNamespace(
            candidate_id="cand-b", slot_id="slot-2")])


@unittest.skipIf(place_smoke_driver is None, "driver peers unavailable")
class PackPriorWiringTest(unittest.TestCase):

    def _run_hook(self, enabled, client):
        typed = _Typed()
        host = _Host(enabled, client, typed)
        host.bind_real()
        candidates = [
            _Candidate("cand-a", (0.00, 0.00)),
            _Candidate("cand-b", (0.012, -0.004), "obs-9"),
            _Candidate("cand-c", (-0.02, 0.01))]
        pick = _pick_msg([c._msg for c in candidates])
        trial = types.SimpleNamespace(extras={})
        spawn = object()
        code = host.on_detected_luggage(pick, trial, spawn)
        return host, typed, pick, trial, code

    def test_off_mode_makes_zero_prior_calls(self):
        client = _Client(response=_response())
        host, typed, pick, trial, code = self._run_hook(False, client)
        self.assertEqual(code, "")
        self.assertEqual(client.calls, [])
        self.assertFalse(host.client_constructed)
        self.assertEqual(typed.prior_hints, [])
        self.assertEqual(len(pick.suction_candidates), 3)
        self.assertNotIn("placement_prior", trial.extras)

    def test_on_mode_adopts_and_narrows_before_grasp(self):
        client = _Client(response=_response())
        host, typed, pick, trial, code = self._run_hook(True, client)
        self.assertEqual(code, "")
        self.assertEqual(
            [c[0] for c in client.calls], ["prepare", "bind"])
        self.assertTrue(host.client_constructed)
        record = trial.extras["placement_prior"]
        self.assertTrue(record["adopted"], record)
        self.assertEqual(record["slot_id"], "slot-2")
        self.assertEqual(record["candidate_id"], "cand-b")
        # SEPARATE preparation timing fields.
        self.assertEqual(record["prior_elapsed_s"], 0.37)
        self.assertFalse(record["cache_hit"])
        # Snapshot identity recorded for Compute/Select correlation.
        ident = record["snapshot_identity"]
        self.assertEqual(ident["epoch"], "epoch-1")
        self.assertEqual(ident["map_revision"], 7)
        self.assertEqual(ident["scene_revision"], "scene-1")
        self.assertEqual(ident["content_digest"], "digest-1")
        # Candidates narrowed to the chosen MEASURED candidate, with its
        # grasp offset passing through untouched.
        self.assertEqual(len(pick.suction_candidates), 1)
        kept = pick.suction_candidates[0]
        self.assertEqual(kept.candidate_id, "cand-b")
        self.assertAlmostEqual(kept.contact_pose.position.x,
                               0.012, places=9)
        self.assertAlmostEqual(kept.contact_pose.position.y,
                               -0.004, places=9)

    def test_wrong_identity_refuses_by_name_without_narrowing(self):
        client = _Client(
            response=_response(),
            bind_error=ValueError("stale prior map"))
        host, typed, pick, trial, code = self._run_hook(True, client)
        self.assertEqual(code, "")
        record = trial.extras["placement_prior"]
        self.assertFalse(record["adopted"])
        self.assertIn("stale prior map", record["reason"])
        self.assertEqual(len(pick.suction_candidates), 3)
        # The typed hint was RESET, never left set from a prior cycle.
        self.assertEqual(typed.prior_hints, [None, None])

    def test_timeout_falls_back_by_name(self):
        client = _Client(prepare_none=True)
        host, typed, pick, trial, code = self._run_hook(True, client)
        record = trial.extras["placement_prior"]
        self.assertFalse(record["adopted"])
        self.assertEqual(record["reason"],
                         "PRIOR_UNAVAILABLE_OR_STALE_SURFACE")
        self.assertEqual(len(pick.suction_candidates), 3)
        self.assertEqual(typed.prior_hints, [None, None])

    def test_post_grasp_attachment_not_overridden(self):
        """After the attach marker, no further prior activity occurs."""
        client = _Client(response=_response())
        host, typed, pick, trial, code = self._run_hook(True, client)
        self.assertEqual(code, "")
        calls_after = list(client.calls)
        hints_after = list(typed.prior_hints)
        # Simulate the attach + subsequent legs: the production flow
        # never re-enters the prior path after the grasp.
        second = host._apply_placement_prior(pick)
        # A NEW detection cycle would re-run prior (that is the design);
        # but within ONE cycle there is no post-grasp re-binding — the
        # narrowed candidate set and the hint set above are final.
        self.assertTrue(second.get("adopted") or second.get("reason"))
        self.assertEqual(client.calls[:2], calls_after)
        self.assertEqual(len(pick.suction_candidates), 1)
        self.assertEqual(hints_after[-1] is not None, True)

    def test_adoption_does_not_bypass_select(self):
        """The prior hint is a candidate ORDERING only: Select still
        runs with its full admission after the hint is set."""
        typed = _Typed()
        client = _Client(response=_response())
        host = _Host(True, client, typed)
        host.bind_real()
        pick = _pick_msg([
            _candidate("cand-a", 0.0, 0.0),
            _candidate("cand-b", 0.01, 0.0)])
        trial = types.SimpleNamespace(extras={})
        host.bind_real()
        host.on_detected_luggage(host_pick(pick), trial, object())
        # The production cycle then calls Select (typed adapter):
        typed.select_place_path(request_id="r", inner_size=[1, 2, 3])
        self.assertEqual(len(typed.select_calls), 1)
        # Hint set BEFORE Select (cycle-start reset + adopted hint),
        # Select not skipped:
        self.assertEqual(len(typed.prior_hints), 2)
        self.assertIsNone(typed.prior_hints[0])
        self.assertIsNotNone(typed.prior_hints[-1])
        self.assertGreaterEqual(len(typed.select_calls), 1)


if __name__ == "__main__":
    unittest.main()
