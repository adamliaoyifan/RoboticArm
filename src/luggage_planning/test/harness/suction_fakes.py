"""Fakes for the suction pick session gates (C2-C6).

Everything is injectable through :func:`make_ports`; every call is
recorded so tests can assert zero unauthorized invocations.
"""

import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", ".."))

from luggage_planning.pose import Point, Pose  # noqa: E402
from luggage_planning.suction_candidate_selection import (  # noqa: E402
    SuctionCandidateView,
)
from luggage_planning.suction_candidate_waypoints import (  # noqa: E402
    quaternion_aligning_z_to,
)
from luggage_planning.suction_pick_session import (  # noqa: E402
    DetectionView,
    SegmentOutcome,
)


class FakeClock:
    def __init__(self, start=1000.0):
        self._now = float(start)

    def now(self):
        return self._now

    def sleep(self, dt):
        self._now += float(dt)

    def advance(self, dt):
        self._now += float(dt)


def make_candidate(candidate_id, rank, xy=(0.65, 0.0), z=0.85,
                   stamp=1234.5, frame="world", instance_id="box-1",
                   generation=7, model_version=1, model_hash="abc123",
                   normal=(0.0, 0.0, 1.0)):
    return SuctionCandidateView(
        candidate_id=candidate_id, rank=rank, stamp=stamp, frame=frame,
        instance_id=instance_id, generation=generation,
        contact=Pose(position=Point(x=xy[0], y=xy[1], z=z),
                     orientation=quaternion_aligning_z_to(normal)),
        model_version=model_version, model_hash=model_hash,
        boundary_clearance=0.02, valid_coverage=0.99,
    )


def make_detection(candidates, stamp=1234.5, frame="world",
                   instance_id="box-1", generation=7, top_surface_valid=True,
                   yaw_valid=False):
    return DetectionView(
        stamp=stamp, frame=frame, candidates=tuple(candidates),
        top_surface_valid=top_surface_valid,
        detection_yaw=0.4, yaw_valid=yaw_valid,
        box_xyz=(0.65, 0.0, 0.6), box_quat=(0.0, 0.0, 0.0, 1.0),
        box_size=(0.5, 0.35, 0.4))


class FakeVacuumService:
    """VacuumCommand double with backend release-on-timeout semantics.

    ``seal_delays`` maps candidate_id -> DI0 rise delay (sec). A delay
    above ``seal_timeout`` (8.0 s) produces the backend timeout path:
    DO0 off + blow-off, then the scripted DI0 behaviour (settle low,
    stuck high, or unknown).
    """

    def __init__(self, clock, seal_delays, seal_timeout=8.0,
                 di0_settle_delay=0.0, stuck_high=False, unknown=False,
                 service_error=False, drop_di0_on_carry=False,
                 di0_flip_flop_sec=0.0):
        self.clock = clock
        self.seal_delays = dict(seal_delays or {})
        self.seal_timeout = float(seal_timeout)
        self.di0_settle_delay = float(di0_settle_delay)
        self.stuck_high = stuck_high
        self.unknown = unknown
        self.service_error = service_error
        self.drop_di0_on_carry = drop_di0_on_carry
        self.di0_flip_flop_sec = float(di0_flip_flop_sec)
        self.do0 = 0
        self.di0_value = 0
        self.enable_calls = 0
        self.release_calls = 0
        self.enable_candidates = []
        self._settle_at = None
        self._flip_last = None

    def _tick_settle(self):
        if (self._settle_at is not None
                and self.clock.now() >= self._settle_at):
            self.di0_value = 0
            self._settle_at = None

    def enable(self, candidate_id):
        self.enable_calls += 1
        self.enable_candidates.append(candidate_id)
        if self.service_error:
            return False, "VACUUM_BACKEND_ERROR: io exception"
        delay = float(self.seal_delays.get(candidate_id, 0.0))
        if delay <= self.seal_timeout:
            self.clock.sleep(max(0.0, delay))
            self.do0 = 1
            self.di0_value = 1
            return True, "sealed in %.2fs" % delay
        # timeout: the backend completes its release sequence internally
        self.clock.sleep(self.seal_timeout)
        self.do0 = 0
        self.release_calls += 1
        if self.stuck_high:
            self.di0_value = 1
        elif self.unknown:
            self.di0_value = None
        elif self.di0_settle_delay > 0.0:
            # pressure decay: DI0 stays high, then settles low
            self.di0_value = 1
            self._settle_at = self.clock.now() + self.di0_settle_delay
        else:
            self.di0_value = 0
        return False, "VACUUM_SEAL_TIMEOUT after %.1fs" % self.seal_timeout

    def release(self):
        self.release_calls += 1
        self.do0 = 0
        self.di0_value = 0
        return True, "released"

    def di0(self):
        self._tick_settle()
        if self.di0_flip_flop_sec > 0.0 and self.do0 == 0:
            # leaky/intermittent release: low never holds long enough
            now = self.clock.now()
            if self._flip_last is None:
                self._flip_last = now
                self.di0_value = 0
            elif now - self._flip_last >= self.di0_flip_flop_sec:
                self._flip_last = now
                self.di0_value = 1 if self.di0_value == 0 else 0
        if (self.drop_di0_on_carry and self.do0 == 1
                and self.di0_value == 1 and self.enable_calls >= 1):
            # pressure loss while carrying (C6 injection)
            self.di0_value = 0
        return self.di0_value


class FakeProbe:
    """ProbeMotionSegment double keyed by (candidate_id, segment_name)."""

    def __init__(self, overrides=None):
        # {(candidate_id, segment_name):
        #     {"ik_ok": bool, "fraction": float, "moveit_error_code": int}}
        self.overrides = dict(overrides or {})
        self.calls = []
        self._active = ""

    def set_active(self, candidate_id):
        self._active = candidate_id

    def probe(self, segment, candidate_id):
        self.calls.append((candidate_id, segment.name))
        default = {"ik_ok": True, "fraction": 1.0, "moveit_error_code": 1}
        return dict(self.overrides.get((candidate_id, segment.name),
                                       default))


class FakeMotion:
    """PlanMotion double keyed by segment_name."""

    def __init__(self, outcomes=None):
        # {segment_name: SegmentOutcome}; unmatched -> ok
        self.outcomes = dict(outcomes or {})
        self.default = SegmentOutcome(ok=True, message="ok", fraction=1.0)
        self.executed = []

    def execute_segment(self, segment, candidate_id):
        self.executed.append((candidate_id, segment.name))
        return self.outcomes.get(segment.name, self.default)


class FakeScenePort:
    def __init__(self, fail_add=False, fail_attach=False):
        self.fail_add = fail_add
        self.fail_attach = fail_attach
        self.calls = []

    def _record(self, name, ok, message=""):
        self.calls.append((name, ok))
        return ok, message or name

    def scene_add_box(self, detection):
        return self._record("add", not self.fail_add)

    def scene_attach(self):
        return self._record("attach", not self.fail_attach)

    def scene_remove(self):
        return self._record("remove", True)

    def set_pickup_touch(self, allowed):
        return self._record("touch", True)


class FakePorts:
    """Assembled session ports; every knob is a public attribute."""

    def __init__(self, clock, candidates, seal_delays=None,
                 probe_overrides=None, segment_outcomes=None,
                 di0_settle_delay=0.0, stuck_high=False, unknown=False,
                 service_error=False, identity=("box-1", 7), graph_ok=True,
                 scene=None, dry_run_detections=None,
                 drop_di0_on_carry=False, di0_flip_flop_sec=0.0):
        self.clock = clock
        self.candidates = list(candidates)
        self.vacuum = FakeVacuumService(
            clock, seal_delays, di0_settle_delay=di0_settle_delay,
            stuck_high=stuck_high, unknown=unknown,
            service_error=service_error,
            drop_di0_on_carry=drop_di0_on_carry,
            di0_flip_flop_sec=di0_flip_flop_sec)
        self.probe_fake = FakeProbe(probe_overrides)
        self.motion = FakeMotion(segment_outcomes)
        self.scene = scene or FakeScenePort()
        self.identity_value = identity
        self.graph_ok_value = graph_ok
        self.log = []
        self.detect_calls = 0
        self.dry_run_detections = dry_run_detections or []

    # -- ports interface ------------------------------------------------------

    def now(self):
        return self.clock.now()

    def sleep(self, dt):
        self.clock.sleep(dt)

    def detect(self):
        self.detect_calls += 1
        if self.dry_run_detections:
            return self.dry_run_detections.pop(0)
        raise RuntimeError("detect not scripted")

    def probe(self, segment, candidate_id):
        return self.probe_fake.probe(segment, candidate_id)

    def execute_segment(self, segment, candidate_id):
        return self.motion.execute_segment(segment, candidate_id)

    def vacuum_enable(self, candidate_id):
        return self.vacuum.enable(candidate_id)

    def vacuum_release(self, candidate_id=""):
        return self.vacuum.release()

    def di0(self):
        return self.vacuum.di0()

    def scene_add_box(self, detection):
        return self.scene.scene_add_box(detection)

    def scene_attach(self):
        return self.scene.scene_attach()

    def scene_remove(self):
        return self.scene.scene_remove()

    def set_pickup_touch(self, allowed):
        return self.scene.set_pickup_touch(allowed)

    def identity_latest(self):
        return self.identity_value

    def graph_health(self):
        return (bool(self.graph_ok_value),
                "ok" if self.graph_ok_value else "unhealthy")

    def log_event(self, record):
        self.log.append(record)


def make_ports(candidates, seal_delays=None, **kwargs):
    return FakePorts(FakeClock(), candidates, seal_delays, **kwargs)
