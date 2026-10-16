"""Behavioural contracts; native collision accuracy is a separate integration gate."""
from dataclasses import replace
import math
import threading
import time

import pytest

from luggage_planning.placement_prior_contracts import (
    Transform, Snapshot, GraspCandidate, PlacementCandidate, PriorRequest,
)
from luggage_planning.placement_prior import CollisionVerdict, compute_prior
from luggage_planning.placement_prior_worker import PriorWorker

I = Transform((0, 0, 0), (0, 0, 0, 1))


def request():
    snapshot = Snapshot('calibration', 2, 'hull', 'robot', I, b'occ', b'world')
    grasps = (
        GraspCandidate('centre', Transform((0, 0, .1), (0, 0, 0, 1)), 'obs', 'cup', 0),
        GraspCandidate('offset', Transform((-.04, 0, .1), (0, 0, 0, 1)), 'obs', 'cup', 1),
    )
    return PriorRequest('req', snapshot, (.4, .3, .2),
                        (PlacementCandidate('slot', I),), grasps, True)


class SensorWall:
    """Independent analytic fixture: sensor is 8 cm right of TCP, wall at 7 cm.

    The TCP itself is clear at both grasps; only the off-centre grasp fits
    the rigid sensor. No claim that this fixture represents real robot geometry.
    """
    def __init__(self):
        self.calls = []

    def check(self, snapshot, tcp, box, size, **kwargs):
        self.calls.append((snapshot.identity, tcp, box, size))
        sensor_x = tcp.translation[0] + .08
        return CollisionVerdict(sensor_x < .07, 'CLEAR' if sensor_x < .07 else 'COLLISION',
                                () if sensor_x < .07 else (('sensor', 'wall'),))


def test_rigid_sensor_rejection_offset_preserves_box_target_and_rotates_pick():
    req = request()
    port = SensorWall()
    original = req.snapshot.identity
    result = compute_prior(req, port)
    assert result.status == 'READY'
    assert [c.grasp.candidate_id for c in result.choices] == ['offset']
    choice = result.adopt(req)[0]
    # pickup box rotated 90 degrees. Local -x offset must become world -y.
    pick_box = Transform((1, 2, .5), (0, 0, math.sqrt(.5), math.sqrt(.5)))
    pick_tcp = choice.pickup_tcp(pick_box)
    assert pick_tcp.translation == pytest.approx((1, 1.96, .6))
    assert choice.world_from_place_tcp.translation == pytest.approx((-.04, 0, .1))
    recovered_box = choice.world_from_place_tcp.compose(choice.grasp.box_from_tcp.inverse())
    assert recovered_box.translation == pytest.approx((0, 0, 0))
    assert recovered_box.quaternion_xyzw == pytest.approx((0, 0, 0, 1))
    assert req.snapshot.identity == original
    assert result.trace[0].verdict.contacts == (('sensor', 'wall'),)


def test_screen_traverse_endpoint_even_when_final_target_is_clear():
    req = request()
    bad_entry = Transform((1, 0, 0), (0, 0, 0, 1))
    req = replace(req, placements=(replace(req.placements[0], critical_box_poses=(bad_entry,)),))
    port = SensorWall()
    result = compute_prior(req, port)
    assert result.status == 'NO_VALID_GRASP'
    assert len(port.calls) == 2  # rejected at entry, not admitted from final pose alone
    assert all(t.stage_index == 0 for t in result.trace)


@pytest.mark.parametrize('change', ['map', 'scene_bytes', 'size', 'grasp', 'model', 'observation', 'axes'])
def test_stale_or_different_geometry_never_adopted(change):
    req = request()
    result = compute_prior(req, SensorWall())
    if change == 'map':
        changed = replace(req, snapshot=replace(req.snapshot, map_revision=3))
    elif change == 'scene_bytes':
        changed = replace(req, snapshot=replace(req.snapshot, scene_cdr=b'changed'))
    elif change == 'size':
        changed = replace(req, box_wdh=(.5, .3, .2))
    elif change == 'grasp':
        changed = replace(req, grasps=(replace(req.grasps[0], box_from_tcp=I),))
    elif change == 'model':
        changed = replace(req, snapshot=replace(req.snapshot, model_hash='other'))
    elif change == 'observation':
        changed = replace(req, grasps=tuple(replace(g, observation_id='new') for g in req.grasps))
    else:
        flipped = Transform((0, 0, 0), (0, 0, 1, 0))
        changed = replace(req, placements=(replace(req.placements[0], container_from_box=flipped),))
    assert result.adopt(changed) == ()


def test_templates_are_not_measured_adoption_authority():
    req = replace(request(), measured=False)
    result = compute_prior(req, SensorWall())
    assert result.choices
    assert result.adopt(req) == ()
    assert result.adopt(replace(req, measured=True)) == ()


def test_snapshot_copies_input_bytes_and_rejects_mixed_candidates():
    req = request()
    source = bytearray(b'occ')
    snap = replace(req.snapshot, surface_cdr=source)
    source[0] = 0
    assert snap.surface_cdr == b'occ'
    with pytest.raises(ValueError, match='mixed'):
        replace(req, grasps=(req.grasps[0], replace(req.grasps[1], observation_id='other')))
    with pytest.raises(ValueError, match='unit'):
        Transform((0, 0, 0), (0, 0, 0, 2))


def test_budget_and_cancellation_discard_late_port_results():
    now = [0.0]
    class LatePort(SensorWall):
        def check(self, *args, **kwargs):
            now[0] = 11.0
            return CollisionVerdict(True, 'CLEAR')
    result = compute_prior(request(), LatePort(), clock=lambda: now[0])
    assert result.status == 'DEADLINE'
    assert not result.choices
    port = SensorWall()
    cancelled = compute_prior(request(), port, cancelled=lambda: True)
    assert cancelled.status == 'CANCELLED'
    assert not port.calls


def test_candidate_limit_is_reported_not_silently_complete():
    req = replace(request(), candidate_limit=1)
    result = compute_prior(req, SensorWall())
    assert result.status == 'CANDIDATE_LIMIT'
    assert result.pairs_checked == 1
    assert not result.choices


def await_result(worker):
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        generation, result, error = worker.poll()
        if result is not None or error is not None:
            return generation, result, error
        threading.Event().wait(.005)
    pytest.fail('worker did not complete')


def test_newest_pending_job_replaces_older_and_never_publishes_stale_result():
    entered, release = threading.Event(), threading.Event()
    seen = []
    def compute(req, port, cancelled):
        seen.append(req.request_id)
        if req.request_id == 'first':
            entered.set()
            assert release.wait(2)
        return compute_prior(req, port, cancelled=cancelled)
    worker = PriorWorker(SensorWall(), compute)
    try:
        worker.submit(replace(request(), request_id='first'))
        assert entered.wait(2)
        worker.submit(replace(request(), request_id='discard'))
        latest = worker.submit(replace(request(), request_id='last'))
        release.set()
        generation, result, error = await_result(worker)
        assert generation == latest
        assert result.request_id == 'last'
        assert error is None
        assert seen == ['first', 'last']
    finally:
        release.set()
        assert worker.close()


def test_exception_and_invalidation_leave_no_adoptable_result_and_worker_recovers():
    class FailsOnce(SensorWall):
        def check(self, *args, **kwargs):
            if not self.calls:
                self.calls.append('failure')
                raise RuntimeError('missing mesh')
            return super().check(*args, **kwargs)
    worker = PriorWorker(FailsOnce())
    try:
        worker.submit(request())
        _, result, error = await_result(worker)
        assert result is None and error == ('RuntimeError', 'missing mesh')
        worker.submit(request())
        _, result, error = await_result(worker)
        assert result.choices and error is None
        worker.invalidate()
        assert worker.poll()[1:] == (None, None)
    finally:
        assert worker.close()
    with pytest.raises(RuntimeError, match='closed'):
        worker.submit(request())
