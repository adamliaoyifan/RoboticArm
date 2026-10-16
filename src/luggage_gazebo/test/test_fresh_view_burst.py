"""Independent acquisitions accumulate evidence without relaxing occupancy."""

from luggage_gazebo.fresh_view_burst import observe_fresh_frames
from luggage_perception.voxel_log_odds import LogOddsGrid, OCCUPIED


def test_saturated_free_voxel_requires_three_distinct_hits():
    grid = LogOddsGrid(1)
    for _ in range(10):
        grid.apply_miss(0)
    revisions = []

    def integrate(frame, ordinal, revision):
        revisions.append(revision)
        occupied = grid.apply_hit(0) == OCCUPIED
        return dict(success=True, integrated=True,
                    expected_map_revision=revision,
                    resulting_map_revision=revision + 1,
                    footprint_hits=int(occupied))

    final, stamp = observe_fresh_frames(
        lambda after, deadline: (after + 1, object()), integrate, 13,
        after_ns=100, clock=lambda: 0)
    assert revisions == [13, 14, 15]
    assert stamp == 103
    assert final['acquisition_burst']['reason'] == 'confirmed'
    assert grid.occupied_threshold == 0.5


def test_duplicate_frame_is_not_integrated():
    calls = []
    final, stamp = observe_fresh_frames(
        lambda after, deadline: (after, object()),
        lambda *args: calls.append(args), 1, after_ns=100, clock=lambda: 0)
    assert not calls
    assert stamp == 100
    assert final['acquisition_burst']['reason'] == 'nonmonotonic_acquisition'


def test_unconfirmed_frames_stop_at_four():
    calls = []

    def integrate(frame, ordinal, revision):
        calls.append(revision)
        return dict(success=True, integrated=True, footprint_hits=0,
                    expected_map_revision=revision, resulting_map_revision=revision + 1)

    final, _ = observe_fresh_frames(
        lambda after, deadline: (after + 1, None), integrate, 1, clock=lambda: 0)
    assert calls == [1, 2, 3, 4]
    assert final['acquisition_burst']['reason'] == 'frame_limit'


def test_acquisition_exceeding_budget_is_not_integrated():
    now = [0]
    calls = []

    def acquire(after, deadline):
        now[0] = deadline
        return after + 1, None

    final, _ = observe_fresh_frames(acquire, lambda *a: calls.append(a), 1,
                                     clock=lambda: now[0])
    assert not calls
    assert final['acquisition_burst']['reason'] == 'deadline'


def test_refused_integration_does_not_retry():
    calls = []

    def integrate(*args):
        calls.append(args)
        return dict(success=False, integrated=False, drop_reason='STALE_REVISION')

    final, _ = observe_fresh_frames(
        lambda after, deadline: (after + 1, None), integrate, 1, clock=lambda: 0)
    assert len(calls) == 1
    assert final['drop_reason'] == 'STALE_REVISION'
    assert final['acquisition_burst']['reason'] == 'integration_refused'


def test_missing_frame_fails_without_integrating():
    calls = []
    final, _ = observe_fresh_frames(lambda *a: None, lambda *a: calls.append(a), 1,
                                     clock=lambda: 0)
    assert not calls
    assert not final['success']
    assert final['acquisition_burst']['reason'] == 'fresh_frame_unavailable'


def test_driver_waits_past_cached_frame_and_pins_each_new_acquisition():
    from types import SimpleNamespace
    from cargo_map_integrate_eval_driver import CargoMapIntegrateDriver

    frames = [SimpleNamespace(header=SimpleNamespace(
        stamp=SimpleNamespace(sec=0, nanosec=n))) for n in (100, 101, 102)]

    class Driver:
        _integrate_fresh_views = CargoMapIntegrateDriver._integrate_fresh_views

        def __init__(self):
            self.frames = iter(frames)
            self.calls = []

        def _latest_buffer_msg(self, key):
            assert key == 'depth'
            return next(self.frames)

        def _integrate_after_place(self, *args, **kwargs):
            self.calls.append(kwargs)
            revision = kwargs['expected_revision']
            return dict(success=True, integrated=True,
                        expected_map_revision=revision, resulting_map_revision=revision + 1,
                        footprint_hits=int(len(self.calls) == 2))

    driver = Driver()
    final = driver._integrate_fresh_views(None, None, None, None, 'aim', expected_revision=13)
    assert [c['depth_msg'] for c in driver.calls] == frames[1:]
    assert [c['expected_revision'] for c in driver.calls] == [13, 14]
    assert [c['view'] for c in driver.calls] == ['aim', 'aim_frame_2']
    assert driver._post_place_depth_stamp_ns == 102
    assert final['acquisition_burst']['reason'] == 'confirmed'
