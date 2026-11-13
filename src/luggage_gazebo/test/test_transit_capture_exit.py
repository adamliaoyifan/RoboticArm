"""Run the actual production place loop with fake boundaries; never moves ROS."""
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import place_smoke_driver as production
from luggage_gazebo.place_metrics import PlaceTrial
from luggage_gazebo.transit_capture_exit import TransitCaptureExit, HOLD_NS


class Driver:
    run_place_from_carry = production.PlaceSmokeDriver.run_place_from_carry
    _abort = production.PlaceSmokeDriver._abort

    def __init__(self, mode):
        self.mode, self.calls, self.now, self.wall = mode, [], 17, 0.
        self._args = NS(use_vacuum=True, plan_only=False)
        self._box_model = 'preserved_payload'
        self._keep_placed = True
        self._scene_config = {}
        self._selected_carry_v1 = [NS(name=name) for name in ('transit','insert','descend')]
        self._selected_carry_v2 = [object()]*3
        self._transit_capture_exit = TransitCaptureExit(
            healthy=self.healthy, now_ns=lambda: self.now,
            sleep=self.advance, monotonic=lambda: self.wall, hold_wall_limit=300.)

    def advance(self, seconds):
        self.wall += seconds
        if self.mode != 'clock_stopped':
            self.now += 10**9
        if self.mode == 'clock_backwards':
            self.now -= 2*10**9

    def healthy(self):
        return not (self.mode == 'fault_before' or
                    (self.mode == 'fault_hold' and self.now >= 10*10**9))

    def _execute_segment(self, segment, trial):
        self.calls.append('execute:' + segment.name)
        if self.mode == 'fault_execute':
            return False, 'EXECUTION_CAPTURE_FAULT', {}
        return self.mode != 'transit_failed', 'ORIGINAL_EXECUTION_FAILURE', {}

    def _carrying_vacuum_holds(self):
        return (self.mode != 'payload_lost', 'PAYLOAD_LOST')

    def _set_state(self, name, **kwargs):
        self.calls.append('state:' + name)
        self._place_state = name

    def _dump_place(self, trial, **kwargs):
        self.calls.append('dump')

    def _release_capture_event(self, *args, **kwargs):
        self.calls.append('abort_event')

    def _stop_release_capture(self):
        self.calls.append('flush_existing_capture')

    def _follow_skipped(self):
        return 0

    def _check_i1(self):
        return True, ''

    def _contact_capture_gate(self, *args):
        return None


@pytest.mark.parametrize('mode,expected', [
    ('success', ''), ('transit_failed', 'ORIGINAL_EXECUTION_FAILURE'),
    ('fault_before', 'DIAGNOSTIC_CAPTURE_UNHEALTHY'),
    ('fault_execute', 'EXECUTION_CAPTURE_FAULT'),
    ('fault_hold', 'DIAGNOSTIC_CAPTURE_UNHEALTHY'),
    ('payload_lost', 'PAYLOAD_LOST'),
    ('clock_stopped', 'DIAGNOSTIC_HOLD_CLOCK_TIMEOUT'),
    ('clock_backwards', 'DIAGNOSTIC_CLOCK_BACKWARDS')])
def test_real_place_loop_exits_once_without_next_leg_or_box_cleanup(monkeypatch, mode, expected):
    monkeypatch.setattr(production, 'point_inside_container_inner_box', lambda *a, **k: True)
    driver = Driver(mode)
    trial = PlaceTrial(index=0)
    trial.extras = {}
    slot = NS(place_pose=NS(position=NS(x=0., y=0., z=0.)))
    returned = driver.run_place_from_carry(None, trial, slot, {})
    assert returned is trial
    assert trial.fail_code == expected
    assert driver._box_model == 'preserved_payload'
    assert all('insert' not in call and 'descend' not in call for call in driver.calls)
    assert driver.calls.count('execute:transit') == (0 if mode == 'fault_before' else 1)
    assert driver.calls.count('dump') == 1
    if mode == 'success':
        assert trial.place_state == 'DIAGNOSTIC_COMPLETE_CARRYING'
        assert trial.extras['diagnostic_hold']['elapsed_ns'] == HOLD_NS
        assert trial.extras['diagnostic_complete']
    else:
        assert trial.place_state == 'ABORT_CARRYING'
        assert driver.calls[-3:] == ['flush_existing_capture', 'state:ABORT_CARRYING', 'dump']


@pytest.mark.parametrize('failure', [False, True])
def test_pack_entry_uses_production_startup_and_returns_before_cleanup(tmp_path, failure):
    from pack_eval_driver import PackEvalDriver
    calls = []
    class Pack:
        run = PackEvalDriver.run
        _out = str(tmp_path)
        _args = NS(sequence_ids='fixture', max_boxes=2, skip_unplaceable=False,
                   seed=42, skip_graph_check=False, await_map_confirmed=True)
        def _write_sequence(self, *args):
            calls.append('sequence')
        def wait_graph(self, **kwargs):
            calls.append('graph')
            return ''
        def _sample_rtf(self):
            return {}
        def _write_rtf(self, *args):
            pass
        def _home_arm(self):
            calls.append('home')
            return True, '', None
        _spawn = object()
        def _call(self, client, request, **kwargs):
            assert client is self._spawn
            calls.append('spawn')
            return NS(success=True, box=NS(width=.2, depth=.3, height=.4))
        def _box_dump_dir(self, *args):
            return str(tmp_path)
        def _await_map_confirmed(self):
            calls.append('map_gate')
            return ''
        def run_trial(self, *args, **kwargs):
            calls.append(('trial', args, kwargs))
            trial = PlaceTrial(index=0)
            trial.extras = {'diagnostic_complete': not failure}
            trial.fail_code = 'SYNTHETIC_FAILURE' if failure else ''
            return trial
        def _diagnostic_trial_exit(self, record, fail, seq):
            calls.append(('exit', fail, seq))
            return 'DIAGNOSTIC_ABORT' if fail else 'DIAGNOSTIC_COMPLETE'
    result = Pack().run()
    assert result == ('DIAGNOSTIC_ABORT' if failure else 'DIAGNOSTIC_COMPLETE')
    assert calls[:5] == ['sequence', 'graph', 'home', 'spawn', 'map_gate']
    trials = [call for call in calls if isinstance(call, tuple) and call[0] == 'trial']
    assert len(trials) == 1
    assert trials[0][2]['already_spawned'] and trials[0][2]['keep_placed']
    assert calls[-1][0] == 'exit'


def test_installed_driver_hooks_capture_typed_boundaries_and_fault_health(tmp_path):
    import diagnostic_transit_driver as entry
    import json
    from std_msgs.msg import String
    from luggage_msgs.srv import SelectPlacePathV2
    from luggage_planning.capture.async_boundary_capture import read_stream
    from rclpy.serialization import deserialize_message
    callbacks, previous, faults = {}, [], []
    class Typed:
        _boundary_recorder = staticmethod(lambda name, msg: previous.append((name, msg)))
        def set_boundary_recorder(self, callback):
            self._boundary_recorder = callback
    class Host:
        _typed = Typed()
        def create_publisher(self, *args):
            return NS(publish=lambda message: faults.append(json.loads(message.data)))
        def get_logger(self):
            return NS(error=lambda text: None)
        def create_subscription(self, kind, topic, callback, qos):
            callbacks[topic] = callback
            return object()
        def get_clock(self):
            return NS(now=lambda: NS(nanoseconds=123))
    driver = Host()
    recorder, healthy = entry.install_capture_exit(driver, tmp_path / 'driver', 'synthetic-hooks')
    assert not healthy()
    callbacks['/diagnostic_capture/health'](String(data=json.dumps(dict(
        run_id='wrong-run', worker_alive=True, closed=False, errors=[], targets_ready=True))))
    assert not healthy()
    callbacks['/diagnostic_capture/health'](String(data=json.dumps(dict(
        run_id='synthetic-hooks', worker_alive=True, closed=False, errors=[], targets_ready=True))))
    assert healthy()
    message = SelectPlacePathV2.Request(request_id='same-run-select')
    driver._typed._boundary_recorder('/select_place_path_v2:request', message)
    assert previous == [('/select_place_path_v2:request', message)]
    assert driver._transit_capture_exit.now_ns() == 123
    complete = NS(extras={'diagnostic_complete': True}, fail_code='')
    assert driver._diagnostic_trial_exit(complete, '', 0) == 'DIAGNOSTIC_COMPLETE'
    assert driver._diagnostic_trial_result is complete
    callbacks['/diagnostic_capture/health'](String(data=json.dumps(dict(
        run_id='synthetic-hooks', worker_alive=True, closed=False, errors=[{'name':'CAPTURE_WRITE_FAILED'}],
        targets_ready=True))))
    assert not healthy()
    recorder.fault('CAPTURE_WRITE_FAILED', 'synthetic driver fault')
    assert faults == [dict(run_id='synthetic-hooks', name='CAPTURE_WRITE_FAILED',
                           detail='synthetic driver fault')]
    with pytest.raises(RuntimeError, match='DIAGNOSTIC_CAPTURE_UNHEALTHY'):
        driver._typed._boundary_recorder('/plan_motion_v2:request', message)
    assert not recorder.close()['recorder_io_complete']
    records = list(read_stream(recorder.root))
    assert len(records) == 1
    assert deserialize_message(records[0][1], SelectPlacePathV2.Request) == message


def test_hold_failure_retains_bounded_post_window_then_aborts(monkeypatch):
    """[2026-10-21 ruling] settle failure keeps <=5 sim-s for the capture
    post-window, then aborts via the normal safety flow (no 60 s wait)."""
    monkeypatch.setattr(production, 'point_inside_container_inner_box', lambda *a, **k: True)
    driver = Driver('payload_lost')
    trial = PlaceTrial(index=0)
    trial.extras = {}
    slot = NS(place_pose=NS(position=NS(x=0., y=0., z=0.)))
    returned = driver.run_place_from_carry(None, trial, slot, {})
    assert returned is trial
    assert trial.fail_code == 'PAYLOAD_LOST'
    hold = trial.extras['diagnostic_hold']
    # advance() gives 1 sim-s per sleep; retention stops at the 5 s cap.
    assert 5 * 10**9 <= hold['retained_after_fail_ns'] <= 6 * 10**9
    assert hold['reason'] == 'PAYLOAD_LOST'
    assert trial.place_state == 'ABORT_CARRYING'


def test_hold_ns_env_override():
    import luggage_gazebo.transit_capture_exit as mod
    import os
    os.environ['JOINT_DIAG_HOLD_NS'] = '5e9'
    try:
        d = Driver('success')
        assert d._transit_capture_exit.hold_ns == 5 * 10**9
    finally:
        del os.environ['JOINT_DIAG_HOLD_NS']
    d2 = Driver('success')
    assert d2._transit_capture_exit.hold_ns == HOLD_NS
