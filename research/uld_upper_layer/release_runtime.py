"""Owned Gazebo/transport lifecycle for independent carriage experiments."""
from collections import deque
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import subprocess
import threading
import time

from release_rehearsal import assert_sim_slot_free, save, COORD
from transport_capture import JsonTopicCapture


class ReleaseRuntime:
    def __init__(self, world, output, policy, physics_step_s):
        self.world = Path(world); self.output = Path(output); self.policy = policy
        self.env = dict(os.environ, IGN_PARTITION='uld_upper_release_trial', ROS_DOMAIN_ID='7')
        self._lock = threading.Lock(); self._queue = deque(); self._armed = False
        self._limit = math.ceil(policy.observation_max_age_s / physics_step_s)*2
        self.latest = None; self.clock = None; self.clock_wall = None; self.native_wall = None
        self.process = None; self.log = None; self.native = None; self.clock_capture = None
        self.max_backlog_s = 0.; self.last_delivered = None
        self.freshness_failure = None

    def _native(self, row):
        try:
            native = json.loads(row['data']); stamp = native['sim_time_s']
            with self._lock:
                if self.latest is not None and stamp <= self.latest['sim_time_s']:
                    raise ValueError('RELEASE_NATIVE_TIME_NOT_INCREASING')
                self.latest = native; self.native_wall = time.monotonic()
                if self._armed:
                    if len(self._queue) >= self._limit:
                        raise RuntimeError('RELEASE_NATIVE_QUEUE_OVERFLOW')
                    self._queue.append(native)
        except Exception:
            save(self.output/'bad_native.json', row)
            raise
        return stamp

    def _clock(self, row):
        sim = row['sim']; stamp = float(sim.get('sec', 0)) + float(sim.get('nsec', 0))*1e-9
        with self._lock:
            self.clock = stamp; self.clock_wall = time.monotonic()
        return stamp

    def start(self):
        assert_sim_slot_free()
        budget = (self.policy.max_case_bytes-self.policy.failure_capture_reserve_bytes)//2
        self.native = JsonTopicCapture(self.env, self.output, 'native', '/upper_layer/vacuum/state', self._native, budget)
        self.clock_capture = JsonTopicCapture(self.env, self.output, 'clock', '/world/airport_loading/clock', self._clock, budget)
        self.native.start(); self.clock_capture.start()
        self.log = (self.output/'launch.log').open('w')
        self.process = subprocess.Popen(['ign', 'gazebo', '-s', '-r', '-v', '3', str(self.world)],
            env=self.env, stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
        (self.output/'launch.pid').write_text(str(self.process.pid))
        Path('/tmp/elfin_upper_release_sim.pid').write_text(str(self.process.pid))

    def arm(self):
        with self._lock:
            if self.latest is None:
                raise ValueError('RELEASE_NATIVE_NOT_READY')
            self._queue.clear(); self._queue.append(self.latest); self._armed = True
            self.last_delivered = None

    def current_native(self):
        """Return an owned snapshot; callers cannot mutate transport state."""
        with self._lock:
            return deepcopy(self.latest)

    @property
    def discovered(self):
        with self._lock:
            return self.latest is not None and self.clock is not None

    def next_native(self, *, check_fresh=True):
        if self.process.poll() is not None:
            raise RuntimeError('RELEASE_SIM_EXITED')
        for capture in (self.native, self.clock_capture):
            if capture.error:
                raise RuntimeError(capture.error)
        with self._lock:
            # observation_max_age_s is in simulation stamp time. A physics
            # pause advances neither cargo nor that clock; use the separate
            # condition-wait WALL budget to detect a stalled transport/server.
            wall_now = time.monotonic()
            if check_fresh and self._armed and (
                    self.clock_wall is None or self.native_wall is None
                    or wall_now-self.clock_wall > self.policy.condition_wait_budget_s
                    or wall_now-self.native_wall > self.policy.condition_wait_budget_s):
                self.freshness_failure = dict(reason='RELEASE_STREAM_WALL_TIMEOUT',
                    clock_silence_wall_s=None if self.clock_wall is None else wall_now-self.clock_wall,
                    native_silence_wall_s=None if self.native_wall is None else wall_now-self.native_wall,
                    condition_wait_budget_wall_s=self.policy.condition_wait_budget_s,
                    queue_count=len(self._queue), clock_stamp_s=self.clock,
                    latest_native_stamp_s=self.latest['sim_time_s'] if self.latest else None)
                save(self.output/'freshness_failure.json', self.freshness_failure)
                raise RuntimeError('RELEASE_STREAM_WALL_TIMEOUT')
            row = self._queue.popleft() if self._armed and self._queue else None
            if not self._armed and self.latest and self.latest['sim_time_s'] != self.last_delivered:
                row = self.latest
            if row is None:
                return None
            if check_fresh:
                if self.clock is None or self.clock_wall is None:
                    raise RuntimeError('RELEASE_CLOCK_MISSING')
                lag = max(self.clock, self.latest['sim_time_s'])-row['sim_time_s']
                self.max_backlog_s = max(self.max_backlog_s, lag)
                if lag > self.policy.observation_max_age_s:
                    self.freshness_failure = dict(reason='RELEASE_OBSERVATION_STALE',
                        processing_backlog_sim_s=lag, observation_max_age_sim_s=self.policy.observation_max_age_s,
                        clock_stamp_s=self.clock, native_stamp_s=row['sim_time_s'],
                        latest_native_stamp_s=self.latest['sim_time_s'], queue_count=len(self._queue))
                    save(self.output/'freshness_failure.json', self.freshness_failure)
                    raise RuntimeError('RELEASE_OBSERVATION_STALE')
            self.last_delivered = row['sim_time_s']
            return row

    def publishers(self):
        checks = {}
        for name, topic, typename in (('state', '/upper_layer/vacuum/state', 'ignition.msgs.StringMsg'),
                ('clock', '/world/airport_loading/clock', 'ignition.msgs.Clock')):
            result = subprocess.run(['ign', 'topic', '-i', '-t', topic], env=self.env,
                capture_output=True, text=True, timeout=10)
            (self.output/(name+'_topic_info.txt')).write_text(result.stdout+result.stderr)
            checks[name] = result.returncode == 0 and result.stdout.split('Subscribers')[0].count(typename) == 1
        if not all(checks.values()):
            raise RuntimeError('RELEASE_PUBLISHER_COUNT_INVALID')
        return checks

    def detach(self):
        return subprocess.Popen(['ign', 'service', '-s', '/upper_layer/vacuum',
            '--reqtype', 'ignition.msgs.Boolean', '--reptype', 'ignition.msgs.Boolean',
            '--timeout', str(math.ceil(self.policy.condition_wait_budget_s*1000)), '--req', 'data: false'],
            env=self.env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    def stop_capture(self):
        for capture in (self.native, self.clock_capture):
            if capture:
                capture.stop()
        return dict(native_samples=self.native.count if self.native else 0,
            native_error=self.native.error if self.native else 'not_started',
            clock_samples=self.clock_capture.count if self.clock_capture else 0,
            clock_error=self.clock_capture.error if self.clock_capture else 'not_started',
            max_processing_backlog_s=self.max_backlog_s)

    def teardown(self):
        if self.process is None:
            return True
        result = subprocess.run(['bash', str(COORD/'scripts/stop_sim.sh'), '--owned-pgid',
            str(self.process.pid), '--pidfile', '/tmp/elfin_upper_release_sim.pid'],
            capture_output=True, text=True, timeout=30)
        (self.output/'teardown.log').write_text(result.stdout+result.stderr)
        self.process.wait(timeout=5)
        if self.log:
            self.log.close()
        return result.returncode == 0
