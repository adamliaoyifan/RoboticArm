"""Persistent carriage joint publication; acknowledgements are not tracking.

The owner supplies an independently checked reference and observes actual
joint/tool motion. This adapter has no box, attachment or planning interface.
"""
import json
import math
import os
from pathlib import Path
import selectors
import subprocess
import time


class CarriageCommandPublisher:
    def __init__(self, executable, topics, env, output, fixture_policy, execution_policy):
        if len(topics) != 2 or len(set(topics)) != 2:
            raise ValueError('CARRIAGE_COMMAND_TOPICS_INVALID')
        self._executable = Path(executable).resolve(strict=True)
        self._topics = tuple(topics); self._env = dict(env); self._output = Path(output)
        self._fixture_policy = fixture_policy; self._policy = execution_policy
        self._process = None; self._sequence = 0
        self._selector = None; self._trace = None; self._stderr = None
        self._buffer = b''

    def _response(self, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if b'\n' in self._buffer:
                line, self._buffer = self._buffer.split(b'\n', 1)
                return json.loads(line)
            remaining = deadline - time.monotonic()
            if not self._selector.select(max(0., remaining)):
                break
            chunk = os.read(self._process.stdout.fileno(), 4096)
            if not chunk:
                raise RuntimeError('CARRIAGE_COMMAND_BRIDGE_EXITED')
            self._buffer += chunk
            if len(self._buffer) > 4096:
                raise RuntimeError('CARRIAGE_COMMAND_ACK_TOO_LARGE')
        raise TimeoutError('CARRIAGE_COMMAND_ACK_TIMEOUT')

    def start(self):
        if self._process is not None:
            raise RuntimeError('CARRIAGE_COMMAND_BRIDGE_ALREADY_STARTED')
        self._stderr = (self._output / 'joint_command_stderr.log').open('w')
        self._trace = (self._output / 'joint_commands.jsonl').open('w')
        timeout = self._policy.condition_wait_budget_s
        self._process = subprocess.Popen([str(self._executable), *self._topics, str(math.ceil(timeout * 1000))],
            env=self._env, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self._stderr,
            start_new_session=True)
        self._selector = selectors.DefaultSelector()
        self._selector.register(self._process.stdout, selectors.EVENT_READ)
        response = self._response(timeout)
        if response != {'ready': True}:
            raise RuntimeError('CARRIAGE_COMMAND_BRIDGE_NOT_READY')

    def publish(self, positions_xz_m, observation_stamp_s):
        positions = tuple(positions_xz_m)
        if (len(positions) != 2 or any(isinstance(v, bool) or not math.isfinite(v) or
                abs(v) > self._fixture_policy.travel_limit_m for v in positions) or
                isinstance(observation_stamp_s, bool) or not math.isfinite(observation_stamp_s) or observation_stamp_s < 0):
            raise ValueError('CARRIAGE_COMMAND_OUTSIDE_CONTRACT')
        if self._process is None:
            raise RuntimeError('CARRIAGE_COMMAND_BRIDGE_NOT_STARTED')
        self._sequence += 1
        request = dict(sequence=self._sequence, positions_xz_m=positions,
                       observation_stamp_s=observation_stamp_s, tracking_confirmed=False)
        self._trace.write(json.dumps(dict(request=request)) + '\n'); self._trace.flush()
        self._process.stdin.write(f'{self._sequence} {positions[0]:.17g} {positions[1]:.17g}\n'.encode())
        self._process.stdin.flush()
        response = self._response(self._policy.observation_max_age_s)
        self._trace.write(json.dumps(dict(response=response)) + '\n'); self._trace.flush()
        if response != dict(sequence=self._sequence, published=True):
            raise RuntimeError('CARRIAGE_COMMAND_ACK_MISMATCH')
        return request

    def close(self):
        try:
            if self._process is not None:
                if self._process.stdin and not self._process.stdin.closed:
                    try:
                        self._process.stdin.close()
                    except BrokenPipeError:
                        pass
                try:
                    self._process.wait(timeout=3.)
                except subprocess.TimeoutExpired:
                    self._process.terminate()
                    self._process.wait(timeout=3.)
        finally:
            if self._selector is not None:
                self._selector.close()
            if self._process is not None and self._process.stdout is not None:
                self._process.stdout.close()
            if self._trace is not None:
                self._trace.close()
            if self._stderr is not None:
                self._stderr.close()
