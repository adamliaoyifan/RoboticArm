"""Transport buffering, snapshot ownership and freshness safeguards."""
from dataclasses import replace
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import time
import unittest

from execution_policy import ExecutionPolicy
from release_runtime import ReleaseRuntime


class ReleaseRuntimeContracts(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.policy = ExecutionPolicy.load()
        self.runtime = ReleaseRuntime(Path('not_launched'), Path(self.temp.name), self.policy, .001)
        self.runtime.process = SimpleNamespace(poll=lambda: None)
        self.runtime.native = SimpleNamespace(error=None)
        self.runtime.clock_capture = SimpleNamespace(error=None)

    def row(self, stamp):
        return dict(data=json.dumps(dict(sim_time_s=stamp, nested=dict(value=1))))

    def test_owned_snapshot_and_contiguous_armed_queue(self):
        r = self.runtime
        r._native(self.row(1.)); r._clock(dict(sim=dict(sec=1)))
        snapshot = r.current_native(); snapshot['nested']['value'] = 99
        self.assertEqual(r.current_native()['nested']['value'], 1)
        self.assertTrue(r.discovered)
        r.arm(); r._native(self.row(1.001)); r._native(self.row(1.002))
        self.assertEqual([r.next_native()['sim_time_s'] for _ in range(3)], [1., 1.001, 1.002])
        self.assertIsNone(r.next_native())

    def test_empty_queue_still_rejects_stale_stream(self):
        r = self.runtime; r._native(self.row(1.)); r._clock(dict(sim=dict(sec=1))); r.arm()
        r.next_native(); r.clock_wall = time.monotonic()-self.policy.observation_max_age_s*2
        with self.assertRaisesRegex(RuntimeError, 'OBSERVATION_STALE'):
            r.next_native()

    def test_processing_backlog_is_not_hidden_by_fresh_receipt(self):
        r = self.runtime; r._native(self.row(1.)); r._clock(dict(sim=dict(sec=2))); r.arm()
        with self.assertRaisesRegex(RuntimeError, 'OBSERVATION_STALE'):
            r.next_native()

    def test_nonincreasing_native_time_and_queue_overflow_preserved(self):
        r = self.runtime; r._native(self.row(1.))
        with self.assertRaisesRegex(ValueError, 'TIME_NOT_INCREASING'):
            r._native(self.row(1.))
        self.assertTrue((Path(self.temp.name)/'bad_native.json').is_file())
        r.arm()
        with self.assertRaisesRegex(RuntimeError, 'QUEUE_OVERFLOW'):
            for index in range(r._limit+1):
                r._native(self.row(1.+(index+1)*.001))


if __name__ == '__main__':
    unittest.main()
