"""Behavioural filesystem budget and lossless compressed trigger checks."""
from pathlib import Path
import sys,json,gzip,tempfile,unittest
from collections import deque
from types import SimpleNamespace
from bounded_trace import BoundedTrace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from runtime_io import RuntimeIO
from execution_monitor import ExecutionMonitor

class CaptureBudgetContracts(unittest.TestCase):
    def test_case_reserve_accounts_for_multiple_streams_before_hard_limit(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            (root/'one.gz').write_bytes(b'a'*700)
            (root/'two.gz').write_bytes(b'b'*700)
            runtime=RuntimeIO(root,root,capture_policy=SimpleNamespace(max_case_bytes=2000,failure_capture_reserve_bytes=500))
            self.assertIsNone(runtime.snapshot()['capture_error'])
            (root/'two.gz').write_bytes(b'b'*900)
            runtime._next_capture_check=0.
            self.assertTrue(runtime.snapshot()['capture_error'].startswith('CASE_CAPTURE_RESERVE_REACHED'))
            self.assertLess(sum(p.stat().st_size for p in root.iterdir()),2000)

    def test_trigger_compression_keeps_all_pre_and_post_samples(self):
        rows=[(i*.02,dict(native='contact '*1000,index=i)) for i in range(150)]
        monitor=object.__new__(ExecutionMonitor);monitor._ring=deque(rows[:100])
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'trigger.jsonl.gz';monitor.save_trigger(path,rows[100:])
            with gzip.open(path,'rt') as stream:actual=[json.loads(line) for line in stream]
            self.assertEqual([r['observation'] for r in actual],[r[1] for r in rows])
            self.assertEqual([r['stamp_s'] for r in actual],[r[0] for r in rows])
            self.assertLess(path.stat().st_size,len(json.dumps(actual))//10)

    def test_every_callback_retained_in_raw_window_and_sampled_baseline_bounded(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'native.jsonl.gz'
            trace=BoundedTrace(path,period_s=.02,pre_s=.2,post_s=.1,lag_s=.02)
            for i in range(501):trace.append(dict(stamp=i*.001,index=i),i*.001)
            trace.start_trigger(.49)
            for i in range(501,601):trace.append(dict(stamp=i*.001,index=i),i*.001)
            stats=trace.metrics();trace.close()
            with gzip.open(path,'rt') as f:baseline=[json.loads(line) for line in f]
            with gzip.open(Path(temp)/'native_trigger.jsonl.gz','rt') as f:window=[json.loads(line) for line in f]
            self.assertEqual(stats['seen'],601);self.assertEqual(len(baseline),31)
            self.assertTrue(stats['trigger_complete'])
            self.assertEqual([r['index'] for r in window],list(range(290,591)))
            self.assertLess(stats['archived'],stats['seen']//10)
