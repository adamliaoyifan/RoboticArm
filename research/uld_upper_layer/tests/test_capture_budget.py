"""Behavioural filesystem budget and lossless compressed trigger checks."""
from pathlib import Path
import sys,json,gzip,tempfile,unittest
from collections import deque
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from runtime_io import RuntimeIO
from execution_monitor import ExecutionMonitor

class CaptureBudgetContracts(unittest.TestCase):
    def test_case_reserve_accounts_for_multiple_streams_before_hard_limit(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            (root/'one.gz').write_bytes(b'a'*700)
            (root/'two.gz').write_bytes(b'b'*700)
            runtime=RuntimeIO(root,root,capture_budget_bytes=2000,capture_reserve_bytes=500)
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
