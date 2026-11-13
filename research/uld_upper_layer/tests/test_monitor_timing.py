"""Timing diagnostics distinguish blocked wall time without changing GC policy."""
from pathlib import Path
import gc,gzip,json,sys,tempfile,time,unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from monitor_timing import MonitorTiming


class MonitorTimingContracts(unittest.TestCase):
    def test_wall_cpu_gc_and_owned_callback_teardown(self):
        callbacks=list(gc.callbacks);enabled=gc.isenabled();frozen=gc.get_freeze_count()
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'timing.jsonl.gz';timing=MonitorTiming(path,slow_s=.005)
            with timing.stage('collision',2.):time.sleep(.01)
            timing._on_gc('start',dict(generation=2))
            time.sleep(.01)
            timing._on_gc('stop',dict(generation=2,collected=3,uncollectable=0))
            with timing.stage('callback',2.01):pass
            timing.close();rows=[json.loads(line) for line in gzip.open(path,'rt')]
            collision=next(row for row in rows if row.get('stage')=='collision')
            self.assertGreaterEqual(collision['wall_s'],.01)
            self.assertLess(collision['thread_cpu_s'],collision['wall_s'])
            self.assertTrue(any(row['event']=='gc' and row['generation']==2 for row in rows))
            self.assertGreaterEqual(timing.metrics()['gc_max_pause_s'][2],.01)
        self.assertEqual(gc.callbacks,callbacks);self.assertEqual(gc.isenabled(),enabled)
        self.assertEqual(gc.get_freeze_count(),frozen)

    def test_slow_event_dump_is_bounded_and_omissions_are_visible(self):
        with tempfile.TemporaryDirectory() as temp:
            timing=MonitorTiming(Path(temp)/'timing.jsonl.gz',slow_s=0.,max_events=1)
            for i in range(3):
                with timing.stage('callback',float(i)):pass
            timing.close();metrics=timing.metrics()
            self.assertEqual(metrics['stages']['callback']['count'],3)
            self.assertEqual(metrics['events_written'],1);self.assertEqual(metrics['events_omitted'],2)


if __name__=='__main__':unittest.main()
