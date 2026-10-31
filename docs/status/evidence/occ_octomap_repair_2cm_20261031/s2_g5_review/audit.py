import contextlib
import io
import json
import os
from pathlib import Path
import sys
import tempfile
from unittest.mock import patch

ROOT = Path('/tmp/elfin-occ-octomap-repair-2cm-g1')
sys.path.insert(0, str(ROOT / 'src/uld_occ_observer/test/benchmark'))
import campaign_executor as ce
import campaign_entry

results = {}
with tempfile.TemporaryDirectory() as temp:
    # Never launch a real process, even when exercising the authorized branch.
    with patch.dict(os.environ, {'ULD_OCC_S3_OPEN': '1',
                                 'ULD_OCC_S2_CONFIRM_OFFICIAL': '1',
                                 'ULD_OCC_S2_ADAPTER': 'live'}), \
            patch('subprocess.Popen', side_effect=AssertionError('real launch forbidden')):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            code = ce.main(['--mode', 'stage', '--stage', 'o5', '--out', temp + '/live'])
        results['live_open'] = {'exit': code, 'result': json.loads(output.getvalue())}
    with patch.dict(os.environ, {'ULD_OCC_S3_OPEN': '1', 'ULD_OCC_S2_ADAPTER': 'injected'}):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            code = ce.main(['--mode', 'stage', '--stage', 'o7', '--out', temp + '/blocked'])
        results['o7_without_o5'] = {'exit': code, 'result': json.loads(output.getvalue())}
    plan = campaign_entry.load_plan(Path('/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures'))
    plan['o7_windows'] = [plan['o7_windows'][0]]
    class SlowQuery(ce.OfflineAdapter):
        def ros_query(self, window, index):
            result = super().ros_query(window, index)
            self.clock.advance(1.0)
            return result
    adapter = SlowQuery()
    ce.schedule(plan, adapter, temp + '/slow', stages=('o7',), input_limit=2,
                allow_incomplete_dependencies=True)
    sends = [entry for entry in adapter.order if entry[0] == 'send']
    results['slow_query_sender'] = {'send_times': [entry[3] for entry in sends],
                                  'expected_interval_s': 0.5,
                                  'actual_interval_s': sends[1][3] - sends[0][3]}
print(json.dumps(results, indent=2))
