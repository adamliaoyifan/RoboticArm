"""Execute the actual nested smoke Adapter against a completed status fixture.

No ROS process/network; the clock is deliberately virtual. This tests client
interpretation only, not whether the historical cloud reached the observer.
"""
import ast
import itertools
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

path = Path('/tmp/elfin-occ-octomap-repair-2cm-g1/src/uld_occ_observer/test/benchmark/campaign_executor.py')
tree = ast.parse(path.read_text())
smoke = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'smoke_ros')
adapter = next(n for n in smoke.body if isinstance(n, ast.ClassDef) and n.name == 'Adapter')
response = SimpleNamespace(status='COMPLETED', receipt=SimpleNamespace(status='INTEGRATED'))
holder = {'observation_id': '1000000000', 'session': SimpleNamespace(query=lambda *args: response)}
namespace = {'holder': holder, 'ManualClock': SimpleNamespace}
exec(compile(ast.Module(body=[adapter], type_ignores=[]), str(path), 'exec'), namespace)
instance = namespace['Adapter']()
ticks = itertools.count()
with patch('time.monotonic', side_effect=lambda: float(next(ticks))), patch('time.sleep'):
    query = instance.ros_query({}, 0)
    collected = instance.ros_collect({'id': 'normal_b00_r0'}, 0, 'score-0')
print(json.dumps({'status_response': {'status': response.status, 'receipt_status': response.receipt.status},
                  'query_artifact': query, 'input_artifact': collected,
                  'smoke_parseable': holder.get('query_status') == 'INTEGRATED',
                  'scope': 'offline client status interpretation; no ROS execution'}, indent=2))
assert collected['disposition'] == 'unknown'
