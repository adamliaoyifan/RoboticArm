"""Offline numeric/missing-observation audit; no ROS or product scoring."""
import argparse
import copy
import json
import subprocess
import sys
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument('--worktree', required=True)
p.add_argument('--output', required=True)
a = p.parse_args()
sys.path.insert(0, str(Path(a.worktree) / 'src/uld_occ_observer/test/benchmark'))
import repair_s0_judge as judge

cases = []
for name, factory in [('o8', judge.healthy_o8), ('o9', judge.healthy_o9),
                      ('o6', lambda: judge.healthy_latency_group('r02_b00_chamfer_c30000_o50_x00', 30000))]:
    cases.append((name + '_healthy', factory(), 'pass'))
for label, value in [('nan', float('nan')), ('negative', -1), ('bool', True), ('null', None)]:
    case = judge.healthy_o8()
    case['runs'][0]['rss_median_kib'][2] = value
    cases.append(('o8_rss_' + label, case, 'not_evaluated' if value is None else 'fail'))
case = judge.healthy_latency_group('r02_b00_chamfer_c30000_o50_x00', 30000)
del case['surface_first_ms']
cases.append(('o6_missing_surface_first_ms', case, 'not_evaluated'))
case = judge.healthy_o9()
del case['operations']['roi'][0]['off_ms']
cases.append(('o9_missing_off_ms', case, 'not_evaluated'))
results = []
for name, case, expected in cases:
    try:
        actual = judge.judge_case(copy.deepcopy(case))
    except Exception as error:
        actual = {'exception': type(error).__name__, 'message': str(error)}
    results.append({'name': name, 'input': case, 'expected_status': expected,
                    'actual': actual, 'matched': actual.get('status') == expected})
out = Path(a.output)
out.write_text(json.dumps({'revision': subprocess.check_output(
    ['git', '-C', a.worktree, 'rev-parse', 'HEAD'], text=True).strip(),
    'results': results}, ensure_ascii=False, indent=2) + '\n')
for row in results:
    print(row['name'], 'expected=' + row['expected_status'], json.dumps(row['actual']))
sys.exit(0 if all(row['matched'] for row in results) else 1)
