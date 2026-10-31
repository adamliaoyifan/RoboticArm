import importlib.util
import json
from pathlib import Path
path = Path('/tmp/elfin-occ-octomap-repair-2cm-g1/src/uld_occ_observer/test/test_repair_s0.py')
spec = importlib.util.spec_from_file_location('matrix_tests', path)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
rows = []
for name, factory, mutate, status, reason in m._numeric_specs():
    case = factory()
    mutate(case)
    direct = m.judge.judge_case(case)
    replay = m.replay.verify_replay(case)
    ok = direct['status'] == replay['status'] == status and reason in direct['reasons'] and reason in replay['reasons'] and replay['replay_possible'] is True
    rows.append({'name': name, 'expected': status, 'reason': reason, 'matched': ok})
report = {'cases': len(rows), 'matched': sum(row['matched'] for row in rows), 'rows': rows}
print(json.dumps(report, indent=2))
assert all(row['matched'] for row in rows)
