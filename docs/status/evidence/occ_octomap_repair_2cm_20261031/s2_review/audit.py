import ast
import json
import math
import sys
from pathlib import Path
root = Path('/home/adamliao/work/elfin_humble_ws')
wt = Path('/tmp/elfin-occ-octomap-repair-2cm-g1')
out = Path(__file__).resolve().parent
source = wt / 'src/uld_occ_observer/test/s2_rehearsal.py'
tree = ast.parse(source.read_text())
functions = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in ('replay_bundle', 'replay_trace')]
ns = {'json': json, 'Path': Path}
exec(compile(ast.Module(body=functions, type_ignores=[]), str(source), 'exec'), ns)
# Exact production helper bodies, no ROS imported and no implementation edits.
fake = {'input_id': 'never-executed', 'epoch': 99, 'status': 'REJECTED', 'reason': 'invented', 'sensor_id': 'cam0', 'capture_complete': True}
p = out/'fabricated_failure.json'; p.write_text(json.dumps(fake)+'\n')
trace = out/'dropped_identity_free_trace.jsonl'
trace.write_text(''.join(json.dumps(x)+'\n' for x in [{'event':'integrate_start'}, {'event':'integrate_end'}, {'event':'trace_footer','status':'999'}]))
results = [
 {'case':'fabricated bundle without input/TF or execution', 'expected_replay_possible':False, 'actual':ns['replay_bundle'](p)},
 {'case':'trace with 999 drops and no identities', 'expected_replay_possible':False, 'actual':ns['replay_trace'](trace)}]
sys.path.insert(0,str(wt/'src/uld_occ_observer/test/benchmark'))
import repair_s0_judge as judge
ops=json.loads((root/'docs/status/evidence/occ_octomap_repair_2cm_20261031/s2/o9_pairs.json').read_text())
ratios={}
for name,pairs in ops.items():
 off=sorted(x['off_ms'] for x in pairs); on=sorted(x['on_ms'] for x in pairs)
 ratios[name]={'pairs':len(pairs),'ratio':on[math.ceil(.95*len(on))-1]/off[math.ceil(.95*len(off))-1]}
report={'scope':'offline review; no product or O9 timing runs','results':results,'o9_arithmetic':ratios,'official_judge':judge.judge_case({'gate':'o9','operations':ops})}
(out/'RESULTS.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
raise SystemExit(1 if any(x['actual']['replay_possible'] for x in results) else 0)
