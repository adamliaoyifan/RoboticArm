import sys,json,tempfile,hashlib
from pathlib import Path
w=Path('/tmp/elfin-occ-freespace-impl-g4')
e=w/'docs/status/evidence/occ_freespace_final_20261102/implementation_g4'
sys.path.insert(0,str(w/'src/uld_occ_observer/test/benchmark'))
import freespace_verdict as v
import freespace_judge as j
out=Path(__file__).resolve().parent
f=json.loads((e/'FREEZE.json').read_text()); checks=[]
for group in ('binaries','tools','fixtures','inputs','commands'):
 entries=f[group]; entries={'self':entries} if 'path' in entries else entries
 for name,row in entries.items():
  p=Path(row['path']); checks.append({'group':group,'name':name,'match':p.is_file() and hashlib.sha256(p.read_bytes()).hexdigest()==row['sha256']})
(out/'freeze_checks.json').write_text(json.dumps(checks,indent=2))
actual=v.judge_evidence_root(e)
(out/'recomputed_verdict.json').write_text(json.dumps(actual,indent=2))
neg=[]
with tempfile.TemporaryDirectory() as temp:
 root=Path(temp)
 for stage in ('o8','o5','o6'):
  d=root/'scoring'/stage;d.mkdir(parents=True)
  for value in (0,True,30):
   (d/'summary.json').write_text(json.dumps({'passed':value,'total':value}))
   neg.append({'stage':stage,'value':value,'result':v._row_formal(root,stage)[:2]})
(out/'formal_summary_counterexamples.json').write_text(json.dumps(neg,indent=2))
print('freeze',sum(x['match'] for x in checks),len(checks))
print('verdict',actual['overall'])
print('summary mutants',neg)
