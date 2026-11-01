import sys,json,copy,base64,hashlib,struct
from pathlib import Path
root=Path('/tmp/elfin-occ-octomap-repair-2cm-g5')
sys.path.insert(0,str(root/'src/uld_occ_observer/test/benchmark'))
import repair_s0_judge as judge
import repair_s0_contract as contract
out=Path(__file__).parent
base=json.loads((out.parent/'s2_gen9/stage_rehearsal/o8/runs.json').read_text())
base['artifact_dir']=str((out.parent/'s2_gen9/stage_rehearsal/o8').resolve())
r={'kind':'offline injection into saved artifact; originals unchanged','healthy':judge.judge_o8(base)}
assert r['healthy']['pass'],r
case=copy.deepcopy(base)
for hold in case['holds']:
 for name in ('probe_before','probe_after'):
  p=hold[name];p['cells']=1
  states=b'\x00';odds=struct.pack('<f',0)
  p['content']={'states_b64':base64.b64encode(states).decode(),'odds_b64':base64.b64encode(odds).decode(),
    'states_sha256':hashlib.sha256(states).hexdigest(),'odds_sha256':hashlib.sha256(odds).hexdigest(),
    'content_sha256':contract.probe_content_sha256(states,odds)}
r['one_cell_with_full_region_label']=judge.judge_o8(case)
assert r['one_cell_with_full_region_label']['pass']
(out/'RESULTS.json').write_text(json.dumps(r,indent=2)+'\n')
print(json.dumps(r,indent=2))
