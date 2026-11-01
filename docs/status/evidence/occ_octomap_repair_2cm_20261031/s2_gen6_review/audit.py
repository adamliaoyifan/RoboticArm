import sys,json,hashlib
from pathlib import Path
from types import SimpleNamespace as NS
root=Path('/tmp/elfin-occ-octomap-repair-2cm-g2')
sys.path.insert(0,str(root/'src/uld_occ_observer/test/benchmark'))
import live_schedule as live
import repair_s0_judge as judge
out=Path(__file__).parent
adapter=object.__new__(live.LiveScheduleAdapter)
adapter.plan_o8={'warmup_s':60,'duration_s':180}
run=NS(origin_ns=0,session=NS(health_events=[(70_000_000_000,'tree_nodes: 71')],receipt_events=[]),rss_samples=[1000]*9)
adapter._stability={'run':run,'stamps':['missing-%d'%i for i in range(30)]}
original=live.executor._receipt_events
live.executor._receipt_events=lambda session,observation: []
row=adapter.stability_run('missing_receipts')
live.executor._receipt_events=original
case=judge.healthy_o8()
case['runs']=[dict(row,run_id=str(i)) for i in range(3)]
verdict=judge.judge_o8(case)
assert verdict['pass'],verdict
adapter._groups={'g':{'samples':[{'marker':i} for i in range(34)],'warm_init_s':1,'resources':None}}
adapter._cold_start=lambda *args: {'init_s':1,'rss_kib':1000}
selected=[adapter.profile_frame({'id':'g'},i,False,out)['marker'] for i in range(3,33)]
assert selected==list(range(30))
evidence=root/'docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_gen6'
freeze=json.loads((evidence/'FREEZE.json').read_text())
sha={k: hashlib.sha256(Path(v['path']).read_bytes()).hexdigest()==v['sha256'] for k,v in freeze['binaries'].items()}
result={'candidate':freeze['head'],'binary_hash_match':sha,'injected_missing_all_30_receipts':{'adapter_row':row,'judge':verdict,'healthy_holds_fixture':True,'live_run':False},'profile_scored_source_indices':selected,'expected_scored_source_indices':list(range(3,33))}
(out/'RESULTS.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
# A filename-complete bundle with a nonexistent input must not be replayable.
base=out/'forged_bundle'
folder=base/'o5'/'cumulative'
folder.mkdir(parents=True,exist_ok=True)
(folder/'samples.yaml').write_text('{}\n')
(folder/'cli.log').write_text('fixture\n')
(folder/'manifest.yaml').write_text('frames:\n- path: missing.yaml\n  sha256: fake\n')
path=live.executor._save_failure(base,'o5','injected_missing_input','offline audit')
manifest=json.loads((path/'manifest.json').read_text())
assert manifest['replay_possible'] is True
result['forged_missing_input_bundle']={k:manifest[k] for k in ('capture_complete','replay_possible','missing')}
(out/'RESULTS.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result['forged_missing_input_bundle']))
