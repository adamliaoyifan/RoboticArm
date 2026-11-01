import sys,json,copy
from pathlib import Path
root=Path('/tmp/elfin-occ-octomap-repair-2cm-g4')
sys.path.insert(0,str(root/'src/uld_occ_observer/test/benchmark'))
import live_schedule as live
import repair_s0_judge as judge
out=Path(__file__).parent
base=json.loads((out.parent/'s2_gen8/stage_rehearsal/o8/runs.json').read_text())
r={'candidate':'03334c0b','kind':'offline injected; saved owner artifact healthy control','healthy_o8':judge.judge_o8(base)}
assert r['healthy_o8']['pass']
for name in ('nan_duration','inf_duration','missing_log_odds_hash','different_probe_region'):
 case=copy.deepcopy(base)
 for h in case['holds']:
  if name=='nan_duration':h['duration_measured_s']=float('nan')
  if name=='inf_duration':h['duration_measured_s']=float('inf')
  if name=='missing_log_odds_hash':
   for key in ('probe_before','probe_after'):h[key].pop('log_odds_hash',None)
  if name=='different_probe_region':
   h['probe_after']['cells']=1
   h['probe_after']['region']={'begin':[0,0,0],'end':[1,1,1]}
 r[name]=judge.judge_o8(case)
 assert r[name]['pass']
usage={'read_view_bytes':1024,'cache_bytes':1024,'read_view_bytes_peak':1024**3,'cache_bytes_peak':1024,
 'read_view_alloc_bytes':2*1024**3,'cache_alloc_bytes':1024,
 'certification':'READ_VIEW_CACHE_ATTRIBUTED','attribution_complete':True,'unmarked_allocations':False,'uncovered_classes':[]}
resources=live._peak_resources_from_documents([{'resource_usage':usage}])
verdict=judge._resources(resources)
r['one_GiB_peak_with_small_current']={'adapter':resources,'judge':verdict}
assert verdict[1]=='pass'
bad=copy.deepcopy(usage);bad['read_view_bytes']=float('nan')
resources=live._peak_resources_from_documents([{'resource_usage':usage},{'resource_usage':bad}])
r['nan_resource_row_dropped']={'adapter':resources,'judge':judge._resources(resources)}
assert r['nan_resource_row_dropped']['judge'][1]=='pass'
(out/'RESULTS.json').write_text(json.dumps(r,indent=2)+'\n')
print(json.dumps({k:v.get('judge',v) for k,v in r.items() if isinstance(v,dict)},indent=2)[:1600])
