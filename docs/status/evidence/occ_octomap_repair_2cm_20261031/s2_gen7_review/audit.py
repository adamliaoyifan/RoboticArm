import sys,json,copy,types
from pathlib import Path
from types import SimpleNamespace as NS
root=Path('/tmp/elfin-occ-octomap-repair-2cm-g3')
sys.path.insert(0,str(root/'src/uld_occ_observer/test/benchmark'))
import repair_s0_judge as judge
import live_schedule as live
results={'candidate':'5643039bca744de2abb5ac5837921170bfe93c0a','kind':'offline injected counterexamples'}
base=json.loads(Path('docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_gen7/stage_rehearsal/o8/runs.json').read_text())
results['healthy']=judge.judge_o8(base)
for name in ['zero_expected','wrong_identity_disposition','duplicate_receipts','health_same_timestamp']:
 case=copy.deepcopy(base)
 for run in case['runs']:
  if name=='zero_expected':
   run['saturation_frames_expected']=0
   run['saturation_frames_verified']=0
   run['saturation_receipts']=[]
  if name=='wrong_identity_disposition':
   for receipt in run['saturation_receipts']:
    receipt['observation_id']='same-wrong-id'
    receipt['disposition']='unknown'
  if name=='duplicate_receipts':
   for receipt in run['saturation_receipts']: receipt['events']=2
  if name=='health_same_timestamp': run['node_health_times_s']=[60.0]*30
 results[name]=judge.judge_o8(case)
 assert results[name]['pass']
# Exercise actual hold probe and hold adapter with an error/empty Surface response.
module=types.ModuleType('uld_occ_observer.srv')
for name in ['GetSnapshot','ReadSurface','ReleaseSnapshot']:
 setattr(module,name,type(name,(),{'Request':NS}))
sys.modules['uld_occ_observer.srv']=module
class Session:
 snapshot='snapshot'
 node=NS(create_client=lambda service,path:path)
 def wait_service(self,*a,**kw): pass
 def call(self,client,request,timeout):
  if client=='snapshot': return NS(status='OK',handle='1:1',identity={'last_integrated_stamp_ns':100})
  if client.endswith('release_snapshot'): return NS(status='OK')
  return NS(status='INTERNAL_ERROR',done=False,x=[],y=[],active_count=[],known_count=[],has_occupied=[],top_z=[])
adapter=object.__new__(live.LiveScheduleAdapter)
run=NS(session=Session(),out=Path(__file__).parent/'injected_hold')
adapter._stability={'run':run,'stamps':[]}
adapter.plan_o8={'no_input_s':0}
adapter._await_terminal=lambda *a,**kw: None
row=adapter.hold('audit','no_input',0)
case=copy.deepcopy(base)
case['holds']=[row if h['kind']=='no_input' else h for h in case['holds']]
results['error_empty_surface_hold']={'row':row,'judge':judge.judge_o8(case)}
assert row['probability_unchanged'] is True
assert results['error_empty_surface_hold']['judge']['pass']
Path(__file__).with_name('RESULTS.json').write_text(json.dumps(results,indent=2)+'\n')
print(json.dumps({k:v.get('judge',v).get('pass') for k,v in results.items() if isinstance(v,dict)},indent=2))
