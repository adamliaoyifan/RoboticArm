"""Finite source behavior witnesses; synthetic geometry, no ROS/robot calls."""
import json
import math
from pathlib import Path
import time
import traceback
from luggage_packing.placement_solver import generate_candidates, solve_placement, _same_layer_reason, aabb_overlap
OUT=Path(__file__).resolve().parent

def surface(nx=20,ny=10,res=.02):
 return dict(resolution=res,nx=nx,ny=ny,inner_size=[nx*res,ny*res,1.],floor_z=0.,center_base=[0.,0.,.5],yaw=0.,height=[[0.]*ny for _ in range(nx)],state=[['free']*ny for _ in range(nx)],confidence=[['sensor']*ny for _ in range(nx)])

results=[]
def record(name,inputs,observations,expected):
 row=dict(case=name,inputs=inputs,observations=observations,expected=expected,passed=bool(expected))
 results.append(row);(OUT/'probes.json').write_text(json.dumps(results,indent=2)+'\n')

started=time.monotonic()
try:
 sm=surface();size=[.15,.30,.10]
 default=generate_candidates(sm,size,params={'top_n':8})
 explicit=generate_candidates(sm,size,allowed_yaws=[0.,math.pi/2],params={'top_n':8})
 n0=sum(c['feasible'] for c in default);n1=sum(c['feasible'] for c in explicit)
 record('quarter_turn_required',{'map':sm,'size':size},dict(default_feasible=n0,explicit_yaws_feasible=n1,explicit_headings=sorted(set(c['heading_theta'] for c in explicit if c['feasible']))),n0==0 and n1>0)

 candidate={'center_container':[0.,0.,.05],'size':[.1,.1,.1],'box_yaw':0.}
 neighbour=(.05,-.05,0.,.15,.05,.1)
 legacy=_same_layer_reason(candidate,[neighbour],0.,.05,.05)
 zero=_same_layer_reason(candidate,[neighbour],0.,0.,.05)
 penetrates=aabb_overlap((-.05,-.05,0.,.05,.05,.1),neighbour)
 record('nominal_contact_and_gap',{'candidate':candidate,'neighbour':neighbour},dict(default_gap_reason=legacy,zero_gap_reason=zero,positive_volume_overlap=penetrates),legacy=='same_layer_clearance' and zero is None and not penetrates)

 sm=surface(nx=12,ny=12);size=[.08,.08,.1]
 def annotate(candidates):
  # Explicit hypothetical atlas classification, same fixed rule across top_n.
  # First retained physical position is unavailable; another position is clear.
  candidates=[dict(c) for c in candidates]
  for c in candidates:
   if c['candidate_id']=='c000_008_f0_h0':c.update(feasible=False,reason='atlas_unreachable')
  return candidates,{'source':'synthetic fixed classification; not deployed atlas'}
 initial=generate_candidates(sm,size,params={'top_n':2})
 blocked=initial[0]['candidate_id']
 def fixed_annotate(candidates):
  copied=[dict(c) for c in candidates]
  for c in copied:
   if c['candidate_id']==blocked:c.update(feasible=False,reason='atlas_unreachable')
  return copied,{'blocked_id':blocked}
 one=solve_placement(sm,size,params={'top_n':1},atlas_annotate=fixed_annotate)
 two=solve_placement(sm,size,params={'top_n':2},atlas_annotate=fixed_annotate)
 record('retention_precedes_atlas',{'map':sm,'size':size,'blocked_id':blocked},dict(top1_success=one['success'],top1_reason=one['reason_code'],top2_success=two['success'],top2_selected=two['preferred_candidate']['candidate_id'] if two['preferred_candidate'] else None),not one['success'] and two['success'])
except Exception:
 (OUT/'probe_failure.txt').write_text(traceback.format_exc());raise
finally:
 (OUT/'probe_summary.json').write_text(json.dumps(dict(cases=len(results),passes=sum(r['passed'] for r in results),elapsed_s=time.monotonic()-started,hardware=False),indent=2)+'\n')
print(json.dumps({'cases':len(results),'passes':sum(r['passed'] for r in results),'observations':[{r['case']:r['observations']} for r in results]}))
if not all(r['passed'] for r in results):raise SystemExit(1)
