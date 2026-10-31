import json,sys
from pathlib import Path
sys.path.insert(0,'/tmp/elfin-occ-octomap-repair-2cm-g1/src/uld_occ_observer/test/benchmark')
import campaign_executor as e
import repair_s0_judge as j
out=Path(__file__).resolve().parent
window={'id':'normal_b00_r0','kind':'normal'}
results=[]
for name,extra in [('healthy_control',{}),('all_rejected',{'disposition':'rejected','reject_reason':'QUEUE_FULL'}),('all_timed_out',{'disposition':'unknown','receipt_received':False,'integrate_ms':100000.0})]:
 row={'disposition':'integrated','terminal_count':1,**extra}
 artifact=e._assemble_window(window,[dict(row) for _ in range(240)])
 result=j.judge_o7_normal(artifact)
 results.append({'case':name,'adapter_row':row,'rows':240,'expected_pass':name=='healthy_control','actual':result,'assembled_first_input':artifact['inputs'][0],'manufactured_queries':len(artifact['queries'])})
report={'scope':'offline assembler test only; no ROS or scored execution','results':results}
(out/'RESULTS.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
raise SystemExit(0 if all(x['actual']['pass']==x['expected_pass'] for x in results) else 1)
