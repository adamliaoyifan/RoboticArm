import json,sys
from pathlib import Path
sys.path.insert(0,'/tmp/elfin-occ-octomap-repair-2cm-g1/src/uld_occ_observer/test/benchmark')
import capture_replay as c
import campaign_entry as e
out=Path(__file__).resolve().parent
footer={'schema_version':1,'run_id':'no-work','phase':'capture','event_seq':1,'event':'trace_footer','mono_ns':1,'pid':1,'tid':1,'thread_cpu_ns':1,'request_id':None,'status':'0'}
p=out/'footer_only.jsonl';p.write_text(json.dumps(footer)+'\n')
results=[{'case':'footer_only_no_operation','expected':False,'actual':c.certify_trace(p)}]
for label,value in [('100000ms',100000.0),('NaN',float('nan')),('bool',True)]:
 doc={'integrate_ms':[value]*30,'resource_certification':'pass'}
 results.append({'case':'window_'+label,'expected':False,'input':doc,'actual':e.judge_recorded_window(doc)})
old=Path('/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_review')
controls={'previous_fabrication':c.replay_failure(old/'fabricated_failure.json',out/'unused'),'previous_drop999':c.certify_trace(old/'dropped_identity_free_trace.jsonl')}
report={'results':results,'previous_controls':controls,'scope':'offline only; no new O9 or official samples'}
(out/'RESULTS.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
raise SystemExit(1 if any(x['actual'].get('pass',x['actual'].get('capture_complete',False)) for x in results) else 0)
