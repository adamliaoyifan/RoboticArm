import json,sys,hashlib
from pathlib import Path
from collections import Counter
from rclpy.serialization import deserialize_message
from luggage_msgs.srv import SelectPlacePathV2
run=Path(sys.argv[1])
suite=json.loads((run/'driver_out/suite.json').read_text())
manifest=json.loads((run/'manifest.json').read_text())
selections=[]
for f in sorted((run/'selection_boundary').glob('*/response.cdr')):
 r=deserialize_message(f.read_bytes(),SelectPlacePathV2.Response)
 rows=json.loads(r.rows_json)
 (f.parent/'response_rows.json').write_text(json.dumps(rows,indent=2)+'\n')
 selected=[row for row in rows if row.get('reason')=='OK']
 attempts=[a for row in rows for a in row.get('topology_attempts',[])]
 assert all(not row.get('topology_attempts') or row['topology_attempts'][0]['topology']=='direct' for row in rows)
 for row in rows:
  if row.get('selected_topology')=='direct' and len(row.get('topology_attempts',[]))==1:
   assert not row.get('staging_endpoints')
 selections.append(dict(request=f.parent.name,success=r.success,motion_segments=len(r.motion_segments),
  stage_timings=json.loads(r.stage_timings_json),selected_topologies=[x.get('selected_topology') for x in selected],
  candidate_count=len([x for x in rows if 'slot_index' in x]),reason_histogram=dict(Counter(x.get('reason') for x in rows)),
  attempts=[dict(slot_index=row.get('slot_index'),**a) for row in rows for a in row.get('topology_attempts',[])],
  segment_durations_s=[dict(name=s.name,seconds=s.joint_trajectory.trajectory.points[-1].time_from_start.sec+s.joint_trajectory.trajectory.points[-1].time_from_start.nanosec/1e9) for s in r.motion_segments if s.joint_trajectory.trajectory.points]))
trials=[]
for f in sorted((run/'driver_out/dumps').glob('*/trial.json')):
 d=json.loads(f.read_text());rp=f.parent/'replay/manifest.json'
 trials.append(dict(case=f.parent.name,**{k:d.get(k) for k in ['fail_code','place_state','vac_attach','segments_planned','segments_succeeded','descend_fraction']},release_kinematic=d.get('extras',{}).get('release_kinematic'),replay=json.loads(rp.read_text()) if rp.exists() else None))
result=dict(revision=manifest['revision'],dirty=manifest['dirty'],boxes_attempted=suite['boxes_attempted'],boxes_packed=suite['boxes_packed'],termination=suite['termination_reason'],wall_s=suite['wall_total_sec'],cycle=suite['cycle_sec'],selections=selections,trials=trials,startup=json.loads((run/'startup_preflight.json').read_text()),suite_pass=suite['boxes_packed']==3 and suite['termination_reason']=='MAX_BOXES')
(run/'metrics.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:result[k] for k in ['revision','boxes_attempted','boxes_packed','termination','wall_s','suite_pass']},indent=2))
