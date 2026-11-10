from pathlib import Path
import json,hashlib,subprocess
from rclpy.serialization import deserialize_message
from luggage_msgs.srv import SelectPlacePathV2
root=Path('/tmp/elfin-master-pack-baseline-g3')
evidence=root/'docs/status/evidence/occ_owner_b1/release_format_fix_20261023';out=evidence/'live_01'
suite=json.loads((out/'driver_out/suite.json').read_text())
metrics={'product_revision':json.loads((out/'manifest.json').read_text())['revision'],'suite':suite,'suite_pass':suite['boxes_packed']==3 and suite['boxes_attempted']==3,'scored_suites':1,'boxes_not_attempted':3-suite['boxes_attempted'],'trials':[],'selections':[]}
for p in sorted((out/'driver_out/dumps').glob('*/trial.json')):
 d=json.loads(p.read_text()); extras=d.get('extras',{});replay=json.loads((p.parent/'replay/manifest.json').read_text())
 metrics['trials'].append({'case':p.parent.name,'fail_code':d.get('fail_code'),'place_state':d.get('place_state'),'picked':d.get('vac_attach'),'segments_certified':d.get('segments_planned'),'segments_executed':d.get('segments_succeeded'),'release':extras.get('release_kinematic'),'replay_boundary':replay,'segment_durations_s':[{'name':leg['name'],'seconds':leg.get('trajectory',{}).get('points',[{}])[-1].get('t')} for leg in extras.get('submitted_legs',[])]})
for p in sorted((out/'selection_boundary').glob('*/response.cdr')):
 response=deserialize_message(p.read_bytes(),SelectPlacePathV2.Response);rows=json.loads(response.rows_json)
 (p.parent/'response_rows.json').write_text(json.dumps(rows,indent=2)+'\n')
 metrics['selections'].append({'request':p.parent.name,'success':response.success,'slot_index':response.slot_index,'path_source':response.path_source,'stage_timings':json.loads(response.stage_timings_json),'selected_topologies':[r.get('selected_topology') for r in rows if r.get('slot_index')==response.slot_index and r.get('reason')=='OK'],'attempts':[attempt for r in rows for attempt in r.get('topology_attempts',[])]})
metrics['startup']=json.loads((out/'startup_preflight.json').read_text())
metrics['capture_limits']={'box1_pose_stream_byte_limited':True,'box1_pose_target_samples':827,'box1_exact_release_cache_pose7_missing':False,'box2_pose_target_samples':1,'box2_rejected_cache_stamp_ns':729582000000,'box2_later_raw_stamp_ns':729616000000,'box2_exact_rejected_cache_pose7_missing':True,'set_pose_request_response_application_timestamps_missing':True,'joint_recorder':json.loads((out/'joint_capture/manifest_recorder.json').read_text()),'contact_absence_proven':False}
(out/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
print(json.dumps({'attempted':suite['boxes_attempted'],'packed':suite['boxes_packed'],'termination':suite['termination_reason'],'wall_s':suite['wall_total_sec'],'selections':metrics['selections']},indent=2))
