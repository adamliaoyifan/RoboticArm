from pathlib import Path
import json,math,re
from rclpy.serialization import deserialize_message
from luggage_msgs.srv import SelectPlacePathV2
from luggage_gazebo.pose_text import PoseTextDecoder
out=Path('/tmp/elfin-master-pack-baseline-g3/docs/status/evidence/occ_owner_b1/release_xy_only_20261023/live_01')
suite=json.loads((out/'driver_out/suite.json').read_text())
log=(out/'driver.log').read_text()
metrics={'product_revision':json.loads((out/'manifest.json').read_text())['revision'],'suite':suite,'suite_pass':False,'suites_run':1,'not_attempted_boxes':2,'release_gate_pass_observed':True,'certified_leg_count':8,'executed_leg_count_observed':8,'home_state_entered_but_return_observe_failed':True,'first_named_runtime_failure':'PEER_LEASE_EXPIRED placement_planner','exact_live_release_check_inputs_durable':False,'selections':[]}
for p in sorted((out/'selection_boundary').glob('*/response.cdr')):
 response=deserialize_message(p.read_bytes(),SelectPlacePathV2.Response);rows=json.loads(response.rows_json)
 (p.parent/'response_rows.json').write_text(json.dumps(rows,indent=2)+'\n')
 metrics['selections'].append({'request':p.parent.name,'success':response.success,'slot_index':response.slot_index,'stage_timings':json.loads(response.stage_timings_json),'selected_topologies':[r.get('selected_topology') for r in rows if r.get('slot_index')==response.slot_index and r.get('reason')=='OK']})
for folder in (out/'release_capture').iterdir():
 decoder=PoseTextDecoder(); first=None; last=None; count=0
 for line in (folder/'poses.raw').read_text().splitlines():
  for sample in decoder.feed_line(line):
   if sample.model==folder.name:first=first or sample;last=sample;count+=1
 decoder.finish()
 (folder/'offline_replay.json').write_text(json.dumps({'model':folder.name,'target_samples':count,'first':first.__dict__ if first else None,'last':last.__dict__ if last else None,'decoder':decoder.diagnostics(),'scope':'Independent raw recorder; byte cap truncates last block. Not the exact sample used by release check.'},indent=2)+'\n')
 metrics['raw_target_samples']=count
metrics['capture_complete']=False
metrics['missing']=['trial.json and its in-memory release_kinematic evidence: exception in _home_arm precedes _dump_place','failure driver joint ring and replay manifest normally emitted by _dump_place','raw runtime identity heartbeat sequence for exact lease replay']
metrics['plugin_capture']=json.loads((out/'joint_capture/manifest_recorder.json').read_text())
(out/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
(out/'exception_boundary.json').write_text(json.dumps({'derived_from':'driver.log plus frozen source call order; not a raw callback trace','first_rejection_wall_s':1792725258.414044487,'first_reported_peer':'placement_planner','first_reported_lease_age_s':2.003704871982336,'call_chain':['run_place_from_carry','_home_arm','goto_observe','_goto_named_pose_v2','typed_driver_adapter.plan_motion','runtime.require_graph_ready'],'schema':'observed_exception_boundary/1','capture_complete':False,'replay_possible':False,'missing':metrics['missing']},indent=2)+'\n')
print(json.dumps({k:metrics[k] for k in ['first_named_runtime_failure','raw_target_samples','selections','missing']},indent=2))
