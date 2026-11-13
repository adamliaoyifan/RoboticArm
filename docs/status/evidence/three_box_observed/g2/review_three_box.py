"""Offline integration audit over retained typed online records and separate physics.

No simulator reference is returned to perception, placement, O, or a planner.
"""
import gzip,json,re,sys,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from luggage_msgs.action import ExecuteJointOperationV2
from luggage_msgs.srv import SpawnNextBox,VacuumCommandV2
from std_msgs.msg import String
from luggage_description.scene_tf_config_utils import load_scene_tf_config,world_transform
root=Path(sys.argv[1]);cycle=root/'cycle';driver=json.loads((cycle/'manifest.json').read_text());launch=json.loads((root/'run/launch.json').read_text())
def read(folder,row,typ):
 raw=(folder/row['file']).read_bytes();raw=gzip.decompress(raw) if row['file'].endswith('.gz') else raw
 return deserialize_message(raw,typ)
def mat(p,q):
 m=np.eye(4);m[:3,:3]=Rotation.from_quat(q).as_matrix();m[:3,3]=p;return m
def pose(msg):return mat([msg.position.x,msg.position.y,msg.position.z],[msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w])
xyz,rpy=world_transform(load_scene_tf_config(launch['resolved']['scene_tf_config']),'container_link');world_container=mat(xyz,Rotation.from_euler('xyz',rpy).as_quat())
# Frames are explicit: the transforms here are container-absolute; size is height.
from luggage_mapping.observer import ObserverConfiguration
cfg=ObserverConfiguration.load(launch['map_owner']['geometry_file'],launch['map_owner']['config_file']);descriptor=cfg.descriptor();floor=descriptor['floor_z'];resolution=cfg.resolution_m
spawns={};release={}
for row in driver['records']:
 if row['file'].endswith('physics_spawn_response.cdr'):
  msg=read(cycle,row,SpawnNextBox.Response);spawns[msg.box.id]=dict(dimensions=[msg.box.width,msg.box.depth,msg.box.height],reference=row['file'])
 elif 'physics_release_report' in row['file']:
  msg=read(cycle,row,String);v=json.loads(msg.data);release[(v.get('model'),v.get('generation'))]=v
pose_rows=json.loads((root/'reference/manifest.json').read_text())['records'];pose_samples=[]
for row in pose_rows:
 if not row.get('file'):continue
 text=gzip.decompress((root/'reference'/row['file']).read_bytes()).decode();poses={}
 for block in text.split('\npose {\n')[1:]:
  n=re.search(r'name: "([^"]+)"',block)
  if n is None:continue
  def values(field,names):
   b=re.search(field+r' \{([^}]*)\}',block,re.S).group(1)
   return [float(m[1]) if (m:=re.search(r'\b'+key+r': ([^\s]+)',b)) else 0. for key in names]
  poses[n[1]]=mat(values('position','xyz'),values('orientation','xyzw'))
 pose_samples.append((row,poses))
contacts={}
p=root/'reference/contacts_manifest.json'
if p.exists():
 for row in json.loads(p.read_text())['records']:
  text=gzip.decompress((root/'reference'/row['file']).read_bytes()).decode();pairs=set()
  for a,b in re.findall(r'collision1 \{(.*?)\}\s*collision2 \{(.*?)\}',text,re.S):
   pairs.add(tuple(sorted(re.search(r'name: "([^"]+)"',x)[1] for x in (a,b))))
  contacts[row['execution']]=dict(file=row['file'],pairs=sorted(pairs),bytes=row['bytes'])
boxes=[]
for manifest in sorted((root/'run/execution').glob('*/manifest.json')):
 folder=manifest.parent;records=json.loads(manifest.read_text())['records'];goal=None;result=None;views=[];geometry=None
 for row in records:
  name=row['file']
  if 'execute_goal' in name:goal=read(folder,row,ExecuteJointOperationV2.Goal)
  elif 'execute_result' in name:result=read(folder,row,ExecuteJointOperationV2.Result)
  elif 'confirm_attachment_response' in name:geometry=read(folder,row,VacuumCommandV2.Response).conservative_geometry
  elif 'explore_view_observation.json' in name:views.append(json.loads(gzip.decompress((folder/name).read_bytes())))
 if goal is None:continue
 plan=goal.plan;instance=goal.top_domain.instance_id;predicted=plan.predicted_attachment;target=np.linalg.inv(world_container)@pose(plan.world_from_settled_box)
 legs=[]
 for leg in plan.legs:
  points=leg.trajectory.trajectory.points;t=points[-1].time_from_start if points else None
  travel=np.abs(np.diff(np.array([point.positions for point in points]),axis=0)).sum(axis=0).tolist() if len(points)>1 else []
  legs.append(dict(name=leg.name,phase=leg.phase,points=len(points),duration_s=(t.sec+t.nanosec/1e9) if t else None,joint_travel_rad=travel))
  import csv
  with (folder/(leg.name+'_trajectory.csv')).open('w') as stream:
   writer=csv.writer(stream);writer.writerow(['time_from_start_s']+list(leg.trajectory.trajectory.joint_names))
   for point in points:writer.writerow([point.time_from_start.sec+point.time_from_start.nanosec/1e9]+list(point.positions))
 actual=[(r,poses[instance]) for r,poses in pose_samples if instance in poses]
 last=actual[-3:];physical=None
 if last:
  values=np.array([x[1][:3,3] for x in last]);r,m=last[-1];physical=dict(scope='simulator reference only',last_reference=r['file'],world_xyz=m[:3,3].tolist(),world_quaternion_xyzw=Rotation.from_matrix(m[:3,:3]).as_quat().tolist(),last_samples=len(last),position_span_m=np.ptp(values,axis=0).tolist(),sample_interval_s=last[-1][0]['wall_finished']-last[0][0]['wall_started'])
 rel=release.get((instance,plan.generation));dynamic=None
 if rel and rel.get('dynamic_sdf'):
  sdf=ET.fromstring(rel['dynamic_sdf']);dynamic=sdf.find('model/static').text=='false'
  (folder/'released_physical.sdf').write_text(rel['dynamic_sdf'])
 item=dict(execution=folder.name,instance_id=instance,generation=plan.generation,certificate_id=plan.certificate_id,plan_digest=plan.plan_digest,model_sha256=plan.model_sha256,target_xyz_container=target[:3,3].tolist(),target_bottom_z_container=float(target[2,3]-predicted.measured_dimensions_wdh[2]/2),target_bottom_above_floor_m=float(target[2,3]-predicted.measured_dimensions_wdh[2]/2-floor),measured_dimensions=list(predicted.measured_dimensions_wdh),measured_pick_offset_in_tool_m=list(predicted.tool_box_translation),legs=legs,observed_views=[dict(view=v['view'],source=v['source'],coverage={k:val for k,val in v['coverage'].items() if k!='missing_points_container'}) for v in views],actual_physics=physical,physical_spawn_reference=spawns.get(instance),release_dynamic=dynamic,settled_contact=contacts.get(folder.name))
 if result:item['result']=dict(success=result.success,outcome=result.outcome,reason=result.reason,motion_terminal=result.motion_terminal,may_have_payload=result.may_have_payload,epoch=result.final_map_source.epoch,map_revision=result.final_map_source.map_revision)
 boxes.append(item)
result=dict(scope=__doc__,driver_outcome=driver.get('outcome','running'),driver_completed_boxes=driver.get('completed_boxes',[]),floor_z_container=floor,resolution_m=resolution,boxes=boxes)
(root/'three_box_review.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(dict(outcome=result['driver_outcome'],boxes=[{k:b.get(k) for k in ['instance_id','target_bottom_above_floor_m','result','release_dynamic']} for b in boxes]),indent=2))
