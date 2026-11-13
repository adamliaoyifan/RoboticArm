"""Eval-only actual SDF collision corners in each box-own loaded interval; not observable-lid reference."""
import gzip,itertools,json,re,sys,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from luggage_msgs.srv import VacuumCommandV2,SpawnNextBox
from luggage_msgs.action import ExecuteJointOperationV2
root=Path(sys.argv[1]);cycle=root/'cycle'
def matrix(p,q):
 m=np.eye(4);m[:3,:3]=Rotation.from_quat(q).as_matrix();m[:3,3]=p;return m
def read(folder,row,typ):
 raw=(folder/row['file']).read_bytes();return deserialize_message(gzip.decompress(raw) if row['file'].endswith('.gz') else raw,typ)
joints={j.find('child').get('link'):j for j in ET.parse(root/'run/robot.urdf').findall('joint')};link_tool=np.eye(4);child='suction_contact_frame'
while child!='elfin_link6':
 j=joints[child];assert j.get('type')=='fixed';origin=j.find('origin');xyz=[float(x) for x in origin.get('xyz','0 0 0').split()];rpy=[float(x) for x in origin.get('rpy','0 0 0').split()];link_tool=matrix(xyz,Rotation.from_euler('xyz',rpy).as_quat())@link_tool;child=j.find('parent').get('link')
spawns={}
for row in json.loads((cycle/'manifest.json').read_text())['records']:
 if row['file'].endswith('physics_spawn_response.cdr'):
  s=read(cycle,row,SpawnNextBox.Response);spawns[s.box.id]=np.array([s.box.width,s.box.depth,s.box.height])
results=[]
for manifest in sorted((root/'run/execution').glob('*/manifest.json')):
 records=json.loads(manifest.read_text())['records'];g=next((r for r in records if 'execute_goal' in r['file']),None);a=next((r for r in records if 'confirm_attachment_response' in r['file']),None)
 if not g or not a:continue
 stages=[json.loads(gzip.decompress((manifest.parent/r['file']).read_bytes())) for r in records if r['file'].endswith('stage.json.gz')]
 loaded_start=min((r['ros_stamp_ns'] for r in stages if r['stage']=='execute_leg' and r['index']==2),default=None)
 loaded_end=min((r['ros_stamp_ns'] for r in stages if r['stage']=='confirm_release'),default=None)
 if loaded_start is None:continue
 goal=read(manifest.parent,g,ExecuteJointOperationV2.Goal);geometry=read(manifest.parent,a,VacuumCommandV2.Response).conservative_geometry;instance=goal.top_domain.instance_id;dimensions=spawns[instance];collision_corners=[];collision_details=[]
 sdf=ET.parse(manifest.parent/'released_physical.sdf').getroot()
 for collision in sdf.findall('model/link/collision'):
  box=collision.find('geometry/box/size');assert box is not None,'ACTUAL_NON_BOX_COLLISION_REQUIRES_VERTEX_READER'
  size=np.array([float(x) for x in box.text.split()]);origin=collision.find('pose');values=[float(x) for x in origin.text.split()] if origin is not None else [0.]*6;local=matrix(values[:3],Rotation.from_euler('xyz',values[3:]).as_quat());vertices=np.array(list(itertools.product(*[(-x/2,x/2) for x in size])));collision_corners.extend((np.c_[vertices,np.ones(8)]@local.T)[:,:3]);collision_details.append(dict(name=collision.get('name'),size=size.tolist(),pose=values))
 corners=np.array(collision_corners);dimensions=np.ptp(corners,axis=0);tool_envelope=matrix(geometry.tool_conservative_translation,geometry.tool_conservative_quaternion_xyzw);half=np.array(geometry.conservative_dimensions_wdh)/2;checks=[]
 for row in json.loads((root/'reference/manifest.json').read_text())['records']:
  if not row.get('file') or not any(x in row.get('phase','') for x in ('EXECUTING_2','EXECUTING_3','CANCELLING','CARRY_FAULT','WAIT_RELEASE')):continue
  text=gzip.decompress((root/'reference'/row['file']).read_bytes()).decode();poses={}
  stamp=re.search(r'stamp \{\s*sec: (\d+)(?:\s*nsec: (\d+))?',text)
  if stamp is None:continue
  stamp_ns=int(stamp[1])*10**9+int(stamp[2] or 0)
  if stamp_ns<loaded_start or (loaded_end is not None and stamp_ns>=loaded_end):continue
  for block in text.split('\npose {\n')[1:]:
   n=re.search(r'name: "([^"]+)"',block)
   if n is None or n[1] not in [instance,'elfin_link6']:continue
   def values(field,names):
    b=re.search(field+r' \{([^}]*)\}',block,re.S).group(1);return [float(m[1]) if (m:=re.search(r'\b'+key+r': ([^\s]+)',b)) else 0. for key in names]
   poses[n[1]]=matrix(values('position','xyz'),values('orientation','xyzw'))
  if set(poses)!={instance,'elfin_link6'}:continue
  envelope=poses['elfin_link6']@link_tool@tool_envelope;local=(np.c_[corners,np.ones(len(corners))]@(np.linalg.inv(envelope)@poses[instance]).T)[:,:3];excess=np.max(abs(local)-half,axis=0);relation=np.linalg.inv(poses['elfin_link6']@link_tool)@poses[instance];checks.append(dict(reference=row['file'],wall_started=row['wall_started'],maximum_corner_excess_by_axis_m=excess.tolist(),all_corners_inside=bool(np.all(excess<=1e-6)),tool_box_translation=relation[:3,3].tolist(),tool_box_quaternion_xyzw=Rotation.from_matrix(relation[:3,:3]).as_quat().tolist()))
 results.append(dict(instance=instance,loaded_start_ros_ns=loaded_start,loaded_end_ros_ns=loaded_end,actual_collision_aabb_dimensions=dimensions.tolist(),actual_collisions=collision_details,observable_reference_dimensions=spawns[instance].tolist(),measured_dimensions=list(geometry.measured_dimensions_wdh),envelope_dimensions=list(geometry.conservative_dimensions_wdh),checks=checks))
result=dict(scope=__doc__,boxes=results);(root/'payload_containment_eval_v5.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps([dict(instance=b['instance'],samples=len(b['checks']),outside=sum(not x['all_corners_inside'] for x in b['checks']),max_excess=np.max([x['maximum_corner_excess_by_axis_m'] for x in b['checks']],axis=0).tolist() if b['checks'] else None,first=b['checks'][:1]) for b in results],indent=2))
