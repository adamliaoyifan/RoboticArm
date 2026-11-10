from pathlib import Path
import json,hashlib
import numpy as np
from mcap.reader import make_reader
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
from scipy.spatial.transform import Rotation
from luggage_perception.ros_message_adapters import cloud_points_from_msg
from luggage_perception.cargo_occ_input import prepare_occ_observation,OccInputPolicy
from luggage_description.container_geometry import normalize_descriptor
p=Path('docs/status/evidence/occ_owner_b1/occ_container_crop_20261023');out=p/'recorded_exterior';out.mkdir(exist_ok=True)
case=json.loads(Path('/tmp/elfin-grid-1cm-g1/docs/status/evidence/occ_owner_b1/grid_1cm_s1_20261023/measured_five_boxes.json').read_text())
hull=normalize_descriptor(case['geometry_descriptor'])
uri='/home/adamliao/work/robotarm_bags/20260916_212503_floor_box_pick/bags/pick/pick_0.mcap'
rows=[];buffer=Buffer(cache_time=Duration(seconds=600))
with open(uri,'rb') as stream:
 reader=make_reader(stream);clouds=[]
 for _,channel,rec in reader.iter_messages(topics=['/luggage/semantic/cargo_points']):
  clouds.append((deserialize_message(rec.data,PointCloud2),rec.log_time))
  if len(clouds)==20:break
 for _,channel,rec in reader.iter_messages(topics=['/tf_static']):
  for t in deserialize_message(rec.data,TFMessage).transforms:buffer.set_transform_static(t,'recorded_bag')
 for _,channel,rec in reader.iter_messages(topics=['/tf'],end_time=clouds[-1][1]+2000000000):
  for t in deserialize_message(rec.data,TFMessage).transforms:buffer.set_transform(t,'recorded_bag')
 for i,(msg,logtime) in enumerate(clouds):
  stamp=msg.header.stamp;raw=bytes(msg.data);points=cloud_points_from_msg(msg)
  tf=buffer.lookup_transform('container_link',msg.header.frame_id,Time.from_msg(stamp)).transform
  q=tf.rotation;v=tf.translation;matrix=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();translation=np.array([v.x,v.y,v.z])
  container=points@matrix.T+translation
  keep=container.copy();container.setflags(write=False)
  prepared=prepare_occ_observation(container,translation,hull,.01,
     OccInputPolicy(hull.floor_z+.05,tuple(case['center_base']),case['yaw']))
  hull_only=prepare_occ_observation(container,translation,hull,.01,
     OccInputPolicy(None,tuple(case['center_base']),case['yaw']))
  assert bytes(msg.data)==raw and np.array_equal(container,keep)
  np.savez_compressed(out/('frame_%02d.npz'%i),points_camera=points,points_container=container,
       rotation=matrix,translation=translation,hit_points=prepared.hit_points)
  rows.append(dict(index=i,stamp_ns=stamp.sec*1000000000+stamp.nanosec,frame=msg.header.frame_id,
     raw_sha256=hashlib.sha256(raw).hexdigest(),counts=dict(prepared.counts),
     hull_only_counts=dict(hull_only.counts),
     minimum_container=container.min(axis=0).tolist(),maximum_container=container.max(axis=0).tolist(),
     shared_input_unchanged=True))
result=dict(source=uri,rows=rows,geometry_descriptor=case['geometry_descriptor'],
 interpretation='recorded pick cargo points with recorded TF; current configured hull used diagnostically, not historical hull certification',
 crop_scene='exterior only; boundary-crossing and full real top/suction replay not evaluated',
 pass_=all(r['shared_input_unchanged'] and r['counts']['inside_hull']==0 for r in rows))
(out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(dict(frames=len(rows),pass_=result['pass_'],first=rows[0])))
