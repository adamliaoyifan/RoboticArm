import json
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
p=Path('/tmp/elfin-occ-crop-g1/docs/status/evidence/occ_owner_b1/occ_container_crop_20261023')
uri='/home/adamliao/work/robotarm_bags/20260916_212503_floor_box_pick/bags/pick'
from mcap.reader import make_reader
frames=[];clouds=[]
with open(uri+'/pick_0.mcap','rb') as stream:
 r=make_reader(stream)
 for _,channel,record in r.iter_messages(topics=['/tf_static']):
  msg=deserialize_message(record.data,TFMessage)
  for tf in msg.transforms:
   frames.append(dict(parent=tf.header.frame_id,child=tf.child_frame_id,xyz=[tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z]))
 for _,channel,record in r.iter_messages(topics=['/luggage/semantic/cargo_points']):
  msg=deserialize_message(record.data,PointCloud2)
  clouds.append(dict(frame=msg.header.frame_id,stamp=[msg.header.stamp.sec,msg.header.stamp.nanosec],n=msg.width*msg.height))
  if len(clouds)>=20:break
record=dict(uri=uri,static_frames=frames,cloud_headers=clouds,
            verdict='not accepted as deployment replay: no frozen matching ULD descriptor/scene profile or requested two-scene coverage',
            no_motion=True)
(p/'historical_bag_inspection.json').write_text(json.dumps(record,indent=2))
print(json.dumps(record))
