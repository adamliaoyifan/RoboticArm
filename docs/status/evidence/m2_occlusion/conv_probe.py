import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time

rclpy.init()
node = Node("conv_probe")
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
got = {}
def cb(msg):
    got['msg'] = msg
sub = node.create_subscription(PointCloud2, "/camera/depth/points", cb, qos)
buf = Buffer()
TransformListener(buf, node, spin_thread=False)

end = time.time() + 15
while time.time() < end and 'msg' not in got:
    rclpy.spin_once(node, timeout_sec=0.1)
msg = got.get('msg')
if msg is None:
    print("NO CLOUD")
    os._exit(0)
print("cloud frame_id:", msg.header.frame_id, "points:", msg.width*msg.height)
arr = np.array(list(pc2.read_points(msg, field_names=("x","y","z"), skip_nans=True)))
if arr.dtype.names:
    pts = np.stack([arr['x'],arr['y'],arr['z']],axis=1).astype(np.float64)
else:
    pts = arr.astype(np.float64)
print("camera-frame stats: x %.2f..%.2f  y %.2f..%.2f  z %.2f..%.2f" % (
    pts[:,0].min(), pts[:,0].max(), pts[:,1].min(), pts[:,1].max(), pts[:,2].min(), pts[:,2].max()))

for frame in ("camera_depth_optical_frame", "camera_link"):
    try:
        t = buf.lookup_transform("world", frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
    except Exception as e:
        print(frame, "TF FAIL", e); continue
    tr, rot = t.transform.translation, t.transform.rotation
    qx,qy,qz,qw = rot.x, rot.y, rot.z, rot.w
    R = np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]])
    w = pts.dot(R.T) + np.array([tr.x, tr.y, tr.z])
    print("via %s: z %.2f..%.2f median %.2f | pts z<1.2: %d  z in[0.8,1.2]: %d  z>1.7: %d" % (
        frame, w[:,2].min(), w[:,2].max(), np.median(w[:,2]),
        int((w[:,2]<1.2).sum()), int(((w[:,2]>0.8)&(w[:,2]<1.2)).sum()), int((w[:,2]>1.7).sum())))
    print("   x %.2f..%.2f y %.2f..%.2f" % (w[:,0].min(), w[:,0].max(), w[:,1].min(), w[:,1].max()))
os._exit(0)
