import os
import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time

rclpy.init()
node = Node("tf_probe")
buf = Buffer()
TransformListener(buf, node, spin_thread=False)
end = time.time() + 5
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.1)

def rot_of(t):
    r = t.transform.rotation
    qx,qy,qz,qw = r.x, r.y, r.z, r.w
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]])

for frame in ("elfin_base_link","elfin_link6","suction_panel","suction_contact_frame",
              "eef_mount_adapter","camera_link","camera_depth_optical_frame"):
    try:
        t = buf.lookup_transform("world", frame, rclpy.time.Time(),
                                 timeout=rclpy.duration.Duration(seconds=0.5))
        tr = t.transform.translation
        R = rot_of(t)
        print("%-28s xyz=(%.3f, %.3f, %.3f)  +X=(%.2f,%.2f,%.2f)  +Z=(%.2f,%.2f,%.2f)" % (
            frame, tr.x, tr.y, tr.z, R[0,0],R[1,0],R[2,0], R[0,2],R[1,2],R[2,2]))
    except Exception as e:
        print(frame, "TF FAIL", e)
os._exit(0)
