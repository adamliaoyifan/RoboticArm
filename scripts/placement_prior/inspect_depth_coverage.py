"""Offline exact-stamp depth/TF coverage inspection; never publishes ROS data."""
import argparse
import json
from pathlib import Path
import sqlite3

import numpy as np
import yaml
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
from scipy.spatial.transform import Rotation
from luggage_description.container_geometry import descriptor_from_scene_config
from luggage_perception.depth_view_rays import admit_depth_ray


def inspect(run, scene_config, resolution):
    if not np.isfinite(resolution) or resolution <= 0:
        raise ValueError('resolution must be positive and finite')
    geometry = descriptor_from_scene_config(yaml.safe_load(scene_config.read_text()))
    buffer = Buffer(cache_time=Duration(seconds=300))
    database = sqlite3.connect('file:' + str(run / 'bag/bag_0.db3') + '?mode=ro', uri=True)
    query = ('SELECT topics.name, messages.data FROM messages JOIN topics '
             'ON messages.topic_id=topics.id WHERE topics.name IN '
             "('/tf','/tf_static') ORDER BY messages.timestamp")
    for topic, data in database.execute(query):
        message = deserialize_message(data, TFMessage)
        for transform in message.transforms:
            setter = buffer.set_transform_static if topic == '/tf_static' else buffer.set_transform
            setter(transform, 'recorded_bag')
    database.close()
    boundary = run / 'cycle/depth_bootstrap_boundaries'
    results = []
    for row in map(json.loads, (boundary / 'boundaries.jsonl').read_text().splitlines()):
        if row['boundary'] != 'optical_cloud':
            continue
        cloud = deserialize_message((boundary / row['file']).read_bytes(), PointCloud2)
        transform = buffer.lookup_transform('container_link', cloud.header.frame_id,
                                            Time.from_msg(cloud.header.stamp))
        translation = transform.transform.translation
        quaternion = transform.transform.rotation
        origin = np.array([translation.x, translation.y, translation.z])
        rotation = Rotation.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        points = np.array([tuple(p) for p in read_points(cloud, field_names=('x','y','z'), skip_nans=True)])
        points = rotation.apply(points) + origin
        admitted = [admit_depth_ray(geometry, origin, point, np.sqrt(3) * resolution) for point in points]
        accepted = [point for point, ray in zip(points, admitted) if ray is not None]
        results.append(dict(file=row['file'], stamp_ns=Time.from_msg(cloud.header.stamp).nanoseconds,
                            camera_container=origin.tolist(), optical_forward_container=rotation.apply([0,0,1]).tolist(),
                            points=len(points), admitted=len(accepted),
                            obstacle_returns=sum(ray is not None and ray.obstacle is not None for ray in admitted),
                            endpoint_percentiles=np.percentile(points,[0,5,50,95,100],axis=0).tolist(),
                            accepted_bounds=None if not accepted else [np.min(accepted,axis=0).tolist(),np.max(accepted,axis=0).tolist()]))
    return dict(frame='container_link', resolution_m=resolution,
                terminal_band_m=float(np.sqrt(3)*resolution), views=results)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--scene-config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--resolution', type=float, required=True,
                        help='Recorded mapper voxel resolution in metres')
    args = parser.parse_args()
    result = inspect(args.run, args.scene_config, args.resolution)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))
