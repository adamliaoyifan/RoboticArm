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


def inspect(run, scene_config, resolution, *, volume_output=None, max_raycast_points=2000):
    if not np.isfinite(resolution) or resolution <= 0:
        raise ValueError('resolution must be positive and finite')
    geometry = descriptor_from_scene_config(yaml.safe_load(scene_config.read_text()))
    mapper = None
    if volume_output is not None:
        from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
        mapper = CargoVolumeMapper(
            (geometry.length, geometry.width, geometry.height),
            (0., 0., (geometry.floor_z+geometry.ceiling_z)/2), 0., resolution,
            geometry_descriptor=geometry.descriptor(), max_raycast_points=max_raycast_points)
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
        if mapper is not None:
            detail = mapper.integrate_depth_view(points, origin)
            results[-1]['integration'] = {key:value for key,value in detail.items() if key!='observed_points'}
            results[-1]['stats'] = mapper.stats()
    report = dict(frame='container_link', resolution_m=resolution,
                  terminal_band_m=float(np.sqrt(3)*resolution), views=results)
    if mapper is not None:
        # x-fastest storage, then y, then z, matching CargoVolumeMapper._index.
        shape = (mapper.nz, mapper.ny, mapper.nx)
        states = np.asarray(mapper._grid, dtype=np.uint8).reshape(shape)
        active = np.asarray(mapper._active, dtype=bool).reshape(shape)
        np.savez_compressed(volume_output, states=states, active=active,
                            origin_container=[-geometry.half_x,-geometry.half_y,geometry.floor_z],
                            resolution_m=resolution)
        report['replay_volume'] = dict(file=str(volume_output), max_raycast_points=max_raycast_points,
            state_encoding={'unknown':0,'free':1,'occupied':2},
            unknown_by_z=[int(np.count_nonzero(active[z] & (states[z]==0))) for z in range(mapper.nz)],
            free_by_z=[int(np.count_nonzero(active[z] & (states[z]==1))) for z in range(mapper.nz)],
            surface=mapper.surface_map_2d())
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--scene-config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--resolution', type=float, required=True,
                        help='Recorded mapper voxel resolution in metres')
    parser.add_argument('--replay-volume', type=Path,
                        help='Save reconstructed owner grid; diagnostic only, never an online map')
    parser.add_argument('--max-raycast-points', type=int, default=2000)
    args = parser.parse_args()
    result = inspect(args.run, args.scene_config, args.resolution,
                     volume_output=args.replay_volume, max_raycast_points=args.max_raycast_points)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))
