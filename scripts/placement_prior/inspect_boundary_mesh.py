#!/usr/bin/env python3
"""Compare captured container mesh intersections with its semantic usable hull.

Read-only geometric diagnostic, not sensed free space or motion authorization.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def pose_matrix(pose):
    q = pose.orientation
    result = np.eye(4)
    result[:3,:3] = Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
    result[:3,3] = [pose.position.x,pose.position.y,pose.position.z]
    return result


def intersections(triangles, origin, direction):
    """Positive ray intersections using triangle barycentric coordinates."""
    edge1 = triangles[:,1] - triangles[:,0]
    edge2 = triangles[:,2] - triangles[:,0]
    cross = np.cross(direction, edge2)
    determinant = np.einsum('ij,ij->i', edge1, cross)
    mask = np.abs(determinant) > 1e-10
    inverse = np.zeros_like(determinant)
    inverse[mask] = 1. / determinant[mask]
    delta = origin - triangles[:,0]
    u = np.einsum('ij,ij->i', delta, cross) * inverse
    cross2 = np.cross(delta, edge1)
    v = np.dot(cross2, direction) * inverse
    distance = np.einsum('ij,ij->i', edge2, cross2) * inverse
    mask &= (u >= -1e-9) & (v >= -1e-9) & (u+v <= 1.+1e-9) & (distance > 1e-9)
    return np.sort(distance[mask])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path, required=True, help='Full PlanningScene CDR')
    parser.add_argument('--scene-config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    from rclpy.serialization import deserialize_message
    from moveit_msgs.msg import PlanningScene
    from luggage_description.scene_tf_config_utils import load_scene_tf_config, world_transform
    from luggage_description.container_geometry import descriptor_from_scene_config
    config = load_scene_tf_config(str(args.scene_config))
    hull = descriptor_from_scene_config(config)
    xyz,rpy = world_transform(config,'container_link')
    world_container = np.eye(4)
    world_container[:3,:3] = Rotation.from_euler('xyz',rpy).as_matrix()
    world_container[:3,3] = xyz
    scene = deserialize_message(args.scene.read_bytes(),PlanningScene)
    obj = next(o for o in scene.world.collision_objects if o.id=='airport_container_real')
    if obj.header.frame_id != 'world' or len(obj.meshes)!=len(obj.mesh_poses) or not obj.meshes:
        raise ValueError('captured world-frame container mesh required')
    triangles = []
    for mesh,pose in zip(obj.meshes,obj.mesh_poses):
        transform = np.linalg.inv(world_container) @ pose_matrix(obj.pose) @ pose_matrix(pose)
        vertices = np.array([[v.x,v.y,v.z] for v in mesh.vertices])
        vertices = vertices @ transform[:3,:3].T + transform[:3,3]
        triangles.append(vertices[np.array([t.vertex_indices for t in mesh.triangles])])
    triangles = np.concatenate(triangles)
    rows = []
    # Interior diagnostic rays isolate the rear face; they are not sensor rays.
    for y in (-.1,.14,.39):
        for z in (.65,1.,1.5):
            origin = np.array([0.,y,z])
            distances = intersections(triangles,origin,np.array([1.,0.,0.]))
            if not len(distances):
                raise ValueError('no rear mesh intersection')
            rows.append(dict(origin_container=origin.tolist(), direction_container=[1.,0.,0.],
                first_mesh_x=float(distances[0]), semantic_back_x=hull.half_x,
                mesh_minus_semantic_m=float(distances[0]-hull.half_x)))
    report = dict(scope=__doc__, geometry_hash=hull.geometry_hash, triangle_count=len(triangles),
        sources={str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest()
                 for p in (args.scene,args.scene_config)}, rays=rows)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(report,stream,indent=2)
        stream.write('\n')
    print(json.dumps({'rays':len(rows),'back_mesh_x_range':[
        min(r['first_mesh_x'] for r in rows),max(r['first_mesh_x'] for r in rows)]}))


if __name__ == '__main__':
    main()
