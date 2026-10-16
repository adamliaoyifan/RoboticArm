"""Bounded offline camera IK proposals; outputs are not motion authorization."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
from luggage_description.scene_tf_config_utils import load_scene_tf_config, world_transform
from luggage_planning.offline_ik import load_urdf, refine_to_pose
from luggage_planning.container_aim_utils import look_at_quaternion


def propose(scene, output, budget_s, profile='initial'):
    started = time.monotonic()
    robot = load_urdf(scene_tf_config=str(scene))
    if robot.root_link != 'world':
        raise ValueError('This probe requires a world-rooted URDF')
    path = robot.path_to('camera_depth_optical_frame')
    if not path:
        raise ValueError('Camera optical frame absent')
    xyz, rpy = world_transform(load_scene_tf_config(str(scene)), 'container_link')
    container_rotation = Rotation.from_euler('xyz', rpy)
    seeds = [[.7412,-1.8043,-1.0461,-1.8668,1.5749,-2.4370],
             [1.8806,-1.7736,-1.0491,4.4439,1.7721,1.8473]]
    attempts = []
    if profile == 'lower':
        # Diagnostic views aimed at the measured floor/side collision regions.
        eyes=[(-.95,-.6,.8),(-1.15,-.6,1.),(-1.,.05,.85)]
        targets=[(.4,-.6,.65),(.5,-.6,1.2)]
    else:
        eyes=[(-1.15,-.27,1.3),(-1.4,-.27,1.3),(-1.15,-.27,1.65)]
        targets=[(0.,-.27,1.3),(.5,-.27,1.0)]
    for eye in eyes:
        for target in targets:
            for roll in [0., np.pi]:
                local_rotation = Rotation.from_quat(look_at_quaternion(eye,target)) * Rotation.from_rotvec([0,0,roll])
                position = container_rotation.apply(eye) + xyz
                orientation = (container_rotation * local_rotation).as_quat()
                for seed in seeds:
                    if time.monotonic()-started > budget_s:
                        break
                    joints, info = refine_to_pose(robot,path,position,orientation,seed)
                    row = dict(eye_container=eye, target_container=target, optical_roll_rad=roll,
                               target_world=position.tolist(), quaternion_world=orientation.tolist(),
                               seed=seed, joints=joints, ik=info, collision_checked=False, path_checked=False)
                    if joints is not None:
                        fk = robot.tcp(joints,path)
                        row['achieved_eye_container'] = container_rotation.inv().apply(fk[:3,3]-xyz).tolist()
                        row['achieved_forward_container'] = container_rotation.inv().apply(fk[:3,2]).tolist()
                    attempts.append(row)
    import xml.etree.ElementTree as ET
    xml = ET.tostring(robot.root)
    output.mkdir(parents=True,exist_ok=False)
    (output/'robot.urdf').write_bytes(xml)
    report = dict(scope='offline kinematic proposals only', profile=profile, scene=str(scene),
                  scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),
                  urdf_sha256=hashlib.sha256(xml).hexdigest(),
                  elapsed_s=time.monotonic()-started, budget_s=budget_s,
                  attempts=attempts, feasible=sum(r['joints'] is not None for r in attempts))
    (output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='attempts'}))


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--budget-s',type=float,default=30.)
    parser.add_argument('--profile',choices=('initial','lower'),default='initial')
    args=parser.parse_args()
    propose(args.scene,args.output,args.budget_s,args.profile)
