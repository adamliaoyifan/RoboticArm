#!/usr/bin/env python3
"""Offline check of saved DESCEND endpoint, using the diagnostic envelope."""
import argparse
from itertools import product
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'scripts'))
from replay_collision_attribution import Urdf
from luggage_description.container_geometry import descriptor_from_scene_config, contains_point
from luggage_planning.payload_envelope_config import load_envelope_limits
from st09_contracts.payload_envelope import build_envelope


def transform(translation, quaternion):
    matrix = np.eye(4)
    matrix[:3,:3] = Rotation.from_quat(quaternion).as_matrix()
    matrix[:3,3] = translation
    return matrix


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--urdf',type=Path,required=True)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    from rclpy.serialization import deserialize_message
    from luggage_msgs.srv import SelectPlacePathV2
    request_file=next((args.run/'selection_boundary').glob('*/request.json'))
    request=json.loads(request_file.read_text())
    response=deserialize_message((request_file.parent/'response.cdr').read_bytes(),SelectPlacePathV2.Response)
    descend=next(s for s in response.motion_segments if s.name=='descend')
    joints=list(descend.joint_trajectory.trajectory.points[-1].positions)
    world_tool=Urdf(args.urdf.read_text()).link_transforms(joints)['suction_contact_frame']
    grasp=request['grasp']
    nominal=np.linalg.inv(transform(grasp['position'],grasp['quaternion_xyzw']))
    envelope=build_envelope(request['box_wdh'],nominal[:3,3],Rotation.from_matrix(nominal[:3,:3]).as_quat(),load_envelope_limits(args.config))
    scene=yaml.safe_load((args.run/'scene_tf_used.yaml').read_text())
    frame=next(s for s in scene['static_transforms'] if s['parent']=='world' and s['child']=='container_link')
    container_world=np.linalg.inv(transform(frame['translation'],Rotation.from_euler('xyz',frame['rotation_rpy']).as_quat()))
    container_envelope=container_world@world_tool@transform(envelope.t_tcp_box_translation,envelope.t_tcp_box_quaternion_xyzw)
    corners=[(container_envelope@np.r_[np.array(sign)*np.array(envelope.dimensions_wdh)/2,1])[:3].tolist() for sign in product((-1,1),repeat=3)]
    geometry=descriptor_from_scene_config(scene)
    result=dict(source='saved selected DESCEND final joint state; FK, not a new route',
        selected_slot_index=response.slot_index,joints=joints,envelope=envelope.to_dict(),
        envelope_digest=envelope.digest(),corners_container=corners,
        floor_z=geometry.floor_z, min_z=min(p[2] for p in corners),
        inside=all(contains_point(geometry,p) for p in corners),
        new_route='not_evaluated',n3='not_evaluated')
    result['below_floor_m']=geometry.floor_z-result['min_z']
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('corners_container','envelope','joints')},indent=2))


if __name__=='__main__':
    main()
