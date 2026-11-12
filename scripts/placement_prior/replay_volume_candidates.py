"""Compare exact saved candidates against reconstructed 3D occupancy offline.

The volume is a diagnostic reconstruction, not a newly owner-admitted snapshot.
No planning, motion, scene publication or map mutation occurs.
"""
import argparse
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message, serialize_message
from luggage_msgs.srv import PreparePlacementPriorV2, CheckPlacementToolV2
from moveit_msgs.msg import PlanningScene
from luggage_description.scene_tf_config_utils import load_scene_tf_config, world_transform
from luggage_planning.placement_prior.contracts import Transform
from luggage_planning.placement_prior.surface import ConfirmedSurface
from luggage_planning.placement_prior.snapshot import build_private_snapshot


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('run','scene','urdf','srdf','native','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args(); args.output.mkdir(parents=True,exist_ok=False)
    boundary=args.run/'prior_boundaries'
    rows=list(map(json.loads,(boundary/'boundaries.jsonl').read_text().splitlines()))
    response_row=next(row for row in rows if row['boundary']=='prepare_response')
    response=deserialize_message((boundary/response_row['file']).read_bytes(),PreparePlacementPriorV2.Response)
    typed=response.surface
    reshape=lambda values:[list(values[i*typed.columns:(i+1)*typed.columns]) for i in range(typed.rows)]
    surface=dict(frame_id='container_link',scene_revision=typed.identity.scene_revision,
        geometry_hash=typed.geometry_hash,map_revision=typed.map_revision,
        origin_xy_container=list(typed.origin_xy_container),floor_z_container=typed.floor_z_container,
        ceiling_z_container=typed.ceiling_z_container,resolution_m=typed.resolution_m,
        state=reshape(typed.state),surface_z_container=reshape([
            z if valid else None for z,valid in zip(typed.surface_z_container,typed.valid)]))
    replay=json.loads((args.run/'volume_replay.json').read_text())
    stamp=replay['views'][-1]['stamp_ns']
    confirmed=ConfirmedSurface(json.dumps(surface).encode(),serialize_message(typed),
                               json.dumps(dict(acquisition_stamp_ns=stamp)).encode())
    data=np.load(args.run/'volume_replay.npz');nz,ny,nx=data['states'].shape
    volume=SimpleNamespace(success=True,frame_id='container_link',scene_revision=surface['scene_revision'],
        geometry_hash=typed.geometry_hash,map_revision=typed.map_revision,acquisition_stamp_ns=stamp,
        map_source='measured',occupancy_confirmed=True,has_pending_changes=False,
        origin_container=data['origin_container'].tolist(),resolution_m=float(data['resolution_m']),
        dimensions_xyz=[nx,ny,nz],states=data['states'].reshape(-1).tolist(),active=data['active'].reshape(-1).tolist())
    baseline_row=next(row for row in rows if row['boundary']=='baseline_scene')
    baseline=deserialize_message((boundary/baseline_row['file']).read_bytes(),PlanningScene)
    xyz,rpy=world_transform(load_scene_tf_config(str(args.scene)),'container_link')
    snapshot=build_private_snapshot(confirmed,baseline,response.model_sha256,
        Transform(xyz,Rotation.from_euler('xyz',rpy).as_quat()),collision_margin_m=.05,volume=volume)
    scene_file=args.output/'volume_scene.cdr';scene_file.write_bytes(snapshot.scene_cdr)
    requests=[str(boundary/row['file']) for row in rows if row['boundary']=='collision_request']
    with (args.output/'native.log').open('w') as log:
        subprocess.run([str(args.native),str(args.urdf),str(args.srdf),str(scene_file),*requests],
                       stdout=log,stderr=subprocess.STDOUT,check=True,timeout=90)
    results=[]
    for line in (args.output/'native.log').read_text().splitlines():
        if line.startswith('RESULT '):
            _,request,mode,index,collision=line.split()
            results.append(dict(request=request,mode=int(mode),index=int(index),collision=bool(int(collision))))
    report=dict(scope='diagnostic reconstructed volume; rigid-tool/payload endpoint checks only',
        request_count=len(requests),results=results,
        old_clear=sum(not r['collision'] for r in results if r['mode']==0),
        volume_clear=sum(not r['collision'] for r in results if r['mode']==1))
    (args.output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='results'}))


if __name__=='__main__':main()
