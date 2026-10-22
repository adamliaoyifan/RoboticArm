#!/usr/bin/env python3
"""Offline payload/arm attribution at saved staging states; no execution."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
import subprocess
import numpy as np
from scipy.spatial.transform import Rotation
import trimesh

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))
from replay_collision_attribution import Urdf, _origin_of, _resolve_package_path
from luggage_planning.payload_envelope_config import load_envelope_limits
from st09_contracts.payload_envelope import build_envelope


def pose(t,q):
    m=np.eye(4);m[:3,:3]=Rotation.from_quat(q).as_matrix();m[:3,3]=t
    return m


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--probe',default='/tmp/phase2_mesh_box_collision_probe')
    args=parser.parse_args();run=args.run
    boundary=json.loads(next((run/'selection_boundary').glob('*/request.json')).read_text())
    rows=json.loads((run/'selection_response.json').read_text())['rows']
    # Exact full-precision endpoint joints; do not replay rounded console anchors.
    states={tuple(boundary['start_joints']): 'start'}
    for row in rows:
        for item in row.get('staging_endpoints',[]):
            states.setdefault(tuple(item['q']),item['node_id'])
    grasp=boundary['grasp'];nominal=np.linalg.inv(pose(grasp['position'],grasp['quaternion_xyzw']))
    env=build_envelope(boundary['box_wdh'],nominal[:3,3],Rotation.from_matrix(nominal[:3,:3]).as_quat(),load_envelope_limits(run/'payload_envelope.yaml'))
    payloads=[('nominal',boundary['box_wdh'],nominal),('envelope',env.dimensions_wdh,pose(env.t_tcp_box_translation,env.t_tcp_box_quaternion_xyzw))]
    urdf=Urdf((run/'robot_expanded.urdf').read_text())
    transforms=[(name,list(q),urdf.link_transforms(q)) for q,name in states.items()]
    hits=[];checked=[];skipped=[]
    for link in ['elfin_base_link']+['elfin_link%d'%i for i in range(1,5)]:
        if link not in urdf.link_els:
            skipped.append(link+': absent');continue
        for number,collision in enumerate(urdf.link_els[link].findall('collision')):
            mesh_element=collision.find('geometry/mesh')
            if mesh_element is None:
                skipped.append(link+': non-mesh geometry');continue
            mesh=trimesh.load(_resolve_package_path(mesh_element.get('filename')),force='mesh',process=False)
            mesh.vertices*=np.array([float(v) for v in mesh_element.get('scale','1 1 1').split()])
            xyz,rotation=_origin_of(collision);origin=np.eye(4);origin[:3,:3]=rotation;origin[:3,3]=xyz
            cases=[];labels={}
            for index,(name,q,tf) in enumerate(transforms):
                for kind,size,offset in payloads:
                    label='%d_%s'%(index,kind)
                    cases.append(' '.join(map(str,[label,*(tf[link]@origin).flat,*size,*(tf['suction_contact_frame']@offset).flat])))
                    labels[label]=dict(state=name,joints=q,kind=kind,link=link,collision_index=number)
            lines=['%d %d'%(len(mesh.vertices),len(mesh.faces))]
            lines.extend(' '.join(map(str,v)) for v in mesh.vertices)
            lines.extend(' '.join(map(str,f)) for f in mesh.faces)
            lines.append(str(len(cases)));lines.extend(cases)
            result=subprocess.run([args.probe],input='\n'.join(lines)+'\n',text=True,capture_output=True,check=True)
            for line in result.stdout.splitlines():
                label,hit,distance=line.split()
                if hit=='1':hits.append(dict(labels[label],distance=float(distance)))
            checked.append(link)
    summary=dict(states=len(states),geometry='URDF arm meshes, link5/6 existing touch exemptions excluded',
        scope='saved candidate states only; not executed trajectories or world-collision proof',
        checked_links=checked,skipped=skipped,
        hit_counts=dict(Counter(x['kind']+'/'+x['link'] for x in hits)),
        unique_hit_states={k:len({x['state'] for x in hits if x['kind']==k}) for k in ('nominal','envelope')},
        envelope=env.to_dict(),hits=hits)
    (run/'offline_anchor_payload_collisions.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps({k:v for k,v in summary.items() if k not in ('hits','envelope')},indent=2))


if __name__=='__main__':main()
