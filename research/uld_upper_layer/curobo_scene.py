"""Source-bound GPU proposals in elfin_base_link; native FCL remains authoritative.

No scene is rebuilt from an independent OCC convention. Every object already
owned by FullArmScene is exported, including hollow ULD mesh and released box.
"""
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from assets import source_mesh
from planning_types import digest

JOINT_NAMES=tuple(f'elfin_joint{i}' for i in range(1,7))


def pose_vector(matrix):
    m=np.asarray(matrix)
    return [*m[:3,3].tolist(),*Rotation.from_matrix(m[:3,:3]).as_quat()[[3,0,1,2]].tolist()]


def pose_matrix(pose):
    transform=np.eye(4);transform[:3,3]=pose[:3]
    transform[:3,:3]=Rotation.from_quat(np.asarray(pose[3:])[[1,2,3,0]]).as_matrix()
    return transform


def mesh_record(mesh,transform):
    import trimesh
    vertices,inverse=np.unique(np.asarray(mesh.vertices),axis=0,return_inverse=True)
    topology=trimesh.Trimesh(vertices,inverse[np.asarray(mesh.faces)],process=False)
    # Preserve source triangle surfaces when a CAD mesh is not a closed solid.
    # In particular, the ULD must never become a filled convex/AABB obstacle.
    record=dict(vertices=np.asarray(mesh.vertices).tolist(),faces=np.asarray(mesh.faces).tolist(),
                pose=pose_vector(transform),solid=bool(topology.is_watertight and
                    topology.is_winding_consistent and topology.volume>0))
    # Exact box recognition preserves the existing e2e base/pedestal pair mask.
    # AABB approximation of a non-box (especially the hollow ULD) is forbidden.
    bounds=np.asarray(mesh.bounds);vertices=np.unique(np.asarray(mesh.vertices),axis=0)
    if (len(vertices)==8 and mesh.is_watertight and mesh.is_convex and
            np.isclose(abs(mesh.volume),np.prod(bounds[1]-bounds[0]),rtol=1e-10,atol=1e-14) and
            np.all(np.logical_or(np.isclose(vertices,bounds[0],atol=1e-12,rtol=0),
                                 np.isclose(vertices,bounds[1],atol=1e-12,rtol=0)))):
        centered=np.eye(4);centered[:3,3]=bounds.mean(axis=0)
        record.update(size=np.diff(bounds,axis=0)[0].tolist(),pose=pose_vector(transform@centered))
    return record


def source_urdf(scene):
    """Prune only ancestors of base; keep every descendant and source geometry."""
    root=ET.fromstring(scene.assets['xml'])
    keep={'elfin_base_link'}
    changed=True
    while changed:
        changed=False
        for joint in root.findall('joint'):
            if joint.find('parent').get('link') in keep and joint.find('child').get('link') not in keep:
                keep.add(joint.find('child').get('link'));changed=True
    for link in list(root.findall('link')):
        if link.get('name') not in keep:root.remove(link)
    for joint in list(root.findall('joint')):
        if joint.find('parent').get('link') not in keep:root.remove(joint)
    for mesh in root.findall('.//mesh'):
        mesh.set('filename',str(source_mesh(scene.assets['repo'],mesh.get('filename'))))
    return ET.tostring(root,encoding='unicode'),keep


def make_request(scene,start,goal,config,contact_tool=False):
    base_world=np.linalg.inv(scene.urdf.tcp([0.]*6,scene.urdf.path_to('elfin_base_link')))
    xml,links=source_urdf(scene)
    model=dict(urdf=xml,signature=digest(model_signature(xml)),joint_names=list(JOINT_NAMES))
    world={name:mesh_record(mesh,base_world@transform) for name,(mesh,transform) in scene.world_parts.items()}
    payload=None
    if scene.payload is not None:
        p=scene.payload
        if p['mode']=='attached':
            if not contact_tool:raise ValueError('ATTACHMENT_WITHOUT_CONTACT_POLICY')
            payload=dict(size=np.asarray(p['size']).tolist(),tool_from_box=pose_vector(np.linalg.inv(p['box_T_tool'])),
                         contact_link=scene.robot_parts[scene.push_geometry.collision_key][0])
        elif p['mode']=='world':
            world['released_payload']=mesh_record(p['mesh'],base_world@p['world_pose'])
        else:raise ValueError('UNKNOWN_PAYLOAD_MODE')
    options=dict(e2e_root=config.curobo_e2e_root,assets_manifest=config.curobo_assets_manifest,
                 margin_m=config.collision_margin_m)
    request=dict(model=model,world=world,payload=payload,options=options,
                 start=list(start),goal=list(goal),frame_id='elfin_base_link')
    # FK proof binds base/world convention and tool transform to this source URDF.
    request['source_fk']=[(base_world@scene.urdf.tcp(q,scene.urdf.path_to('suction_contact_frame'))).tolist()
                          for q in (start,goal)]
    request['identity']=digest(request)
    return request


def model_signature(xml, mesh_root=None):
    """Compare base descendants, collision bytes, fixed tooling and joint limits.

    Paths/visuals/inertias do not determine geometric free-motion equivalence.
    A different source collision mesh or kinematic tree rejects old GPU assets.
    """
    root=ET.fromstring(xml);links={'elfin_base_link'}
    while True:
        expanded=links|{j.find('child').get('link') for j in root.findall('joint')
                        if j.find('parent').get('link') in links}
        if expanded==links:break
        links=expanded
    def values(text,default):return tuple(float(v) for v in (text or default).split())
    def origin(element):
        o=element.find('origin')
        return (values(o.get('xyz') if o is not None else None,'0 0 0'),
                values(o.get('rpy') if o is not None else None,'0 0 0'))
    collisions=[]
    for link in root.findall('link'):
        if link.get('name') not in links:continue
        for c in link.findall('collision'):
            g=c.find('geometry');mesh=g.find('mesh')
            if mesh is not None:
                name=mesh.get('filename');path=Path(name)
                if name.startswith('package://'):path=Path(mesh_root)/name.removeprefix('package://')
                elif not path.is_absolute():path=Path(mesh_root)/path
                shape=('mesh',hashlib.sha256(path.read_bytes()).hexdigest(),values(mesh.get('scale'),'1 1 1'))
            else:
                primitive=list(g)[0]
                shape=(primitive.tag,tuple(sorted((k,values(v,'')) for k,v in primitive.attrib.items())))
            collisions.append((link.get('name'),origin(c),shape))
    joints=[]
    for j in root.findall('joint'):
        if j.find('parent').get('link') not in links:continue
        axis=j.find('axis');limit=j.find('limit')
        joints.append((j.get('name'),j.get('type'),j.find('parent').get('link'),j.find('child').get('link'),
                       origin(j),values(axis.get('xyz') if axis is not None else None,'1 0 0'),
                       tuple(sorted((k,float(v)) for k,v in limit.attrib.items())) if limit is not None else ()))
    return sorted(collisions),sorted(joints)
