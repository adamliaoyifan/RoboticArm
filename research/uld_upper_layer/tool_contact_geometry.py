"""Immutable source-CAD contact geometry; seal TCP and rigid push face differ.

The push face is the outward, planar +Z boundary of the source p04 collision
mesh, expressed relative to suction_contact_frame. No measured contact, force,
or stability is inferred from this geometric construction.
"""
from dataclasses import dataclass
from pathlib import Path
import numpy as np
import trimesh
from shapely.geometry import Polygon
from shapely.ops import unary_union
from assets import origin, source_mesh, sha256
from planning_types import digest


@dataclass(frozen=True)
class ToolContactGeometry:
    geometry_id: str
    collision_key: str
    source_mesh_sha256: str
    tool_T_push_surface: tuple
    push_polygon_xy: tuple
    cad_tolerance_m: float


def derive_push_surface(assets, cad_tolerance_m=1e-6):
    """Fail closed if p04 no longer supplies one connected, planar front face.

    Tolerance accommodates float32 STL export, never physical penetration.
    The rounded CAD outline is retained; its bounding rectangle is not a
    claim that the corners contain plate material.
    """
    if not 0 < cad_tolerance_m <= 1e-5:
        raise ValueError('INVALID_CAD_TOLERANCE')
    urdf=assets['urdf'];zero=[0.]*6
    link=urdf.root.find("link[@name='suction_panel']")
    matches=[]
    for index,collision in enumerate(link.findall('collision')):
        mesh=collision.find('geometry/mesh')
        if mesh is not None and Path(mesh.get('filename')).name=='p04.stl':
            matches.append((index,collision,mesh))
    if len(matches)!=1:raise ValueError('PUSH_PLATE_SOURCE_NOT_UNIQUE')
    index,collision,node=matches[0]
    path=source_mesh(assets['repo'],node.get('filename'))
    mesh=trimesh.load_mesh(path,process=False)
    mesh.apply_scale([float(v) for v in node.get('scale','1 1 1').split()])
    tool_T_link=np.linalg.inv(urdf.tcp(zero,urdf.path_to('suction_contact_frame')))@urdf.tcp(zero,urdf.path_to('suction_panel'))
    mesh.apply_transform(tool_T_link@origin(collision))
    front=float(mesh.vertices[:,2].max())
    selected=(mesh.face_normals[:,2]>1-1e-8)&(np.max(np.abs(mesh.triangles[:,:,2]-front),axis=1)<=cad_tolerance_m)
    if not np.any(selected):raise ValueError('PUSH_PLATE_FRONT_NOT_PLANAR')
    polygon=unary_union([Polygon(triangle[:,:2]) for triangle in mesh.triangles[selected]])
    if polygon.geom_type!='Polygon' or not polygon.is_valid or polygon.area<=0 or len(polygon.interiors):
        raise ValueError('PUSH_PLATE_FRONT_NOT_SINGLE_SOLID_POLYGON')
    transform=np.eye(4);transform[2,3]=front
    points=tuple(tuple(float(v) for v in xy) for xy in polygon.exterior.coords[:-1])
    fields=dict(collision_key=f'suction_panel:{index}',source_mesh_sha256=sha256(path),
                tool_T_push_surface=tuple(transform.ravel()),push_polygon_xy=points,
                cad_tolerance_m=cad_tolerance_m)
    return ToolContactGeometry(geometry_id=digest(fields),**fields)
