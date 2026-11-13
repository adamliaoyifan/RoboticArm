"""Conservative fixed-tool projection from the same URDF used by MoveIt.

Articulating forearm links are deliberately excluded and remain native collision
checks. Bounding boxes are proposal geometry, never substitutes for mesh checks.
"""
from pathlib import Path
from urllib.parse import unquote, urlparse

import numpy as np

from luggage_planning.joint_operation.aperture_geometry import box_vertices
from luggage_planning.motion.offline_ik import _origin_of


def _mesh_path(uri):
    if uri.startswith('package://'):
        from ament_index_python.packages import get_package_share_directory
        package, relative = uri[len('package://'):].split('/', 1)
        return Path(get_package_share_directory(package))/relative
    if uri.startswith('file://'):
        return Path(unquote(urlparse(uri).path))
    path = Path(uri)
    if not path.is_absolute():
        raise ValueError('absolute or package tool mesh URI required')
    return path


def fixed_tool_vertices(model):
    urdf = model.urdf
    component = {'suction_contact_frame'}
    changed = True
    while changed:
        changed = False
        for joint in urdf.joints.values():
            if joint['type'] == 'fixed' and component.intersection((joint['parent'], joint['child'])):
                before = len(component)
                component.update((joint['parent'], joint['child']))
                changed |= len(component) != before
    zero = (0.,)*6
    world_from_tool = urdf.tcp(zero, model.path)
    tool_from_world = np.linalg.inv(world_from_tool)
    result = []
    for link in urdf.root.findall('link'):
        if link.get('name') not in component:
            continue
        tool_from_link = tool_from_world @ urdf.tcp(zero, urdf.path_to(link.get('name')))
        for collision in link.findall('collision'):
            geometry = collision.find('geometry')
            if geometry is None or len(geometry) != 1:
                raise ValueError('explicit tool collision geometry required')
            shape = geometry[0]
            if shape.tag == 'box':
                vertices = box_vertices([float(v) for v in shape.get('size').split()])
            elif shape.tag == 'sphere':
                vertices = box_vertices((2*float(shape.get('radius')),)*3)
            elif shape.tag == 'cylinder':
                diameter = 2*float(shape.get('radius'))
                vertices = box_vertices((diameter, diameter, float(shape.get('length'))))
            elif shape.tag == 'mesh':
                from luggage_description.stl_geometry import read_stl_arrays
                vertices, _ = read_stl_arrays(str(_mesh_path(shape.get('filename'))))
                vertices = np.asarray(vertices)*np.asarray([float(v) for v in shape.get('scale', '1 1 1').split()])
                lower, upper = vertices.min(axis=0), vertices.max(axis=0)
                vertices = box_vertices(upper-lower)+(upper+lower)/2
            else:
                raise ValueError('unsupported fixed-tool collision geometry: '+shape.tag)
            xyz, rotation = _origin_of(collision)
            vertices = vertices @ rotation.T+xyz
            result.extend(vertices @ tool_from_link[:3, :3].T+tool_from_link[:3, 3])
    values = np.asarray(result)
    if values.ndim != 2 or values.shape[1] != 3 or not np.all(np.isfinite(values)):
        raise ValueError('finite fixed-tool collision vertices required')
    return values
