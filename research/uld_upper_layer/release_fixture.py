"""Original rigid tool on an X/Z carriage for independent release calibration.

This builds fixture geometry, never a robot reachability certificate. The source
tool is welded into one link with equivalent mass/COM/inertia and exact source
collision geometry. Only carriage joints are controllable; there is no payload
command or attachment implementation here.
"""
from copy import deepcopy
from dataclasses import dataclass, fields
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from assets import origin, source_mesh
from candidates import matrix
from scene import text


def _numbers(values):
    # Collision transforms and joint axes retain double round-trip precision.
    return ' '.join(format(float(value), '.17g') for value in values)


@dataclass(frozen=True)
class ReleaseFixturePolicy:
    carriage_mass_kg: float
    carriage_inertia_kg_m2: float
    travel_limit_m: float
    velocity_limit_mps: float
    effort_limit_n: float
    position_p_gain: float
    position_d_gain: float

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if isinstance(value, bool) or not np.isfinite(value) or value <= 0:
                raise ValueError('INVALID_RELEASE_FIXTURE_POLICY:' + field.name)

    @classmethod
    def load(cls, path=None):
        return cls(**yaml.safe_load(Path(path or Path(__file__).with_name(
            'release_fixture.yaml')).read_text()))


def _pose_text(parent, transform):
    return text(parent, 'pose', _numbers([*transform[:3, 3],
        *Rotation.from_matrix(transform[:3, :3]).as_euler('xyz')]))


def _inertial(link, mass, center, inertia):
    node = ET.SubElement(link, 'inertial')
    text(node, 'mass', mass)
    text(node, 'pose', _numbers([*center, 0, 0, 0]))
    tensor = ET.SubElement(node, 'inertia')
    for key, row, col in [('ixx', 0, 0), ('iyy', 1, 1), ('izz', 2, 2),
                          ('ixy', 0, 1), ('ixz', 0, 2), ('iyz', 1, 2)]:
        text(tensor, key, float(inertia[row, col]))


def source_tool_link(assets):
    """Return one exact rigid tool link at its source suction_contact_frame."""
    urdf = assets['urdf']
    tcp = urdf.tcp([0.] * 6, urdf.path_to('suction_contact_frame'))
    names = {'suction_panel'}
    while True:
        children = {j['child'] for j in urdf.joints.values() if j['parent'] in names}
        for joint in urdf.joints.values():
            if joint['parent'] in names and joint['type'] != 'fixed':
                raise ValueError('NONRIGID_SOURCE_TOOL')
        expanded = names | children
        if expanded == names:
            break
        names = expanded
    link = ET.Element('link', name='tool')
    parts = []; masses = []; ledger = []
    for source in urdf.root.findall('link'):
        name = source.get('name')
        if name not in names:
            continue
        relative = np.linalg.inv(tcp) @ urdf.tcp([0.] * 6, urdf.path_to(name))
        inertial = source.find('inertial')
        if inertial is not None:
            mass = float(inertial.find('mass').get('value'))
            inertia_pose = relative @ origin(inertial)
            values = inertial.find('inertia').attrib
            inertia = np.array([
                    [float(values['ixx']), float(values['ixy']), float(values['ixz'])],
                    [float(values['ixy']), float(values['iyy']), float(values['iyz'])],
                    [float(values['ixz']), float(values['iyz']), float(values['izz'])]])
            rotated = inertia_pose[:3, :3] @ inertia @ inertia_pose[:3, :3].T
            if mass <= 0 or not np.isfinite(mass) or not np.isfinite(rotated).all():
                raise ValueError('INVALID_SOURCE_TOOL_INERTIA:' + name)
            masses.append((mass, inertia_pose[:3, 3], rotated))
        for index, collision in enumerate(source.findall('collision')):
            key = f'{name}:{index}'
            part = ET.SubElement(link, 'collision', name=f'{name}_{index}')
            transform = relative @ origin(collision)
            _pose_text(part, transform)
            geometry = deepcopy(collision.find('geometry'))
            mesh = geometry.find('mesh')
            if mesh is not None:
                uri = source_mesh(assets['repo'], mesh.get('filename')).resolve()
                scale = mesh.get('scale', '1 1 1')
                mesh.attrib.clear()
                text(mesh, 'uri', str(uri)); text(mesh, 'scale', scale)
            else:
                box = geometry.find('box')
                if box is None:
                    raise ValueError('UNSUPPORTED_SOURCE_TOOL_COLLISION:' + key)
                size = box.attrib.pop('size'); text(box, 'size', size)
            part.append(geometry)
            visual = deepcopy(part); visual.tag = 'visual'
            visual.set('name', part.get('name') + '_visual'); link.append(visual)
            parts.append(key)
            ledger.append(dict(source_part=key, sdf_collision=part.get('name'),
                               tool_T_collision=transform.reshape(-1).tolist()))
    if set(parts) != {body.name for body in assets['tool']}:
        raise ValueError('INCOMPLETE_SOURCE_TOOL_COLLISIONS')
    mass = sum(item[0] for item in masses)
    if mass <= 0:
        raise ValueError('MISSING_SOURCE_TOOL_MASS')
    center = sum(m * c for m, c, _ in masses) / mass
    inertia = sum(i + m * ((d @ d) * np.eye(3) - np.outer(d, d))
                  for m, c, i in masses for d in [c - center])
    _inertial(link, mass, center, inertia)
    return link, dict(parts=ledger, mass_kg=mass, com_tool_m=center.tolist(),
                      inertia_com_tool_kg_m2=inertia.tolist())


def release_carriage_model(assets, case, policy):
    """Generate a dynamic carriage, at zero joint positions, for one legal case.

    Positive x/z joint displacement is container +X/+Z independent of tool yaw.
    Controllers act on those joints using forces, never link pose commands.
    """
    if not case.geometry_available or case.container_T_tool is None:
        raise ValueError('RELEASE_GEOMETRY_UNAVAILABLE')
    container_tool = matrix(case.container_T_tool)
    world_tool = assets['world_container'] @ container_tool
    model = ET.Element('model', name='release_carriage')
    text(model, 'static', 'false'); _pose_text(model, world_tool)
    anchor = ET.SubElement(model, 'link', name='anchor')
    text(anchor, 'gravity', 'false')
    carriage = ET.SubElement(model, 'link', name='x_carriage')
    _inertial(carriage, policy.carriage_mass_kg, np.zeros(3),
              np.eye(3) * policy.carriage_inertia_kg_m2)
    link, ledger = source_tool_link(assets); model.append(link)
    joint = ET.SubElement(model, 'joint', name='world_anchor', type='fixed')
    text(joint, 'parent', 'world'); text(joint, 'child', 'anchor')
    axes = {}
    for label, parent, child, direction in [('x', 'anchor', 'x_carriage', [1, 0, 0]),
                                          ('z', 'x_carriage', 'tool', [0, 0, 1])]:
        name = 'release_' + label
        joint = ET.SubElement(model, 'joint', name=name, type='prismatic')
        text(joint, 'parent', parent); text(joint, 'child', child)
        axis = ET.SubElement(joint, 'axis')
        local = container_tool[:3, :3].T @ np.array(direction)
        text(axis, 'xyz', _numbers(local)); axes[label] = local.tolist()
        limit = ET.SubElement(axis, 'limit')
        for key, value in dict(lower=-policy.travel_limit_m, upper=policy.travel_limit_m,
                effort=policy.effort_limit_n, velocity=policy.velocity_limit_mps).items():
            text(limit, key, value)
        plugin = ET.SubElement(model, 'plugin',
            filename='ignition-gazebo-joint-position-controller-system',
            name='ignition::gazebo::systems::JointPositionController')
        for key, value in dict(joint_name=name, topic='/upper_layer/release/' + label,
                p_gain=policy.position_p_gain, i_gain=0, d_gain=policy.position_d_gain,
                cmd_max=policy.effort_limit_n, cmd_min=-policy.effort_limit_n).items():
            text(plugin, key, value)
    return model, dict(**ledger, joint_axes_tool=axes,
        scope='independent_release_fixture_not_arm_reachability_or_execution_admission')
