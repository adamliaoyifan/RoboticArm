"""Load source-revision assets without falling back to installed meshes."""
import hashlib
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import trimesh
import yaml
from scipy.spatial.transform import Rotation
from geometry import Box, pose


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def origin(element):
    node = element.find('origin')
    if node is None:
        return np.eye(4)
    xyz = [float(x) for x in node.get('xyz', '0 0 0').split()]
    rpy = [float(x) for x in node.get('rpy', '0 0 0').split()]
    return pose(xyz, Rotation.from_euler('xyz', rpy).as_matrix())


def source_mesh(repo, uri):
    if not uri.startswith('package://'):
        raise ValueError(f'only source package meshes supported: {uri}')
    package, relative = uri[len('package://'):].split('/', 1)
    if package == 'luggage_description' and relative.startswith('meshes/'):
        rest = relative[len('meshes/'):]
        if rest.startswith('suction_panel/'):
            path = repo/'src/luggage_gazebo/models/suction_panel/collision'/rest.split('/', 1)[1]
        else:
            path = repo/'src/luggage_gazebo/models'/rest
    else:
        path = repo/'src'/package/relative
    if not path.is_file():
        raise FileNotFoundError(path)
    return path


def load(repo, scene_path=None):
    repo = Path(repo).resolve()
    sys.path[:0] = [str(repo/'src/luggage_planning'), str(repo/'src/luggage_description')]
    import xacro
    import xacro.substitution_args
    from luggage_planning.motion.offline_ik import Urdf
    from luggage_description.scene_tf_config_utils import urdf_world_base_pose
    from luggage_description.xacro_robot_with_scene_base import patch_world_base
    from luggage_description.container_geometry import descriptor_from_scene_config

    scene_path = (repo/'src/luggage_description/config/scene_tf.yaml' if scene_path is None
                  else Path(scene_path).resolve())
    scene = yaml.safe_load(scene_path.read_text())
    catalog_path = scene_path.parent/scene['container']['box_catalog_config']
    contact_path = scene_path.parent/'suction_contact_model.yaml'
    source = repo/'src/luggage_description/urdf/elfin_s20_with_camera.urdf.xacro'
    static_models=repo/'src/luggage_description/luggage_description/configured_scene_models.py'
    inputs = {p: sha256(p) for p in (scene_path, catalog_path, contact_path, source,static_models)}
    original_find = xacro.substitution_args._eval_find

    def find_source(package):
        path = repo/'src'/package
        if not path.is_dir():
            raise FileNotFoundError(f'source package missing: {package}')
        return str(path)

    try:
        xacro.substitution_args._eval_find = find_source
        document = xacro.process_file(str(source), mappings={'fixed_world': 'true'})
        for name in xacro.all_includes:
            path = Path(name).resolve()
            inputs[path] = sha256(path)
    finally:
        xacro.substitution_args._eval_find = original_find
    root = ET.fromstring(document.toxml())
    xyz, rpy = urdf_world_base_pose(scene)
    if not patch_world_base(root, xyz, rpy):
        raise ValueError('world_base missing from expanded URDF')
    xml = ET.tostring(root, encoding='unicode')
    urdf = Urdf(xml)
    tcp = urdf.tcp([0]*6, urdf.path_to('suction_contact_frame'))
    fixed_links = {'suction_panel'}
    changed = True
    while changed:
        changed = False
        for joint in urdf.joints.values():
            if joint['parent'] in fixed_links and joint['child'] not in fixed_links:
                if joint['type'] != 'fixed':
                    raise ValueError('moving joint below panel unsupported')
                fixed_links.add(joint['child']); changed = True
    tool = []
    for link in root.findall('link'):
        name = link.get('name')
        if name not in fixed_links:
            continue
        relative = np.linalg.inv(tcp) @ urdf.tcp([0]*6, urdf.path_to(name))
        for index, collision in enumerate(link.findall('collision')):
            transform = relative @ origin(collision)
            geometry = collision.find('geometry')
            mesh = geometry.find('mesh'); box = geometry.find('box')
            if mesh is not None:
                path = source_mesh(repo, mesh.get('filename'))
                inputs[path] = sha256(path)
                vertices = np.asarray(trimesh.load_mesh(str(path), process=False).vertices)
                vertices *= np.array([float(x) for x in mesh.get('scale', '1 1 1').split()])
                low, high = vertices.min(axis=0), vertices.max(axis=0)
            elif box is not None:
                high = np.array([float(x)/2 for x in box.get('size').split()]); low = -high
            else:
                raise ValueError(f'unsupported collision primitive: {name}')
            local = Box(f'{name}:{index}', (low+high)/2, np.eye(3), (high-low)/2)
            tool.append(local.moved(transform))
    required = {'suction_panel', 'eef_mount_adapter', 'camera_link', 'mid360_mount_frame'}
    if not required.issubset({b.name.split(':')[0] for b in tool}):
        raise ValueError('incomplete fixed-tool geometry')
    transform_row = next(x for x in scene['static_transforms'] if x['child']=='container_link')
    if transform_row['parent'] != scene['world_frame']:
        raise ValueError('expected direct world/container transform')
    world_container = pose(transform_row['translation'], Rotation.from_euler(
        'xyz', transform_row['rotation_rpy']).as_matrix())
    return dict(repo=repo, scene=scene, catalog=yaml.safe_load(catalog_path.read_text()),
                contact=yaml.safe_load(contact_path.read_text()), urdf=urdf, xml=xml,
                hull=descriptor_from_scene_config(scene), tool=tool,
                world_container=world_container,
                hashes={str(p.relative_to(repo)) if p.is_relative_to(repo) else str(p):h
                        for p,h in inputs.items()})
