#!/usr/bin/env python3
"""Build a deterministic research fixture; truth never enters production perception.

The overlay world is consumed by sim_world.launch.py, which supplies the original
container, platforms and controlled robot. The standalone world adds those static
scene assets for an independent gravity-settling check (no robot motion).
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

from assets import load, sha256
from geometry import axis_box, separated

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/luggage_description'))
from luggage_description.container_geometry import (
    aabb_intersection_volume, contains_oriented_box, y_bounds_for_z_interval,
)
from luggage_description.scene_tf_config_utils import (
    gazebo_container_spawn_pose, gazebo_pedestal_spawn_pose,
    gazebo_pickup_platform_spawn_pose,
)


def text(parent, name, value):
    child = ET.SubElement(parent, name)
    child.text = str(value)
    return child


def numbers(values):
    return ' '.join(f'{float(v):.9g}' for v in values)


def fixture(assets, ceiling_gap=.03, layers=4, columns=3):
    """Rectangular boxes; upper positive-Y rows overhang the chamfer safely.

    Both Y rows share the same seam. No wedge, shelf, hidden slab or fixed lower
    box supplies support. Dimensions are synthetic fixture inputs, not catalog
    claims. Adjacent layers start 1 mm apart to avoid initial mesh penetration.
    """
    hull = assets['hull']
    payload = next(r for r in assets['catalog']['box_catalog'] if r['id'] == 'carryon')
    top = hull.ceiling_z - payload['size'][2] - ceiling_gap
    pitch_z = (top - hull.floor_z) / layers
    if pitch_z <= .05 or ceiling_gap <= 0:
        raise ValueError('invalid support height or ceiling gap')
    wall_gap, seam_gap, vertical_gap = .006, .004, .001
    # Mesh ribs/jambs protrude beyond the semantic hull; measured source margins.
    x_min, x_max = -hull.half_x+.015, hull.half_x-.025
    pitch_x = (x_max-x_min) / columns
    boxes = []
    for layer in range(layers):
        bottom = hull.floor_z + layer * pitch_z + vertical_gap
        height = pitch_z - vertical_gap
        ymin, ymax = y_bounds_for_z_interval(hull, bottom, bottom+height, margin=wall_gap)
        # The wall has an inward rail at Y~0.930 around Z=1.5 m.
        ymax = min(ymax, hull.half_y-.065)
        for col in range(columns):
            for row, (ya, yb) in enumerate(((ymin, -seam_gap/2), (seam_gap/2, ymax))):
                xa = x_min + col*pitch_x + seam_gap/2
                xb = xa + pitch_x - seam_gap
                size = [xb-xa, yb-ya, height]
                center = [(xa+xb)/2, (ya+yb)/2, bottom+height/2]
                if not contains_oriented_box(hull, center, size):
                    raise ValueError('fixture box outside canonical hull')
                world = (assets['world_container'] @ np.r_[center, 1])[:3]
                yaw = Rotation.from_matrix(assets['world_container'][:3,:3]).as_euler('xyz')[2]
                boxes.append(dict(name=f'lower_l{layer}_x{col}_y{row}', layer=layer,
                                  size_m=size, center_container_m=center,
                                  pose_world=[*world, 0., 0., yaw], mass_kg=160*np.prod(size)))
    for i, a in enumerate(boxes):
        for b in boxes[i+1:]:
            ac, bc = np.array(a['center_container_m']), np.array(b['center_container_m'])
            ah, bh = np.array(a['size_m'])/2, np.array(b['size_m'])/2
            if not separated(axis_box('a', ac-ah, ac+ah), axis_box('b', bc-bh, bc+bh)):
                raise ValueError('overlapping fixture boxes')
    lower_volume = aabb_intersection_volume(
        hull, [-hull.half_x, -hull.half_y, hull.floor_z], [hull.half_x, hull.half_y, top])
    fill = sum(np.prod(b['size_m']) for b in boxes)/lower_volume
    tool_depth = -min(b.corners()[:,2].min() for b in assets['tool'])
    panel_depth = -min(b.corners()[:,2].min() for b in assets['tool']
                       if b.name.startswith('suction_panel:'))
    if fill < .90 or ceiling_gap >= panel_depth:
        raise ValueError('fixture must be dense and exclude downward panel clearance')
    return dict(schema_version=1, physics_step_s=.001, world='airport_loading', frame='container_link',
                geometry_hash=hull.geometry_hash, support_z_m=top,
                ceiling_z_m=hull.ceiling_z, upper_headroom_m=hull.ceiling_z-top,
                lower_fill_fraction=fill, boxes=boxes,
                payload=dict(id='carryon', size_m=payload['size'], mass_kg=payload['mass_kg']),
                payload_ceiling_gap_m=ceiling_gap, fixed_tool_depth_from_contact_m=tool_depth,
                suction_panel_depth_from_contact_m=panel_depth,
                allowed_action='upward-panel carry / release / rear push (not implemented)',
                online_geometry_source=False, canonical_floor_contact=True,
                source_hashes=assets['hashes'])


def box_model(spec, color):
    model = ET.Element('model', name=spec['name'])
    text(model, 'static', 'false')
    text(model, 'pose', numbers(spec['pose_world']))
    link = ET.SubElement(model, 'link', name='body')
    text(link, 'gravity', 'true')
    inertia = ET.SubElement(link, 'inertial')
    mass = float(spec['mass_kg'])
    text(inertia, 'mass', mass)
    tensor = ET.SubElement(inertia, 'inertia')
    x, y, z = spec['size_m']
    for key, value in dict(ixx=mass*(y*y+z*z)/12, iyy=mass*(x*x+z*z)/12,
                           izz=mass*(x*x+y*y)/12, ixy=0, ixz=0, iyz=0).items():
        text(tensor, key, value)
    for kind in ('collision', 'visual'):
        part = ET.SubElement(link, kind, name=kind)
        text(ET.SubElement(ET.SubElement(part, 'geometry'), 'box'), 'size', numbers((x,y,z)))
        if kind == 'visual':
            material = ET.SubElement(part, 'material')
            text(material, 'ambient', color)
            text(material, 'diffuse', color)
        else:
            friction = ET.SubElement(ET.SubElement(part, 'surface'), 'friction')
            ode = ET.SubElement(friction, 'ode')
            text(ode, 'mu', .8)
            text(ode, 'mu2', .8)
    return model


def original_model(repo, name, pose):
    model = deepcopy(ET.parse(repo/f'src/luggage_gazebo/models/{name}/model.sdf').getroot().find('model'))
    old_pose = model.find('pose')
    if old_pose is not None:
        model.remove(old_pose)
    text(model, 'pose', numbers(pose[k] for k in ('x','y','z','R','P','Y')))
    for uri in model.iter('uri'):
        if uri.text.startswith('model://'):
            uri.text = str(repo/'src/luggage_gazebo/models'/uri.text[len('model://'):])
    return model


def intersecting_triangles(center, half, triangles):
    """Exact box/triangle SAT at the initial, axis-aligned fixture pose.

    Hull containment alone omits mesh ribs. Check actual source triangles;
    mesh AABB or center-ray checks do not establish non-intersection.
    """
    center, half = np.asarray(center), np.asarray(half)
    mask = np.all(triangles.max(axis=1) >= center-half, axis=1)
    mask &= np.all(triangles.min(axis=1) <= center+half, axis=1)
    hits = []
    for index in np.flatnonzero(mask):
        triangle = triangles[index]-center
        edges = [triangle[1]-triangle[0], triangle[2]-triangle[1], triangle[0]-triangle[2]]
        axes = [*np.eye(3), np.cross(edges[0], edges[1])]
        axes += [np.cross(edge, axis) for edge in edges for axis in np.eye(3)]
        for axis in axes:
            if np.linalg.norm(axis) < 1e-12:
                continue
            projection = triangle @ axis
            radius = np.abs(axis) @ half
            if projection.min() > radius+1e-9 or projection.max() < -radius-1e-9:
                break
        else:
            hits.append(int(index))
    return hits


def write_scene(repo, output):
    if output.exists() and any(output.iterdir()):
        raise ValueError('output must be new or empty; do not overwrite a previous scene')
    assets = load(repo)
    spec = fixture(assets)
    container_model = assets['scene']['gazebo']['container_model']
    mesh = repo/f'src/luggage_gazebo/models/{container_model}/meshes/container_collision_physics.stl'
    triangles = np.asarray(trimesh.load_mesh(mesh, process=False).triangles)
    spec['source_hashes'][str(mesh.relative_to(repo))] = sha256(mesh)
    for box in spec['boxes']:
        hits = intersecting_triangles(box['center_container_m'], np.array(box['size_m'])/2, triangles)
        if hits:
            raise ValueError(f"{box['name']} intersects source collision mesh triangles {hits[:8]}")
    spec['initial_mesh_intersections'] = 0
    output.mkdir(parents=True, exist_ok=True)
    root = ET.parse(repo/'src/luggage_gazebo/worlds/airport_loading.sdf').getroot()
    world = root.find('world')
    # Preserve collision physics; use a declared 1 ms research stepping profile.
    world.find('physics/max_step_size').text = str(spec['physics_step_s'])
    # The default world contains off-scene mesh-cache preloads; omit them.
    for include in list(world.findall('include')):
        world.remove(include)
    colors = ['0.65 0.34 0.16 1', '0.76 0.49 0.23 1', '0.48 0.32 0.20 1']
    for box in spec['boxes']:
        world.append(box_model(box, colors[box['layer']%len(colors)]))
    # The usual spawner remains a service, so this named test payload is explicit.
    pickup = gazebo_pickup_platform_spawn_pose(assets['scene'])
    platform = assets['scene']['pickup_platform']
    target = dict(name='upper_layer_payload', size_m=spec['payload']['size_m'],
                  mass_kg=spec['payload']['mass_kg'],
                  pose_world=[pickup['x'], pickup['y'], pickup['z']+platform['size'][2]
                              +spec['payload']['size_m'][2]/2+.001, 0,0,0])
    spec['pickup_payload'] = target
    world.append(box_model(target, '0.15 0.55 0.85 1'))
    # Add a slow all-model pose stream without changing dynamics.
    for model in world.findall('model'):
        if model.get('name', '').startswith('lower_'):
            plugin = ET.SubElement(model, 'plugin', filename='ignition-gazebo-pose-publisher-system',
                                   name='ignition::gazebo::systems::PosePublisher')
            text(plugin, 'publish_model_pose', 'true')
            text(plugin, 'publish_link_pose', 'false')
            text(plugin, 'use_pose_vector_msg', 'true')
            text(plugin, 'update_frequency', 20)
    # Declare a planar container-floor contact at the canonical floor datum.
    # The imported decimated floor varies by millimetres below this plane.
    # This is the ULD floor, never an elevated shelf supporting the upper layer.
    hull = assets['hull']
    center = (assets['world_container'] @ np.array([0,0,hull.floor_z-.005,1]))[:3]
    floor = box_model(dict(name='canonical_cargo_floor', size_m=[hull.length,hull.width,.01],
                           mass_kg=1., pose_world=[*center,0,0,0]), '0.25 0.35 0.45 1')
    floor.find('static').text = 'true'
    world.append(floor)
    ET.indent(root)
    ET.ElementTree(root).write(output/'overlay.sdf', encoding='utf-8', xml_declaration=True)
    for name, spawn in (
        (assets['scene']['gazebo']['container_model'], gazebo_container_spawn_pose(assets['scene'])),
        (assets['scene']['pedestal']['gazebo_model'], gazebo_pedestal_spawn_pose(assets['scene'])),
        (platform['gazebo_model'], pickup),
    ):
        world.append(original_model(repo, name, spawn))
    ET.indent(root)
    ET.ElementTree(root).write(output/'standalone.sdf', encoding='utf-8', xml_declaration=True)
    (output/'fixture.json').write_text(json.dumps(spec, indent=2)+'\n')
    return spec


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, default=ROOT)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    spec = write_scene(args.repo.resolve(), args.output.resolve())
    print(json.dumps({k:spec[k] for k in ('support_z_m','upper_headroom_m','lower_fill_fraction',
                                          'payload_ceiling_gap_m','suction_panel_depth_from_contact_m')}, indent=2))
    print(f"Scene: {args.output.resolve()} (24 dynamic support boxes; no action commands)")


if __name__ == '__main__':
    main()
