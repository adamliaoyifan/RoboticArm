#!/usr/bin/env python3
"""Read-only replay of frozen payload geometry and rejected candidate states.

Run from a sourced Humble + workspace environment. No ROS node, simulator,
policy construction, planning call, or online scene mutation is performed.
"""
import argparse
import csv
from collections import Counter
import hashlib
from itertools import product
import json
from pathlib import Path
import subprocess
import sys

# Match distro matplotlib/mplot3d while retaining user-site plotly/trimesh.
sys.path.insert(0, '/usr/lib/python3/dist-packages')
import numpy as np
from scipy.spatial.transform import Rotation
import trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import plotly.graph_objects as go
from rclpy.serialization import deserialize_message
from moveit_msgs.srv import GetPlanningScene

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
from replay_collision_attribution import Urdf, _origin_of, _resolve_package_path

COLORS = {'measured': '#2274dc', 'attachment': '#ed8b00',
          'planning_input': '#00a878', 'scene_snapshot': '#b531c4',
          'historical_envelope': '#e33333'}
CORNERS = np.array(list(product((-1, 1), repeat=3)))
EDGES = [(i, j) for i in range(8) for j in range(i) if (i ^ j).bit_count() == 1]


def pose(t, q):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(q).as_matrix()
    result[:3, 3] = t
    return result


def message_pose(p):
    return pose([p.position.x, p.position.y, p.position.z],
                [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])


def vertices(size, transform):
    return CORNERS * np.asarray(size) / 2 @ transform[:3, :3].T + transform[:3, 3]


def load_meshes(urdf):
    meshes = []
    # Replay all six arm links geometrically, without changing the saved ACM.
    for number in range(1, 7):
        link = f'elfin_link{number}'
        for collision in urdf.link_els[link].findall('collision'):
            element = collision.find('geometry/mesh')
            if element is None:
                raise ValueError(f'Unsupported collision primitive on {link}')
            mesh = trimesh.load(_resolve_package_path(element.get('filename')),
                                force='mesh', process=False)
            mesh.vertices *= np.fromstring(element.get('scale', '1 1 1'), sep=' ')
            xyz, rotation = _origin_of(collision)
            origin = np.eye(4)
            origin[:3, :3], origin[:3, 3] = rotation, xyz
            meshes.append((link, mesh, origin))
    return meshes


def replay(run, payloads, probe):
    boundary = json.loads(next((run / 'selection_boundary').glob('*/request.json')).read_text())
    rows = json.loads((run / 'selection_response.json').read_text())['rows']
    states = {tuple(boundary['start_joints']): {'node_id': 'attach_start', 'slots': []}}
    for row in rows:
        for endpoint in row.get('staging_endpoints', []):
            state = states.setdefault(tuple(endpoint['q']), {'node_id': endpoint['node_id'], 'slots': []})
            if row.get('slot_index') not in state['slots']:
                state['slots'].append(row.get('slot_index'))
    urdf = Urdf((run / 'robot_expanded.urdf').read_text())
    meshes = load_meshes(urdf)
    records = [dict(id=i, q=list(q), **state) for i, (q, state) in enumerate(states.items())]
    transforms = [urdf.link_transforms(s['q']) for s in records]
    hits = []
    for link, mesh, origin in meshes:
        lines = [f'{len(mesh.vertices)} {len(mesh.faces)}']
        lines += [' '.join(map(str, v)) for v in mesh.vertices]
        lines += [' '.join(map(str, f)) for f in mesh.faces]
        cases = []
        for state, tf in zip(records, transforms):
            for kind, (size, offset) in payloads.items():
                cases.append(' '.join(map(str, [f'{state["id"]}:{kind}',
                    *(tf[link] @ origin).flat, *size,
                    *(tf['suction_contact_frame'] @ offset).flat])))
        lines += [str(len(cases)), *cases]
        result = subprocess.run([str(probe)], input='\n'.join(lines) + '\n',
                                text=True, capture_output=True, check=True)
        for line in result.stdout.splitlines():
            label, hit, distance, count, *raw = line.split()
            sid, kind = label.split(':')
            if hit == '1':
                hits.append(dict(state_id=int(sid), kind=kind, link=link,
                                 contacts=np.array(raw, float).reshape(-1, 4).tolist()))
    counts = {kind: {'link1_4_states': len({h['state_id'] for h in hits
                if h['kind'] == kind and h['link'] in [f'elfin_link{i}' for i in range(1, 5)]}),
                'all_six_by_link': dict(Counter(h['link'] for h in hits if h['kind'] == kind))}
              for kind in payloads}
    return dict(states=records, hits=hits, counts=counts, root_frame=urdf.root_link), transforms, meshes


def plot_case(out, name, result, transforms, meshes, payloads, sid):
    tf = transforms[sid]
    fig = plt.figure(figsize=(15, 6))
    ax = fig.add_subplot(121, projection='3d')
    section = fig.add_subplot(122)
    interactive = go.Figure()
    all_points = []
    for link, mesh, origin in meshes:
        t = tf[link] @ origin
        vv = mesh.vertices @ t[:3, :3].T + t[:3, 3]
        all_points.extend([vv.min(0), vv.max(0)])
        color = '#c9a340' if link == 'elfin_link2' else '#9ca5b0'
        ax.add_collection3d(Poly3DCollection(vv[mesh.faces], facecolor=color, alpha=.22))
        section.scatter(vv[:, 0], vv[:, 2], s=.1, color=color, alpha=.25)
        # Bounded HTML display only; FCL above always uses the complete mesh.
        display = trimesh.Trimesh(vertices=vv, faces=mesh.faces[::max(1, len(mesh.faces)//4000)], process=False)
        display.remove_unreferenced_vertices()
        dv = np.round(display.vertices, 6); df = display.faces
        interactive.add_trace(go.Mesh3d(x=dv[:, 0], y=dv[:, 1], z=dv[:, 2],
            i=df[:, 0], j=df[:, 1], k=df[:, 2],
            name=link, color=color, opacity=.25, showlegend=True))
    for kind, (size, offset) in payloads.items():
        vv = vertices(size, tf['suction_contact_frame'] @ offset)
        all_points.extend(vv)
        xyz = np.array([v for i, j in EDGES for v in [vv[i], vv[j], [np.nan]*3]])
        linestyle = '--' if kind == 'planning_input' else '-'
        for index, (i, j) in enumerate(EDGES):
            ax.plot(*vv[[i, j]].T, color=COLORS[kind], ls=linestyle,
                    lw=2 if kind == 'planning_input' else 1,
                    label=kind if index == 0 else None)
            section.plot(vv[[i, j], 0], vv[[i, j], 2], color=COLORS[kind], ls=linestyle)
        interactive.add_trace(go.Scatter3d(x=xyz[:, 0], y=xyz[:, 1], z=xyz[:, 2],
            mode='lines', name=kind, line=dict(color=COLORS[kind], width=4, dash='dash' if linestyle == '--' else 'solid')))
    contacts = np.array([c[:3] for h in result['hits'] if h['state_id'] == sid
                         and h['kind'] in ('attachment', 'historical_envelope')
                         and h['link'] in ('elfin_link1', 'elfin_link2', 'elfin_link3', 'elfin_link4')
                         for c in h['contacts']])
    if len(contacts):
        ax.scatter(*contacts.T, color='red', s=14, label='FCL contact samples')
        section.scatter(contacts[:, 0], contacts[:, 2], c='red', s=15)
        interactive.add_trace(go.Scatter3d(x=contacts[:, 0], y=contacts[:, 1], z=contacts[:, 2],
            mode='markers', name='FCL contact samples', marker=dict(color='red', size=4)))
    pp = np.array(all_points)
    center = (pp.max(0) + pp.min(0)) / 2
    radius = max(pp.max(0) - pp.min(0)) * .55
    ax.set(xlim=(center[0]-radius, center[0]+radius), ylim=(center[1]-radius, center[1]+radius),
           zlim=(center[2]-radius, center[2]+radius), xlabel='X [m]', ylabel='Y [m]', zlabel='Z [m]')
    ax.set_box_aspect((1, 1, 1)); ax.view_init(23, -55); ax.legend(fontsize=7)
    section.set(xlabel='X [m]', ylabel='Z [m]', title='XZ projection (projection overlap alone is not a collision)')
    section.set_aspect('equal'); section.grid(alpha=.2)
    links = sorted({h['link'] for h in result['hits'] if h['state_id'] == sid and h['kind'] in ('attachment', 'historical_envelope')})
    title = f'{name}: saved state {sid}; envelope contacts: {", ".join(links) or "none"}'
    fig.suptitle(title + '\nOffline candidate pose, NOT executed motion. Frame: ' + result['root_frame'], fontsize=11)
    fig.tight_layout(); fig.savefig(out / f'{name}.png', dpi=170); plt.close(fig)
    interactive.update_layout(title=title + '<br>Toggle legend layers; drag to rotate. Offline candidate pose.',
        scene=dict(aspectmode='data', xaxis_title='X m', yaxis_title='Y m', zaxis_title='Z m'))
    interactive.write_html(out / f'{name}.html', include_plotlyjs='directory')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--probe', type=Path, default=Path('/tmp/payload_contact_probe'))
    args = parser.parse_args(); out = args.output; out.mkdir(parents=True, exist_ok=True)
    evidence = ROOT / 'docs/status/evidence/occ_owner_b1'
    run = evidence / 'attachment_geometry_g4_20261022/live_02'
    boundary_path = next((run / 'selection_boundary').glob('*/request.json'))
    boundary = json.loads(boundary_path.read_text())
    raw = boundary['conservative_geometry']; env = raw['conservative']
    measured = (raw['measured_dimensions_wdh'], pose(raw['t_tool_box_translation'], raw['t_tool_box_quaternion_xyzw']))
    conservative = (env['dimensions_wdh'], pose(env['t_tcp_box_translation'], env['t_tcp_box_quaternion_xyzw']))
    scene_path = boundary_path.parent / 'planning_scene.cdr'
    scene = deserialize_message(scene_path.read_bytes(), GetPlanningScene.Response).scene
    attached = next(a for a in scene.robot_state.attached_collision_objects if a.object.id == 'pickup_box')
    assert attached.object.header.frame_id == attached.link_name == 'suction_contact_frame'
    assert len(attached.object.primitives) == len(attached.object.primitive_poses) == 1
    scene_payload = (list(attached.object.primitives[0].dimensions),
                     message_pose(attached.object.pose) @ message_pose(attached.object.primitive_poses[0]))
    payloads = dict(measured=measured, attachment=conservative, planning_input=conservative,
                    scene_snapshot=scene_payload)
    result, transforms, meshes = replay(run, payloads, args.probe)
    # Deterministic strongest link1/2 collision among captured staging candidates.
    ranks = Counter(h['state_id'] for h in result['hits'] if h['kind'] == 'attachment'
                    and h['link'] in ('elfin_link1', 'elfin_link2') and h['state_id'] != 0)
    sid = ranks.most_common(1)[0][0] if ranks else 1
    plot_case(out, 'g4_collision', result, transforms, meshes, payloads, sid)
    plot_case(out, 'g4_attach_start', result, transforms, meshes, payloads, 0)
    np.testing.assert_array_equal(scene_payload[0], conservative[0])
    result['selected_state_id'] = sid
    result['dimensions_mm'] = {k: (np.array(v[0])*1000).tolist() for k, v in payloads.items()}
    result['tool_transforms'] = {k: v[1].tolist() for k, v in payloads.items()}
    result['scene_vs_attachment_center_distance_m'] = float(np.linalg.norm(scene_payload[1][:3, 3]-conservative[1][:3, 3]))
    result['scene_vs_attachment_rotation_deg'] = float(np.rad2deg(Rotation.from_matrix(scene_payload[1][:3,:3].T @ conservative[1][:3,:3]).magnitude()))
    # Tool-normal near face must stay fixed; far face is permitted to grow.
    result['tool_z_bounds_m'] = {k: [float(vertices(*v)[:, 2].min()), float(vertices(*v)[:, 2].max())]
                                for k, v in payloads.items()}
    result['source_files_sha256'] = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in (boundary_path, scene_path, run/'robot_expanded.urdf', run/'selection_response.json')}
    (out/'g4_replay.json').write_text(json.dumps(result, indent=2)+'\n')
    # Original failure uses its saved historical envelope, without reconstruction.
    old_run = evidence / 'payload_envelope_g2_20261022/live_03'
    old_boundary = json.loads(next((old_run/'selection_boundary').glob('*/request.json')).read_text())
    grasp = old_boundary['grasp']
    nominal = (old_boundary['box_wdh'], np.linalg.inv(pose(grasp['position'], grasp['quaternion_xyzw'])))
    old_env = json.loads((old_run/'offline_anchor_payload_collisions.json').read_text())['envelope']
    old_payloads = dict(measured=nominal, historical_envelope=(old_env['dimensions_wdh'],
        pose(old_env['t_tcp_box_translation'], old_env['t_tcp_box_quaternion_xyzw'])))
    old_result, old_tf, old_mesh = replay(old_run, old_payloads, args.probe)
    assert old_result['counts']['measured']['link1_4_states'] == 16
    assert old_result['counts']['historical_envelope']['link1_4_states'] == 20
    slot_332_states = {s['id'] for s in old_result['states'] if 332 in s['slots']}
    old_sid = next(h['state_id'] for h in old_result['hits'] if h['kind']=='historical_envelope'
                   and h['link']=='elfin_link2' and h['state_id'] in slot_332_states)
    plot_case(out, 'original_g2_collision', old_result, old_tf, old_mesh, old_payloads, old_sid)
    old_result['selected_state_id'] = old_sid
    (out/'original_g2_replay.json').write_text(json.dumps(old_result, indent=2)+'\n')
    with (out/'state_collision_table.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['run', 'state_id', 'node_id', 'slot_indices', 'geometry', 'colliding_links'])
        for name, data in [('g4_live02', result), ('original_g2_live03', old_result)]:
            for state in data['states']:
                for kind in data['counts']:
                    links = sorted({h['link'] for h in data['hits'] if h['state_id']==state['id'] and h['kind']==kind})
                    writer.writerow([name, state['id'], state['node_id'], ' '.join(map(str,state['slots'])), kind, ' '.join(links)])
    # Local view exposes coincident attachment/planning layers without arm occlusion.
    fig, axes = plt.subplots(1, 2, figsize=(11, 5))
    for ax, indices, names in zip(axes, ((0, 1), (0, 2)), (('tool X', 'tool Y'), ('tool X', 'tool Z (away from contact)'))):
        for kind in ('measured', 'attachment', 'planning_input'):
            vv = vertices(*payloads[kind])*1000
            for i, j in EDGES:
                ax.plot(vv[[i,j],indices[0]], vv[[i,j],indices[1]], color=COLORS[kind],
                        ls='--' if kind=='planning_input' else '-', lw=2 if kind=='planning_input' else 1)
            ax.plot([], [], color=COLORS[kind], ls='--' if kind=='planning_input' else '-', label=kind)
        ax.scatter([0], [0], c='black', s=15); ax.set_aspect('equal'); ax.grid(alpha=.2)
        ax.set(xlabel=names[0]+' [mm]', ylabel=names[1]+' [mm]'); ax.legend(fontsize=8)
    fig.suptitle('Frozen geometry in tool frame | attachment and planning input coincide\nTool +Z points world -Z in these tool-down poses; contact face unchanged')
    fig.tight_layout(); fig.savefig(out/'envelope_comparison.png', dpi=170); plt.close(fig)
    print(json.dumps({'g4_states':len(result['states']), 'g4_counts':result['counts'],
        'scene_center_delta_m':result['scene_vs_attachment_center_distance_m'],
        'scene_rotation_delta_deg':result['scene_vs_attachment_rotation_deg'],
        'original_counts':old_result['counts'], 'output':str(out)}, indent=2))


if __name__ == '__main__':
    main()
