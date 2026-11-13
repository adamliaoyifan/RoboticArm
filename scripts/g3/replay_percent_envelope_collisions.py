#!/usr/bin/env python3
"""Compare nominal, frozen G2, and current G3 payloads at saved states."""
import argparse
from collections import Counter
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
from replay_collision_attribution import Urdf, _origin_of, _resolve_package_path
from luggage_planning.attachment.attachment_geometry_policy import load_attachment_geometry_policy as load_envelope_limits
from luggage_planning.planning_contract.payload_envelope import build_envelope


def pose(translation, quaternion):
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    matrix[:3, 3] = translation
    return matrix


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--probe', default='/tmp/phase2_mesh_box_collision_probe')
    args = parser.parse_args()

    boundary = json.loads(next((args.run / 'selection_boundary').glob('*/request.json')).read_text())
    rows = json.loads((args.run / 'selection_response.json').read_text())['rows']
    states = {}
    for row in rows:
        for item in row.get('staging_endpoints', []):
            states.setdefault(tuple(item['q']), item['node_id'])

    grasp = boundary['grasp']
    nominal = np.linalg.inv(pose(grasp['position'], grasp['quaternion_xyzw']))
    previous = json.loads((args.run / 'offline_anchor_payload_collisions.json').read_text())['envelope']
    current = build_envelope(
        boundary['box_wdh'], nominal[:3, 3],
        Rotation.from_matrix(nominal[:3, :3]).as_quat(),
        load_envelope_limits(args.config))
    payloads = {
        'nominal': (boundary['box_wdh'], nominal),
        'g2_fixed_margin': (
            previous['dimensions_wdh'],
            pose(previous['t_tcp_box_translation'], previous['t_tcp_box_quaternion_xyzw'])),
        'g3_percent_downward': (
            current.dimensions_wdh,
            pose(current.t_tcp_box_translation, current.t_tcp_box_quaternion_xyzw)),
    }

    urdf = Urdf((args.run / 'robot_expanded.urdf').read_text())
    transforms = [(name, list(q), urdf.link_transforms(q)) for q, name in states.items()]
    hits = []
    checked = []
    for link in ['elfin_link%d' % index for index in range(1, 5)]:
        for collision_index, collision in enumerate(urdf.link_els[link].findall('collision')):
            mesh_element = collision.find('geometry/mesh')
            if mesh_element is None:
                continue
            mesh = trimesh.load(_resolve_package_path(mesh_element.get('filename')),
                                force='mesh', process=False)
            mesh.vertices *= np.array([
                float(value) for value in mesh_element.get('scale', '1 1 1').split()])
            xyz, rotation = _origin_of(collision)
            origin = np.eye(4)
            origin[:3, :3] = rotation
            origin[:3, 3] = xyz
            cases = []
            labels = {}
            for state_index, (name, joints, transform) in enumerate(transforms):
                for kind, (size, offset) in payloads.items():
                    label = '%d_%s' % (state_index, kind)
                    cases.append(' '.join(map(str, [
                        label, *(transform[link] @ origin).flat, *size,
                        *(transform['suction_contact_frame'] @ offset).flat])))
                    labels[label] = dict(state=name, joints=joints, kind=kind,
                                         link=link, collision_index=collision_index)
            lines = ['%d %d' % (len(mesh.vertices), len(mesh.faces))]
            lines.extend(' '.join(map(str, vertex)) for vertex in mesh.vertices)
            lines.extend(' '.join(map(str, face)) for face in mesh.faces)
            lines.append(str(len(cases)))
            lines.extend(cases)
            result = subprocess.run(
                [args.probe], input='\n'.join(lines) + '\n', text=True,
                capture_output=True, check=True)
            for line in result.stdout.splitlines():
                label, hit, distance = line.split()
                if hit == '1':
                    hits.append(dict(labels[label], distance=float(distance)))
            checked.append(link)

    counts = {}
    for kind in payloads:
        kind_hits = [hit for hit in hits if hit['kind'] == kind]
        counts[kind] = {
            'unique_hit_states': len({hit['state'] for hit in kind_hits}),
            'hit_counts_by_link': dict(Counter(hit['link'] for hit in kind_hits)),
        }
    output = {
        'scope': '20 saved staging states; links1-4 meshes; no trajectory/world proof',
        'states': len(states),
        'checked_links': checked,
        'nominal_dimensions_wdh': boundary['box_wdh'],
        'g2_envelope': previous,
        'g3_envelope': current.to_dict(),
        'counts': counts,
        'hits': hits,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2) + '\n')
    print(json.dumps({key: value for key, value in output.items() if key != 'hits'}, indent=2))


if __name__ == '__main__':
    main()
