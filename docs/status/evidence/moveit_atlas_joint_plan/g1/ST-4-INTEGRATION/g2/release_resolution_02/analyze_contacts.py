"""Map retained native contact objects to actual O states; no new scene verdict."""
from pathlib import Path
from collections import Counter
import hashlib
import json

from rclpy.serialization import deserialize_message
from uld_occ_observer.srv import ReadCells, ReadSurface, IntegrateObservation, GetSnapshot, ReleaseSnapshot
from luggage_msgs.srv import ValidateRouteV2
from luggage_planning.joint_operation_observer import ObserverConfiguration, assemble_snapshot, identity_record
from luggage_planning.placement_prior_volume import blocked_volume_runs

ROOT = Path(__file__).resolve().parent


def main():
    folder = ROOT / 'resolution_0.02'
    cfg = ObserverConfiguration.load(folder / 'geometry.yaml', folder / 'observer.yaml')
    cells = [deserialize_message(p.read_bytes(), ReadCells.Response)
             for p in sorted(folder.glob('*observer_read_cells_response.cdr'))]
    surfaces = [deserialize_message(p.read_bytes(), ReadSurface.Response)
                for p in sorted(folder.glob('*observer_read_surface_response.cdr'))]
    states = [v for r in cells for v in r.states]
    fields = ('x', 'y', 'active_count', 'known_count', 'has_occupied', 'top_z')
    columns = [dict(zip(fields, row)) for s in surfaces
               for row in zip(*(getattr(s, k) for k in fields))]
    snap = assemble_snapshot(cfg, identity_record(cells[0].identity), cells[0].validity.generation,
                            columns, states, scene_revision='scene', request_id='retained-diagnostic')
    boxes = blocked_volume_runs(snap.volume, snap.surface)
    nx, ny, nz = cfg.dimensions_xyz
    runs = []
    for y in range(ny):
        for x in range(nx):
            start = None
            for z in range(nz + 1):
                index = x + nx * (y + ny * z)
                blocked = z < nz and snap.volume.active[index] and states[index] != 1
                if blocked and start is None:
                    start = z
                if not blocked and start is not None:
                    values = [states[x + nx * (y + ny * j)] for j in range(start, z)]
                    runs.append(dict(cell_xy=[x, y], z_indices=[start, z], state_counts=dict(Counter(values))))
                    start = None
    assert len(runs) == len(boxes)
    summary = json.loads((ROOT / 'summary.json').read_text())
    output = []
    for variant in summary['variants'][1:]:
        margin = variant['padding_m']
        request_file = next(folder.glob('*native_padding_' + str(margin) + '_request.cdr'))
        request = deserialize_message(request_file.read_bytes(), ValidateRouteV2.Request)
        scene = request.load_state_scenes[request.legs[0].load_state_index]
        objects = {o.id: o for o in scene.world.collision_objects}
        contacts = []
        for contact in variant['legs'][0]['contacts']:
            index = int(contact['body1'].rsplit('-', 1)[1])
            centre, dimensions = boxes[index]
            obj = objects[contact['body1']]
            position = obj.primitive_poses[0].position
            assert all(abs(a-b) < 1e-12 for a,b in zip(centre, (position.x, position.y, position.z)))
            assert all(abs(a+2*margin-b) < 1e-12 for a,b in zip(dimensions, obj.primitives[0].dimensions))
            contacts.append(dict(**contact, source_run=runs[index], unpadded_centre=centre,
                                 unpadded_dimensions=dimensions))
        counts = Counter('mixed' if len(c['source_run']['state_counts']) > 1 else
                         ('unknown' if 0 in c['source_run']['state_counts'] else 'occupied') for c in contacts)
        output.append(dict(padding_m=margin, reported_contacts=len(contacts),
                           reported_contact_source_counts=dict(counts), contacts=contacts))
    (ROOT / 'contact_source_analysis.json').write_text(json.dumps(output, indent=2))
    manifest = json.loads((ROOT / 'manifest.json').read_text())
    decoders = {'integrate': IntegrateObservation, 'observer_get_snapshot': GetSnapshot,
                'observer_read_cells': ReadCells, 'observer_read_surface': ReadSurface,
                'observer_release': ReleaseSnapshot, 'native_padding': ValidateRouteV2}
    for item in manifest['records']:
        path = ROOT / item['file']
        data = path.read_bytes()
        assert hashlib.sha256(data).hexdigest() == item['sha256']
        service = next(v for k,v in decoders.items() if path.name[5:].startswith(k))
        cls = service.Request if path.name.endswith('_request.cdr') else service.Response
        deserialize_message(data, cls)
    assert manifest['capture_complete'] and manifest['certifier_exit'] == 0
    assert all(x['observer_exit'] == 0 for x in manifest['native_exits'])
    manifest['capture_health'] = dict(cdr_hashes_and_decodes=len(manifest['records']),
                                     contact_source_geometry_parity=True, native_teardown=True)
    manifest['diagnostic_source_sha256'] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                           for p in [ROOT / 'replay.py', Path(__file__)]}
    (ROOT / 'manifest.json').write_text(json.dumps(manifest, indent=2))
    print(json.dumps([dict(padding_m=v['padding_m'], contact_sources=v['reported_contact_source_counts'])
                      for v in output], indent=2))


if __name__ == '__main__':
    main()
