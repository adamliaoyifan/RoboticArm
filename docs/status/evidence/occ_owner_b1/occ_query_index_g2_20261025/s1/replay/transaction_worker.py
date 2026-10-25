"""Isolated baseline/candidate oracle worker; no module mixing, no ROS graph."""
import argparse
import hashlib
import json
from pathlib import Path
import random
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--seeds', type=int, default=20)
    parser.add_argument('--transactions', type=int, default=500)
    args = parser.parse_args()
    root = Path(args.root).resolve()
    sys.path[:0] = [str(root), str(root/'src/luggage_perception'), str(root/'src/luggage_description')]
    from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
    from luggage_perception.occupancy_map_store import OccupancyMapStore, MapUpdate, UpdateKind
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open('w') as stream:
        for seed in range(args.seeds):
            rng = random.Random(seed)
            mapper = CargoVolumeMapper((.12, .12, .12), (0, 0, .06), 0, .02, max_raycast_points=4)
            store = OccupancyMapStore(mapper, epoch='oracle')
            previous = None
            for transaction in range(args.transactions):
                choice = rng.randrange(10)
                point = (rng.uniform(-.055, .055), rng.uniform(-.055, .055), rng.uniform(.005, .115))
                op = str(transaction)
                box = ((0, 0, .06), (.06, .06, .06), 0.)
                if choice < 4:
                    update = MapUpdate(operation_id=op, kind=UpdateKind.INTEGRATE_VIEW, settled=True,
                        schema_version=1, geometry_hash=mapper.geometry_hash, points=(point,),
                        origin=(0, 0, .2), stamp_key=(seed, transaction), acquisition_stamp_ns=transaction+1)
                elif choice == 4:
                    update = MapUpdate(operation_id=op, kind=UpdateKind.ADD_BOX_V1, box=box)
                elif choice == 5:
                    update = MapUpdate(operation_id=op, kind=UpdateKind.REMOVE_BOX_V1, box=box)
                elif choice == 6 and previous is not None:
                    update = previous
                elif choice == 7:
                    update = MapUpdate(operation_id=op, kind=UpdateKind.RESET)
                elif choice == 8:
                    update = MapUpdate(operation_id='', kind=UpdateKind.RESET)
                else:
                    update = MapUpdate(operation_id=op, kind=UpdateKind.NOTE_EVENT,
                                       event='unobserved_change', event_stamp_ns=transaction+1)
                result = store.apply_update(update)
                previous = update
                digest = hashlib.sha256()
                for field in ('_grid', '_source', '_labels', '_instance_ids', '_sensor_seen',
                              '_sensor_values', '_sensor_labels', '_sensor_instance_ids'):
                    digest.update(bytes(getattr(mapper, field)))
                digest.update(repr(mapper._occupancy.values).encode())
                digest.update(repr(mapper._occupancy.geometry_locked).encode())
                # Reset epoch is intentionally random in both implementations;
                # compare semantic digest/revision, not independently created UUIDs.
                stream.write(json.dumps(dict(seed=seed, transaction=transaction, choice=choice,
                    status=result.status.value, reason=result.reason,
                    revision=mapper.map_revision, content_digest=result.snapshot.identity.content_digest,
                    facts=digest.hexdigest(), stats=mapper.stats()), sort_keys=True)+'\n')


if __name__ == '__main__':
    main()
