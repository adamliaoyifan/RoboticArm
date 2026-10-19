#!/usr/bin/env python3
"""Recover the exact pre-Select surface publication from a captured sqlite bag.

No ROS graph, scene mutation, inferred geometry or execution authorization.
The typed publication must equal Select's embedded surface. The accompanying
legacy publication must agree with its grid and the preceding captured stats.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sqlite3


def check_legacy(surface, record, stats, inner_size):
    from luggage_planning.occupancy_place_paths import _legacy_view_of_surface_contract
    expected = _legacy_view_of_surface_contract(record, inner_size, surface['geometry_descriptor'])
    for key in ('map_revision', 'geometry_hash', 'resolution', 'nx', 'ny',
                'inner_size', 'origin_local', 'state', 'confidence'):
        if surface.get(key) != expected[key]:
            raise ValueError('surface differs from typed request: ' + key)
    for i, row in enumerate(expected['height']):
        for j, height in enumerate(row):
            if expected['state'][i][j] != 'unknown':
                measured = surface['height'][i][j]
                if not math.isfinite(measured) or abs(measured - height) > 1e-9:
                    raise ValueError('surface differs from typed request: height')
    if not stats.success:
        raise ValueError('captured stats unsuccessful')
    for key, value in (
        ('map_revision', stats.map_revision), ('geometry_hash', stats.geometry_hash),
        ('acquisition_stamp_ns', stats.last_acquisition_stamp_ns),
        ('map_source', stats.map_source), ('placed_count', stats.placed_count),
        ('cargo_observed', stats.cargo_observed),
    ):
        if surface.get(key) != value:
            raise ValueError('surface differs from preceding stats: ' + key)
    if (surface.get('occupancy_confirmed') is not True
            or surface.get('pending_changes') != [] or surface.get('change_event')):
        raise ValueError('surface not settled/confirmed')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--select-index', type=int, default=0)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.select_index < 0:
        parser.error('select-index must be nonnegative')
    from rclpy.serialization import deserialize_message, serialize_message
    from luggage_msgs.srv import SelectPlacePathV2, GetCargoMapStats
    from luggage_msgs.msg import SurfaceContractV2
    from std_msgs.msg import String
    from luggage_planning.place_service import request_dict
    boundary = args.capture / 'dumps/typed_service_boundaries'
    rows = [json.loads(line) for line in (boundary / 'boundaries.jsonl').read_text().splitlines()]
    indices = [i for i, r in enumerate(rows)
               if r['boundary'] == '/trajectory_selector/select_place_path_v2:request']
    index = indices[args.select_index]
    request_path = boundary / rows[index]['file']
    request = deserialize_message(request_path.read_bytes(), SelectPlacePathV2.Request)
    record = request_dict(request)['surface']
    stats_row = next(r for r in reversed(rows[:index])
                     if r['boundary'] == '/cargo_map/get_stats:response')
    stats_path = boundary / stats_row['file']
    stats = deserialize_message(stats_path.read_bytes(), GetCargoMapStats.Response)
    typed, legacy, rejected = [], {}, []
    bags = sorted((args.capture / 'bag').glob('*.db3'))
    if not bags:
        raise ValueError('recorded sqlite bag required')
    for bag in bags:
        with sqlite3.connect(bag.resolve().as_uri() + '?mode=ro', uri=True) as connection:
            topics = connection.execute('SELECT id,name,type FROM topics WHERE name IN (?,?)',
                ('/luggage/cargo_map/surface_2d', '/luggage/cargo_map/surface_v2')).fetchall()
            for topic_id, name, kind in topics:
                expected_kind = 'std_msgs/msg/String' if name.endswith('surface_2d') else 'luggage_msgs/msg/SurfaceContractV2'
                if kind != expected_kind:
                    raise ValueError('unexpected recorded topic type')
                for stamp, data in connection.execute(
                        'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp', (topic_id,)):
                    source = dict(bag=str(bag.resolve()), topic=name, receive_ns=stamp,
                                  cdr_sha256=hashlib.sha256(data).hexdigest())
                    if name.endswith('surface_v2'):
                        if deserialize_message(data, SurfaceContractV2) == request.surface:
                            typed.append(source)
                        continue
                    text = deserialize_message(data, String).data
                    surface = json.loads(text)
                    if surface.get('map_revision') != request.map_revision:
                        continue
                    try:
                        check_legacy(surface, record, stats, list(request.inner_size))
                    except (ValueError, KeyError, IndexError, TypeError) as exc:
                        rejected.append(dict(source, reason=str(exc)))
                        continue
                    digest = hashlib.sha256(json.dumps(surface, sort_keys=True).encode()).hexdigest()
                    legacy.setdefault(digest, (text, source, bytes(data)))
    if not typed or len(legacy) != 1:
        raise ValueError('surface join requires typed match and unique legacy content: typed=%d legacy=%d rejected=%d'
                         % (len(typed), len(legacy), len(rejected)))
    text, source, data = next(iter(legacy.values()))
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / 'surface_2d.json').write_text(text)
    (args.output / 'surface_2d.cdr').write_bytes(data)
    (args.output / 'surface_v2.cdr').write_bytes(serialize_message(request.surface))
    manifest = dict(scope='exact recorded pre-Select surface, not execution authorization',
        select_index=args.select_index, map_revision=request.map_revision,
        inputs={str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in (request_path, stats_path)},
        legacy_source=source, typed_sources=typed, rejected_same_revision=rejected)
    (args.output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps(dict(map_revision=request.map_revision, typed_matches=len(typed), output=str(args.output))))


if __name__ == '__main__':
    main()
