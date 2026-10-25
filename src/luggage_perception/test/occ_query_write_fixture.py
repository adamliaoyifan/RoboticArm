"""Frozen-root whole-write worker, with explicit raw and selected-ray loads.

Each measured transaction starts from a newly constructed identical map.
Setup and result hashing are excluded; Store update and both legacy JSON
publication preparations are included. No production fusion or ray changes.
"""
import argparse
import hashlib
import itertools
import json
from pathlib import Path
import resource
import sys
import time


def make_points(hull, count, outside, crossing):
    import numpy as np
    # 1000 distinct endpoints per class at every tested resolution. Raw
    # multiplicity is explicit, not confused with sampled ray throughput.
    inside = np.asarray([(x, y, hull.floor_z+z) for z in np.linspace(.10, .91, 10)
        for y in np.linspace(-.70, .11, 10) for x in np.linspace(-.405, .405, 10)])
    exterior = inside.copy()
    if crossing:
        exterior[:, 0] += 1.6
    else:
        # Height-filtered exterior points do not spend a ray budget. This
        # realizes zero crossing with a nonzero raw-outside fraction.
        exterior[:, 2] = hull.floor_z-.20
    n_out = int(round(count*outside))
    points = np.concatenate((np.resize(inside, (count-n_out, 3)), np.resize(exterior, (n_out, 3))))
    return points, (0., 0., hull.floor_z+.10)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--resolution', type=float, required=True)
    p.add_argument('--boxes', type=int, required=True)
    p.add_argument('--points', type=int, required=True)
    p.add_argument('--outside', type=float, required=True)
    p.add_argument('--crossing', type=float, required=True)
    p.add_argument('--index', type=int, required=True)
    p.add_argument('--input', required=True)
    p.add_argument('--profile', action='store_true')
    a = p.parse_args()
    root = Path(a.root).resolve()
    sys.path[:0] = [str(root/'src/luggage_perception'), str(root/'src/luggage_description'), str(root)]
    import numpy as np
    import yaml
    from luggage_description.container_geometry import descriptor_from_scene_config
    from luggage_description.container_geometry_batch import contains_points
    from luggage_perception.cargo_occ_input import prepare_occ_observation, OccInputPolicy
    from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
    from luggage_perception.occupancy_map_store import OccupancyMapStore, MapUpdate, UpdateKind
    from occ_query_oracle_fixture import primary_arrays
    out = Path(a.out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    loaded = Path(sys.modules[CargoVolumeMapper.__module__].__file__).resolve()
    if not loaded.is_relative_to(root):
        raise RuntimeError('mixed product roots')
    scene = yaml.safe_load((root/'src/luggage_description/config/scene_tf.yaml').read_text())
    hull = descriptor_from_scene_config(scene)
    begin = time.perf_counter_ns()
    m = CargoVolumeMapper((hull.length, hull.width, hull.ceiling_z-hull.floor_z),
        (0., 0., (hull.ceiling_z+hull.floor_z)/2), 0., a.resolution,
        geometry_descriptor=hull.descriptor(), max_raycast_points=2000)
    s = OccupancyMapStore(m, epoch='paired-write', clock_ns=lambda: 1_000_000_000)
    for i, (z, y, x) in enumerate(itertools.product(range(4), (-.48, .18), (-.50, 0., .50))):
        if i >= a.boxes:
            break
        result = s.apply_update(MapUpdate(operation_id='box-'+str(i), kind=UpdateKind.ADD_BOX_V1,
            box=((x, y, hull.floor_z+.11+.24*z), (.35, .53, .22), 0.), box_id='write-box-'+str(i)))
        assert result.accepted
    assert len(m._placed_boxes) == a.boxes
    setup_ns = time.perf_counter_ns()-begin
    points, origin = make_points(hull, a.points, a.outside, a.crossing)
    minimum_z = hull.floor_z+.05
    prepared = prepare_occ_observation(points, origin, hull, a.resolution,
        OccInputPolicy(minimum_z, tuple(m.center), m.yaw))
    rays = prepared.ray_points
    if len(rays) > m.max_raycast_points:
        rays = rays[::max(1, len(rays)//m.max_raycast_points)]
    # For this convex hull and strictly interior origin, exterior endpoints
    # cross a boundary; interior endpoints remain inside for the whole ray.
    actual_crossing = float((~contains_points(hull, rays)).mean()) if len(rays) else 0.
    actual_outside = float((~contains_points(hull, points)).mean())
    assert actual_crossing == a.crossing and abs(actual_outside-a.outside) < 1e-12
    input_path = Path(a.input)
    if input_path.exists():
        assert np.array_equal(np.load(input_path), points)
    else:
        np.save(input_path, points)
    update = MapUpdate(operation_id='measured', kind=UpdateKind.INTEGRATE_VIEW,
        expected_snapshot=s.identity, geometry_hash=m.geometry_hash, points=points,
        origin=origin, minimum_z=minimum_z, stamp_key=(1, 0), acquisition_stamp_ns=1_000_000_000,
        settled=True, schema_version=1)
    before = primary_arrays(m)
    if a.profile:
        import cProfile
        profiler = cProfile.Profile()
        profiler.enable()
    begin = time.perf_counter_ns()
    result = s.apply_update(update)
    updated = time.perf_counter_ns()
    v1 = json.dumps(s.v1_surface_dict(result.snapshot), sort_keys=True)
    ledger = json.dumps(s.committed_dict('container_link', {'translation': [0., 0., 0.],
        'rotation_rpy': [0., 0., 0.]}, result.snapshot), sort_keys=True)
    ended = time.perf_counter_ns()
    if a.profile:
        profiler.disable()
        profiler.dump_stats(str(out/'write.cprofile'))
    row = dict(index=a.index, resolution=a.resolution, boxes=a.boxes, raw_points=len(points),
        outside=a.outside, crossing=a.crossing, actual_outside=actual_outside,
        actual_crossing=actual_crossing, selected_rays=len(rays), candidate_rays=len(prepared.ray_points),
        hit_points=len(prepared.hit_points), input_sha256=hashlib.sha256(points.tobytes()).hexdigest(),
        setup_ns=setup_ns, update_ns=updated-begin, publication_preparation_ns=ended-updated,
        elapsed_ns=ended-begin, accepted=result.accepted, reason=result.reason,
        identity=list(result.identity), v1_sha256=hashlib.sha256(v1.encode()).hexdigest(),
        ledger_sha256=hashlib.sha256(ledger.encode()).hexdigest(),
        facts={k:hashlib.sha256(v.tobytes()).hexdigest() for k,v in primary_arrays(m).items()},
        stats=s.mapper.stats(), rss_peak_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        loaded_mapper=str(loaded))
    if hasattr(m, '_query_read_owner') and m._query_read_owner is not None:
        row['index_work'] = dict(m._query_read_owner.work)
        row['resources'] = dict(s._query_registry.resource_summary())
    (out/'result.json').write_text(json.dumps(row, indent=2)+'\n')
    if a.index == -1 or not result.accepted:
        np.savez_compressed(out/'before.npz', **before)
        np.savez_compressed(out/'after.npz', **primary_arrays(m))
    if not result.accepted:
        raise RuntimeError('write refused: '+result.reason)


if __name__ == '__main__':
    main()
