"""Isolated installed-node harness, using its unchanged production main().

Only setup writes synthetic facts, through Store.apply_update, before spinning.
No simulator, motion client, executor override or callback-group change.
SIGUSR1 requests a durable, transaction-locked failed-case capture.
"""
import argparse
from collections import deque
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import resource
import signal
import time


def save(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2, default=str)+'\n')
    temporary.replace(path)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out', required=True)
    p.add_argument('--scene', required=True)
    p.add_argument('--install', required=True)
    p.add_argument('--source', required=True)
    p.add_argument('--domain', type=int, required=True)
    p.add_argument('--resolution', type=float, choices=(.01, .02, .05), required=True)
    p.add_argument('--boxes', type=int, choices=(0, 5, 20), required=True)
    p.add_argument('--trace', action='store_true')
    p.add_argument('--profile-distance', action='store_true')
    a = p.parse_args()
    if not 100 <= a.domain <= 232:
        p.error('dedicated ROS domain required')
    os.environ.update(ROS_DOMAIN_ID=str(a.domain), ROS_LOCALHOST_ONLY='1')
    import numpy as np
    from ament_index_python.packages import get_package_prefix
    import luggage_perception.cargo_volume_mapper as mapper_module
    from luggage_perception.occupancy_map_store import MapUpdate, UpdateKind
    from luggage_perception.cargo_map_query_service import MapQueryService
    from luggage_perception.cargo_map_query_service import set_identity
    from luggage_perception.cargo_map_query import CellKey, QueryError
    from luggage_msgs.msg import CargoMapQueryIdentityV2
    from rosidl_runtime_py.convert import message_to_ordereddict

    out = Path(a.out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    install, source = Path(a.install).resolve(), Path(a.source).resolve()
    loaded = Path(mapper_module.__file__).resolve()
    if not loaded.is_relative_to(install):
        raise RuntimeError('mapper imported outside independent install: '+str(loaded))
    # Compare every installed perception Python module with this frozen source.
    hashes = {}
    for original in sorted((source/'src/luggage_perception/luggage_perception').glob('*.py')):
        installed = loaded.parent/original.name
        digest = hashlib.sha256(installed.read_bytes()).hexdigest()
        if digest != hashlib.sha256(original.read_bytes()).hexdigest():
            raise RuntimeError('installed/source mismatch: '+original.name)
        hashes[original.name] = digest
    node_path = Path(get_package_prefix('luggage_perception'))/'lib/luggage_perception/cargo_volume_mapper_node.py'
    if node_path.read_bytes() != (source/'src/luggage_perception/scripts/cargo_volume_mapper_node.py').read_bytes():
        raise RuntimeError('installed node differs from frozen source')
    spec = importlib.util.spec_from_file_location('g2_installed_mapper_node', node_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    manifest = dict(started_ns=time.time_ns(), setup='synthetic measured hits and actual geometry records',
        production_main=True, executor_override=False, domain=a.domain, resolution=a.resolution,
        requested_boxes=a.boxes, loaded_mapper=str(loaded), loaded_node=str(node_path),
        source_hashes=hashes, scene_sha256=hashlib.sha256(Path(a.scene).read_bytes()).hexdigest(),
        trace=a.trace, cpu_affinity=sorted(os.sched_getaffinity(0)))
    save(out/'manifest.json', manifest)
    owner = []
    events = deque(maxlen=100_000)
    if a.profile_distance:
        import cProfile
        import luggage_perception.cargo_distance_query as distance_module
        original_distance = distance_module.read_nearest
        profiled = []
        def profile_distance(handle,data,points,*args,**kwargs):
            if len(points) < 256 or profiled:
                return original_distance(handle,data,points,*args,**kwargs)
            profiled.append(True)
            profiler = cProfile.Profile()
            try:
                return profiler.runcall(original_distance,handle,data,points,*args,**kwargs)
            finally:
                profiler.dump_stats(str(out/'distance.cprofile'))
        distance_module.read_nearest = profile_distance
    if a.trace:
        original_open = MapQueryService._open

        def traced_open(self, query):
            begin = time.perf_counter_ns()
            try:
                return original_open(self, query)
            finally:
                events.append(dict(stage='snapshot', request_id=query.request_id,
                    started_ns=begin, elapsed_ns=time.perf_counter_ns()-begin))
        MapQueryService._open = traced_open
    original_init = module.CargoVolumeMapperNode.__init__

    def initialize(node):
        begin = time.perf_counter_ns()
        original_init(node)
        manifest['node_init_ns'] = time.perf_counter_ns()-begin
        owner.append(node)
        s, m = node._store, node._mapper
        reset_start = time.perf_counter_ns()
        reset = s.apply_update(MapUpdate(operation_id='setup-reset',kind=UpdateKind.RESET))
        if not reset.accepted:
            raise RuntimeError('setup reset refused: '+reset.reason)
        manifest['reset_ns'] = time.perf_counter_ns()-reset_start
        # These sites lie inside even the floor section of the deployed hull.
        boxes = [((x, y, m._hull.floor_z+.11+.24*z), (.35, .53, .22), 0.)
            for z in range(4) for y in (-.48, .18) for x in (-.50, 0., .50)]
        for i, box in enumerate(boxes[:a.boxes]):
            result = s.apply_update(MapUpdate(operation_id='setup-box-'+str(i),
                kind=UpdateKind.ADD_BOX_V1, box=box, box_id='runtime-box-'+str(i)))
            if not result.accepted:
                raise RuntimeError('setup geometry refused: '+result.reason)
        if len(m._placed_boxes) != a.boxes:
            raise RuntimeError('actual box count differs from requested load')
        # Hit-only setup makes many genuine unknown/observed frontier cells;
        # the Optional origin API is unchanged. This is not a ray throughput run.
        step = max(2, int(round(.08/a.resolution)))
        xyz = np.asarray([(x, y, z) for z in range(1, m.nz-1, step)
            for y in range(1, m.ny-1, step) for x in range(1, m.nx-1, step)])
        points = np.asarray([-m.inner_l/2, -m.inner_w/2, m._hull.floor_z])+(xyz+.5)*a.resolution
        result = s.apply_update(MapUpdate(operation_id='setup-hits', kind=UpdateKind.INTEGRATE_VIEW,
            expected_snapshot=s.identity, geometry_hash=m.geometry_hash,
            points=points, origin=None, stamp_key=(1000, 0), acquisition_stamp_ns=1_000_000_000,
            schema_version=1, settled=True))
        if not result.accepted:
            raise RuntimeError('setup hits refused: '+result.reason)
        with s.acquire_query_snapshot() as snapshot:
            old = CargoMapQueryIdentityV2()
            set_identity(old, snapshot.identity)
        patch_points = np.asarray([(-m.inner_l/2+(x+.5)*a.resolution,
            -m.inner_w/2+(y+.5)*a.resolution, m._hull.floor_z+(m.nz-.5)*a.resolution)
            for y in range(10) for x in range(10)])
        result = s.apply_update(MapUpdate(operation_id='setup-patch', kind=UpdateKind.INTEGRATE_VIEW,
            expected_snapshot=s.identity, geometry_hash=m.geometry_hash,
            points=patch_points, origin=None, stamp_key=(1001, 0), acquisition_stamp_ns=1_001_000_000,
            schema_version=1, settled=True))
        if not result.accepted:
            raise RuntimeError('setup patch refused')
        node._publish_all()
        # Isolated snapshot acquisition benchmark: no service or trace overhead.
        snapshots = []
        for _ in range(34):
            t = time.perf_counter_ns()
            snapshot = s.acquire_query_snapshot()
            snapshots.append(time.perf_counter_ns()-t)
            snapshot.close()
        rng = np.random.default_rng(20261025)
        point_reads = hashlib.sha256()
        with s.acquire_query_snapshot() as snapshot:
            for flat in rng.integers(0, m.nx*m.ny*m.nz, 10_000):
                flat = int(flat)
                key = CellKey(snapshot.identity.epoch, flat % m.nx, flat//m.nx % m.ny, flat//(m.nx*m.ny))
                try:
                    cell = snapshot.get_cell(key)
                    assert m._active[flat]
                    assert (cell.state, cell.source, cell.label, cell.instance_id) == (
                        m._grid[flat], m._source[flat], m._labels[flat], m._instance_ids[flat])
                    point_reads.update(str((flat, cell.state, cell.source, cell.label, cell.instance_id)).encode())
                except QueryError as error:
                    assert not m._active[flat] and error.reason == 'OUTSIDE_HULL'
                    point_reads.update(str((flat, 'OUTSIDE_HULL')).encode())
        manifest.update(ready_ns=time.time_ns(), actual_boxes=len(m._placed_boxes),
            setup_elapsed_ns=time.perf_counter_ns()-begin, grid_shape=[m.nx, m.ny, m.nz],
            floor_z=m._hull.floor_z, ceiling_z=m._hull.ceiling_z,
            initial_hits=len(points), patch_columns=len(patch_points),
            since_identity=message_to_ordereddict(old), snapshot_ns=snapshots,
            point_reads=dict(count=10_000, seed=20261025, exact_native_match=True, sha256=point_reads.hexdigest()),
            resources=dict(s._query_registry.resource_summary()),
            rss_peak_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
        save(out/'ready.json', manifest)
    module.CargoVolumeMapperNode.__init__ = initialize

    def capture(_signum=None, _frame=None):
        if not owner:
            return
        from occ_query_oracle_fixture import primary_arrays
        node = owner[0]
        with node._store._txn_lock:
            np.savez_compressed(out/'boundary_map.npz', **primary_arrays(node._mapper))
            save(out/'boundary_exports.json', dict(v1=node._store.v1_surface_dict(),
                stats=node._mapper.stats()))
        save(out/'capture.json', dict(capture_complete=True, replay_possible=True,
            captured_ns=time.time_ns(), input='ready.json synthetic setup + request CDR files',
            window='transaction boundary after stopped scoring; heartbeat window in parent trace'))
    signal.signal(signal.SIGUSR1, capture)
    stop_requested = []
    def stop_spin(_signum, _frame):
        # SIGINT also shuts rclpy's context, while production main performs
        # its own shutdown. A private test signal only exits its spin loop.
        stop_requested.append(time.time_ns())
        raise KeyboardInterrupt()
    signal.signal(signal.SIGUSR2, stop_spin)
    try:
        module.main(['--ros-args', '-p', 'scene_tf_config:='+str(Path(a.scene).resolve()),
                     '-p', 'resolution:='+str(a.resolution)])
    except Exception as error:
        # Constructor failures happen before production main's finally.
        # Tear down only resources created by this fixture in that case.
        import rclpy
        if stop_requested and not rclpy.ok() and 'rcl_shutdown already called' in str(error):
            manifest['post_scoring_teardown_error'] = repr(error)
        else:
            capture()
            for node in owner:
                node.shutdown_tf()
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            raise
    finally:
        manifest.update(ended_ns=time.time_ns(),
            rss_peak_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
        save(out/'manifest.json', manifest)
        with (out/'trace.jsonl').open('w') as stream:
            for event in events:
                stream.write(json.dumps(event)+'\n')


if __name__ == '__main__':
    main()
