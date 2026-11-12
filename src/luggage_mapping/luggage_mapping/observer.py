"""Read-only O snapshot adapter: one lease for surface and volume.

Owner configuration bytes pin resolution and hull; its native digest contract
detects an accidental pairing with another observer. No G confirmation flags
are invented. All RPCs use the caller's one deadline and single-flight port.
"""
from dataclasses import asdict, dataclass
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import threading

import yaml

from luggage_description.cargo_grid_contract import MAX_VOLUME_CELLS
from luggage_description.container_geometry import normalize_descriptor
from luggage_mapping.source import OperationMapSource
from luggage_mapping.surface import ConfirmedSurface, freeze
from luggage_mapping.regions import CellBlock, CellRegion, NativeGrid


@dataclass(frozen=True)
class ObserverConfiguration:
    descriptor_json: bytes
    resolution_m: float
    config_digest: str
    dimensions_xyz: tuple

    @classmethod
    def load(cls, geometry_file, config_file):
        geometry_bytes, config_bytes = Path(geometry_file).read_bytes(), Path(config_file).read_bytes()
        envelope, config = yaml.safe_load(geometry_bytes), yaml.safe_load(config_bytes)
        canonical = envelope['descriptor_json'].encode()
        if (envelope['schema'] != 'uld_geometry_export_v1'
                or hashlib.sha256(canonical).hexdigest() != envelope['descriptor_sha256']):
            raise ValueError('OBSERVER_GEOMETRY_ARTIFACT_CORRUPT')
        descriptor = json.loads(canonical)
        if normalize_descriptor(descriptor).descriptor() != descriptor:
            raise ValueError('OBSERVER_NORMALIZED_GEOMETRY_REQUIRED')
        allowed = {'resolution_m', 'depth_z_min_m', 'depth_z_max_m', 'clock_domain', 'discretize_endpoints'}
        resolution = float(config['resolution_m'])
        if (set(config)-allowed or config['clock_domain'] != 'ros'
                or not math.isfinite(resolution) or resolution <= 0):
            raise ValueError('OBSERVER_CONFIGURATION_INVALID')
        # Match UldDomain's exact ceil; an epsilon can silently drop its last
        # voxel layer for a floating-point extent just above an integer.
        dimensions = tuple(math.ceil(v/resolution) for v in
            (descriptor['length'], descriptor['width'], descriptor['ceiling_z']-descriptor['floor_z']))
        if min(dimensions) <= 0 or math.prod(dimensions) > MAX_VOLUME_CELLS:
            raise ValueError('OBSERVER_VOLUME_CONTRACT_LIMIT')
        digest = hashlib.sha256(geometry_bytes+b'\n'+config_bytes+b'\nprobability-profile-v1').hexdigest()
        return cls(canonical, resolution, digest, dimensions)

    def descriptor(self):
        return json.loads(self.descriptor_json)


IDENTITY_FIELDS = ('epoch', 'map_revision', 'observed_revision', 'last_integrated_stamp_ns',
                   'geometry_hash', 'config_digest', 'clock_domain')
# ReadCells/ReadSurface owner transport maximum, not a spatial coverage limit.
OBSERVER_PAGE_CELLS = 65536


def identity_record(identity):
    return {name: getattr(identity, name) for name in IDENTITY_FIELDS}


def admit_response(response, identity, generation):
    if (response.status != 'OK' or identity_record(response.identity) != identity
            or response.validity.state != 'VALID' or response.validity.epoch != identity['epoch']
            or response.validity.generation != generation):
        raise ValueError('OBSERVER_RESPONSE_IDENTITY_OR_VALIDITY_MISMATCH')


@dataclass(frozen=True)
class ObserverVolume:
    """Admitted O volume; inactive voxels are separate from observed free."""
    source: OperationMapSource
    resolution_m: float
    origin_container: tuple
    dimensions_xyz: tuple
    states: bytes
    active: tuple


@dataclass(frozen=True)
class ObserverSnapshot:
    source: OperationMapSource
    surface: ConfirmedSurface
    volume: ObserverVolume


def _owner_bridge():
    # O ships the canonical pure converters alongside its executable. Resolve
    # its ament-owned file explicitly; do not require a script-dir PYTHONPATH.
    from ament_index_python.packages import get_package_prefix
    path = Path(get_package_prefix('uld_occ_observer'))/'lib/uld_occ_observer/occ_surface_bridge.py'
    spec = importlib.util.spec_from_file_location('_operation_observer_bridge', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def assemble_snapshot(config, identity, generation, columns, states, *, scene_revision, request_id):
    """Convert only complete identity-admitted pages, preserving owner semantics."""
    from rclpy.serialization import serialize_message
    from luggage_description.frame_diagnostics import BoundaryContext
    from luggage_description.ros_frame_bindings.surfaces import surface_to_message
    nx, ny, nz = config.dimensions_xyz
    if (len(columns) != nx*ny or len(states) != nx*ny*nz
            or any(v not in (0, 1, 2, 3) for v in states)):
        raise ValueError('OBSERVER_INCOMPLETE_GRID')
    by_index = {}
    for column in columns:
        x, y = column['x'], column['y']
        if not 0 <= x < nx or not 0 <= y < ny or (x,y) in by_index:
            raise ValueError('OBSERVER_AMBIGUOUS_SURFACE_INDEX')
        values = [states[x+nx*(y+ny*z)] for z in range(nz)]
        active = sum(v != 3 for v in values)
        known = sum(v in (1,2) for v in values)
        occupied = 2 in values
        # The independent surface and volume views must describe the same lease.
        if (column['active_count'] != active or column['known_count'] != known
                or column['has_occupied'] != occupied):
            raise ValueError('OBSERVER_SURFACE_VOLUME_DISAGREEMENT')
        if occupied:
            descriptor = config.descriptor()
            top = min(descriptor['ceiling_z'], descriptor['floor_z']+
                      (max(z for z,v in enumerate(values) if v == 2)+1)*config.resolution_m)
            if not math.isfinite(column['top_z']) or abs(column['top_z']-top) > 1e-8:
                raise ValueError('OBSERVER_SURFACE_TOP_DISAGREEMENT')
        by_index[x,y] = column
    geometry = config.descriptor()
    geometry['_resolution'] = config.resolution_m
    bridge = _owner_bridge()
    identity = dict(identity, validity_generation=generation)
    legacy = bridge.build_surface_map(columns, geometry, identity)
    if 'error' in legacy:
        raise ValueError(legacy['error'])
    record = bridge.build_surface_v2_record(legacy, geometry, identity, scene_revision, request_id)
    # Record the actual acquisition, never a query/confirmation wall time.
    record['stamp_ns'] = identity['last_integrated_stamp_ns']
    record['time_basis'] = 'acquisition'
    content = hashlib.sha256(freeze(dict(identity=identity, surface=record))+bytes(states)).hexdigest()
    source = OperationMapSource(scene_revision=scene_revision, content_sha256=content, **identity)
    record['content_digest'] = content
    context = BoundaryContext('JOINT_O_OCC', 'uld_occ_observer', 'joint_operation', request_id,
        scene_revision, 'container_link', '', 'acquisition', source.geometry_hash, source.map_revision)
    cdr = serialize_message(surface_to_message(record, context))
    metadata = dict(legacy, source='observer', acquisition_stamp_ns=source.last_integrated_stamp_ns,
                    observer_source=asdict(source))
    surface = ConfirmedSurface(freeze(record), cdr, freeze(metadata))
    volume = ObserverVolume(source, config.resolution_m,
        (*record['origin_xy_container'], record['floor_z_container']), config.dimensions_xyz,
        bytes(0 if v == 3 else v for v in states), tuple(v != 3 for v in states))
    return ObserverSnapshot(source, surface, volume)


class ObserverSnapshotPort:
    def __init__(self, node, *, configuration, call, prefix='/uld_occ_observer', max_age_ms=1000,
                 recorder=None):
        from uld_occ_observer.srv import GetSnapshot, ReadSurface, ReadCells, ReleaseSnapshot
        if type(max_age_ms) is not int or max_age_ms < 0 or type(configuration) is not ObserverConfiguration:
            raise ValueError('EXPLICIT_OBSERVER_CONFIGURATION_AND_AGE_REQUIRED')
        self.config, self.call, self.max_age_ms = configuration, call, max_age_ms
        self.recorder = recorder
        self.services = dict(get_snapshot=GetSnapshot, read_surface=ReadSurface, read_cells=ReadCells,
                             release_snapshot=ReleaseSnapshot)
        self.clients = {name:node.create_client(service,prefix.rstrip('/')+'/'+name)
                        for name,service in self.services.items()}
        self._pending_releases, self._release_lock = [], threading.Lock()
        self._capture_fault = None

    def _release(self, lease):
        if lease.handle:
            with self._release_lock:
                self._pending_releases = [f for f in self._pending_releases if not self._released(f)]
                request = self.services['release_snapshot'].Request(handle=lease.handle)
                # Evidence failure cannot prevent mandatory resource release.
                try:
                    if self.recorder:self.recorder('observer_release_request',request)
                except Exception as error:self._capture_fault = error
                try:future = self.clients['release_snapshot'].call_async(request)
                except Exception as error:
                    self._capture_fault = error
                    raise
                self._pending_releases.append(future)
                if self.recorder:
                    def record_result(completed):
                        try:
                            if completed.exception() is None:
                                self.recorder('observer_release_response',completed.result())
                        except Exception as error:self._capture_fault = error
                    future.add_done_callback(record_result)

    @staticmethod
    def _released(future):
        return (future.done() and future.exception() is None and future.result() is not None
                and future.result().status == 'OK')

    def _get(self, request, deadline):
        return self.call(self.clients['get_snapshot'], request, deadline, 'observer_get_snapshot',
                         late_response=self._release)

    def capture(self, *, scene_revision, request_id, expected_epoch, expected_revision, deadline,
                require_latest=True):
        if expected_epoch <= 0:
            raise ValueError('OBSERVER_EXPECTED_EPOCH_REQUIRED')
        if self._capture_fault is not None:
            raise RuntimeError('OBSERVER_RELEASE_CAPTURE_FAILED') from self._capture_fault
        # A timed-out release remains owned until its response arrives; do not
        # repeatedly allocate leases while the previous lifetime is uncertain.
        with self._release_lock:
            if any(not self._released(f) for f in self._pending_releases):
                raise RuntimeError('OBSERVER_RELEASE_BACKEND_BUSY')
        exact = expected_revision is not None
        request = self.services['get_snapshot'].Request(expected_epoch=expected_epoch,
            exact_revision=exact, expected_revision=expected_revision if exact else 0,
            max_age_ms=self.max_age_ms, allow_stale=False)
        lease = self._get(request,deadline)
        handle = lease.handle
        try:
            identity = identity_record(lease.identity)
            admit_response(lease,identity,lease.validity.generation)
            if (not handle or lease.stale or not 0 <= lease.age_ns <= self.max_age_ms*1_000_000
                    or identity['epoch'] != expected_epoch
                    or (exact and identity['map_revision'] != expected_revision)
                    or identity['config_digest'] != self.config.config_digest
                    or identity['geometry_hash'] != self.config.descriptor()['geometry_hash']
                    or identity['clock_domain'] != 'ros'):
                raise ValueError('OBSERVER_LEASE_CONFIGURATION_OR_FRESHNESS_MISMATCH')
            generation = lease.validity.generation
            nx, ny, nz = self.config.dimensions_xyz
            columns = self._pages('read_surface',handle,nx*ny,identity,generation,deadline)
            block = self.read_region(handle, identity, generation,
                                     CellRegion((0, 0, 0), (nx, ny, nz)), deadline=deadline)
            # The surface bridge uses the owner's explicit out-of-domain code;
            # shared blocks instead expose it as unknown plus inactive coverage.
            states = bytes(s if a else 3 for s, a in zip(block.states, block.active))
            snapshot = assemble_snapshot(self.config,identity,generation,columns,states,
                                        scene_revision=scene_revision,request_id=request_id)
            if require_latest and not self.current(snapshot.source,deadline=deadline):
                raise ValueError('OBSERVER_SOURCE_CHANGED_DURING_CAPTURE')
            return snapshot
        finally:
            self._release(lease)

    def read_region(self, handle, identity, generation, region, *, deadline):
        """Read explicit coverage inside an already admitted, still-owned lease.

        This low-level operation never acquires/releases/rebinds a snapshot. The
        capture owner must validate freshness and release even after a failure.
        """
        descriptor = self.config.descriptor()
        grid = NativeGrid(self.config.resolution_m,
                          (-descriptor['length']/2, -descriptor['width']/2, descriptor['floor_z']),
                          self.config.dimensions_xyz)
        region.validate_grid(grid)
        raw = self._pages('read_cells', handle, region.count, identity, generation,
                          deadline, region=region)
        provenance = freeze(dict(producer='observer', identity=identity,
                                 validity_generation=generation))
        return CellBlock(grid, region, provenance, bytes(0 if s == 3 else s for s in raw),
                         bytes(s != 3 for s in raw))

    def _pages(self, name, handle, count, identity, generation, deadline, *, region=None):
        if name == 'read_cells':
            region = region or CellRegion((0, 0, 0), self.config.dimensions_xyz)
            if (region.count != count or any(b > n for b, n in
                    zip(region.end, self.config.dimensions_xyz))):
                raise ValueError('OBSERVER_REGION_COUNT_OR_BOUNDS')
        elif region is not None:
            raise ValueError('OBSERVER_SURFACE_IS_FULL_DOMAIN')
        values, cursor = (bytearray() if name == 'read_cells' else []), 0
        for _ in range(math.ceil(count/OBSERVER_PAGE_CELLS)):
            args = dict(handle=handle,cursor=cursor,limit=OBSERVER_PAGE_CELLS)
            if name == 'read_cells':
                args.update(begin=list(region.begin),end=list(region.end))
            page = self.call(self.clients[name],self.services[name].Request(**args),deadline,'observer_'+name)
            admit_response(page,identity,generation)
            if name == 'read_cells':
                chunk = page.states
                if any(v not in (0, 1, 2, 3) for v in chunk):
                    raise ValueError('OBSERVER_INVALID_CELL_STATE')
            else:
                fields = ('x','y','active_count','known_count','has_occupied','top_z')
                if len({len(getattr(page,field)) for field in fields}) != 1:
                    raise ValueError('OBSERVER_SURFACE_PAGE_SHAPE')
                chunk = [dict(zip(fields,row)) for row in zip(*(getattr(page,f) for f in fields))]
            if (not len(chunk) or len(chunk) > OBSERVER_PAGE_CELLS
                    or page.next_cursor != cursor+len(chunk) or cursor+len(chunk) > count
                    or page.done != (cursor+len(chunk) == count)):
                raise ValueError('OBSERVER_PAGE_PROGRESS_OR_COMPLETENESS')
            # Owner pages fill limit except for the final tail. Accepting a short
            # intermediate page would invalidate the dimension-derived page bound.
            if not page.done and len(chunk) != OBSERVER_PAGE_CELLS:
                raise ValueError('OBSERVER_SHORT_INTERMEDIATE_PAGE')
            values.extend(chunk)
            if page.done:
                return bytes(values) if name == 'read_cells' else values
            cursor = page.next_cursor
        raise ValueError('OBSERVER_PAGE_LIMIT')

    def current(self, source, *, deadline):
        # GetValidity reports the transition-time version, not the latest map.
        # A fresh lease also checks observation age in the owner's clock.
        if self._capture_fault is not None:return False
        request = self.services['get_snapshot'].Request(expected_epoch=source.epoch,
            exact_revision=False, max_age_ms=self.max_age_ms, allow_stale=False)
        lease = self._get(request,deadline)
        try:
            return (lease.status == 'OK' and bool(lease.handle) and not lease.stale
                and 0 <= lease.age_ns <= self.max_age_ms*1_000_000
                and lease.validity.state == 'VALID' and lease.validity.epoch == source.epoch
                and lease.validity.generation == source.validity_generation
                and all(getattr(lease.identity,name) == getattr(source,name) for name in IDENTITY_FIELDS))
        finally:
            self._release(lease)
