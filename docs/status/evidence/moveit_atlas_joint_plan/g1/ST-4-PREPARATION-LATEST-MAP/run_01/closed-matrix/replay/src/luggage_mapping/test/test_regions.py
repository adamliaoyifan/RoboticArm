"""Native-grid region contract. Deterministic recipes are replay inputs."""
from dataclasses import replace
from types import SimpleNamespace
import hashlib
import json
import math
import time

import pytest

from luggage_mapping.regions import CellBlock, CellRegion, NativeGrid, block_from_volume
from luggage_mapping.observer import ObserverSnapshotPort, ObserverConfiguration, OBSERVER_PAGE_CELLS
from uld_occ_observer.msg import SnapshotIdentity, ValidityStamp
from uld_occ_observer.srv import ReadCells


@pytest.mark.parametrize('resolution', [.01, .02])
@pytest.mark.parametrize('origin', [(-.3, -.4, .53), (.2, -.1, -.7)])
def test_o_g_blocks_match_scalar_native_grid(resolution, origin):
    dims = (7, 5, 4)
    states = bytes(i % 3 for i in range(math.prod(dims)))
    active = tuple(i % 11 != 0 for i in range(len(states)))
    volume = SimpleNamespace(resolution_m=resolution, origin_container=origin,
                             dimensions_xyz=dims, states=states, active=active)
    region = CellRegion((2, 1, 1), (6, 4, 3))
    # Same admitted data through either source adapter; neither invents identity.
    o = block_from_volume(volume, region, identity=b'observer:frozen:7')
    g = block_from_volume(volume, region, identity=b'grid_g:frozen:7')
    assert (o.grid, o.region, o.states, o.active) == (g.grid, g.region, g.states, g.active)
    assert o.identity != g.identity
    expected = []
    for z in range(1, 3):
        for y in range(1, 4):
            for x in range(2, 6):
                flat = x+7*(y+5*z)
                cell = (states[flat] if active[flat] else 0, active[flat])
                assert o.cell((x, y, z)) == cell
                expected.append(cell[0])
    assert o.states == bytes(expected)
    assert o.grid.origin_container == origin and o.grid.resolution_m == resolution
    assert o.grid.dimensions_xyz == dims and not hasattr(o, 'dimensions_xyz')
    with pytest.raises(ValueError, match='NOT_COVERED'):
        o.cell((1, 1, 1))
    # Detached result cannot change when the transport object is replaced.
    volume.states = b'\2'*len(states)
    assert o.states == bytes(expected)


@pytest.mark.parametrize('begin,end', [((0, 0, 0), (0, 1, 1)), ((-1, 0, 0), (1, 1, 1)),
    ((0., 0, 0), (1, 1, 1)), ((False, 0, 0), (1, 1, 1)), ((0, 0), (1, 1, 1))])
def test_invalid_bounds_refuse(begin, end):
    with pytest.raises(ValueError, match='REGION_'):
        CellRegion(begin, end)


@pytest.mark.parametrize('damage', ['outside', 'zero_resolution', 'nan_origin', 'huge',
    'missing_identity', 'short_states', 'bad_state', 'bad_active', 'inactive_free'])
def test_invalid_grid_or_block_refuses(damage):
    grid = NativeGrid(.01, (-.2, -.3, .53), (2, 2, 2))
    region = grid.full_region()
    identity, states, active = b'frozen', b'\1'*8, b'\1'*8
    with pytest.raises(ValueError, match='REGION_'):
        if damage == 'outside':region = CellRegion((0, 0, 0), (3, 2, 2))
        elif damage == 'zero_resolution':grid = replace(grid, resolution_m=0.)
        elif damage == 'nan_origin':grid = replace(grid, origin_container=(0., 0., float('nan')))
        elif damage == 'huge':grid = replace(grid, dimensions_xyz=(1000, 1000, 1000))
        elif damage == 'missing_identity':identity = b''
        elif damage == 'short_states':states = states[:-1]
        elif damage == 'bad_state':states = b'\3'+states[1:]
        elif damage == 'bad_active':active = b'\2'+active[1:]
        elif damage == 'inactive_free':active = b'\0'+active[1:]
        CellBlock(grid, region, identity, states, active)


def make_port(dimensions, *, damage=None):
    descriptor = dict(length=dimensions[0]*.01, width=dimensions[1]*.01, floor_z=.53)
    config = ObserverConfiguration(json.dumps(descriptor).encode(), .01, 'digest', dimensions)
    identity = dict(epoch=1, map_revision=7, observed_revision=9,
        last_integrated_stamp_ns=12_000_000_000, geometry_hash='geometry',
        config_digest='digest', clock_domain='ros')
    port = object.__new__(ObserverSnapshotPort)
    port.config = config
    port.clients = {'read_cells':object()}
    port.services = {'read_cells':ReadCells}
    requests = []
    def call(client, request, deadline, stage):
        requests.append((request, deadline, stage))
        count = int(math.prod(b-a for a, b in zip(request.begin, request.end)))
        size = min(request.limit, count-request.cursor)
        cells = bytes((request.cursor+i) % 4 for i in range(size))
        page = ReadCells.Response(status='OK', identity=SnapshotIdentity(**identity),
            validity=ValidityStamp(state='VALID', epoch=1, generation=2),
            next_cursor=request.cursor+size, done=request.cursor+size == count, states=cells)
        if damage == 'identity':page.identity.map_revision += 1
        elif damage == 'generation':page.validity.generation += 1
        elif damage == 'validity':page.validity.state = 'BUILDING'
        elif damage == 'status':page.status = 'STALE'
        elif damage == 'cursor':page.next_cursor += 1
        elif damage == 'empty':page.states = []
        elif damage == 'short':page.states = cells[:-1];page.next_cursor -= 1
        elif damage == 'long':page.states = cells+b'\1';page.next_cursor += 1
        elif damage == 'done':page.done = not page.done
        elif damage == 'state':page.states = b'\4'+cells[1:]
        elif damage == 'timeout':raise TimeoutError('fixture deadline')
        return page
    port.call = call
    return port, identity, requests


def test_offset_region_pages_reset_local_cursor_and_preserve_deadline():
    port, identity, requests = make_port((120, 100, 20))
    region = CellRegion((10, 5, 3), (110, 95, 13))  # 90,000 cells
    block = port.read_region('lease', identity, 2, region, deadline=123.5)
    assert [r.cursor for r, _, _ in requests] == [0, OBSERVER_PAGE_CELLS]
    assert all(tuple(r.begin) == region.begin and tuple(r.end) == region.end for r, _, _ in requests)
    assert all(d == 123.5 and s == 'observer_read_cells' for _, d, s in requests)
    assert len(block.states) == 90000
    assert block.states[:8] == bytes([0, 1, 2, 0]*2)
    assert block.active[:8] == bytes([1, 1, 1, 0]*2)
    assert block.grid.origin_container == (-.6, -.5, .53)
    second = CellRegion((30, 40, 10), (32, 43, 12))
    port.read_region('lease', identity, 2, second, deadline=123.5)
    assert requests[-1][0].cursor == 0 and tuple(requests[-1][0].begin) == second.begin
    assert json.loads(block.identity)['identity'] == identity


@pytest.mark.parametrize('damage', ['identity', 'generation', 'validity', 'status', 'cursor',
                                   'empty', 'short', 'long', 'done', 'state', 'timeout'])
def test_malformed_page_never_returns_partial_block(damage):
    port, identity, requests = make_port((120, 100, 20), damage=damage)
    with pytest.raises((ValueError, TimeoutError)):
        port.read_region('lease', identity, 2, CellRegion((10, 5, 3), (110, 95, 13)), deadline=123.)
    assert len(requests) == 1


def test_region_count_and_bounds_rejected_before_rpc():
    port, identity, requests = make_port((4, 4, 4))
    with pytest.raises(ValueError, match='OUTSIDE'):
        port.read_region('lease', identity, 2, CellRegion((0, 0, 0), (5, 4, 4)), deadline=123.)
    with pytest.raises(ValueError, match='COUNT_OR_BOUNDS'):
        port._pages('read_cells', 'lease', 63, identity, 2, 123.)
    assert requests == []


def test_full_one_cm_grid_requires_67_pages(record_property):
    port, identity, requests = make_port((149, 197, 148))
    started = time.monotonic()
    block = port.read_region('lease', identity, 2, CellRegion((0, 0, 0), (149, 197, 148)), deadline=1000.)
    elapsed = time.monotonic()-started
    assert len(block.states) == 4_344_244 and len(requests) == 67
    assert [r.cursor for r, _, _ in requests] == list(range(0, 4_344_244, 65536))
    assert requests[-1][0].cursor == 4_325_376
    record_property('fixture_profile', 'typed RPC double, no live throughput claim')
    record_property('pages', len(requests))
    record_property('cells', len(block.states))
    record_property('elapsed_s', elapsed)
    record_property('states_sha256', hashlib.sha256(block.states).hexdigest())


@pytest.mark.parametrize('fail_cells', [False, True])
def test_complete_capture_uses_region_and_releases_lease(tmp_path, fail_cells):
    import threading
    from concurrent.futures import Future
    from test_joint_operation_observer import configuration, identity as fixture_identity, columns_for
    from uld_occ_observer.srv import GetSnapshot, ReadSurface, ReleaseSnapshot
    config, _, _ = configuration(tmp_path)
    ident = fixture_identity(config)
    validity = ValidityStamp(state='VALID', epoch=1, generation=2)
    lease = GetSnapshot.Response(status='OK', handle='frozen-lease',
        identity=SnapshotIdentity(**ident), validity=validity, stale=False, age_ns=0)
    port = object.__new__(ObserverSnapshotPort)
    port.config, port.max_age_ms = config, 1000
    port._pending_releases, port._release_lock, port._capture_fault = [], threading.Lock(), None
    port.recorder = None
    port.services = dict(get_snapshot=GetSnapshot, read_surface=ReadSurface,
                         read_cells=ReadCells, release_snapshot=ReleaseSnapshot)
    released = []
    def release(request):
        released.append(request.handle)
        future = Future();future.set_result(ReleaseSnapshot.Response(status='OK'))
        return future
    port.clients = dict(read_surface=object(), read_cells=object(),
                        release_snapshot=SimpleNamespace(call_async=release))
    port._get = lambda request, deadline: lease
    states = [1]*64;states[-1] = 3
    columns = columns_for(config, states)
    requests = []
    def call(client, request, deadline, stage):
        requests.append(request)
        if type(request) is ReadSurface.Request:
            fields = ('x', 'y', 'active_count', 'known_count', 'has_occupied', 'top_z')
            return ReadSurface.Response(status='OK', identity=SnapshotIdentity(**ident), validity=validity,
                next_cursor=16, done=True, **{f:[c[f] for c in columns] for f in fields})
        assert tuple(request.begin) == (0, 0, 0) and tuple(request.end) == (4, 4, 4)
        if fail_cells:raise TimeoutError('fixture cells failed')
        return ReadCells.Response(status='OK', identity=SnapshotIdentity(**ident), validity=validity,
                                  next_cursor=64, done=True, states=states)
    port.call = call
    def capture():
        return port.capture(scene_revision='scene', request_id='req', expected_epoch=1,
            expected_revision=7, deadline=123., require_latest=False)
    if fail_cells:
        with pytest.raises(TimeoutError):capture()
    else:
        snapshot = capture()
        assert snapshot.volume.dimensions_xyz == (4, 4, 4)
        assert snapshot.volume.active[-1] is False and snapshot.volume.states[-1] == 0
        assert snapshot.volume.source == snapshot.source
    assert len(requests) == 2 and released == ['frozen-lease']


def test_real_o_g_volume_dtos_share_region_result(tmp_path):
    from luggage_msgs.srv import GetCargoVolumeV2
    from test_joint_operation_observer import configuration, snapshot
    config, _, _ = configuration(tmp_path)
    raw = [1]*64;raw[5] = 2;raw[21] = 3
    observed = snapshot(config, raw).volume
    grid_g = GetCargoVolumeV2.Response(resolution_m=observed.resolution_m,
        origin_container=observed.origin_container, dimensions_xyz=observed.dimensions_xyz,
        states=observed.states, active=observed.active)
    region = CellRegion((0, 1, 0), (3, 3, 3))
    a = block_from_volume(observed, region, identity=b'admitted-O')
    b = block_from_volume(grid_g, region, identity=b'admitted-G')
    assert (a.grid, a.region, a.states, a.active) == (b.grid, b.region, b.states, b.active)


def test_second_page_failure_does_not_return_first_page():
    port, identity, requests = make_port((120, 100, 20))
    transport = port.call
    def changed_second_page(*args):
        page = transport(*args)
        if len(requests) == 2:page.identity.map_revision += 1
        return page
    port.call = changed_second_page
    with pytest.raises(ValueError, match='OBSERVER_'):
        port.read_region('lease', identity, 2, CellRegion((10, 5, 3), (110, 95, 13)), deadline=123.)
    assert len(requests) == 2


@pytest.mark.parametrize('dimensions', [(2., 2, 2), (True, 2, 2)])
def test_source_adapter_does_not_truncate_or_accept_bool_dimensions(dimensions):
    volume = SimpleNamespace(resolution_m=.01, origin_container=(0., 0., .53),
        dimensions_xyz=dimensions, states=b'\1'*8, active=(True,)*8)
    with pytest.raises(ValueError, match='INTEGER_CELL'):
        block_from_volume(volume, CellRegion((0, 0, 0), (2, 2, 2)), identity=b'frozen')
