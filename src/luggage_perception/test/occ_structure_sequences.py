"""Fixed M1–M8 and S1–S9 sequences for the structure-oracle.

The production path under test is the vectorized integrate. The scalar
reference is invoked only where the acceptance names it.
"""

import math
import random

from luggage_perception.cargo_map_query import QueryError
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.depth_view_rays import admit_depth_ray
from luggage_perception.occupancy_map_store import (
    MapUpdate,
    OccupancyMapStore,
    SnapshotIdentity,
    UpdateKind,
    UpdateStatus,
)
from luggage_description.container_geometry import cuboid_from_inner_size
from occ_structure_state import attach_delta, canon, surface_record, take_delta
from scalar_integrate_reference import scalar_integrate


class Step(object):
    def __init__(self, name, meta, payload, delta):
        self.name = name
        self.meta = meta
        self.payload = payload
        self.delta = delta


def _m1_mapper():
    mapper = CargoVolumeMapper((0.12, 0.12, 0.12), (0, 0, 0.06), 0, 0.02)
    attach_delta(mapper)
    return mapper


def _call(mapper, name, fn):
    error = None
    result = None
    try:
        result = fn()
    except Exception as exc:
        error = {"type": type(exc).__name__, "message": str(exc)}
    delta = take_delta(mapper)
    meta = surface_record(mapper)
    meta["name"] = name
    meta["result"] = canon(result)
    meta["error"] = error
    from occ_structure_state import mapper_payload
    return Step(name, meta, mapper_payload(mapper, delta), delta)


def sequence_m1():
    mapper = _m1_mapper()
    steps = [_call(mapper, "init", lambda: None)]
    point = (0.01, 0.01, 0.05)
    for index in range(100):
        steps.append(_call(
            mapper, "hit-%03d" % index,
            lambda point=point: mapper.integrate_points([point])))
    origin = (-0.2, 0.01, 0.05)
    end = (0.2, 0.01, 0.05)
    for index in range(100):
        steps.append(_call(
            mapper, "miss-%03d" % index,
            lambda origin=origin, end=end: mapper.integrate_points(
                [end], origin=origin)))
    return steps


def assert_m1_threshold_coverage(steps):
    """Preparation fails unless the trace actually crosses both thresholds."""
    values = []
    for step in steps:
        values.append(step.payload["logodds"])
    stacked = values[0]
    occupied = 0.50
    free = -0.35
    lo =  -2.0
    hi = 3.5
    saw_below_occ = saw_above_occ = False
    saw_below_free = saw_above_free = False
    saw_lo = saw_hi = False
    for array in values:
        if np_any_compare(array, occupied, "ge") and np_any_compare(array, occupied, "lt"):
            saw_below_occ = saw_above_occ = True
        if float(array.max()) >= occupied:
            saw_above_occ = True
        if float(array.min()) < occupied:
            saw_below_occ = True
        if float(array.max()) > free:
            saw_above_free = True
        if float(array.min()) <= free:
            saw_below_free = True
        if float(array.max()) >= hi - 1e-9:
            saw_hi = True
        if float(array.min()) <= lo + 1e-9:
            saw_lo = True
    ok = all((saw_below_occ, saw_above_occ, saw_below_free, saw_above_free, saw_lo, saw_hi))
    return {
        "ok": ok,
        "saw_below_occupied": saw_below_occ,
        "saw_above_occupied": saw_above_occ,
        "saw_below_free": saw_below_free,
        "saw_above_free": saw_above_free,
        "saw_min_saturation": saw_lo,
        "saw_max_saturation": saw_hi,
        "global_min": float(min(float(a.min()) for a in values)),
        "global_max": float(max(float(a.max()) for a in values)),
    }


def np_any_compare(array, bound, op):
    if op == "ge":
        return bool((array >= bound).any())
    if op == "lt":
        return bool((array < bound).any())
    raise ValueError(op)


def sequence_m2_prefix():
    mapper = _m1_mapper()
    steps = [_call(mapper, "init", lambda: None)]
    center = (0.01, 0.01, 0.05)
    size = (0.04, 0.04, 0.04)
    steps.append(_call(
        mapper, "place",
        lambda: mapper.mark_placed_box(center, size, yaw=0.0)))
    origin = (-0.2, 0.01, 0.05)
    end = (0.2, 0.01, 0.05)
    for index in range(100):
        steps.append(_call(
            mapper, "miss-%03d" % index,
            lambda origin=origin, end=end: mapper.integrate_points(
                [end], origin=origin)))
    return mapper, steps


def sequence_m2():
    mapper, steps = sequence_m2_prefix()
    center = (0.01, 0.01, 0.05)
    size = (0.04, 0.04, 0.04)
    steps.append(_call(
        mapper, "unmark",
        lambda: mapper.unmark_placed_box(center, size, yaw=0.0)))
    return steps


def sequence_m3():
    mapper = _m1_mapper()
    steps = [_call(mapper, "init", lambda: None)]
    steps.append(_call(
        mapper, "duplicate",
        lambda: mapper.integrate_points(
            [(0.01, 0.01, 0.03), (0.01, 0.01, 0.03)],
            labels=[1, 2], instance_ids=[11, 22])))
    steps.append(_call(
        mapper, "label-1",
        lambda: mapper.integrate_points(
            [(0.01, 0.01, 0.03)], labels=[1], instance_ids=[11])))
    steps.append(_call(
        mapper, "label-2",
        lambda: mapper.integrate_points(
            [(0.01, 0.01, 0.07)], labels=[2], instance_ids=[22])))
    return steps


def sequence_m4_grid():
    steps_by_name = {}
    for resolution in (0.01, 0.02, 0.05):
        mapper = CargoVolumeMapper((0.2, 0.15, 0.3), (0.0, 0.0, 0.15), 0.0, resolution)
        attach_delta(mapper)
        rng = random.Random(193)
        def paint(mapper=mapper, rng=rng):
            for i in range(len(mapper._grid)):
                mapper._grid[i] = rng.randrange(3)
                mapper._source[i] = rng.randrange(3)
                mapper._labels[i] = rng.randrange(4)
                mapper._active[i] = rng.randrange(2)
            return {"resolution": resolution, "seed": 193}
        steps_by_name["grid-%.2f" % resolution] = [
            _call(mapper, "painted", paint)]
    return steps_by_name


def sequence_m4_m7_depth():
    """Every depth-view case from test_depth_view_rays, one step per call."""
    groups = {}

    def run(name, builder):
        groups[name] = builder()

    def wall():
        mapper = CargoVolumeMapper((1., 1., 1.), (0., 0., .5), 0., .1)
        attach_delta(mapper)
        steps = []
        for index in range(5):
            steps.append(_call(
                mapper, "view-%d" % index,
                lambda: mapper.integrate_depth_view([(.5, 0., .5)], (-1., 0., .5))))
        return steps

    def obstacle():
        mapper = CargoVolumeMapper((1., 1., 1.), (0., 0., .5), 0., .1)
        attach_delta(mapper)
        steps = []
        for index in range(5):
            steps.append(_call(
                mapper, "view-%d" % index,
                lambda: mapper.integrate_depth_view([(0., 0., .5)], (-1., 0., .5))))
        return steps

    def rejected(tag, origin, end):
        mapper = CargoVolumeMapper((1., 1., 1.), (0., 0., .5), 0., .1)
        attach_delta(mapper)
        return [_call(
            mapper, "view",
            lambda origin=origin, end=end: mapper.integrate_depth_view([end], origin))]

    def rotated():
        a = CargoVolumeMapper((1., 1., 1.), (0., 0., .5), 0., .1)
        b = CargoVolumeMapper((1., 1., 1.), (2., 3., 1.5), math.pi / 2, .1)
        attach_delta(a)
        attach_delta(b)
        steps = []
        for index in range(5):
            steps.append(_call(
                a, "a-%d" % index,
                lambda: a.integrate_depth_view([(.035, .035, .565)], (-1.03, .035, .565))))
            steps.append(_call(
                b, "b-%d" % index,
                lambda: b.integrate_depth_view([(1.965, 3.035, 1.565)], (1.965, 1.97, 1.565))))
        return steps

    def bad_band():
        hull = cuboid_from_inner_size((1., 1., 1.), floor_z=0.)
        meta = {"name": "admit", "error": None, "result": None}
        try:
            admit_depth_ray(hull, (-1., 0., .5), (.5, 0., .5), 0.)
            meta["result"] = True
        except Exception as exc:
            meta["error"] = {"type": type(exc).__name__, "message": str(exc)}
        return [Step("admit", meta, {}, None)]

    def covered():
        mapper = CargoVolumeMapper((1., 1., 1.), (0., 0., .5), 0., .1)
        attach_delta(mapper)
        steps = []
        for iz in range(10):
            z = (iz + .5) * .1
            for repeat in range(5):
                steps.append(_call(
                    mapper, "z%d-%d" % (iz, repeat),
                    lambda z=z: mapper.integrate_depth_view([(.5, .035, z)], (-1.03, .035, z))))
        return steps

    def door():
        mapper = CargoVolumeMapper((1., 1., 1.), (0., 0., .5), 0., .1)
        attach_delta(mapper)
        steps = []
        for index in range(5):
            steps.append(_call(
                mapper, "view-%d" % index,
                lambda: mapper.integrate_depth_view([(-.48, .035, .565)], (-1.03, .035, .565))))
        return steps

    def bounded():
        mapper = CargoVolumeMapper((1., 1., 1.), (0., 0., .5), 0., .05, max_raycast_points=1)
        attach_delta(mapper)
        points = [(.5, -.3, .5), (.5, .3, .5)]
        steps = []
        for index in range(10):
            steps.append(_call(
                mapper, "view-%d" % index,
                lambda: mapper.integrate_depth_view(points, (-1., 0., .5))))
        return steps

    run("wall", wall)
    run("obstacle", obstacle)
    rejected_cases = [
        ("rear", (1., 0., .5), (0., 0., .5)),
        ("side", (0., 1., .5), (0., 0., .5)),
        ("before-door", (-1., 0., .5), (-.7, 0., .5)),
        ("beyond-rear", (-1., 0., .5), (1., 0., .5)),
        ("nan", (-1., 0., .5), (float("nan"), 0., .5)),
    ]
    for tag, origin, end in rejected_cases:
        run("rejected-" + tag, lambda origin=origin, end=end: rejected(tag, origin, end))
    run("rotated", rotated)
    run("bad-band", bad_band)
    run("covered", covered)
    run("door", door)
    run("bounded", bounded)
    return groups


def sequence_m5():
    groups = {}
    invalid = [None, ("nan", float("nan")), ("+inf", float("inf")), ("-inf", float("-inf"))]

    def one(with_origin):
        mapper = _m1_mapper()
        steps = [_call(mapper, "init", lambda: None)]
        origin = (-0.2, 0.01, 0.05) if with_origin else None
        steps.append(_call(
            mapper, "empty",
            lambda origin=origin: mapper.integrate_points([], origin=origin)))
        for label, value in invalid[1:]:
            point = (value, 0.01, 0.05)
            steps.append(_call(
                mapper, label,
                lambda point=point, origin=origin: mapper.integrate_points(
                    [point], origin=origin)))
        steps.append(_call(
            mapper, "valid",
            lambda origin=origin: mapper.integrate_points(
                [(0.01, 0.01, 0.05)], origin=origin)))
        return steps

    groups["no-origin"] = one(False)
    groups["with-origin"] = one(True)
    return groups


def sequence_m6():
    groups = {}
    for flag in (True, False):
        mapper, steps = sequence_m2_prefix()
        steps.append(_call(
            mapper, "reset-%s" % flag,
            lambda flag=flag: mapper.reset(preserve_placed=flag)))
        steps.append(_call(
            mapper, "hit",
            lambda: mapper.integrate_points([(0.01, 0.01, 0.05)])))
        steps.append(_call(
            mapper, "miss",
            lambda: mapper.integrate_points(
                [(0.2, 0.01, 0.05)], origin=(-0.2, 0.01, 0.05))))
        groups["preserve-%s" % flag] = steps
    return groups


class _Clock(object):
    def __init__(self):
        self.now = 1_000_000_000_000000000

    def __call__(self):
        return self.now

    def advance(self, ns):
        self.now += int(ns)


def _store():
    mapper = CargoVolumeMapper([1.0, 1.0, 1.0], [0.0, 0.0, 0.5], 0.0, 0.1)
    attach_delta(mapper)
    clock = _Clock()
    store = OccupancyMapStore(
        mapper, require_confirmation=True, clock_ns=clock, epoch="e1")
    return store, clock


def _store_meta(store, name, result, error, handles):
    snap = store.snapshot()
    fields = store.pending.surface_fields()
    query_error = None
    try:
        store.summary()
        query_state = "available"
    except Exception as exc:
        query_state = "unavailable"
        query_error = {"type": type(exc).__name__, "message": str(exc)}
    projections = {
        "v1": canon(store.v1_surface_dict(snap)),
        "v2": canon(store.v2_record("oracle", "oracle", snap)),
        "committed": canon(store.committed_dict("container_link", [0, 0, 0, 1, 0, 0, 0], snap)),
    }
    meta = {
        "name": name,
        "status": None if result is None else result.status.value,
        "reason": None if result is None else result.reason,
        "error": error,
        "identity": None if snap.identity is None else {
            "epoch": snap.identity.epoch,
            "revision": int(snap.identity.revision),
            "content_digest": snap.identity.content_digest,
        },
        "pending": canon(fields),
        "query_state": query_state,
        "query_error": query_error,
        "handles": handles,
        "projections": projections,
        "surface": surface_record(store.mapper),
    }
    return meta


def _apply(store, name, update, handles):
    error = None
    result = None
    try:
        result = store.apply_update(update)
    except Exception as exc:
        error = {"type": type(exc).__name__, "message": str(exc)}
    delta = take_delta(store.mapper)
    meta = _store_meta(store, name, result, error, handles)
    from occ_structure_state import mapper_payload
    return Step(name, meta, mapper_payload(store.mapper, delta), delta), result


def _handle_view(store, token, identity):
    try:
        snap = store.acquire_query_snapshot(identity, token=token)
        summary = snap.summary()
    except QueryError as exc:
        return {"ok": False, "error": str(exc)}
    ident = tuple(snap.identity)
    return {
        "ok": True,
        "identity": [item.item() if hasattr(item, "item") else item for item in ident],
        "summary_status": str(summary.status),
    }


def sequence_store():
    """S1–S9. Each group has an initial read, a trigger, and a follow-up read."""
    groups = {}

    def integrate(store, op, points, stamp_key):
        return MapUpdate(
            operation_id=op, kind=UpdateKind.INTEGRATE_VIEW,
            expected_snapshot=SnapshotIdentity("", store.map_revision, ""),
            geometry_hash=store.mapper.geometry_hash,
            settled=True, schema_version=1,
            points=[list(p) for p in points], origin=(2.0, 0.0, 1.0),
            stamp_key=stamp_key, acquisition_stamp_ns=int(stamp_key[0]) * 1_000_000_000)

    store, _clock = _store()
    points = [(0.0, 0.0, 0.5)]
    steps = []
    step, _ = _apply(store, "init", integrate(store, "s1-a", points, (1, 0)), [])
    # init above consumed an update; record a pure read by re-dumping without update
    steps = [step]
    step, first = _apply(store, "trigger", integrate(store, "s1-a", points, (1, 0)), [])
    steps.append(step)
    step, _ = _apply(store, "follow", integrate(store, "s1-b", [(0.2, 0.0, 0.5)], (2, 0)), [])
    steps.append(step)
    groups["S1-same-id"] = steps

    store, _clock = _store()
    steps = []
    step, _ = _apply(store, "init", integrate(store, "s2-a", points, (1, 0)), [])
    steps.append(step)
    step, result = _apply(store, "trigger", integrate(store, "s2-a", [(0.4, 0.0, 0.5)], (3, 0)), [])
    steps.append(step)
    if result is None or result.status is not UpdateStatus.REJECTED:
        raise AssertionError("S2 expected rejection, got %s %s" % (result, step.meta.get("error")))
    step, _ = _apply(store, "follow", integrate(store, "s2-b", [(0.2, 0.0, 0.5)], (4, 0)), [])
    steps.append(step)
    groups["S2-different-payload"] = steps

    store, _clock = _store()
    steps = []
    step, _ = _apply(store, "init", integrate(store, "s3-a", points, (1, 0)), [])
    steps.append(step)
    bad = MapUpdate(
        operation_id="s3-bad", kind=UpdateKind.ADD_BOX_V2,
        expected_snapshot=SnapshotIdentity("e1", 999, ""),
        geometry_hash=store.mapper.geometry_hash,
        box=((0.0, 0.0, 0.1), (0.2, 0.2, 0.2), 0.0),
        measured_source="verify_placed_box",
        verified_center=(0.0, 0.0, 0.1), verify_tolerance_m=0.1)
    step, result = _apply(store, "trigger", bad, [])
    steps.append(step)
    assert result.status is UpdateStatus.REJECTED
    step, _ = _apply(store, "follow", integrate(store, "s3-b", [(0.2, 0.0, 0.5)], (5, 0)), [])
    steps.append(step)
    groups["S3-premise"] = steps

    store, clock = _store()
    steps = []
    step, _ = _apply(store, "init", integrate(store, "s4-a", points, (1, 0)), [])
    steps.append(step)
    late = MapUpdate(
        operation_id="s4-late", kind=UpdateKind.NOTE_EVENT,
        expected_snapshot=SnapshotIdentity("e1", 0, ""),
        geometry_hash=store.mapper.geometry_hash,
        event="box_released", event_stamp_ns=clock() + 5,
        box_id="box-1")
    step, result = _apply(store, "trigger", late, [])
    steps.append(step)
    assert result.status is UpdateStatus.RECORDED_PENDING
    step, _ = _apply(store, "follow", integrate(store, "s4-b", [(0.2, 0.0, 0.5)], (6, 0)), [])
    steps.append(step)
    groups["S4-late-fact"] = steps

    store, clock = _store()
    steps = []
    step, first = _apply(store, "init", integrate(store, "s5-a", points, (1, 0)), [])
    steps.append(step)
    held = store.snapshot()
    held_digest = held.identity.content_digest
    note = MapUpdate(
        operation_id="s5-note", kind=UpdateKind.NOTE_EVENT,
        expected_snapshot=SnapshotIdentity(
            held.identity.epoch, held.identity.revision, held.identity.content_digest),
        geometry_hash=store.mapper.geometry_hash,
        event="box_released", event_stamp_ns=clock() + 7,
        box_id="box-2")
    step, _ = _apply(store, "trigger", note, [])
    steps.append(step)
    # Old snapshot object must keep the digest it had when it was read.
    steps[-1].meta["held_digest_unchanged"] = held.identity.content_digest == held_digest
    step, _ = _apply(store, "follow", integrate(store, "s5-b", [(0.2, 0.0, 0.5)], (7, 0)), [])
    steps.append(step)
    groups["S5-old-snapshot"] = steps

    store, _clock = _store()
    steps = []
    step, _ = _apply(store, "init", integrate(store, "s6-a", points, (1, 0)), [])
    steps.append(step)
    snap = store.snapshot()
    before = snap.identity.content_digest
    v1 = store.v1_surface_dict(snap)
    v2 = store.v2_record("oracle", "oracle", snap)
    committed = store.committed_dict("container_link", [0, 0, 0, 1, 0, 0, 0], snap)
    v1["height"][0][0] = -1.0
    v2["state"][0][0] = "mutated"
    committed["boxes"].append({"center": [9, 9, 9]})
    step, _ = _apply(store, "trigger", integrate(store, "s6-hold", points, (1, 0)), [])
    steps.append(step)
    steps[-1].meta["snapshot_untouched"] = store.snapshot() is not None and before == snap.identity.content_digest
    step, _ = _apply(store, "follow", integrate(store, "s6-b", [(0.2, 0.0, 0.5)], (8, 0)), [])
    steps.append(step)
    groups["S6-projection-isolation"] = steps

    store, _clock = _store()
    steps = []
    step, _ = _apply(store, "init", integrate(store, "s7-a", points, (1, 0)), [])
    opened = store.acquire_query_snapshot()
    token = opened._token
    identity = opened.identity
    handle = _handle_view(store, token, identity)
    steps.append(step)
    steps[-1].meta["handles"] = [handle]

    original_publish = store._query_registry.publish

    def fail_publish(data, publish_status=None):
        store._query_registry.publish = original_publish
        if publish_status is not None:
            publish_status()
        store._query_registry._error = "RESOURCE_LIMIT"
        return False

    store._query_registry.publish = fail_publish
    step, result = _apply(
        store, "trigger", integrate(store, "s7-b", [(0.3, 0.0, 0.5)], (9, 0)), [])
    follow_handle = _handle_view(store, token, identity)
    try:
        store.acquire_query_snapshot()
        new_open = "opened"
    except QueryError as exc:
        new_open = str(exc)
    steps.append(step)
    steps[-1].meta["handles"] = [follow_handle]
    steps[-1].meta["new_open"] = new_open
    assert result.status is UpdateStatus.ACCEPTED
    assert follow_handle["ok"]
    assert new_open != "opened"
    step, _ = _apply(store, "reset", MapUpdate(
        operation_id="s7-reset", kind=UpdateKind.RESET,
        geometry_hash=store.mapper.geometry_hash), [])
    try:
        restored = store.acquire_query_snapshot()
        restored_state = "opened"
        restored_identity = tuple(restored.identity)
    except QueryError as exc:
        restored_state = str(exc)
        restored_identity = None
    steps.append(step)
    steps[-1].meta["restored_open"] = restored_state
    steps[-1].meta["restored_identity"] = canon(restored_identity)
    groups["S7-S9-query-failure-reset"] = steps
    return groups


def sequence_m8(evidence_path):
    import json
    with open(evidence_path) as handle:
        saved = json.load(handle)
    config = saved["config"]
    mapper = CargoVolumeMapper(
        tuple(config["inner_size"]), tuple(config["center"]),
        float(config["yaw"]), float(config["resolution"]),
        max_raycast_points=config["max_raycast_points"])
    attach_delta(mapper)
    cloud = [tuple(point) for point in saved["cloud"]]
    rays = [tuple(point) for point in saved["rays"]]
    origin = tuple(saved["origin"])
    step = _call(
        mapper, "vectorized",
        lambda: mapper.integrate_points(cloud, origin=origin, ray_points=rays))
    scalar_mapper = CargoVolumeMapper(
        tuple(config["inner_size"]), tuple(config["center"]),
        float(config["yaw"]), float(config["resolution"]),
        max_raycast_points=config["max_raycast_points"])
    attach_delta(scalar_mapper)
    scalar_result = scalar_integrate(
        scalar_mapper, cloud, origin=origin, ray_points=rays)
    from occ_structure_state import mapper_payload
    scalar_delta = take_delta(scalar_mapper)
    scalar_step = Step(
        "scalar",
        {"name": "scalar", "result": canon(scalar_result), "error": None,
         **surface_record(scalar_mapper)},
        mapper_payload(scalar_mapper, scalar_delta),
        scalar_delta)
    return [step, scalar_step], saved
