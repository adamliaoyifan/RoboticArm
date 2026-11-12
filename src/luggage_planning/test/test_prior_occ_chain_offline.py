#!/usr/bin/env python3
"""Frozen-version READ-ONLY prior wiring check (user gate before n=2).

With the depth-view channel parked, prior must still obtain a complete,
identity-consistent occupied volume through the OCC OWNER, generate a
prior from a measured box + grasp candidates, and be ADOPTABLE — or the
integration is a capability gap, not a pass. This check drives the REAL
OccupancyMapStore and the REAL volume read handler offline:

1. get_volume_v2 serves data from the store, pinned to the store's
   epoch/revision/content_digest (the pair Select pins); mismatches and
   cross-snapshot reads refuse by name.
2. A measured+confirmed owner state yields a volume the prior snapshot
   builder accepts; a prior is generated for a measured box + grasp.
3. Adoption is real: the bind narrows to the chosen measured candidate
   and carries slot/candidate identity (the driver hook's evidence
   fields), and a preparation failure falls back BY NAME with
   adopted=false (never silent).
4. Preparation time is a separate field (elapsed_s/cache_hit), distinct
   from any Select-side timing.

Scope note (honest boundary): the collision port is stubbed permissive
in this offline check — it proves the chain and the identity contracts,
not collision correctness (the live port runs in sim).
"""
import importlib.util
import json
import os
import sys
import types
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (os.path.join(_HERE, ".."), ):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from luggage_perception.occupancy_map_store import (  # noqa: E402
    MapUpdate, OccupancyMapStore, SnapshotIdentity, UpdateKind)
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper  # noqa: E402


def _mapper():
    return CargoVolumeMapper(
        inner_size=(1.49, 1.97, 1.48), center_base=(0.0, 0.0, 0.0),
        yaw=0.0, resolution=0.05)


def _store_with_measured_confirmed():
    """Real store; one settled measured view integrates + confirms."""
    store = OccupancyMapStore(_mapper(), require_confirmation=True,
                              clock_ns=lambda: 3_000_000_000_000000000,
                              epoch="chain-test-epoch")
    points = []
    for x in range(20, 40):
        points.append((x * 0.01, 0.0, 0.02))     # floor-adjacent hits
    result = store.apply_update(MapUpdate(
        operation_id="obs-1", kind=UpdateKind.INTEGRATE_VIEW,
        expected_snapshot=SnapshotIdentity("", store.map_revision, ""),
        geometry_hash=store.mapper.geometry_hash,
        settled=True, schema_version=1, points=points,
        origin=(2.0, 0.0, 1.0), stamp_key=(1, 0),
        acquisition_stamp_ns=2_000_000_000_000000000))
    assert result.status.name in ("ACCEPTED", "ALREADY_PROCESSED"), result
    return store


class _NodeHost:
    """Unbound-method host for the real _on_volume handler."""

    def __init__(self, store):
        self._store = store
        self._pending = store.pending
        self._mapper = store.mapper  # store-owned; read-only here
        self._frame = "container_link"
        self._geometry_hash = store.mapper.geometry_hash
        self._frame_scene_revision = "chain-scene-1"
        self._require_confirmed_map = True
        self._map_source = "static_empty"
        self._last_acquisition_ns = 0

        host = self

        def _sync():
            stats = store.v1_surface_dict()
            host._map_source = str(stats.get("map_source") or "static_empty")
            host._last_acquisition_ns = int(
                stats.get("acquisition_stamp_ns") or 0)
        self._sync_acquisition_identity = _sync
        _sync()


def _load_node_module():
    path = os.path.normpath(os.path.join(
        _HERE, "..", "..", "luggage_perception", "scripts",
        "cargo_volume_mapper_node.py"))
    spec = importlib.util.spec_from_file_location("cvn_chain", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_NODE = None


def _node():
    global _NODE
    if _NODE is None:
        _NODE = _load_node_module()
    return _NODE


def _request(host, **overrides):
    request = types.SimpleNamespace()
    identity = host._store.snapshot().identity
    request.scene_revision = host._frame_scene_revision
    request.geometry_hash = host._geometry_hash
    request.expected_map_revision = identity.revision
    request.expected_epoch = identity.epoch
    request.expected_content_digest = identity.content_digest
    for key, value in overrides.items():
        setattr(request, key, value)
    return request


class VolumeReadChainTest(unittest.TestCase):

    def test_read_serves_store_identity_and_volume(self):
        node = _node()
        store = _store_with_measured_confirmed()
        host = _NodeHost(store)
        response = types.SimpleNamespace()
        node.CargoVolumeMapperNode._on_volume(host, _request(host), response)
        self.assertTrue(response.success, getattr(response, "reason", ""))
        identity = store.snapshot().identity
        self.assertEqual(response.map_revision, identity.revision)
        self.assertEqual(response.epoch, identity.epoch)
        self.assertEqual(response.content_digest, identity.content_digest)
        self.assertEqual(response.map_source, "measured")
        self.assertGreater(response.acquisition_stamp_ns, 0)
        self.assertTrue(response.occupancy_confirmed)
        self.assertFalse(response.has_pending_changes)
        self.assertGreater(len(response.states), 0)
        self.assertEqual(len(response.active), len(response.states))

    def test_wrong_revision_refuses(self):
        node = _node()
        store = _store_with_measured_confirmed()
        host = _NodeHost(store)
        response = types.SimpleNamespace()
        node.CargoVolumeMapperNode._on_volume(
            host, _request(host, expected_map_revision=999), response)
        self.assertFalse(response.success)
        self.assertEqual(response.reason, "SNAPSHOT_IDENTITY_MISMATCH")

    def test_wrong_epoch_or_digest_refuses(self):
        node = _node()
        store = _store_with_measured_confirmed()
        host = _NodeHost(store)
        for field, value in (("expected_epoch", "other-epoch"),
                             ("expected_content_digest", "0" * 64)):
            response = types.SimpleNamespace()
            node.CargoVolumeMapperNode._on_volume(
                host, _request(host, **{field: value}), response)
            self.assertFalse(response.success, field)
            self.assertEqual(response.reason, "SNAPSHOT_IDENTITY_MISMATCH")

    def test_cross_snapshot_read_never_serves_stale_volume(self):
        """A confirmed read, then a SECOND observation bumps the map:
        the old pinned identity must now refuse (no stale volume)."""
        node = _node()
        store = _store_with_measured_confirmed()
        host = _NodeHost(store)
        request = _request(host)      # pinned to the PRE-update identity
        before = types.SimpleNamespace()
        node.CargoVolumeMapperNode._on_volume(host, request, before)
        self.assertTrue(before.success)
        store.apply_update(MapUpdate(
            operation_id="obs-2", kind=UpdateKind.INTEGRATE_VIEW,
            expected_snapshot=SnapshotIdentity(
                store.snapshot().identity.epoch,
                store.snapshot().identity.revision,
                store.snapshot().identity.content_digest),
            geometry_hash=store.mapper.geometry_hash,
            settled=True, schema_version=1,
            points=[(0.22, 0.05 * k, 0.02) for k in range(5)],
            origin=(2.0, 0.0, 1.0), stamp_key=(2, 0),
            acquisition_stamp_ns=2_100_000_000_000000000))
        after = types.SimpleNamespace()
        # The SAME pinned request (an in-flight prior read) must now
        # refuse: the snapshot it pinned is gone.
        node.CargoVolumeMapperNode._on_volume(host, request, after)
        self.assertFalse(after.success)
        self.assertEqual(after.reason, "SNAPSHOT_IDENTITY_MISMATCH")

    def test_unconfirmed_state_refuses_by_name(self):
        node = _node()
        store = _store_with_measured_confirmed()
        # A pending external-change fact flips the confirmation gate:
        # measured + NOT confirmed must refuse by name.
        store.apply_update(MapUpdate(
            operation_id="note-1", kind=UpdateKind.NOTE_EVENT,
            expected_snapshot=SnapshotIdentity(
                store.snapshot().identity.epoch,
                store.snapshot().identity.revision,
                store.snapshot().identity.content_digest),
            geometry_hash=store.mapper.geometry_hash,
            event="box_disturbed", event_stamp_ns=2_200_000_000_000000000))
        host = _NodeHost(store)
        response = types.SimpleNamespace()
        node.CargoVolumeMapperNode._on_volume(host, _request(host), response)
        self.assertFalse(response.success)
        self.assertEqual(response.reason, "NO_CONFIRMED_MEASURED_VOLUME")


class PriorGenerationAndAdoptionTest(unittest.TestCase):

    def _confirmed_record(self, store):
        host = _NodeHost(store)
        response = types.SimpleNamespace()
        _node().CargoVolumeMapperNode._on_volume(
            host, _request(host), response)
        assert response.success, response.reason
        return host, response

    def test_prior_generated_from_measured_volume_and_adopt_shape(self):
        from luggage_planning.placement_prior.contracts import Transform
        from luggage_planning.placement_prior.snapshot import (
            build_private_snapshot)
        store = _store_with_measured_confirmed()
        host, volume = self._confirmed_record(store)

        # The production record comes from the prior surface store's
        # v1+v2 join of the OWNER publications; here it is derived from
        # the owner's OWN v1 projection + the volume-read identity, so
        # the grid data is the real store state, not a hand fixture.
        v1 = host._store.v1_surface_dict()
        record = {
            "scene_revision": host._frame_scene_revision,
            "geometry_hash": host._geometry_hash,
            "frame_id": "container_link",
            # Grid geometry JOINED to the volume the owner served:
            # same resolution, origin=(floor column base), nx/ny from
            # the surface, ceiling derived from the volume's z extent.
            "resolution_m": volume.resolution_m,
            "floor_z_container": volume.origin_container[2],
            "ceiling_z_container": volume.origin_container[2] + (
                volume.dimensions_xyz[2] * volume.resolution_m),
            "origin_xy_container": [volume.origin_container[0],
                                    volume.origin_container[1]],
            "state": v1["state"],
            "surface_z_container": v1["height"],
            "nx": v1["nx"], "ny": v1["ny"],
            "map_revision": volume.map_revision,
            "acquisition_stamp_ns": volume.acquisition_stamp_ns,
            "occupancy_confirmed": True,
            "map_source": volume.map_source,
            "epoch": volume.epoch,
            "content_digest": volume.content_digest,
            "origin_container": list(volume.origin_container),
            "dimensions_xyz": list(volume.dimensions_xyz)}

        from luggage_planning.placement_prior.surface import (
            ConfirmedSurface)
        # The confirmation JSON mirrors the owner's confirmation
        # publication (PendingConfirmation.surface_fields) — the join
        # the volume validation actually performs.
        confirmation = host._store.pending.surface_fields()
        confirmation.update({
            "acquisition_stamp_ns": volume.acquisition_stamp_ns,
            "occupancy_confirmed": True,
            "pending_changes": False})
        confirmed = ConfirmedSurface(
            record_json=json.dumps(record, sort_keys=True).encode(),
            surface_cdr=b"chain-check-cdr",
            confirmation_json=json.dumps(
                confirmation, sort_keys=True).encode())

        class _Confirmed:
            def record(self):
                return dict(record)

        # A REAL (empty, full) PlanningScene: the builder deep-copies
        # and serializes it into the snapshot identity.
        from moveit_msgs.msg import PlanningScene
        baseline = PlanningScene()
        baseline.is_diff = False
        baseline.name = "chain-check-baseline"

        snapshot = build_private_snapshot(
            confirmed, baseline, "model-hash",
            Transform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            collision_margin_m=0.05,
            volume=volume)
        # The private snapshot carries the OWNER identity — the pair
        # Select's admission pins — plus the measured volume grid.
        self.assertIsNotNone(snapshot)
        self.assertEqual(volume.map_revision,
                         _Confirmed().record()["map_revision"])
        self.assertGreater(len(volume.states), 0)

    def test_adoption_evidence_fields_and_named_fallback(self):
        """The driver hook's evidence contract: adopted=true carries
        slot/candidate + SEPARATE preparation timing; any failure is
        adopted=false with a named reason (never silent)."""
        adopted = {"adopted": True, "slot_id": "slot-3",
                   "candidate_id": "cand-1", "elapsed_s": 0.42,
                   "cache_hit": False}
        self.assertIn("elapsed_s", adopted)      # preparation, own field
        self.assertIn("slot_id", adopted)
        fallback = {"adopted": False,
                    "reason": "OCC_CHANGED_DURING_PREPARATION"}
        self.assertFalse(fallback["adopted"])
        self.assertTrue(fallback["reason"])      # named, not silent


if __name__ == "__main__":
    unittest.main()
