"""G4-A query replica: read-only receiver of immutable generations.

Protocol 1c0049d4 section 5. State machine:
STARTING -> SYNCING -> READY; any bad hash / missing object / wrong
base / unknown schema -> RESYNC_REQUIRED (a fresh FULL generation is
requested through the control channel); resync failure -> UNAVAILABLE.
No partial generation is ever published to the registry: the receiver
rebuilds the complete snapshot, verifies the target identity, and only
then atomically swaps.

The receiver reads objects read-only; its only writes are its own
control files (ack / resync request). It never imports the mapper or
any write path.
"""

from __future__ import annotations

import json
import os

from luggage_perception.cargo_query_snapshot_wire import (
    decode_array_object, decode_json_object, rebuild_snapshot)
from luggage_perception.cargo_map_query import (
    CELL_DTYPE, GridInfo, QueryIdentity, QuerySnapshotData, VoxelBlock)
from luggage_perception.cargo_distance_query import (
    DistanceLeafLayout, freeze_distance_members)
from luggage_perception.cargo_query_wire import (
    WireError, decode_array_object as wire_decode_array,
    decode_json_object as wire_decode_json, sha256_hex, validate_manifest)
from types import MappingProxyType

import numpy as np

MAX_OBJECT_BYTES = 16 * 1024 * 1024


class ReplicaState:
    STARTING = 'STARTING'
    SYNCING = 'SYNCING'
    READY = 'READY'
    RESYNC_REQUIRED = 'RESYNC_REQUIRED'
    UNAVAILABLE = 'UNAVAILABLE'


class QueryReplica:
    """Receiver side. Polls the export root; holds the rebuilt registry."""

    def __init__(self, root, resync_requester=None):
        self.root = os.path.abspath(root)
        self.state = ReplicaState.STARTING
        self.snapshot = None
        self.manifest = None
        self._resync_requester = resync_requester
        self._resync_attempts = 0
        self._resync_deadline_ns = None
        self.last_error = ''
        # C2 receive-side: decoded-object cache keyed by content address
        # (bounded by the two-generation lifecycle: entries for objects
        # not referenced by the current or previous manifest are dropped
        # at apply time). Unchanged blocks reuse decoded objects instead
        # of re-decoding -- DELTA generations never call the full
        # rebuild_snapshot.
        self._object_cache = {}
        self.cache_hits = 0
        self.cache_misses = 0
        self.control_age_s = None
        self.control_healthy = False
        # G4-A gen3 (plan d87c5731 section 6.1): bounded receive-side
        # event history -- received / validated / ready / resync /
        # unavailable with the target's FULL identity, so measurement
        # can prove commit-to-READY delivery instead of guessing it
        # from the latest observed state.
        import collections as _collections
        self.event_log = _collections.deque(maxlen=8192)

    def _record(self, event, **fields):
        import time as _time
        self.event_log.append(dict({'event': event,
                                    't_ns': _time.monotonic_ns()},
                                   **fields))

    def events(self):
        """Immutable copy of the receive-side event history."""
        return tuple(dict(entry) for entry in self.event_log)

    # -- polling -------------------------------------------------------


    # Slack for the publisher's 0.25s head heartbeat under back-to-back
    # commits: a real manifest never runs more than this far ahead of
    # the publisher's own heartbeat sequence.
    HEAD_SEQ_SLACK = 8

    @property
    def _head_seq(self):
        try:
            head = self.head_state()
        except Exception:  # noqa: BLE001 - heartbeat is advisory
            return None
        if not head:
            return None
        producer = head.get('producer')
        seq = head.get('seq')
        if not isinstance(seq, int):
            return None
        return (str(producer), int(seq))

    def _seq_plausible(self, manifest):
        """True when the manifest's seq is consistent with what the
        PUBLISHER itself has heartbeat-ed (guards the forged-manifest
        wedge: a valid-checksum forged FULL with a huge seq would be
        applied and then wedge the receiver against all real
        generations). No heartbeat or a different producer -> judge
        unknown, stay permissive (other checks govern)."""
        head = self._head_seq
        if head is None:
            return True
        producer, head_seq = head
        if producer != str(manifest['producer_incarnation']):
            return True
        return int(manifest['seq']) <= head_seq + self.HEAD_SEQ_SLACK

    def poll(self):
        """One non-blocking poll cycle. Returns the state after it."""
        try:
            manifest = self._read_published()
        except (WireError, OSError, ValueError) as error:
            return self._fail(str(getattr(error, 'reason', error)))
        if manifest is None:
            if self.manifest is not None:
                # The stream vanished while we HELD a generation: an
                # observable control-plane loss (fault-gate detection
                # event), not silent idling.
                self._record('no_published',
                             seq=self.manifest.get('seq'))
            self.state = ReplicaState.SYNCING
            self.last_error = 'NO_PUBLISHED_GENERATION'
            return self.state
        if (self.manifest is None or int(manifest['seq'])
                != int(self.manifest['seq'])):
            self._record('received', seq=manifest['seq'],
                         kind=manifest['kind'],
                         target_identity=list(
                             manifest['target_identity']))
        if self.manifest is not None:
            # Producer/epoch/schema first, THEN seq (protocol 5.3): a
            # new incarnation restarts the sequence legitimately, and a
            # FULL generation is self-contained so any gap is healed by
            # applying it directly (the resync answer).
            if manifest['producer_incarnation'] != \
                    self.manifest['producer_incarnation']:
                if manifest['kind'] != 'FULL':
                    return self._resync('PRODUCER_CHANGED',
                                        'delta from a new incarnation')
            elif manifest['kind'] == 'FULL' and \
                    int(manifest['seq']) > int(self.manifest['seq']):
                # Self-contained resync answer: apply directly -- but a
                # manifest whose seq runs far AHEAD of the publisher's
                # own heartbeat was never published by this producer
                # (A5 forged-manifest injection): applying it would
                # wedge the receiver, which would then ignore every
                # real lower-seq generation as "old". Reject + resync.
                # Slack covers the 0.25s heartbeat cadence under
                # back-to-back commits.
                if not self._seq_plausible(manifest):
                    return self._resync('MANIFEST_SEQ_AHEAD',
                                        'seq %s vs head %s'
                                        % (manifest['seq'],
                                           self._head_seq))
                pass  # self-contained resync answer: apply directly
            elif int(manifest['seq']) <= int(self.manifest['seq']):
                if (int(manifest['seq']) == int(self.manifest['seq'])
                        and manifest.get('checksum')
                        != self.manifest.get('checksum')):
                    # Same seq with DIFFERENT content: never silently
                    # ignore a rewritten history (gen3 plan 6.3) --
                    # named detection + resync.
                    return self._resync('MANIFEST_CONTENT_CONFLICT',
                                        'seq %s re-published with a '
                                        'different checksum'
                                        % manifest['seq'])
                self.state = ReplicaState.READY
                return self.state
            elif int(manifest['seq']) != int(self.manifest['seq']) + 1:
                return self._resync('SEQ_GAP',
                                    '%s -> %s' % (self.manifest['seq'],
                                                  manifest['seq']))
        if self.manifest is None and manifest['kind'] == 'FULL' \
                and not self._seq_plausible(manifest):
            # A fresh receiver must not adopt a forged high-seq FULL
            # either (same wedge as below).
            return self._resync('MANIFEST_SEQ_AHEAD',
                                'seq %s vs head %s'
                                % (manifest['seq'], self._head_seq))
        if manifest['kind'] == 'DELTA':
            if self.manifest is None:
                return self._resync('DELTA_WITHOUT_BASE')
            if (list(manifest['base_identity'])
                    != list(self.manifest['target_identity'])
                    or manifest['base_manifest_sha']
                    != self.manifest['checksum']):
                return self._resync('BASE_MISMATCH')
            if manifest['producer_incarnation'] != \
                    self.manifest['producer_incarnation']:
                return self._resync('PRODUCER_CHANGED')
        try:
            self.state = ReplicaState.SYNCING
            if (manifest['kind'] == 'DELTA' and self.snapshot is not None
                    and self.manifest is not None):
                snapshot = self._apply_delta(manifest)
            else:
                objects = self._load_objects_all(manifest)
                snapshot = rebuild_snapshot(
                    objects,
                    expected_identity=manifest['target_identity'])
        except WireError as error:
            self._record('validate_failed', seq=manifest['seq'],
                         reason=str(error.reason))
            return self._resync(error.reason, error.detail)
        # Atomic swap: registry state changes only after full validation.
        self._record('validated', seq=manifest['seq'],
                     target_identity=list(manifest['target_identity']))
        self.snapshot = snapshot
        self.manifest = manifest
        self._ack(manifest)
        self.state = ReplicaState.READY
        self.last_error = ''
        self._resync_attempts = 0
        self._record('ready', seq=manifest['seq'],
                     target_identity=list(manifest['target_identity']))
        return self.state

    # -- internals -----------------------------------------------------

    def _read_published(self):
        pointer = os.path.join(self.root, 'published.json')
        if not os.path.exists(pointer):
            return None
        with open(pointer, 'rb') as handle:
            target = json.loads(handle.read().decode('utf-8'))
        manifest_path = os.path.join(self.root, target['manifest'])
        with open(manifest_path, 'rb') as handle:
            body = handle.read(8 * 1024 * 1024 + 1)
        if len(body) > 8 * 1024 * 1024:
            raise WireError('MANIFEST_TOO_LARGE', manifest_path)
        if sha256_hex(body) != target['manifest_sha256']:
            raise WireError('POINTER_MISMATCH', manifest_path)
        manifest = json.loads(body.decode('utf-8'))
        validate_manifest(manifest)
        return manifest

    def _load_payload(self, digest, name):
        """Load+verify one object, or reuse the cached copy when the
        content address was already decoded into this receiver."""
        cached = self._object_cache.get(digest)
        if cached is not None:
            self.cache_hits += 1
            return cached
        self.cache_misses += 1
        path = os.path.join(self.root, 'objects', digest)
        try:
            with open(path, 'rb') as handle:
                payload = handle.read(MAX_OBJECT_BYTES + 1)
        except FileNotFoundError:
            # A vanished object file is a TRANSPORT fault (A5 lost-
            # object injection): a named failure into the resync path,
            # never an uncaught crash of the receiver process. The
            # digest rides the resync request so the publisher can
            # heal exactly this object (precise healing).
            self._resync_object_hint = digest
            raise WireError('OBJECT_MISSING', name) from None
        except OSError as error:
            self._resync_object_hint = digest
            raise WireError('OBJECT_IO_ERROR', str(error)) from None
        if len(payload) > MAX_OBJECT_BYTES:
            raise WireError('OBJECT_TOO_LARGE', name)
        if sha256_hex(payload) != digest:
            self._resync_object_hint = digest
            raise WireError('OBJECT_HASH_MISMATCH', name)
        self._object_cache[digest] = payload
        return payload

    def _load_objects_all(self, manifest):
        return {name: self._load_payload(entry['sha256'], name)
                for name, entry in manifest['entries'].items()}

    def _apply_delta(self, manifest):
        """Incremental assembly: only objects whose content address
        differs from the previous manifest are decoded; unchanged
        blocks reuse the previous snapshot's immutable block objects
        directly. The result must equal a full rebuild of the same
        generation (pinned by tests)."""
        previous_entries = self.manifest['entries']
        changed_names = [
            name for name, entry in manifest['entries'].items()
            if previous_entries.get(name, {}).get('sha256')
            != entry['sha256']]
        # Non-block payloads that changed: decode fresh.
        fresh = {}
        for name in changed_names:
            if name.startswith('block.'):
                continue
            fresh[name] = self._load_payload(
                manifest['entries'][name]['sha256'], name)
        grid_meta = wire_decode_json(self._load_payload(
            manifest['entries']['grid']['sha256'], 'grid'))
        identity_legs = wire_decode_json(self._load_payload(
            manifest['entries']['identity']['sha256'], 'identity'))

        changed_blocks = sorted({
            int(name.split('.')[1]) for name in changed_names
            if name.startswith('block.')})
        new_blocks = list(self.snapshot.blocks)
        for block_id in changed_blocks:
            new_blocks[block_id] = self._rebuild_block(
                manifest, block_id, grid_meta)
        summary = MappingProxyType(dict(wire_decode_json(
            self._load_payload(
                manifest['entries']['summary']['sha256'], 'summary'))))
        provenance = wire_decode_json(self._load_payload(
            manifest['entries']['provenance']['sha256'], 'provenance'))
        from luggage_perception.cargo_query_snapshot_wire import (
            _MapSnapshotView, wire_hull)
        map_snapshot = _MapSnapshotView(provenance)
        grid = GridInfo(
            tuple(grid_meta['shape']), tuple(grid_meta['origin']),
            grid_meta['resolution'], grid_meta['frame_id'],
            grid_meta['geometry_hash'], wire_hull(grid_meta))
        sensor_blocks = MappingProxyType({
            int(k): tuple(v) for k, v in wire_decode_json(
                self._load_payload(manifest['entries']
                                   ['sensor_blocks']['sha256'],
                                   'sensor_blocks')).items()})
        box_blocks = MappingProxyType({
            k: tuple(v) for k, v in wire_decode_json(
                self._load_payload(manifest['entries']
                                   ['box_blocks']['sha256'],
                                   'box_blocks')).items()})
        # Derived payloads (gen3 fix): the manifest transports the
        # CURRENT surface and distance index with EVERY generation;
        # carrying the previous snapshot's copies forward silently
        # served STALE derived data under the new identity (found by
        # the fault gate's full-snapshot wire fingerprint -- blocks and
        # summary were correct, only surface/distance lagged).
        from luggage_perception.cargo_query_snapshot_wire import (
            decode_distance_object, decode_surface_object)
        surface_projection = self.snapshot.surface_projection
        if 'surface' in fresh:
            surface_projection = decode_surface_object(fresh['surface'])
        distance_index = self.snapshot.distance_index
        if 'distance_index.lower' in fresh:
            distance_index = decode_distance_object(
                fresh['distance_index.lower'],
                fresh['distance_index.upper'])
        data = QuerySnapshotData(
            QueryIdentity(identity_legs['epoch'],
                          int(identity_legs['revision']),
                          identity_legs['content_digest'],
                          identity_legs['voxel_digest'],
                          int(identity_legs['index_schema_version'])),
            grid, tuple(new_blocks), sensor_blocks, box_blocks, summary,
            map_snapshot, b'',
            owner_evidence_bytes=int(wire_decode_json(
                self._load_payload(
                    manifest['entries']['owner_evidence_bytes']
                    ['sha256'], 'owner_evidence_bytes'))),
            unbound_geometry_count=int(wire_decode_json(
                self._load_payload(
                    manifest['entries']['unbound_geometry_count']
                    ['sha256'], 'unbound_geometry_count'))),
            surface_projection=surface_projection,
            distance_index=distance_index)
        if tuple(data.identity) != tuple(manifest['target_identity']):
            raise WireError('IDENTITY_MISMATCH',
                            'delta assembly identity mismatch')
        # Bound the cache: keep only objects referenced by the new or
        # previous manifest (two-generation lifecycle).
        referenced = {entry['sha256']
                      for entry in manifest['entries'].values()}
        referenced.update(
            entry['sha256'] for entry in previous_entries.values())
        self._object_cache = {
            digest: payload for digest, payload
            in self._object_cache.items() if digest in referenced}
        return data

    def _rebuild_block(self, manifest, block_id, grid_meta):
        prefix = 'block.%d' % block_id
        entries = manifest['entries']

        def payload(suffix):
            name = prefix + suffix
            if name not in entries:
                raise WireError('MISSING_OBJECT', name)
            return self._load_payload(entries[name]['sha256'], name)

        meta = wire_decode_json(payload('.meta'))
        evidence = bytes(payload('.evidence'))
        active = bytes(payload('.active'))
        frontier = bytes(payload('.frontier'))
        flat = decode_array_object(payload('.flat_indices'))
        members = wire_decode_json(payload('.members'))
        sensor_members = MappingProxyType({
            int(k): bytes.fromhex(v) for k, v in members['sensor'].items()})
        box_members = MappingProxyType({
            k: bytes.fromhex(v) for k, v in members['box'].items()})
        shape = tuple(int(v) for v in meta['shape'])
        layout = DistanceLeafLayout.initialize(shape)
        records = np.frombuffer(evidence, dtype=CELL_DTYPE)
        allowed = np.frombuffer(active, 'u1').astype(bool)
        distance_members = freeze_distance_members(layout, records,
                                                   allowed)
        counts = tuple(int(v) for v in meta['state_counts'])
        recomputed = tuple(int(v) for v in np.bincount(
            records['state'][allowed], minlength=3))
        if counts != recomputed:
            raise WireError('STATE_COUNT_MISMATCH', str(block_id))
        frontier_count = int(np.frombuffer(frontier, 'u1').sum())
        if frontier_count != int(meta['frontier_count']):
            raise WireError('FRONTIER_COUNT_MISMATCH', str(block_id))
        return VoxelBlock(
            int(block_id), tuple(int(v) for v in meta['start']), shape,
            flat, evidence, active, frontier, counts, frontier_count,
            sensor_members, box_members, meta['fact_digest'], layout,
            distance_members)

    def _resync(self, reason, detail=''):
        self._record('resync', reason=str(reason),
                     seq=(self.manifest or {}).get('seq'))
        self.state = ReplicaState.RESYNC_REQUIRED
        self.last_error = '%s %s' % (reason, detail)
        request = os.path.join(self.root, 'control', 'resync')
        hint = getattr(self, '_resync_object_hint', None)
        self._resync_object_hint = None
        try:
            with open(request + '.tmp', 'w') as handle:
                handle.write(json.dumps({'reason': reason,
                                         'seq': (self.manifest or {})
                                         .get('seq', 0),
                                         'object': hint}))
            os.replace(request + '.tmp', request)
        except OSError:
            pass
        if self._resync_requester is not None:
            self._resync_requester()
        return self.state

    def _fail(self, reason):
        self._record('unavailable', error=str(reason))
        self.state = ReplicaState.UNAVAILABLE
        self.last_error = str(reason)
        return self.state

    def head_state(self):
        """Latest head.json contents (the writer's committed view).

        A fresh heartbeat here is NECESSARY but NOT SUFFICIENT for map
        freshness (user 2026-10-26): callers must also compare the
        committed identity against the local READY identity and check
        the error surface and lag bound."""
        head_path = os.path.join(self.root, 'head.json')
        try:
            with open(head_path, 'rb') as handle:
                return json.loads(handle.read().decode('utf-8'))
        except (OSError, ValueError):
            return {}

    def refresh_control_freshness(self, lag_bound_s):
        """Track whether the head heartbeat keeps advancing; a stalled
        writer must degrade service even if the last head file looks
        recent (staleness is computed against the polling clock, not
        the file's own claim)."""
        import time
        head = self.head_state()
        published = int(head.get('published_monotonic_ns', 0))
        now = time.monotonic_ns()
        self.control_age_s = (now - published) / 1e9 if published else None
        self.control_healthy = (
            bool(head) and not head.get('error')
            and self.control_age_s is not None
            and self.control_age_s <= lag_bound_s)

    def _ack(self, manifest):
        ack = os.path.join(self.root, 'control', 'ack')
        try:
            with open(ack + '.tmp', 'w') as handle:
                handle.write(json.dumps(
                    {'seq': manifest['seq'],
                     'identity': list(manifest['target_identity'])}))
            os.replace(ack + '.tmp', ack)
        except OSError:
            pass
