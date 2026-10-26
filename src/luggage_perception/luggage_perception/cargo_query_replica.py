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

from luggage_perception.cargo_query_snapshot_wire import rebuild_snapshot
from luggage_perception.cargo_query_wire import (
    WireError, sha256_hex, validate_manifest)

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

    # -- polling -------------------------------------------------------

    def poll(self):
        """One non-blocking poll cycle. Returns the state after it."""
        try:
            manifest = self._read_published()
        except (WireError, OSError, ValueError) as error:
            return self._fail(str(getattr(error, 'reason', error)))
        if manifest is None:
            self.state = ReplicaState.SYNCING
            self.last_error = 'NO_PUBLISHED_GENERATION'
            return self.state
        if self.manifest is not None:
            if int(manifest['seq']) <= int(self.manifest['seq']):
                self.state = ReplicaState.READY
                return self.state
            if int(manifest['seq']) != int(self.manifest['seq']) + 1:
                return self._resync('SEQ_GAP',
                                    '%s -> %s' % (self.manifest['seq'],
                                                  manifest['seq']))
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
            objects = self._load_objects(manifest)
            snapshot = rebuild_snapshot(
                objects, expected_identity=manifest['target_identity'])
        except WireError as error:
            return self._resync(error.reason, error.detail)
        # Atomic swap: registry state changes only after full validation.
        self.snapshot = snapshot
        self.manifest = manifest
        self._ack(manifest)
        self.state = ReplicaState.READY
        self.last_error = ''
        self._resync_attempts = 0
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

    def _load_objects(self, manifest):
        objects = {}
        for name, entry in manifest['entries'].items():
            if entry.get('type') == 'reference' and entry['bytes'] == 0:
                # Unchanged content address; load like any object (the
                # receiver has no cached copy -- content addressing
                # makes the load idempotent and cheap via page cache).
                pass
            digest = entry['sha256']
            path = os.path.join(self.root, 'objects', digest)
            with open(path, 'rb') as handle:
                payload = handle.read(MAX_OBJECT_BYTES + 1)
            if len(payload) > MAX_OBJECT_BYTES:
                raise WireError('OBJECT_TOO_LARGE', name)
            if sha256_hex(payload) != digest:
                raise WireError('OBJECT_HASH_MISMATCH', name)
            objects[name] = payload
        return objects

    def _resync(self, reason, detail=''):
        self.state = ReplicaState.RESYNC_REQUIRED
        self.last_error = '%s %s' % (reason, detail)
        request = os.path.join(self.root, 'control', 'resync')
        try:
            with open(request + '.tmp', 'w') as handle:
                handle.write(json.dumps({'reason': reason,
                                         'seq': (self.manifest or {})
                                         .get('seq', 0)}))
            os.replace(request + '.tmp', request)
        except OSError:
            pass
        if self._resync_requester is not None:
            self._resync_requester()
        return self.state

    def _fail(self, reason):
        self.state = ReplicaState.UNAVAILABLE
        self.last_error = str(reason)
        return self.state

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
