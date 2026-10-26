"""G4-A query export publisher: immutable generations on a private root.

Protocol 1c0049d4 sections 3-4 (A0 freeze): the publisher is a pure
module called by the store AFTER a commit. It never mutates map facts;
a failed export is a named status, never a silent success.

Root layout (all writes by the publisher only, except replica ACKs):

    owner.json                     exclusive owner lock (O_CREAT|O_EXCL)
    head.json                      latest committed head (atomic replace)
    published.json                 pointer to the newest complete manifest
    latest_full.json               pointer to the newest FULL manifest
    objects/<sha256>               immutable content-addressed payloads
    generations/<inc>/<seq>/manifest.json
    staging/                       bounded in-flight export
    control/ack, control/resync    replica-owned control messages

Bounded lifecycle: pending <= 2 un-published offered generations,
staging <= 1 in-flight export, published readable generations <= 2.
Overflow returns EXPORT_BACKLOG_FULL (blocking, never dropping).

Every manifest is a SELF-CONTAINED entry table (name -> content
address); a DELTA writes only the changed objects and points every
unchanged entry at its existing address -- identical blocks hash
identically because payloads are pure functions of the block.
"""

from __future__ import annotations

import json
import os
import shutil
import time

from luggage_perception.cargo_query_snapshot_wire import (
    DISTANCE_RECIPE_VERSION, encode_snapshot)
from luggage_perception.cargo_query_wire import (
    WIRE_SCHEMA_VERSION, WireError, canonical_json_bytes, manifest_checksum,
    new_producer_incarnation, sha256_hex, validate_manifest)

PENDING_LIMIT = 2
STAGING_LIMIT = 1
PUBLISHED_LIMIT = 2
MAX_PENDING_ENTRIES = 1 << 20


class ExportError(RuntimeError):
    def __init__(self, reason, detail=''):
        super().__init__(reason)
        self.reason = reason
        self.detail = detail


class QueryExportPublisher:
    """Owns one export root. One publisher per root (exclusive lock)."""

    def __init__(self, root, producer_incarnation=None, clock_ns=None):
        self.root = os.path.abspath(root)
        self._clock_ns = clock_ns or time.monotonic_ns
        os.makedirs(os.path.join(self.root, 'objects'), exist_ok=True)
        os.makedirs(os.path.join(self.root, 'generations'), exist_ok=True)
        os.makedirs(os.path.join(self.root, 'staging'), exist_ok=True)
        os.makedirs(os.path.join(self.root, 'control'), exist_ok=True)
        owner = os.path.join(self.root, 'owner.json')
        if os.path.exists(owner):
            raise ExportError('OWNER_LOCK_HELD', owner)
        self.incarnation = (producer_incarnation
                            or new_producer_incarnation())
        fd = os.open(owner, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
        with os.fdopen(fd, 'w') as handle:
            handle.write(canonical_json_bytes({
                'incarnation': self.incarnation,
                'pid': os.getpid(),
                'started_ns': self._clock_ns(),
            }).decode('utf-8'))
        os.chmod(self.root, 0o700)
        self._published = []      # [(seq, manifest_sha, path)] newest last
        self._entries = {}        # name -> sha of the newest generation
        self._seq = 0
        self._pending = 0
        self._unpublished_from = None
        self._recover()

    # -- recovery ------------------------------------------------------

    def _recover(self):
        pointer = os.path.join(self.root, 'published.json')
        if not os.path.exists(pointer):
            return
        try:
            manifest = self._read_manifest(pointer)
        except (WireError, OSError, ValueError):
            # A torn pointer must never expose a half generation; treat
            # as no published state (resync path re-establishes FULL).
            return
        if manifest['producer_incarnation'] != self.incarnation:
            return
        self._seq = int(manifest['seq'])
        self._entries = {name: entry['sha256']
                         for name, entry in manifest['entries'].items()}
        self._last_target_identity = tuple(manifest['target_identity'])
        self._published = [(self._seq, manifest['checksum'], pointer)]

    def _read_manifest(self, path):
        with open(path, 'rb') as handle:
            raw = handle.read(8 * 1024 * 1024 + 1)
        if len(raw) > 8 * 1024 * 1024:
            raise WireError('MANIFEST_TOO_LARGE', path)
        manifest = json.loads(raw.decode('utf-8'))
        validate_manifest(manifest)
        return manifest

    # -- publishing ----------------------------------------------------

    def offer_committed(self, snapshot, dirty_blocks=None,
                        provenance_changed=True, force_full=False):
        """Encode + publish one committed snapshot. Returns a result dict.

        dirty_blocks: block ids whose payloads changed (the write path's
        dirty set INCLUDING frontier neighbourhoods). None -> FULL.
        """
        if self._pending >= PENDING_LIMIT:
            self._mark_unhealthy('EXPORT_BACKLOG_FULL')
            return {'status': 'EXPORT_BACKLOG_FULL',
                    'unpublished_from': self._unpublished_from}
        staging = os.path.join(self.root, 'staging')
        if len(os.listdir(staging)) > STAGING_LIMIT:
            self._mark_unhealthy('EXPORT_BACKLOG_FULL')
            return {'status': 'EXPORT_BACKLOG_FULL', 'staging_busy': True}
        self._pending += 1
        try:
            return self._publish(snapshot, dirty_blocks,
                                 provenance_changed, force_full)
        except WireError as error:
            self._mark_unhealthy('EXPORT_' + error.reason)
            return {'status': 'EXPORT_' + error.reason,
                    'detail': error.detail}
        except OSError as error:
            self._mark_unhealthy('EXPORT_IO_ERROR')
            return {'status': 'EXPORT_IO_ERROR', 'detail': str(error)}
        finally:
            self._pending = max(0, self._pending - 1)

    def publish_resync_full(self, snapshot):
        """Answer a replica resync request with a FULL generation."""
        return self.offer_committed(snapshot, dirty_blocks=None,
                                    force_full=True)

    def _publish(self, snapshot, dirty_blocks, provenance_changed,
                 force_full):
        kind = 'FULL'
        base_identity = None
        base_sha = None
        entries = {}
        if (self._published and not force_full
                and (dirty_blocks is not None or not provenance_changed)):
            kind = 'DELTA'
            base_identity = list(self._last_target_identity)
            base_sha = self._published[-1][1]
            entries = dict(self._entries)
        staged = os.path.join(self.root, 'staging',
                              'gen-%d' % (self._seq + 1))
        os.makedirs(staged, exist_ok=True)
        try:
            payloads = encode_snapshot(
                snapshot, self.incarnation, self._seq + 1,
                dirty_blocks=(None if kind == 'FULL' else
                              set(dirty_blocks or ())),
                provenance_changed=provenance_changed)
            emitted = {}
            for name, payload in payloads.items():
                digest = sha256_hex(payload)
                entries[name] = digest
                emitted[name] = (digest, payload)
            manifest = {
                'wire_schema_version': WIRE_SCHEMA_VERSION,
                'producer_incarnation': self.incarnation,
                'map_epoch': snapshot.identity.epoch,
                'seq': self._seq + 1,
                'kind': kind,
                'base_identity': base_identity,
                'base_manifest_sha': base_sha,
                'target_identity': list(snapshot.identity),
                'geometry_hash': snapshot.grid.geometry_hash,
                'grid': {
                    'shape': [int(v) for v in snapshot.grid.shape],
                    'origin': [float(v) for v in snapshot.grid.origin],
                    'resolution': float(snapshot.grid.resolution),
                },
                'commit_monotonic_ns': self._clock_ns(),
                'distance_recipe_version': DISTANCE_RECIPE_VERSION,
                'entries': {name: {'sha256': digest, 'bytes': len(payload),
                                   'type': 'object'}
                            for name, (digest, payload)
                            in emitted.items()},
                'deletes': [],
            }
            if kind == 'DELTA':
                # Self-contained table: unchanged entries keep their
                # existing addresses; only changed objects were emitted.
                for name, digest in self._entries.items():
                    manifest['entries'].setdefault(
                        name, {'sha256': digest, 'bytes': 0,
                               'type': 'reference'})
            manifest['checksum'] = manifest_checksum(manifest)
            validate_manifest(manifest)
            if len(manifest['entries']) > MAX_PENDING_ENTRIES:
                raise WireError('ENTRY_TABLE_TOO_LARGE')
            # Write new objects (content-addressed; unchanged skipped).
            written = skipped = 0
            for name, (digest, payload) in emitted.items():
                path = os.path.join(self.root, 'objects', digest)
                if os.path.exists(path):
                    skipped += 1
                    continue
                temp = path + '.tmp'
                with open(temp, 'wb') as handle:
                    handle.write(payload)
                    handle.flush()
                os.replace(temp, path)
                written += 1
            gen_dir = os.path.join(
                self.root, 'generations', self.incarnation,
                str(manifest['seq']))
            os.makedirs(gen_dir, exist_ok=True)
            manifest_path = os.path.join(gen_dir, 'manifest.json')
            body = canonical_json_bytes(manifest)
            with open(manifest_path, 'wb') as handle:
                handle.write(body)
            self._atomic_pointer('published.json', manifest_path, body)
            if kind == 'FULL':
                self._atomic_pointer('latest_full.json', manifest_path,
                                     body)
            self._write_head(snapshot, manifest)
        finally:
            shutil.rmtree(staged, ignore_errors=True)
        self._seq += 1
        # Keep the complete table from the manifest (addresses for all).
        self._entries = {
            name: entry['sha256']
            for name, entry in manifest['entries'].items()}
        self._last_target_identity = tuple(snapshot.identity)
        self._published.append((self._seq, manifest['checksum'],
                                manifest_path))
        while len(self._published) > PUBLISHED_LIMIT:
            self._published.pop(0)
        self._unpublished_from = None
        return {'status': 'OK', 'seq': self._seq, 'kind': kind,
                'objects_written': written, 'objects_deduped': skipped,
                'entries': len(manifest['entries'])}

    def _atomic_pointer(self, name, manifest_path, body):
        pointer = os.path.join(self.root, name)
        temp = pointer + '.tmp'
        with open(temp, 'wb') as handle:
            handle.write(canonical_json_bytes(
                {'manifest': os.path.relpath(manifest_path, self.root),
                 'manifest_sha256': sha256_hex(body)}))
        os.replace(temp, pointer)

    def _write_head(self, snapshot, manifest):
        head = os.path.join(self.root, 'head.json')
        temp = head + '.tmp'
        with open(temp, 'wb') as handle:
            handle.write(canonical_json_bytes({
                'producer': self.incarnation,
                'seq': manifest['seq'],
                'target_identity': list(snapshot.identity),
                'commit_monotonic_ns': manifest['commit_monotonic_ns'],
                'published_monotonic_ns': self._clock_ns(),
                'error': '',
            }))
        os.replace(temp, head)

    def _mark_unhealthy(self, reason):
        self._unpublished_from = (self._unpublished_from or self._seq + 1)
        head = os.path.join(self.root, 'head.json')
        if not os.path.exists(head):
            return
        try:
            with open(head, 'rb') as handle:
                state = json.loads(handle.read().decode('utf-8'))
            state['error'] = reason
            temp = head + '.tmp'
            with open(temp, 'wb') as handle:
                handle.write(canonical_json_bytes(state))
            os.replace(temp, head)
        except (OSError, ValueError):
            pass

    def close(self):
        try:
            os.unlink(os.path.join(self.root, 'owner.json'))
        except OSError:
            pass

    # -- replica-facing reads ------------------------------------------

    def read_published_manifest(self):
        pointer = os.path.join(self.root, 'published.json')
        if not os.path.exists(pointer):
            return None
        with open(pointer, 'rb') as handle:
            target = json.loads(handle.read().decode('utf-8'))
        manifest_path = os.path.join(self.root, target['manifest'])
        with open(manifest_path, 'rb') as handle:
            body = handle.read()
        if sha256_hex(body) != target['manifest_sha256']:
            raise WireError('POINTER_MISMATCH', manifest_path)
        manifest = json.loads(body.decode('utf-8'))
        validate_manifest(manifest)
        return manifest

    def load_object(self, digest):
        path = os.path.join(self.root, 'objects', digest)
        with open(path, 'rb') as handle:
            payload = handle.read()
        if sha256_hex(payload) != digest:
            raise WireError('OBJECT_HASH_MISMATCH', digest)
        return payload
