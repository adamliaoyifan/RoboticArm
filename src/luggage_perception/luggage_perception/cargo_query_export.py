"""G4-A query export publisher: immutable generations on a private root.

Protocol 1c0049d4 sections 3-4 (A0 freeze) + gen3 boundary contract
(plan d87c5731 section 5, review_8a372dee P1-4): the publisher is a
pure module called by the store AFTER a commit. It never mutates map
facts; a failed export is a named status, never a silent success.

Root layout (all writes by the publisher only, except replica ACKs):

    owner.json                     exclusive owner lock (O_CREAT|O_EXCL)
    head.json                      latest committed head (atomic replace)
    published.json                 pointer to the newest complete manifest
    latest_full.json               pointer to the newest FULL manifest
    objects/<sha256>               immutable content-addressed payloads
    generations/<inc>/<seq>/manifest.json
    staging/                       bounded in-flight export
    control/ack, control/resync    replica-owned control messages

Gen3 lifecycle contract (approved coalescing, d87c5731):
- Facts commit per-update; the EXPORTER may coalesce queued work to
  the latest committed state. A replaced task never claims READY: the
  per-fact ledger records exact_visible / coalesced_into /
  export_failed (+ timeout computed at read time).
- Coalescing is attempted BEFORE any capacity rejection: one item may
  encode (in flight) while at most one newer item waits; a new offer
  merges with the waiting item (dirty union, provenance OR, any FULL
  request stays FULL). Tasks from a different world (epoch/producer)
  never merge; the older task is dropped as superseded_by_reset.
- An encode failure keeps its dirty set and is retried with bounded
  backoff even with no further writes, or keeps failing NAMED -- the
  head never claims facts stopped at the previous accepted task.
- wait_pending drains only when queued AND in-flight AND unresolved
  failures are all clear. close() stops new offers, drains bounded;
  if the encoder thread does not exit, close FAILS and the owner lock
  is RETAINED (no second publisher on the same root).

Every manifest is a SELF-CONTAINED entry table (name -> content
address); a DELTA writes only the changed objects and points every
unchanged entry at its existing address -- identical blocks hash
identically because payloads are pure functions of the block.
"""

from __future__ import annotations

import collections
import json
import os
import shutil
import threading
import time

from luggage_perception.cargo_query_snapshot_wire import (
    DISTANCE_RECIPE_VERSION, encode_snapshot)
from luggage_perception.cargo_query_wire import (
    WIRE_SCHEMA_VERSION, WireError, canonical_json_bytes, manifest_checksum,
    new_producer_incarnation, sha256_hex, validate_manifest)

PENDING_LIMIT = 2            # in-flight (1) + queued (1)
STAGING_LIMIT = 1
PUBLISHED_LIMIT = 2
MAX_PENDING_ENTRIES = 1 << 20
RETRY_BACKOFF_S = (0.25, 0.5, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0)


class ExportError(RuntimeError):
    def __init__(self, reason, detail=''):
        super().__init__(reason)
        self.reason = reason
        self.detail = detail


def _world_of(snapshot):
    identity = getattr(snapshot, 'identity', None)
    return identity[0] if identity is not None else None


class QueryExportPublisher:
    """Owns one export root. One publisher per root (exclusive lock)."""

    HEAD_UPDATE_PERIOD_NS = 250_000_000  # protocol: 0.25 s

    def __init__(self, root, producer_incarnation=None, clock_ns=None,
                 async_mode=False):
        self.root = os.path.abspath(root)
        self._clock_ns = clock_ns or time.monotonic_ns
        self._async = bool(async_mode)
        self._encoder = None
        self._encoder_error = ''
        self.healed_objects = 0
        self._latest_committed = None   # (snapshot, dirty, provenance)
        self._commit_ns = 0             # offer time of the newest FACT
        self._head_written_ns = 0
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
        self._unpublished_from = None
        # Gen3 lifecycle state (short lock; all IO outside):
        self._state_lock = threading.Lock()
        self._queued = collections.deque()   # at most ONE waiting item
        self._in_flight = None               # item currently encoding
        self._failure = None                 # unresolved encode failure
        self._closing = False
        self._wake = threading.Event()
        self._ledger = []                    # per committed offered fact
        self._ledger_seq = 0
        self._recover()
        if self._async:
            self._encoder = threading.Thread(
                target=self._encode_loop, name='occ-query-export',
                daemon=True)
            self._encoder.start()

    # -- lifecycle accounting (plan section 5.1) ------------------------

    @property
    def _pending(self):
        """Compat surface: queued + in-flight + unresolved failure."""
        with self._state_lock:
            return (len(self._queued) + (self._in_flight is not None)
                    + (self._failure is not None))

    def _ledger_open(self, snapshot, commit_ns):
        self._ledger_seq += 1
        entry = {'ledger_id': self._ledger_seq,
                 'identity': (list(getattr(snapshot, 'identity', ()))
                              or None),
                 'commit_monotonic_ns': commit_ns,
                 'status': 'pending',
                 'published_seq': None}
        self._ledger.append(entry)
        return entry

    def _ledger_set(self, entries, status, seq=None):
        for entry in entries:
            entry['status'] = status
            entry['published_seq'] = seq

    def ledger_snapshot(self, timeout_s=None, now_ns=None):
        """Per-fact delivery ledger (approved coalescing contract):
        exact_visible / coalesced_into / export_failed / superseded_by_reset
        / pending (+ 'timeout' derived at read time when timeout_s set)."""
        now_ns = self._clock_ns() if now_ns is None else now_ns
        with self._state_lock:
            out = []
            for entry in self._ledger:
                item = dict(entry)
                if (timeout_s is not None and item['status'] == 'pending'
                        and item['commit_monotonic_ns']
                        and now_ns - item['commit_monotonic_ns']
                        > int(timeout_s * 1e9)):
                    item['status'] = 'timeout'
                out.append(item)
            return out

    def _coalesce(self, old, new):
        """Merge offer ``new`` into waiting item ``old`` (same world).
        Returns the merged item. Any FULL request stays FULL."""
        dirty = new['dirty']
        previous = old['dirty']
        if dirty is None or previous is None:
            dirty = None
        else:
            dirty = set(dirty) | set(previous)
        merged = dict(old)
        merged.update({
            'snapshot': new['snapshot'],
            'dirty': dirty,
            'provenance': bool(new['provenance'] or old['provenance']),
            'force_full': bool(new['force_full'] or old['force_full']),
            'commit_ns': new['commit_ns'],
            'facts': old['facts'] + new['facts'],
            'fresh': True})
        return merged

    # -- background encoder ---------------------------------------------

    def _encode_loop(self):
        """The UNIQUE encoder thread (protocol section 5.1): all
        encoding and IO happen here, outside the state lock. Facts are
        already committed -- a failure here is a named export status,
        never a rollback."""
        try:
            import ctypes
            libc = ctypes.CDLL('libc.so.6', use_errno=True)
            libc.prctl(15, b'occ-query-export', 0, 0, 0)  # PR_SET_NAME
        except Exception:  # noqa: BLE001 - cosmetic naming only
            pass
        while True:
            self._answer_resync_requests()
            item = None
            with self._state_lock:
                now = self._clock_ns()
                if self._failure is not None and now >= self._failure[
                        'next_retry_ns']:
                    item = self._failure
                    self._failure = None
                    self._in_flight = item
                elif self._queued:
                    item = self._queued.popleft()
                    self._in_flight = item
                elif self._closing:
                    return
            if item is None:
                self._head_loop_tick()
                self._wake.wait(0.25)
                self._wake.clear()
                continue
            # Encoding and disk IO happen with NO state lock held.
            try:
                result = self._publish(item['snapshot'], item['dirty'],
                                       item['provenance'],
                                       item['force_full'],
                                       commit_ns=item['commit_ns'])
                published_seq = result.get('seq') if result else None
            except (WireError, OSError, ValueError, RuntimeError) as error:
                self._encoder_error = str(error)
                self._mark_unhealthy(
                    'EXPORT_' + getattr(error, 'reason', 'IO_ERROR'))
                with self._state_lock:
                    attempts = item.get('attempts', 0) + 1
                    backoff = RETRY_BACKOFF_S[
                        min(attempts - 1, len(RETRY_BACKOFF_S) - 1)]
                    self._failure = dict(
                        item, attempts=attempts, error=self._encoder_error,
                        next_retry_ns=self._clock_ns() + int(backoff * 1e9))
                    self._in_flight = None
                    self._ledger_set(item['facts'], 'pending_retry')
            else:
                # A successful (re)publish clears the sticky encoder
                # error only after the recovery itself succeeded.
                self._encoder_error = ''
                with self._state_lock:
                    self._in_flight = None
                    facts = item['facts']
                    if facts:
                        self._ledger_set(facts[:-1], 'coalesced_into',
                                         seq=published_seq)
                        self._ledger_set(facts[-1:], 'exact_visible',
                                         seq=published_seq)
            # The heartbeat must also refresh under SUSTAINED load.
            self._head_loop_tick()

    def _answer_resync_requests(self):
        """The receiver's RESYNC request (control/resync) is answered
        with a FULL generation of the LATEST committed snapshot
        (protocol section 5; the request file is consumed exactly once)."""
        request = os.path.join(self.root, 'control', 'resync')
        if not os.path.exists(request):
            return False
        latest = self._latest_committed
        heal_hint = None
        try:
            with open(request, 'rb') as handle:
                heal_hint = (json.loads(handle.read() or b'{}')
                             .get('object'))
        except (OSError, ValueError):
            heal_hint = None
        try:
            os.unlink(request)
        except OSError:
            return False
        if latest is None:
            return False
        snapshot, dirty_blocks, provenance_changed = latest
        try:
            self._publish(snapshot, None, provenance_changed, True,
                          heal_hint=heal_hint)
            return True
        except (WireError, OSError, ValueError, RuntimeError) as error:
            self._encoder_error = str(error)
            self._mark_unhealthy(
                'EXPORT_' + getattr(error, 'reason', 'IO_ERROR'))
            return False

    def _head_loop_tick(self):
        """head.json heartbeat: refreshes the HEARTBEAT stamp only --
        the commit stamp always carries the real offer time of the
        newest committed fact (a heartbeat never resets the
        propagation clock; plan section 4)."""
        now = self._clock_ns()
        if now - self._head_written_ns < self.HEAD_UPDATE_PERIOD_NS:
            return
        self._head_written_ns = now
        latest = self._latest_committed
        if latest is None:
            return
        snapshot = latest[0]
        head = os.path.join(self.root, 'head.json')
        temp = head + '.tmp'
        try:
            with open(temp, 'wb') as handle:
                handle.write(canonical_json_bytes({
                    'producer': self.incarnation,
                    'seq': self._seq,
                    'target_identity': list(snapshot.identity),
                    'commit_monotonic_ns': self._commit_ns,
                    'heartbeat_monotonic_ns': now,
                    'published_monotonic_ns': now,
                    'error': self._encoder_error,
                }))
            os.replace(temp, head)
        except OSError:
            pass

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
        """Register one committed snapshot for export. Returns a result
        dict. Never claims READY: delivery truth lives in the ledger.

        dirty_blocks: block ids whose payloads changed (the write path's
        dirty set INCLUDING frontier neighbourhoods). None -> FULL.
        """
        now = self._clock_ns()
        # The facts-latest-commit observation surface is updated even
        # when the offer is rejected: head must never imply facts
        # stopped at the previous accepted task (plan section 5.1).
        self._commit_ns = now
        self._latest_committed = (snapshot, dirty_blocks,
                                   provenance_changed)
        self._head_loop_tick()
        if self._async:
            entry = self._ledger_open(snapshot, now)
            item = {'snapshot': snapshot, 'dirty': dirty_blocks,
                    'provenance': bool(provenance_changed),
                    'force_full': bool(force_full), 'commit_ns': now,
                    'facts': [entry], 'attempts': 0}
            with self._state_lock:
                if self._closing:
                    self._ledger_set([entry], 'export_failed')
                    return {'status': 'EXPORT_CLOSING',
                            'unpublished_from': self._unpublished_from}
                # Fold an unresolved failure into this NEWER committed
                # state: the union of dirty sets must still export.
                if self._failure is not None:
                    failed = self._failure
                    item = self._coalesce(failed, item)
                    item['facts'] = failed['facts'] + [entry]
                    item['attempts'] = failed.get('attempts', 0)
                    self._failure = None
                if self._queued:
                    waiting = self._queued.popleft()
                    if (_world_of(waiting['snapshot']) is not None
                            and _world_of(snapshot) is not None
                            and _world_of(waiting['snapshot'])
                            != _world_of(snapshot)):
                        # Reset boundary: different worlds never merge;
                        # the older world's task is dropped NOT-delivered
                        # (a new-world READY must never write it off as
                        # delivered -- plan section 1.1).
                        self._ledger_set(waiting['facts'],
                                         'superseded_by_reset')
                        self._queued.append(item)
                        status = 'WORLD_RESET'
                    else:
                        merged = self._coalesce(waiting, item)
                        self._queued.append(merged)
                        status = 'COALESCED'
                else:
                    self._queued.append(item)
                    status = 'ACCEPTED'
                self._wake.set()
            return {'status': status, 'async': True,
                    'seq': self._seq + 1,
                    'coalesced': status == 'COALESCED'}
        staging = os.path.join(self.root, 'staging')
        if len(os.listdir(staging)) > STAGING_LIMIT:
            self._mark_unhealthy('EXPORT_BACKLOG_FULL')
            return {'status': 'EXPORT_BACKLOG_FULL', 'staging_busy': True}
        try:
            return self._publish(snapshot, dirty_blocks,
                                 provenance_changed, force_full,
                                 commit_ns=now)
        except WireError as error:
            self._mark_unhealthy('EXPORT_' + error.reason)
            return {'status': 'EXPORT_' + error.reason,
                    'detail': error.detail}
        except OSError as error:
            self._mark_unhealthy('EXPORT_IO_ERROR')
            return {'status': 'EXPORT_IO_ERROR', 'detail': str(error)}

    def publish_resync_full(self, snapshot):
        """Answer a replica resync request with a FULL generation."""
        return self.offer_committed(snapshot, dirty_blocks=None,
                                    force_full=True)

    def _publish(self, snapshot, dirty_blocks, provenance_changed,
                 force_full, heal_hint=None, commit_ns=None):
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
                'commit_monotonic_ns': (commit_ns
                                        if commit_ns is not None
                                        else self._clock_ns()),
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
            # Precise healing (2026-10-27): the receiver's resync
            # request names the object digest it failed on; only that
            # file is verified by hash and rewritten.
            written = skipped = 0
            for name, (digest, payload) in emitted.items():
                path = os.path.join(self.root, 'objects', digest)
                if os.path.exists(path):
                    if digest != heal_hint:
                        skipped += 1
                        continue
                    with open(path, 'rb') as handle:
                        if sha256_hex(handle.read()) == digest:
                            skipped += 1
                            continue
                    self.healed_objects += 1
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
        now = self._clock_ns()
        with open(temp, 'wb') as handle:
            handle.write(canonical_json_bytes({
                'producer': self.incarnation,
                'seq': manifest['seq'],
                'target_identity': list(snapshot.identity),
                'commit_monotonic_ns': manifest['commit_monotonic_ns'],
                'heartbeat_monotonic_ns': now,
                'published_monotonic_ns': now,
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

    def wait_pending(self, timeout_s=30.0):
        """True only when queued, in-flight AND unresolved failures are
        all clear (a blocked encoder keeps the item in flight)."""
        if not self._async:
            return True
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self._state_lock:
                drained = (not self._queued and self._in_flight is None
                           and self._failure is None)
            if drained and not self._encoder_error:
                return True
            time.sleep(0.005)
        return False

    def close(self, timeout_s=5.0):
        """Stop accepting offers, then drain bounded. SUCCESS releases
        the owner lock; if the encoder did not exit (or the drain did
        not finish) close FAILS and the owner lock is RETAINED so no
        second publisher can write this root."""
        if self._async and self._encoder is not None:
            with self._state_lock:
                self._closing = True
                self._wake.set()
            self._encoder.join(timeout=timeout_s)
            if self._encoder.is_alive():
                return False
            with self._state_lock:
                drained = (not self._queued and self._in_flight is None
                           and self._failure is None)
            if not drained:
                return False
        try:
            os.unlink(os.path.join(self.root, 'owner.json'))
        except OSError:
            pass
        return True

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
