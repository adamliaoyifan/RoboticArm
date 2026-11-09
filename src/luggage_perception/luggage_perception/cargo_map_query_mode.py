"""G4-A C3: query-mode wiring for the store and launch surfaces.

cargo_map_query_mode = in_process (DEFAULT, unchanged) | isolated.

- in_process: the store keeps serving queries from its own registry
  exactly as before (zero behavior change; the default must never
  silently flip).
- isolated: the store's typed query services are NOT registered by the
  mapper process; a single CargoMapQueryReplicaNode owns them. The
  store hook offers committed snapshots to the async export publisher
  AFTER commit: enqueue-only (no encode/IO on the caller thread);
  a full queue or an export failure is a named, non-blocking status --
  the physical fact is already committed and never rolls back.
"""

import os


class QueryMode:
    IN_PROCESS = 'in_process'
    ISOLATED = 'isolated'
    VALID = (IN_PROCESS, ISOLATED)

    @classmethod
    def from_launch(cls, value):
        mode = str(value or cls.IN_PROCESS).strip().lower()
        if mode not in cls.VALID:
            raise ValueError('cargo_map_query_mode must be one of %s, '
                             'got %r' % (cls.VALID, value))
        return mode


class StoreExportHook:
    """Post-commit hook owning the async export publisher lifecycle.

    The hook never raises into the commit path: every publisher status
    is recorded (named), a full queue returns EXPORT_BACKLOG_FULL
    immediately, and the store's accepted update result is untouched.
    """

    def __init__(self, export_root, publisher_factory=None):
        from luggage_perception.cargo_query_export import (
            QueryExportPublisher)
        factory = publisher_factory or (
            lambda root: QueryExportPublisher(root, async_mode=True))
        self._publisher = factory(export_root)
        self.last_status = None
        self.status_history = []

    def offer_committed(self, snapshot, dirty_blocks, provenance_changed):
        status = self._publisher.offer_committed(
            snapshot, dirty_blocks, provenance_changed)
        self.last_status = status
        self.status_history.append(status)
        return status

    def wait_pending(self, timeout_s=30.0):
        return self._publisher.wait_pending(timeout_s)

    def close(self):
        self._publisher.close()

    @property
    def export_root(self):
        return self._publisher.root


def wire_store_export(store, export_root, dirty_tracker=None):
    """Attach the post-commit export hook to an OccupancyMapStore.

    dirty_tracker: optional callable(snapshot, previous_snapshot) ->
    set of dirty block ids (the write path's dirty closure). Defaults
    to an identity/digest comparison which is correct though less
    precise -- callers that can supply the transaction dirty set
    should.
    """
    hook = StoreExportHook(export_root)
    store._query_export_hook = hook
    previous = {'snapshot': None}

    def dirty_from_compare(before, after):
        if before is None:
            return None
        return {b1.block_id for b0, b1 in zip(before.blocks, after.blocks)
                if b0.evidence != b1.evidence
                or b0.frontier != b1.frontier
                or b0.fact_digest != b1.fact_digest}

    tracker = dirty_tracker or dirty_from_compare

    original_build = store._build_snapshot

    def build_with_hook(update=None, details=None):
        snapshot = original_build(update, details)
        try:
            # Priority-2 (user 2026-10-27): the change set is computed
            # ONCE by the write path (QueryIndexOwner._update_blocks)
            # and consumed here directly -- no all-block re-scan on the
            # commit path.
            #
            # Review finding 1 (2026-10-27): one transaction can refresh
            # the indexes twice (mutate-time + snapshot-build); the
            # per-refresh _last_dirty_blocks only held the LAST (empty)
            # set, so the exported DELTA dropped the real changes and
            # the replica kept stale blocks under the new identity. The
            # owner now ACCUMULATES dirty ids across refreshes; we peek
            # the transaction set and clear it only once the exporter
            # accepts the generation (a backlogged/failed offer keeps
            # the dirt for the next generation). None = full rebuild
            # pending -> FULL generation.
            owner = getattr(store, '_query_indexes', None)
            if hasattr(owner, 'peek_export_dirty'):
                dirty = owner.peek_export_dirty()
                data = snapshot_query_data(store)
            else:
                # Owners predating the accumulator: compare-tracker
                # fallback (correct though less precise).
                data = snapshot_query_data(store)
                dirty = tracker(previous['snapshot'], data)
            status = hook.offer_committed(
                data, set(dirty) if dirty is not None else None,
                previous['snapshot'] is None
                or tuple(data.identity)
                != tuple(previous['snapshot'].identity))
            # Consume the accumulated set ONLY on acceptance; a
            # backlogged/failed offer keeps the dirt so it ships with
            # the next generation (never a silently empty DELTA).
            if not str(status.get('status', '')).startswith('EXPORT_'):
                if hasattr(owner, 'clear_export_dirty'):
                    owner.clear_export_dirty()
            previous['snapshot'] = data
        except Exception as error:  # noqa: BLE001
            # Export must never break the commit path.
            hook.last_status = {'status': 'EXPORT_HOOK_ERROR',
                                'detail': str(error)}
        return snapshot

    store._build_snapshot = build_with_hook
    return hook


def snapshot_query_data(store):
    with store.acquire_query_snapshot() as handle:
        return handle._data()


def unwire_store_export(store):
    """Remove the hook (test isolation; product keeps it wired for the
    process lifetime in isolated mode)."""
    hook = getattr(store, '_query_export_hook', None)
    if hook is not None:
        hook.close()
        store._query_export_hook = None
    build = getattr(store, '_build_snapshot', None)
    if hasattr(build, '__wrapped_original__'):
        store._build_snapshot = build.__wrapped_original__
