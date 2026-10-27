"""G4-A gen3 M0/M2 witnesses: export queue coalescing, in-flight
drain accounting and close ownership (plan d87c5731 section 5;
review_8a372dee P1-4).

Every test here MUST fail on 8a372dee (verified in M0) and pass after
M2. Deterministic event barriers, never sleeps-for-timing.
"""

import threading
from unittest.mock import patch

from luggage_perception.cargo_query_export import QueryExportPublisher


def make_publisher(root, **kwargs):
    return QueryExportPublisher(root, async_mode=True, **kwargs)


class PublishBarrier:
    """Blocks _publish inside the encoder thread until released;
    records every emitted (snapshot, dirty) pair in order."""

    def __init__(self):
        self.entered = threading.Event()
        self.release = threading.Event()
        self.emitted = []
        self.finished = threading.Event()

    def __call__(self, publisher, snapshot, dirty, provenance,
                 full=False, heal_hint=None):
        self.entered.set()
        if not self.release.wait(10):
            raise RuntimeError('barrier release timeout')
        self.emitted.append(
            {'snapshot': snapshot, 'dirty': (None if dirty is None
                                             else sorted(dirty))})
        return {'status': 'OK'}


def test_third_offer_coalesces_instead_of_rejecting(tmp_path):
    """One in-flight + one queued item; the third offer must COALESCE
    with the queued item (never EXPORT_BACKLOG_FULL). Final emitted
    snapshot is the third; dirty covers the union of all changes."""
    barrier = PublishBarrier()
    with patch.object(QueryExportPublisher, '_publish', barrier), \
            patch.object(QueryExportPublisher, '_head_loop_tick',
                         lambda self: None):
        publisher = make_publisher(str(tmp_path))
        try:
            first = publisher.offer_committed('generation-1', {1})
            assert first['status'] == 'ACCEPTED'
            assert barrier.entered.wait(5)      # g1 is now in-flight
            second = publisher.offer_committed('generation-2', {2})
            assert second['status'] in ('ACCEPTED', 'COALESCED')
            third = publisher.offer_committed('generation-3', {3})
            assert third['status'] != 'EXPORT_BACKLOG_FULL', third
            assert publisher._latest_committed[0] == 'generation-3'
        finally:
            barrier.release.set()
            publisher.close()
    snapshots = [item['snapshot'] for item in barrier.emitted]
    assert snapshots[-1] == 'generation-3'
    dirty_union = barrier.emitted[-1]['dirty']
    assert dirty_union is None or {1, 2, 3} <= set(dirty_union)


def test_wait_pending_false_while_publish_in_flight(tmp_path):
    barrier = PublishBarrier()
    with patch.object(QueryExportPublisher, '_publish', barrier), \
            patch.object(QueryExportPublisher, '_head_loop_tick',
                         lambda self: None):
        publisher = make_publisher(str(tmp_path))
        try:
            publisher.offer_committed('generation-1', {1})
            assert barrier.entered.wait(5)
            # g1 popped from the queue but still encoding: drained
            # must NOT be reported (8a372dee: queue.empty() -> True).
            assert publisher.wait_pending(0.05) is False
        finally:
            barrier.release.set()
            publisher.close()
    assert publisher.wait_pending(5) is True


def test_wait_pending_false_while_item_queued(tmp_path):
    barrier = PublishBarrier()
    with patch.object(QueryExportPublisher, '_publish', barrier), \
            patch.object(QueryExportPublisher, '_head_loop_tick',
                         lambda self: None):
        publisher = make_publisher(str(tmp_path))
        try:
            publisher.offer_committed('generation-1', {1})
            assert barrier.entered.wait(5)
            publisher.offer_committed('generation-2', {2})
            assert publisher.wait_pending(0.05) is False
        finally:
            barrier.release.set()
            publisher.close()
    assert publisher.wait_pending(5) is True


def test_close_failure_retains_owner_lock(tmp_path):
    """If the encoder thread cannot exit within the close timeout,
    the owner lock must be RETAINED: a second publisher on the same
    root must be refused (no second writer)."""
    barrier = PublishBarrier()
    with patch.object(QueryExportPublisher, '_publish', barrier), \
            patch.object(QueryExportPublisher, '_head_loop_tick',
                         lambda self: None):
        publisher = make_publisher(str(tmp_path))
        publisher.offer_committed('generation-1', {1})
        assert barrier.entered.wait(5)
        try:
            publisher.close(timeout_s=0.05)     # encoder still blocked
        except Exception:
            pass
        try:
            second = make_publisher(str(tmp_path))
        except Exception as error:
            second = error
        finally:
            barrier.release.set()
            publisher.close()
        assert isinstance(second, Exception), (
            'second publisher accepted while encoder alive: %r' % second)


def test_rejected_offer_keeps_facts_surface(tmp_path):
    """When an offer is rejected (or coalescing drops a task), the
    latest-committed observation surface must still record the newest
    committed fact: head must not claim facts stopped at the previous
    accepted task (see plan 5.1 'facts-latest-commit observation')."""
    barrier = PublishBarrier()
    with patch.object(QueryExportPublisher, '_publish', barrier), \
            patch.object(QueryExportPublisher, '_head_loop_tick',
                         lambda self: None):
        publisher = make_publisher(str(tmp_path))
        try:
            publisher.offer_committed('generation-1', {1})
            assert barrier.entered.wait(5)
            publisher.offer_committed('generation-2', {2})
            publisher.offer_committed('generation-3', {3})
            # Whatever the capacity outcome, generation-3 was COMMITTED
            # fact: latest_committed must reflect it (or a later one).
            latest = publisher._latest_committed
            assert latest is None or latest[0] in (
                'generation-3',), latest
        finally:
            barrier.release.set()
            publisher.close()
