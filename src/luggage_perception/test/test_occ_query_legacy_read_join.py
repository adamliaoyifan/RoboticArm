"""Behavioural coverage for offline legacy callback association."""

import pytest

from occ_query_legacy_read_join import resolve_legacy_volume_revisions


def request(name, begin=0.0, ended=5.0, premise=None):
    return dict(name=name, begin=begin, ended=ended, premise=premise or {"expected_map_revision": 1})


def callback(begin, ended, revision, thread=1, *, read=True, epoch="epoch", premise=None):
    events = [dict(stage="_on_volume_begin", thread=thread, monotonic_ns=int(begin * 1e9))]
    if read:
        events.append(dict(stage="snapshot_read", thread=thread, monotonic_ns=int((begin + ended) * 0.5e9),
                           snapshot_identity=[epoch, revision, "digest-" + str(revision)]))
    events.append(dict(stage="_on_volume_end", thread=thread, monotonic_ns=int(ended * 1e9),
                       volume_premise=premise or {"expected_map_revision": 1},
                       snapshot_identity=[epoch, revision + 1, "later-digest"]))
    return events


def test_global_bijection_resolves_locally_ambiguous_versions():
    requests = [request("first", ended=2), request("second"), request("third")]
    events = callback(1, 1.5, 2) + callback(3, 3.5, 3) + callback(4, 4.5, 3)
    revisions, diagnostic = resolve_legacy_volume_revisions(requests, events)
    assert revisions == dict(first=2, second=3, third=3)
    assert diagnostic["reason"] == "COMPLETE_ASSOCIATION"


def test_true_version_ambiguity_is_unjoinable():
    revisions, diagnostic = resolve_legacy_volume_revisions(
        [request("a"), request("b")], callback(1, 2, 2) + callback(3, 4, 3))
    assert revisions == dict(a=None, b=None)
    assert all(row["reason"] == "AMBIGUOUS_IDENTITY" for row in diagnostic["rows"])


def test_callback_permutations_with_same_identity_are_allowed():
    revisions, _ = resolve_legacy_volume_revisions(
        [request("a"), request("b")], callback(1, 2, 3) + callback(3, 4, 3))
    assert revisions == dict(a=3, b=3)


def test_same_revision_different_epoch_remains_ambiguous():
    revisions, _ = resolve_legacy_volume_revisions(
        [request("a"), request("b")], callback(1, 2, 3) + callback(3, 4, 3, epoch="reset"))
    assert revisions == dict(a=None, b=None)


@pytest.mark.parametrize("events,reason", [
    ([], "CALLBACK_CARDINALITY"),
    (callback(1, 2, 3) + callback(3, 4, 3), "CALLBACK_CARDINALITY"),
    (callback(1, 2, 3)[:-1], "INCOMPLETE_CALLBACK_TRACE"),
    (callback(1, 2, 3)[1:], "INCOMPLETE_CALLBACK_TRACE"),
    (callback(1, 2, 3, premise={"expected_map_revision": 99}), "NO_PERFECT_MATCHING"),
    (callback(6, 7, 3), "NO_PERFECT_MATCHING"),
])
def test_missing_extra_or_unmatched_callback_never_guessed(events, reason):
    revisions, diagnostic = resolve_legacy_volume_revisions([request("a")], events)
    assert revisions == dict(a=None)
    assert diagnostic["reason"] == reason


def test_first_snapshot_not_later_snapshot_or_callback_end():
    events = callback(1, 2, 3)
    events.insert(-1, dict(stage="snapshot_read", thread=1, monotonic_ns=1900000000,
                           snapshot_identity=["epoch", 10, "other"]))
    revisions, _ = resolve_legacy_volume_revisions([request("a")], events)
    assert revisions == dict(a=3)


def test_missing_first_read_does_not_use_callback_end_identity():
    revisions, diagnostic = resolve_legacy_volume_revisions([request("a")], callback(1, 2, 3, read=False))
    assert revisions == dict(a=None)
    assert diagnostic["rows"][0]["reason"] == "MISSING_FIRST_SNAPSHOT"


def test_request_start_must_precede_callback_start():
    revisions, diagnostic = resolve_legacy_volume_revisions([request("a", begin=1.5)], callback(1, 2, 3))
    assert revisions == dict(a=None)
    assert diagnostic["reason"] == "NO_PERFECT_MATCHING"


@pytest.mark.parametrize("limits", [dict(max_work=0), dict(max_requests=0)])
def test_exhausted_work_does_not_return_partial_association(limits):
    revisions, diagnostic = resolve_legacy_volume_revisions([request("a")], callback(1, 2, 3), **limits)
    assert revisions == dict(a=None)
    assert diagnostic["reason"] == "JOIN_WORK_BUDGET"


def test_interleaved_threads_keep_their_own_first_snapshot():
    events = sorted(callback(1, 4, 2, thread=1) + callback(2, 3, 3, thread=2),
                    key=lambda event: event["monotonic_ns"])
    revisions, _ = resolve_legacy_volume_revisions(
        [request("short", begin=1.5, ended=3.5), request("long")], events)
    assert revisions == dict(short=3, long=2)


def test_forced_edge_search_matches_exhaustive_small_graphs():
    # Enumerate every 3x3 temporal window graph through an independent
    # permutation oracle, including failed matchings and alternate identities.
    from itertools import permutations, product

    events = callback(1, 1.5, 1) + callback(2, 2.5, 2) + callback(3, 3.5, 3)
    windows = [(0, 1.8), (0, 2.8), (0, 4), (1.8, 4), (2.8, 4), (4, 5)]
    for bounds in product(windows, repeat=3):
        requests = [request(str(i), begin=b, ended=e) for i, (b, e) in enumerate(bounds)]
        feasible = [assignment for assignment in permutations(range(3)) if all(
            b <= assignment[i] + 1 and assignment[i] + 1.5 <= e
            for i, (b, e) in enumerate(bounds))]
        expected = {}
        for i in range(3):
            possible = {assignment[i] + 1 for assignment in feasible}
            expected[str(i)] = next(iter(possible)) if len(possible) == 1 else None
        observed, _ = resolve_legacy_volume_revisions(requests, events)
        assert observed == expected
