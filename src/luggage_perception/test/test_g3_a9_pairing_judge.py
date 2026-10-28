"""G4-A gen3 A9 pairing judge behavior tests (review_d9a28e4e P1):
the reducer must have real discriminating power -- synthetic
distributions produce the true ratio, and per-request join violations
(missing / duplicated / wrong-identity records) FAIL, never pass.
"""

import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))),
    'scripts', 'g3'))

from occ_a9_capture_pairing_v2 import nearest_rank_p95  # noqa: E402


def reducer(off_times, on_times, ratio_bound=1.10, min_samples=30):
    """The pairing verdict as a pure function (mirrors main())."""
    off_p95 = nearest_rank_p95(off_times, min_samples)
    on_p95 = nearest_rank_p95(on_times, min_samples)
    if off_p95 is None or on_p95 is None:
        return 4, None
    ratio = round(on_p95 / off_p95, 4)
    return (3 if ratio > ratio_bound else 0), ratio


def join_check(records, expected_count):
    """Per-request completeness as a pure function: unique ids, every
    record carries an identity, exact count."""
    ids = [r['request_id'] for r in records]
    if len(ids) != len(set(ids)):
        return False, 'duplicate ids'
    if any(not r.get('identity') for r in records):
        return False, 'missing identity'
    if len(records) != expected_count:
        return False, 'count %d != %d' % (len(records), expected_count)
    return True, 'ok'


def times(value, count=30):
    return [value] * count


def test_reducer_true_ratio_2x():
    code, ratio = reducer(times(1.0), times(2.0))
    assert code == 3 and ratio == 2.0


def test_reducer_equal_passes():
    code, ratio = reducer(times(5.0), times(5.0))
    assert code == 0 and ratio == 1.0


def test_reducer_ten_percent_boundary():
    code, ratio = reducer(times(10.0), times(11.0))
    assert code == 0 and ratio == 1.1


def test_reducer_just_over_fails():
    code, ratio = reducer(times(10.0), times(11.01))
    assert code == 3


def test_reducer_insufficient_samples_not_evaluated():
    code, ratio = reducer(times(1.0)[:29], times(2.0)[:29])
    assert code == 4 and ratio is None


def test_join_duplicate_id_fails():
    records = ([{'request_id': 'r%d' % i, 'identity': [1]}
                for i in range(29)]
               + [{'request_id': 'r0', 'identity': [1]}])
    ok, why = join_check(records, 30)
    assert not ok and why == 'duplicate ids'


def test_join_missing_one_fails():
    records = [{'request_id': 'r%d' % i, 'identity': [1]}
               for i in range(29)]
    ok, why = join_check(records, 30)
    assert not ok


def test_join_wrong_count_extra_records_fail():
    records = [{'request_id': 'r%d' % i, 'identity': [1]}
               for i in range(31)]
    ok, why = join_check(records, 30)
    assert not ok


def test_join_missing_identity_fails():
    records = [{'request_id': 'r%d' % i, 'identity': [1]}
               for i in range(29)]
    records.append({'request_id': 'r29', 'identity': None})
    ok, why = join_check(records, 30)
    assert not ok and why == 'missing identity'


def test_join_clean_passes():
    records = [{'request_id': 'r%d' % i, 'identity': [1, 2]}
               for i in range(30)]
    ok, why = join_check(records, 30)
    assert ok
