"""G4-A gen3 A9 pairing judge behavior tests (review_d9a28e4e P1 +
reviews follow-up on ed44accb): the reducer must have real
discriminating power, and the capture-completeness judge is the REAL
one the CLI calls (capture_join_complete / identical_recorder_off
imported from the script -- no re-written mirror). Join violations
(missing / duplicated / unknown-id / wrong-identity records, wrong
ids with non-empty wrong identities, duplicated ON samples) FAIL,
never pass.
"""

import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))),
    'scripts', 'g3'))

from occ_a9_capture_pairing_v2 import (  # noqa: E402
    capture_join_complete, identical_recorder_off, nearest_rank_p95,
    pair_schedule, position_order_effect)


def reducer(off_times, on_times, ratio_bound=1.10, min_samples=30):
    """The pairing verdict as a pure function (mirrors main())."""
    off_p95 = nearest_rank_p95(off_times, min_samples)
    on_p95 = nearest_rank_p95(on_times, min_samples)
    if off_p95 is None or on_p95 is None:
        return 4, None
    ratio = round(on_p95 / off_p95, 4)
    return (3 if ratio > ratio_bound else 0), ratio


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


def a9_kinds(pairs=30):
    """samples_by_kind shaped like the v4.1 worker output: per kind
    the full pair-schedule samples with the request identity stamped
    on every sample (identity = [pair, position])."""
    kinds = {}
    for kind in ('roi', 'distance', 'surface'):
        samples = samples_from_schedule(
            pair_schedule(pairs), lambda a, p, i: 1.0)
        for sample in samples:
            sample['identity'] = [sample['pair'], sample['position']]
        kinds[kind] = samples
    return kinds


def on_samples(kinds, kind):
    return [s for s in kinds[kind]
            if s['arm'] == 'on' and not s['warmup']]


def clean_records(kinds):
    """The records a correct v4.1 worker would emit: one per ON
    measurement request, identity taken from that request's sample."""
    return [{'request_id': '%s-p%d' % (kind, s['pair']),
             'identity': list(s['identity'])}
            for kind in ('roi', 'distance', 'surface')
            for s in on_samples(kinds, kind)]


def test_join_clean_passes_real_judge():
    kinds = a9_kinds()
    detail = capture_join_complete(kinds, clean_records(kinds), 30)
    assert detail['complete'] is True
    assert detail['joined'] == 90 and detail['expected'] == 90
    assert detail['on_measurement_samples'] == 90


def test_join_wrong_ids_with_nonempty_identities_fails():
    # THE reviews probe on v4: 90 records, ids unique and of the right
    # shape but assigned to the WRONG requests, each carrying a
    # non-empty (but wrong) identity -- v4's unique/non-empty/count
    # check passed this; the real join must FAIL it.
    kinds = a9_kinds()
    shifted = []
    for kind in ('roi', 'distance', 'surface'):
        for sample in on_samples(kinds, kind):
            shifted.append({
                'request_id': '%s-p%d' % (kind, (sample['pair'] + 1) % 30),
                'identity': list(sample['identity'])})
    assert len(shifted) == 90
    assert len({r['request_id'] for r in shifted}) == 90
    assert all(r['identity'] for r in shifted)
    detail = capture_join_complete(kinds, shifted, 30)
    assert detail['complete'] is False
    assert detail['identity_mismatch']


def test_join_wrong_identity_nonempty_fails():
    kinds = a9_kinds()
    records = clean_records(kinds)
    for record in records:
        record['identity'] = [999, 999]        # non-empty but wrong
    detail = capture_join_complete(kinds, records, 30)
    assert detail['complete'] is False
    assert detail['identity_mismatch']


def test_join_duplicate_id_fails():
    kinds = a9_kinds()
    records = clean_records(kinds)
    records.append(dict(records[0]))           # duplicate request_id
    detail = capture_join_complete(kinds, records, 30)
    assert detail['complete'] is False
    assert detail['duplicate_ids']


def test_join_missing_one_fails():
    kinds = a9_kinds()
    records = clean_records(kinds)[:-1]
    detail = capture_join_complete(kinds, records, 30)
    assert detail['complete'] is False
    assert detail['joined'] == 89 and detail['missing']


def test_join_unknown_id_fails():
    kinds = a9_kinds()
    records = clean_records(kinds)
    records.append({'request_id': 'roi2-p0', 'identity': [1, 1]})
    detail = capture_join_complete(kinds, records, 30)
    assert detail['complete'] is False
    assert detail['unmatched_ids'] == ['roi2-p0']


def test_join_missing_identity_fails():
    kinds = a9_kinds()
    records = clean_records(kinds)
    records[3]['identity'] = None
    detail = capture_join_complete(kinds, records, 30)
    assert detail['complete'] is False
    assert detail['missing_identity']


def test_join_duplicate_on_sample_fails():
    # harness structural error: two ON measurement samples for the
    # same (kind, pair) -- even clean records must not pass
    kinds = a9_kinds()
    records = clean_records(kinds)
    extra = dict(on_samples(kinds, 'roi')[0])
    kinds['roi'].append(extra)
    detail = capture_join_complete(kinds, records, 30)
    assert detail['complete'] is False
    assert detail['duplicate_on_samples']


def test_identical_recorder_off_contract():
    assert identical_recorder_off([])['recorder_fully_off'] is True
    detail = identical_recorder_off(
        [{'request_id': 'roi-p0', 'identity': [1, 1]}])
    assert detail['recorder_fully_off'] is False
    assert detail['records'] == 1


def samples_from_schedule(schedule, value_for, warmup_pairs=2):
    """Materialize per-sample records the way the v4 worker does."""
    out = []
    for pair_index, order in enumerate(schedule):
        arms = ('on', 'off') if order == 'AB' else ('off', 'on')
        warmup = pair_index < warmup_pairs
        for position, arm in enumerate(arms, start=1):
            out.append({'pair': pair_index - warmup_pairs,
                        'order': order, 'arm': arm,
                        'position': position, 'warmup': warmup,
                        'full_ms': value_for(arm, position, pair_index),
                        'direct_ms': 0.0})
    return out


def test_pair_schedule_is_balanced_and_prefixed():
    schedule = pair_schedule(30)
    assert schedule[:2] == ['AB', 'BA']        # warmup opens AB, BA
    assert len(schedule) == 32
    measurement = schedule[2:]
    assert measurement.count('AB') == 15
    assert measurement.count('BA') == 15
    # alternating, no adaptation
    assert all(a != b for a, b in zip(measurement, measurement[1:]))
    # each arm lands in each position exactly 15 times
    samples = samples_from_schedule(schedule, lambda a, p, i: 1.0)
    for arm in ('on', 'off'):
        counts = {1: 0, 2: 0}
        for s in samples:
            if s['arm'] == arm and not s['warmup']:
                counts[s['position']] += 1
        assert counts == {1: 15, 2: 15}


def test_position_order_effect_measures_position_delta():
    # position 2 samples cost exactly 0.2ms more: the decomposition
    # must read the effect off the balanced schedule
    samples = samples_from_schedule(
        pair_schedule(30), lambda a, p, i: 1.0 + (0.2 if p == 2 else 0.0))
    effect = position_order_effect(samples, 'on')
    assert effect['n_by_position'] == {'1': 15, '2': 15}
    assert effect['mean_by_position_ms'] == {'1': 1.0, '2': 1.2}
    assert effect['position_effect_ms'] == 0.2
    # for the ON arm order and position coincide (AB puts ON first):
    # order means differ by exactly the position effect; the mirror
    # arm (off) shows the reversed pattern -- so a pure position bias
    # moves BOTH arms equally and cancels in the on/off ratio
    assert effect['mean_by_order_ms'] == {'AB': 1.0, 'BA': 1.2}
    off_effect = position_order_effect(samples, 'off')
    assert off_effect['mean_by_order_ms'] == {'AB': 1.2, 'BA': 1.0}
    assert off_effect['position_effect_ms'] == 0.2


def test_position_order_effect_excludes_warmup_and_arm():
    samples = samples_from_schedule(
        pair_schedule(30), lambda a, p, i: 2.0)
    # warmup pairs would inflate counts if not excluded
    effect = position_order_effect(samples, 'off')
    assert effect['n_by_position'] == {'1': 15, '2': 15}
    assert effect['position_effect_ms'] == 0.0
    # a foreign arm label contributes nothing
    assert position_order_effect(
        samples, 'other')['n_by_position'] == {'1': 0, '2': 0}


def test_position_order_effect_missing_position_is_none():
    samples = [{'arm': 'on', 'position': 1, 'order': 'AB',
                'warmup': False, 'full_ms': 1.0}] * 10
    effect = position_order_effect(samples, 'on')
    assert effect['position_effect_ms'] is None
    assert effect['mean_by_position_ms']['2'] is None
