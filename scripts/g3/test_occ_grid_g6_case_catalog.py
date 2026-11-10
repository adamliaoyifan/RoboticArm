"""Synthetic positive/negative controls for the gen6 catalog builder.

The real build already ran against the historical G2/gen5 sources; these
controls pin the abort behavior so a future rebuild cannot silently absorb a
count or dimension drift.
"""
import pytest

import occ_grid_g6_case_catalog as builder


def _group(resolution=0.01, boxes=0, shape='cuboid', cases=('summary', 'surface'),
           not_applicable=()):
    instrumentation = {case: {'outcome': 'pass'} for case in cases}
    for case in not_applicable:
        instrumentation[case] = {'outcome': 'not_applicable'}
    return dict(resolution=resolution, boxes=boxes, shape=shape,
                instrumentation=instrumentation)


APPLICABLE = ('distance_1', 'distance_256', 'frontier', 'frontier_count', 'instance',
              'instance_empty', 'patch', 'roi_dense', 'roi_empty', 'summary', 'surface')


def test_capture_rejects_when_not_applicable_is_not_excluded():
    manifest = dict(groups=[_group(r, b, s, APPLICABLE, ('stats', 'snapshot'))
                            for r in (.01, .02, .05) for b in (0, 5, 20)
                            for s in ('cuboid', 'chamfer')])
    ids, classes, excluded = builder.capture_198(manifest)
    assert len(ids) == 198 and classes == list(APPLICABLE)
    assert excluded == ['snapshot', 'stats']
    # Promoting an excluded class must drift the count and abort the build.
    manifest['groups'][0]['instrumentation']['stats'] = {'outcome': 'pass'}
    with pytest.raises(RuntimeError):
        builder.capture_198(manifest)


def test_capture_rejects_duplicate_case_keys():
    groups = [_group(r, b, s, ('summary',)) for r in (.01, .02, .05)
              for b in (0, 5, 20) for s in ('cuboid', 'chamfer')]
    groups.append(groups[0])  # duplicate group
    with pytest.raises(RuntimeError):
        builder.capture_198(dict(groups=groups))


def test_response_rejects_wrong_rows_per_group():
    rows = [{'name': f'{i:03d}.response.cdr', 'baseline_sha256': 'x',
             'candidate_sha256': 'x', 'different_fields': [],
             'semantic_equal': True, 'wire_equal': True} for i in range(33)]
    manifest = dict(groups=[dict(resolution=.01, boxes=0, path='hit',
                                 response_comparison=dict(rows=rows))])
    with pytest.raises(RuntimeError):
        builder.response_612(manifest)


def test_oracle_rejects_dimension_drift(tmp_path, monkeypatch):
    monkeypatch.setattr(builder, 'EXPECTED_ORACLE_ROWS', 1)
    om10 = dict(inputs=dict(seeds=1, transactions=1, resolutions=[.01],
                            boxes=[0], shapes=['cuboid']), expected_rows=1,
                fixture_sha256='f'*64, exact_match=True, input_journals_equal=True,
                baseline_rows_sha256='a', candidate_rows_sha256='b', workers=[])
    journal = tmp_path/'inputs.jsonl'
    journal.write_text('{"seed": 0, "transaction": 0, "resolution": 0.02, '
                       '"initial_boxes": 0, "shape": "cuboid"}\n')
    with pytest.raises(RuntimeError, match='dimension drift'):
        builder.oracle_180000(om10, journal, tmp_path/'keys.jsonl')


def test_generator_rejects_gen5_mismatch():
    gen5 = dict(writes_72=dict(ids=['r0.01_b0_p30000_o0.5_c0.0']),
                placement_18=dict(ids=[]))
    with pytest.raises(RuntimeError, match='writes_72'):
        builder.generator_ids(gen5)
