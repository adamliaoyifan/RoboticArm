"""Behavior controls for the gen7 2cm era plumbing (plan 9b7d465b section 3).

Covers: the era resolution sets, the formal oracle gate refusing 1cm in the
g7_2cm matrix, CLI surface, numeric catalog filtering with abort controls,
excluded items marked out_of_scope (never PASS), and gen6 outputs untouched.
"""
import argparse
import json
import subprocess
import sys
from pathlib import Path

import pytest

import occ_query_eval as runner
import occ_grid_g7_catalog as g7

W = Path(__file__).resolve().parents[2]


def _oracle_args(**over):
    base = dict(rehearsal=False, seed_start=0, seeds=20, transactions=500,
                distance_points=1000, resolutions=[.02, .05], boxes=[0, 5, 20],
                shapes=['cuboid', 'chamfer'], matrix_era='g7_2cm')
    base.update(over)
    return argparse.Namespace(**base)


def test_era_sets_and_unknown():
    assert runner.formal_resolution_set('g6_full') == {.01, .02, .05}
    assert runner.formal_resolution_set('g7_2cm') == {.02, .05}
    assert runner.formal_representative_resolution('g7_2cm') == .02
    with pytest.raises(RuntimeError):
        runner.formal_resolution_set('g8')


def test_g7_oracle_formal_accepts_2_5_only():
    assert runner.oracle_formal(_oracle_args()) is True
    # 1cm must not enter the g7_2cm matrix.
    assert runner.oracle_formal(_oracle_args(resolutions=[.01, .02, .05])) is False
    # The g6 era still demands the full 1/2/5cm set.
    assert runner.oracle_formal(_oracle_args(matrix_era='g6_full')) is False
    assert runner.oracle_formal(
        _oracle_args(matrix_era='g6_full', resolutions=[.01, .02, .05])) is True
    assert runner.oracle_formal(_oracle_args(rehearsal=True)) is False


def test_cli_surface():
    for sub in ('oracle', 'benchmark', 'planning', 'concurrency'):
        result = subprocess.run(
            [sys.executable, str(W/'scripts/g3/occ_query_eval.py'), sub, '--help'],
            cwd='/tmp/g6_neutral', capture_output=True, text=True, timeout=60)
        assert '--matrix-era' in result.stdout, sub
        assert '--evidence-root' in result.stdout, sub
    result = subprocess.run(
        [sys.executable, str(W/'scripts/g3/occ_query_eval.py'), 'concurrency', '--help'],
        cwd='/tmp/g6_neutral', capture_output=True, text=True, timeout=60)
    assert '--resolution' in result.stdout
    # concurrency without --resolution is refused (no implicit 1cm default)
    result = subprocess.run(
        [sys.executable, str(W/'scripts/g3/occ_query_eval.py'), 'concurrency',
         '--out', '/tmp/never'],
        cwd='/tmp/g6_neutral', capture_output=True, text=True, timeout=60)
    assert result.returncode != 0 and '--resolution' in result.stderr


def test_resolution_of_numeric_not_string():
    assert g7.resolution_of('r0.01_b0_cuboid_summary') == .01
    assert g7.resolution_of('r0.05_b20_chamfer_surface') == .05
    with pytest.raises(RuntimeError):
        g7.resolution_of('boxes10_r1_x')      # no leading r<resolution>
    with pytest.raises(RuntimeError):
        g7.split_ids(['r0.03_b0_x'], 'weird')  # outside the contract entirely


def test_split_ids_excludes_only_1cm():
    ids = ['r0.01_b0_cuboid_summary', 'r0.02_b0_cuboid_summary',
           'r0.05_b20_chamfer_surface']
    kept, excluded = g7.split_ids(ids, 'synthetic')
    assert kept == ['r0.02_b0_cuboid_summary', 'r0.05_b20_chamfer_surface']
    assert excluded == ['r0.01_b0_cuboid_summary']


def test_check_aborts_on_wrong_counts_and_duplicates():
    with pytest.raises(RuntimeError):
        g7.check('capture', ['a']*10, [], 'capture_kept', 'capture_excluded')
    with pytest.raises(RuntimeError):
        g7.check('capture', ['r0.02_a']*132, [], 'capture_kept', 'capture_excluded')


def _run_g7_catalog(tmp_path, g6_catalog, keys_in, mutate=None):
    catalog = json.loads(Path(g6_catalog).read_text())
    if mutate:
        mutate(catalog)
    mutated = tmp_path/'g6_mutated.json'
    mutated.write_text(json.dumps(catalog))
    out = tmp_path/'CASE_G7.json'
    scope = tmp_path/'SCOPE.json'
    result = subprocess.run(
        [sys.executable, str(W/'scripts/g3/occ_grid_g7_catalog.py'),
         '--g6-catalog', str(mutated), '--oracle-keys-in', str(keys_in),
         '--out-catalog', str(out), '--oracle-keys-out', str(tmp_path/'keys.jsonl'),
         '--scope-out', str(scope)],
        cwd='/tmp/g6_neutral', capture_output=True, text=True, timeout=300)
    return result, out, scope


G6_ROOT = Path('/home/adamliao/work/elfin_humble_ws/docs/status/evidence/'
               'occ_owner_b1/occ_grid_acceptance_g6_20261030')


def test_g7_catalog_real_build_and_g6_untouched(tmp_path):
    before = sorted(str(p) for p in G6_ROOT.rglob('*'))
    result, out, scope = _run_g7_catalog(tmp_path, G6_ROOT/'CASE_CATALOG.json',
                                         G6_ROOT/'oracle_180000_row_keys.jsonl')
    assert result.returncode == 0, result.stderr[-400:]
    catalog = json.loads(out.read_text())
    assert catalog['capture_132']['count'] == 132
    assert catalog['response_408']['count'] == 408
    assert catalog['writes_48']['count'] == 48
    assert catalog['placement_12']['count'] == 12
    assert catalog['oracle_120000']['row_count'] == 120000
    kept_ids = (catalog['capture_132']['ids'] + catalog['response_408']['ids']
                + catalog['writes_48']['ids'] + catalog['placement_12']['ids'])
    assert all(not i.startswith('r0.01_') for i in kept_ids)
    scope_data = json.loads(scope.read_text())
    assert scope_data['oracle_row_counts_by_resolution']['0.01'] == 60000
    assert all(section['disposition'] == 'out_of_scope_by_user_resolution_change'
               for section in scope_data['sections'].values())
    after = sorted(str(p) for p in G6_ROOT.rglob('*'))
    assert after == before


def test_g7_catalog_aborts_on_duplicate_injection(tmp_path):
    def inject(catalog):
        catalog['placement_18']['ids'] = (
            catalog['placement_18']['ids'][:-1] + [catalog['placement_18']['ids'][0]])
    result, _, _ = _run_g7_catalog(tmp_path, G6_ROOT/'CASE_CATALOG.json',
                                   G6_ROOT/'oracle_180000_row_keys.jsonl',
                                   mutate=inject)
    assert result.returncode != 0
    assert 'duplicate' in result.stderr or 'kept' in result.stderr
