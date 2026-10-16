from copy import deepcopy
import pytest
from luggage_planning.prior_benchmark import BUDGETS, METRICS, analyze, schedule


def fixture():
    manifest = dict(schema='placement-prior-benchmark/1', revision='fixture-revision',
        hardware_id='fixture-machine', profile='isolated-fixture', installed_hash='fixture-install',
        budgets=deepcopy(BUDGETS), repetitions=5,
        cases=[dict(id=f'{snapshot}-{kind}', snapshot_id=snapshot, situation=kind,
                    pickup_orientation_id=snapshot, measured=True, artifacts={'fixture.cdr': 'fixture-hash'})
               for snapshot in ('a', 'b') for kind in ('central_feasible', 'offset_required', 'infeasible')])
    samples = []
    for item in schedule(manifest):
        sample = dict(item, **{k: manifest[k] for k in ('revision', 'hardware_id', 'profile', 'installed_hash', 'budgets')})
        sample.update({metric: 1. for metric in METRICS})
        sample.update(dirty=False, input_artifacts={'fixture.cdr': 'fixture-hash'},
            artifacts={'output.cdr': 'fixture-output'}, resource_scope='planner_and_prior_process_trees',
            status='infeasible' if item['case_id'].endswith('-infeasible') else 'success',
            cache_hit=item['arm']=='prior', fallback=False, prior_exhausted=False,
            route_certified=True, endpoint_ok=True, safety_violations=0,
            exposed_s=10. if item['arm']=='baseline' else 8.)
        samples.append(sample)
    return manifest, samples


def test_complete_pairs_improve_without_hiding_preparation():
    manifest, samples = fixture()
    report = analyze(manifest, samples)
    assert report['verdict'] == 'improved'
    assert report['samples'] == 60 and report['expected_pairs'] == 30
    assert report['cases'][0]['zero_overlap_s']['prior']['median'] == 2.
    assert report['cases'][0]['simulated_overlap_s']['prior']['median'] == 1.
    assert report['cases'][0]['paired_exposed_delta_s'] == [-2.] * 5
    assert [s['arm'] for s in schedule(manifest)[:4]] == ['baseline', 'prior', 'prior', 'baseline']


def test_missing_and_empty_samples_cannot_pass():
    manifest, samples = fixture()
    assert analyze(manifest, [])['verdict'] == 'not_evaluated'
    report = analyze(manifest, samples[:-1])
    assert report['verdict'] == 'not_evaluated' and len(report['missing']) == 1


@pytest.mark.parametrize('key,value', [('dirty', True), ('installed_hash', 'changed'),
    ('cpu_s', None), ('peak_rss_bytes', float('nan')), ('order', 1),
    ('resource_scope', 'client_only'), ('input_artifacts', {}), ('safety_violations', None)])
def test_incomplete_or_mixed_evidence_is_not_evaluated(key, value):
    manifest, samples = fixture()
    samples[0][key] = value
    report = analyze(manifest, samples)
    assert report['verdict'] == 'not_evaluated' and report['problems']


@pytest.mark.parametrize('key,value', [('status', 'timeout'), ('route_certified', False),
    ('endpoint_ok', False), ('safety_violations', 1)])
def test_no_loss_of_feasibility_or_safety(key, value):
    manifest, samples = fixture()
    samples[1][key] = value
    report = analyze(manifest, samples)
    assert report['verdict'] == 'failed'
    assert report['raw_samples'][1][key] == value


def test_cache_miss_and_tail_regression_do_not_claim_warm_gain():
    manifest, samples = fixture()
    samples[1]['cache_hit'] = False
    assert analyze(manifest, samples)['verdict'] == 'no_demonstrated_improvement'
    manifest, samples = fixture()
    samples[1]['exposed_s'] = 30.
    assert analyze(manifest, samples)['verdict'] == 'no_demonstrated_improvement'


def test_duplicate_pair_and_incomplete_matrix_rejected():
    manifest, samples = fixture()
    with pytest.raises(ValueError, match='duplicate'):
        analyze(manifest, samples + samples[:1])
    manifest['cases'][0]['situation'] = 'infeasible'
    with pytest.raises(ValueError, match='each snapshot'):
        analyze(manifest, [])


def test_infeasible_workloads_cannot_be_hidden_as_transport_errors():
    manifest, samples = fixture()
    for sample in samples:
        if sample['case_id'].endswith('-infeasible'):
            sample['status'] = 'error'
    report = analyze(manifest, samples)
    assert report['verdict'] == 'no_demonstrated_improvement'
    assert report['cases'][2]['outcomes']['prior'] == ['error'] * 5


def test_cli_checks_artifact_bytes_and_empty_schedule_is_not_a_result(tmp_path):
    import hashlib
    import json
    import subprocess
    import sys
    from pathlib import Path
    manifest, _ = fixture()
    artifact = tmp_path / 'fixture.cdr'
    artifact.write_bytes(b'unit-test-fixture-only')
    for case in manifest['cases']:
        case['artifacts'] = {artifact.name: hashlib.sha256(artifact.read_bytes()).hexdigest()}
    source = tmp_path / 'manifest.json'
    source.write_text(json.dumps(manifest))
    script = Path(__file__).resolve().parents[3] / 'scripts/placement_prior/benchmark.py'
    output = tmp_path / 'report.json'
    command = [sys.executable, str(script), '--manifest', str(source), '--output', str(output)]
    result = subprocess.run(command, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    report = json.loads(output.read_text())
    assert report['verdict'] == 'not_evaluated' and len(report['schedule']) == 60
    artifact.write_bytes(b'changed')
    command[-1] = str(tmp_path / 'tampered.json')
    result = subprocess.run(command, capture_output=True, text=True)
    assert result.returncode != 0 and 'artifact hash mismatch' in result.stderr
    assert not (tmp_path / 'tampered.json').exists()
