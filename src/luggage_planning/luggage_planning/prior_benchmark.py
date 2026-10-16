"""Strict paired benchmark accounting; no ROS, motion or inferred measurements."""
import math
import statistics

SITUATIONS = {'central_feasible', 'offset_required', 'infeasible'}
BUDGETS = {'select_s': 60, 'certification_s': 2, 'client_watchdog_s': 5,
           'prior_s': 10, 'prior_candidates': 32}
METRICS = ('exposed_s', 'full_request_s', 'prior_wall_s', 'cpu_s', 'peak_rss_bytes',
           'prior_cpu_s', 'prior_peak_rss_bytes', 'certification_s',
           'collision_calls', 'ik_calls', 'service_calls', 'overlap_available_s')


def validate_manifest(manifest):
    if manifest.get('schema') != 'placement-prior-benchmark/1':
        raise ValueError('unsupported benchmark schema')
    for key in ('revision', 'hardware_id', 'profile', 'installed_hash'):
        if not isinstance(manifest.get(key), str) or not manifest[key]:
            raise ValueError('missing identity: ' + key)
    if manifest.get('budgets') != BUDGETS or manifest.get('repetitions') != 5:
        raise ValueError('benchmark requires pinned budgets and five pairs per workload')
    cases = manifest.get('cases', [])
    if len(cases) != 6 or len({c['id'] for c in cases}) != 6:
        raise ValueError('six distinct measured workloads required')
    snapshots = {c['snapshot_id'] for c in cases}
    if len(snapshots) != 2:
        raise ValueError('two snapshot identities required')
    if {(c['snapshot_id'], c['situation']) for c in cases} != {
            (s, k) for s in snapshots for k in SITUATIONS}:
        raise ValueError('each snapshot requires central, offset and infeasible workloads')
    if len({c['pickup_orientation_id'] for c in cases}) < 2:
        raise ValueError('at least two captured pickup orientations required')
    for case in cases:
        if case.get('measured') is not True or not case.get('artifacts'):
            raise ValueError('measured source and frozen input artifacts required')
    return {c['id']: c for c in cases}


def schedule(manifest):
    validate_manifest(manifest)
    return [dict(case_id=c['id'], repetition=i, arm=arm, order=order)
            for c in manifest['cases'] for i in range(5)
            for order, arm in enumerate(('baseline', 'prior') if i % 2 == 0 else ('prior', 'baseline'))]


def percentile95(values):
    # Linear interpolation, explicitly fixed for reproducible small samples.
    values = sorted(values)
    index = .95 * (len(values) - 1)
    low = int(index)
    return values[low] + (values[math.ceil(index)] - values[low]) * (index - low)


def summary(values):
    return dict(n=len(values), median=statistics.median(values), p95=percentile95(values))


def analyze(manifest, samples):
    cases = validate_manifest(manifest)
    expected = {(r['case_id'], r['repetition'], r['arm']): r for r in schedule(manifest)}
    observed, problems = {}, []
    for sample in samples:
        key = (sample.get('case_id'), sample.get('repetition'), sample.get('arm'))
        if key not in expected or key in observed:
            raise ValueError('unexpected or duplicate paired sample: ' + repr(key))
        observed[key] = sample
        for field in ('revision', 'hardware_id', 'profile', 'installed_hash', 'budgets'):
            if sample.get(field) != manifest[field]:
                problems.append(dict(sample=key, reason='identity mismatch: ' + field))
        if sample.get('dirty') is not False or sample.get('order') != expected[key]['order']:
            problems.append(dict(sample=key, reason='dirty runtime or incorrect arm order'))
        if sample.get('input_artifacts') != cases[key[0]]['artifacts']:
            problems.append(dict(sample=key, reason='workload hashes differ'))
        if sample.get('status') not in ('success', 'infeasible', 'timeout', 'error'):
            problems.append(dict(sample=key, reason='missing terminal outcome'))
        if not sample.get('artifacts') or sample.get('resource_scope') != 'planner_and_prior_process_trees':
            problems.append(dict(sample=key, reason='missing outcome artifacts or complete resource scope'))
        for field in METRICS:
            value = sample.get(field)
            if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
                problems.append(dict(sample=key, reason='missing/invalid metric: ' + field))
        for field in ('cache_hit', 'fallback', 'prior_exhausted', 'route_certified', 'endpoint_ok'):
            if type(sample.get(field)) is not bool:
                problems.append(dict(sample=key, reason='missing boolean: ' + field))
        if type(sample.get('safety_violations')) is not int or sample['safety_violations'] < 0:
            problems.append(dict(sample=key, reason='missing safety outcome'))
    missing = sorted(set(expected) - set(observed))
    report = dict(verdict='not_evaluated', expected_pairs=30, samples=len(samples),
                  missing=missing, problems=problems, raw_samples=samples, cases=[])
    if problems:
        return report
    regressions, latency_pass, pooled = [], [], {'baseline': [], 'prior': []}
    for case_id, case in cases.items():
        arms = {arm: [observed[(case_id, i, arm)] for i in range(5)
                      if (case_id, i, arm) in observed] for arm in ('baseline', 'prior')}
        row = dict(case_id=case_id, situation=case['situation'], counts={a: len(v) for a, v in arms.items()},
                   verdict='not_evaluated', outcomes={a: [s['status'] for s in v] for a, v in arms.items()})
        report['cases'].append(row)
        if any(len(v) < 5 for v in arms.values()):
            continue
        row['metrics'] = {a: {metric: summary([s[metric] for s in values]) for metric in METRICS}
                          for a, values in arms.items()}
        row['zero_overlap_s'] = {a: summary([s['full_request_s'] + s['prior_wall_s'] for s in v])
                                for a, v in arms.items()}
        row['simulated_overlap_s'] = {a: summary([s['full_request_s'] + max(
            0, s['prior_wall_s'] - s['overlap_available_s']) for s in v]) for a, v in arms.items()}
        row['paired_exposed_delta_s'] = [p['exposed_s'] - b['exposed_s']
                                        for b, p in zip(arms['baseline'], arms['prior'])]
        for arm, values in arms.items():
            for s in values:
                if s['safety_violations'] or (s['status'] == 'success' and
                        not (s['route_certified'] and s['endpoint_ok'])):
                    regressions.append((case_id, s['repetition'], arm, 'safety/certification'))
        for b, p in zip(arms['baseline'], arms['prior']):
            if b['status'] == 'success' and p['status'] != 'success':
                regressions.append((case_id, p['repetition'], 'prior', 'lost feasible case'))
        if case['situation'] == 'infeasible':
            if all(s['status'] == 'infeasible' for values in arms.values() for s in values):
                row['verdict'] = 'reported_infeasible_workload'
            else:
                row['verdict'] = 'infeasible_classification_not_established'
                latency_pass.append(False)
            if any(s['status'] == 'success' for values in arms.values() for s in values):
                regressions.append((case_id, None, None, 'infeasible workload classification contradicted'))
            continue
        warm = all(s['status'] == 'success' for v in arms.values() for s in v) and all(
            s['cache_hit'] and not s['fallback'] and not s['prior_exhausted'] for s in arms['prior'])
        if not warm:
            row['verdict'] = 'no_demonstrated_improvement'
            latency_pass.append(False)
            continue
        baseline = [s['exposed_s'] for s in arms['baseline']]
        prior = [s['exposed_s'] for s in arms['prior']]
        passed = (statistics.median(baseline) > 0 and
                  statistics.median(prior) <= .9 * statistics.median(baseline) and
                  percentile95(prior) <= 1.1 * percentile95(baseline))
        row['verdict'] = 'improved' if passed else 'no_demonstrated_improvement'
        latency_pass.append(passed)
        pooled['baseline'].extend(baseline)
        pooled['prior'].extend(prior)
    report['regressions'] = regressions
    report['pooled_warm_exposed_s'] = {a: summary(v) for a, v in pooled.items() if v}
    if not missing:
        report['verdict'] = ('failed' if regressions else 'improved' if len(latency_pass) == 4
                             and all(latency_pass) else 'no_demonstrated_improvement')
    return report
