"""Join separately measured diagnostic phases without inventing a benchmark."""
import math


def _seconds(value):
    if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
        raise ValueError('missing or invalid measured elapsed time')
    return value


def account_phases(prepare, route, resources, shared_prior_artifact):
    """Inputs must be hash-verified by the caller. Estimates are not samples.

    In particular, cached preparation does not replace cold preparation cost,
    independent process peaks do not establish concurrent memory, and a missing
    planner counter cannot be treated as zero.
    """
    if not prepare.get('revision') or prepare['revision'] != route.get('revision'):
        raise ValueError('phase runtime revisions differ')
    if prepare.get('dirty') != '' or route.get('dirty') != '':
        raise ValueError('phase runtime must be recorded clean')
    left = prepare.get('inputs', {}).get(shared_prior_artifact)
    right = route.get('inputs', {}).get(shared_prior_artifact)
    if not left or left != right:
        raise ValueError('phases do not share the exact captured prior artifact')
    if not prepare.get('success') or not route.get('success'):
        return dict(verdict='not_evaluated', reason='phase did not succeed',
                    prepare_success=prepare.get('success'), route_success=route.get('success'),
                    benchmark_sample=False)
    labels = {}
    for item in prepare.get('results', []):
        if item['label'] in labels:
            raise ValueError('duplicate preparation phase')
        labels[item['label']] = item
    cold, cached = labels['cold'], labels['cached']
    if (not cold['success'] or not cached['success'] or cold['cache_hit']
            or not cached['cache_hit']):
        raise ValueError('cold/cache observations do not match declared phases')
    elapsed = {name:_seconds(item['elapsed_s']) for name,item in
               [('cold',cold),('cached',cached),('route',route)]}
    phase_resources = {}
    for name in elapsed:
        resource = resources[name]
        if resource.get('measurement_complete') is not True:
            raise ValueError('incomplete resource measurement: '+name)
        if resource.get('resource_scope') != 'planner_and_prior_process_trees':
            raise ValueError('incorrect resource scope: '+name)
        if resource.get('prior_process_present') is not (name != 'route'):
            raise ValueError('unexpected prior process accounting: '+name)
        phase_resources[name] = {key:resource[key] for key in (
            'cpu_s','prior_cpu_s','peak_rss_bytes','prior_peak_rss_bytes',
            'measurement_kind','interval_s','limitations')}
        for key in ('cpu_s','prior_cpu_s','peak_rss_bytes','prior_peak_rss_bytes'):
            _seconds(resource[key])
    return dict(verdict='not_evaluated', benchmark_sample=False,
        scope='Separate diagnostic phase measurements; arithmetic estimates only',
        revision=prepare['revision'], shared_prior_sha256=left,
        measured_phase_elapsed_s=elapsed, phase_resources=phase_resources,
        serial_cold_estimate_s=elapsed['cold']+elapsed['route'],
        serial_cached_estimate_s=elapsed['cached']+elapsed['route'],
        missing_for_scoring=['measured concurrent overlap', 'paired baseline',
            'six classified measured workloads and five pairs each',
            'installed/profile/hardware identity across arms',
            'whole-request process-tree resource window', 'collision and IK counters',
            'functional gate evidence'],
        full_request_measured_s=None, concurrent_peak_rss_bytes=None)
