from copy import deepcopy
import pytest
from luggage_planning.placement_prior.prior_phase_accounting import account_phases


def fixture():
    identity = dict(revision='commit',dirty='',inputs={'prior.cdr':'digest'},success=True)
    prepare = dict(identity,results=[dict(label='cold',success=True,cache_hit=False,elapsed_s=8.),
                                    dict(label='cached',success=True,cache_hit=True,elapsed_s=.5)])
    route = dict(deepcopy(identity),elapsed_s=4.)
    resource = dict(measurement_complete=True,resource_scope='planner_and_prior_process_trees',
                    cpu_s=2.,prior_cpu_s=1.,peak_rss_bytes=100,prior_peak_rss_bytes=50,
                    measurement_kind='sampled_proc_stat',interval_s=.05,limitations=['sampled'])
    resources={name:dict(resource,prior_process_present=name!='route') for name in ('cold','cached','route')}
    return prepare,route,resources


def test_cold_cost_not_replaced_by_cache_and_no_invented_overlap():
    result=account_phases(*fixture(),'prior.cdr')
    assert result['serial_cold_estimate_s']==12.
    assert result['serial_cached_estimate_s']==4.5
    assert result['full_request_measured_s'] is None
    assert result['concurrent_peak_rss_bytes'] is None
    assert result['verdict']=='not_evaluated' and not result['benchmark_sample']


@pytest.mark.parametrize('mutation',['revision','hash','dirty','resources','prior_presence','nan'])
def test_incompatible_or_incomplete_measurements_refused(mutation):
    prepare,route,resources=deepcopy(fixture())
    if mutation=='revision':route['revision']='other'
    if mutation=='hash':route['inputs']['prior.cdr']='other'
    if mutation=='dirty':route['dirty']='M product'
    if mutation=='resources':resources['cold']['measurement_complete']=False
    if mutation=='prior_presence':resources['cold']['prior_process_present']=False
    if mutation=='nan':route['elapsed_s']=float('nan')
    with pytest.raises(ValueError):account_phases(prepare,route,resources,'prior.cdr')


def test_failed_route_is_not_a_latency_success():
    prepare,route,resources=fixture();route['success']=False
    result=account_phases(prepare,route,resources,'prior.cdr')
    assert result['verdict']=='not_evaluated'
    assert 'serial_cold_estimate_s' not in result
