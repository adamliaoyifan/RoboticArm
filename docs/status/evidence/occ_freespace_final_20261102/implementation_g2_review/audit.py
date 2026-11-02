import copy, importlib.util, json, sys
from pathlib import Path
ROOT=Path('/tmp/elfin-occ-freespace-impl-g2')
sys.path.insert(0,str(ROOT/'src/uld_occ_observer/test/benchmark'))
spec=importlib.util.spec_from_file_location('controls',ROOT/'src/uld_occ_observer/test/test_freespace_g2_offline.py')
t=importlib.util.module_from_spec(spec);spec.loader.exec_module(t)
j=t.judge
results={}
def check(name, factory, judge, mutate=None):
 a=factory()
 if mutate: mutate(a)
 results[name]=judge(a)
check('p_control',t.healthy_p_window,j.judge_p_window)
check('r_control',t.healthy_r_artifact,j.judge_r_stage)
check('l_control',t.healthy_l_artifact,j.judge_l_stage)
check('p_all_receipts_same_unrelated_id',t.healthy_p_window,j.judge_p_window,lambda a:[r.update(input_id='unrelated') for r in a['input_timeline']])
check('p_negative_query_time',t.healthy_p_window,j.judge_p_window,lambda a:[r.update(ended_mono_ns=r['planned_mono_ns']-1) for r in a['queries']])
check('r_no_recovery_receipts_or_queries',t.healthy_r_artifact,j.judge_r_stage,lambda a:a.update(recovery_timeline=[],recovery_queries=[],overload_queries=[]))
check('r_second_query_error',t.healthy_r_artifact,j.judge_r_stage,lambda a:a['recovery_queries'].append({'status':'INTERNAL_ERROR'}))
check('l_all_but_first_query_error',t.healthy_l_artifact,j.judge_l_stage,lambda a:a.update(query_records=[a['query_records'][0]]+[{'status':'INTERNAL_ERROR'}]*(len(a['query_records'])-1)))
check('l_drop_10pct',t.healthy_l_artifact,j.judge_l_stage,lambda a:a.update(sent_inputs=16200,query_records=a['query_records'][:12960]))
check('l_all_rss_same_timestamp',t.healthy_l_artifact,j.judge_l_stage,lambda a:[s.update(t_mono_ns=600*10**9) for s in a['rss_series'][1:]])
check('l_missing_10min_bucket',t.healthy_l_artifact,j.judge_l_stage,lambda a:a.update(rss_series=[s for s in a['rss_series'] if not 1800*10**9<=s['t_mono_ns']<2400*10**9]))
print(json.dumps(results,indent=2))
Path(__file__).with_name('RESULTS.json').write_text(json.dumps(results,indent=2))
