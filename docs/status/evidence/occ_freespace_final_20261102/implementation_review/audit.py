import json, sys
from pathlib import Path
sys.path.insert(0, '/tmp/elfin-occ-freespace-impl-g1/src/uld_occ_observer/test/benchmark')
import freespace_judge as j
p = {'formal':True, 'sender':[{'input_id':str(i),'planned_mono_ns':i*400000000,'publish_mono_ns':i*400000000} for i in range(300)], 'queries':[{'planned_mono_ns':i*500000000,'ended_mono_ns':i*500000000+1000000,'status':'INTERNAL_ERROR'} for i in range(240)], 'terminals':{str(i):'COMPLETED' for i in range(300)}}
r = {'formal':True,'overload':[{}]*600,'overload_terminals':{'unrelated':'COMPLETED'},'first_wait':{'status':'OK','elapsed_ms':float('nan')},'epoch_after':1}
l = {'formal':True,'seconds':0,'read_rounds':0,'read_rounds_results':[],'baseline':{'view_directory_entries':0},'after':{'view_directory_entries':0,'read_view_cache_bytes_peak':float('nan')}}
results = { 'all_240_queries_internal_error':j.judge_p_window(p), 'overload_599_missing_no_recovery_nan_wait':j.judge_r_stage(r), 'zero_second_no_traffic_nan_peak':j.judge_l_stage(l)}
print(json.dumps(results, indent=2))
assert all(not v['reasons'] for v in results.values()), results
Path(__file__).with_name('RESULTS.json').write_text(json.dumps(results,indent=2))
from types import SimpleNamespace as S
import freespace_campaign as c
c._snapshot_request = lambda x: x
c._freespace_request = lambda *x: x
c._release_request = lambda x: x
identity = S(epoch=99,map_revision=99,observed_revision=99,geometry_hash='wrong',last_integrated_stamp_ns=0)
class Fake:
    get_snapshot='get'; freespace='query'; release='release'
    def __init__(self, raises=False): self.released=False; self.raises=raises
    def call(self, service, req):
        if service=='get': return S(status='OK',handle='1:1')
        if service=='release': self.released=True; return S(status='INTERNAL_ERROR')
        if self.raises: raise TimeoutError('injected query timeout')
        return S(status='OK',identity=identity,cell_x=[1],next_cursor=100,done=True,feasible_total=100)
f=Fake()
results['one_of_100_wrong_identity_release_error'] = c.full_query(f,(.3,.2,.2))
f=Fake(True)
try: c.full_query(f,(.3,.2,.2))
except TimeoutError: pass
results['query_timeout_released'] = f.released
assert results['one_of_100_wrong_identity_release_error']['status']=='OK'
assert not f.released
Path(__file__).with_name('RESULTS.json').write_text(json.dumps(results,indent=2))
print(json.dumps({k:v for k,v in results.items() if k not in ('all_240_queries_internal_error','overload_599_missing_no_recovery_nan_wait','zero_second_no_traffic_nan_peak')},indent=2))
