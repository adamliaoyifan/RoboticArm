import sys, json, hashlib, copy
from pathlib import Path
ROOT=Path('/tmp/elfin-occ-freespace-impl-g3')
sys.path.insert(0,str(ROOT/'src/uld_occ_observer/test'))
import test_freespace_g3_offline as f
j=f.judge
out=[]
def run(name, factory, judge, mutate=None):
    a=factory()
    if mutate: mutate(a)
    reasons=judge(a)['reasons']
    out.append(dict(name=name,reasons=reasons[:20],accepted=not reasons))
run('healthy_p',f.full_p_window,j.judge_p_window)
run('healthy_l',f.full_l_artifact,j.judge_l_stage)
run('p_all_no_data',f.full_p_window,j.judge_p_window,lambda a:[q.update(status='ACQUIRE_NO_DATA',release_status=None) for q in a['queries']])
run('p_missing_materialized_candidates',f.full_p_window,j.judge_p_window,lambda a:[q.update(candidates=[],content_sha256=j.candidate_digest([]),feasible_total=100000) for q in a['queries']])
run('l_all_queries_age_30_seconds',f.full_l_artifact,j.judge_l_stage,lambda a:[q.update(age_ns=30_000_000_000) for q in a['query_records']])
run('l_no_health',f.full_l_artifact,j.judge_l_stage,lambda a:a.update(health_samples=0,health_gap_max_s=100000))
run('l_round_cache_residual_100MiB',f.full_l_artifact,j.judge_l_stage,lambda a:[r['ownership_after'].update(read_view_cache_bytes=100*1024*1024) for r in a['read_rounds_results']])
run('p_wrong_epoch_all_queries',f.full_p_window,j.judge_p_window,lambda a:[q['lease_identity'].update(epoch=-999) for q in a['queries']])
run('l_all_read_rounds_no_lease',f.full_l_artifact,j.judge_l_stage,lambda a:[r.update(status='ACQUIRE_INTERNAL_ERROR',release_status=None) for r in a['read_rounds_results']])
base=Path(__file__).resolve().parent
(base/'RESULTS.json').write_text(json.dumps(out,indent=2))
freeze=json.loads((ROOT/'docs/status/evidence/occ_freespace_final_20261102/implementation_g3/FREEZE.json').read_text())
checks=[]
for group in ('binaries','tools','fixtures','inputs','commands'):
    entries=freeze[group]
    if 'path' in entries: entries={'self':entries}
    for name,entry in entries.items():
        p=Path(entry['path']); ok=p.exists() and hashlib.sha256(p.read_bytes()).hexdigest()==entry['sha256']
        checks.append(dict(group=group,name=name,match=ok))
(base/'freeze_checks.json').write_text(json.dumps(checks,indent=2))
print(json.dumps(out,indent=2))
print('freeze',sum(x['match'] for x in checks),'/',len(checks))
