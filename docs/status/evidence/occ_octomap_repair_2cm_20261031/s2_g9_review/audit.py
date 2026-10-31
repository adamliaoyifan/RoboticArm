import sys,json,hashlib,tempfile
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0,'/tmp/elfin-occ-octomap-repair-2cm-g1/src/uld_occ_observer/test/benchmark')
import campaign_executor as e
import campaign_entry
plan=campaign_entry.load_plan(Path('/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures'))
r={}
for name,value in [('healthy',1.0),('nan',float('nan')),('inf',float('inf'))]:
    campaign=e.LiveCampaign('unused','unused','unused','unused','unused')
    p=SimpleNamespace(processes=0,receipt_status='INTEGRATED',surfaces={k:value for k in ['surface_first_ms','surface_repeat_ms','surface_after_update_ms']})
    def run(*args):
        p.processes+=1
        return [{'integrate_ms':1.0,'acquire_ms':1.0,'roi_ms':1.0,'surface_first_ms':value,'surface_repeat_ms':value} for _ in range(2)]
    p.run_cumulative=run
    campaign.profile=p
    with tempfile.TemporaryDirectory() as out:
        result=campaign._wire_profile(plan,Path(out),'o5',{})
    r[name]={'executed':result['executed'],'formal_judge_pass':result['judge']['pass']}
root=Path('/tmp/elfin-occ-octomap-repair-2cm-g1/docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g9')
r['chain_hash_matches']=[]
for i in range(2):
    c=json.loads((root/f'o7/chain-{i}.json').read_text())
    r['chain_hash_matches'].append(e._content_sha256(c['content_material'])==c['content_sha256'])
r['scope']='Offline adapter injection; does not claim product emitted nonfinite timings. No live or formal run.'
Path(__file__).with_name('RESULTS.json').write_text(json.dumps(r,indent=2)+'\n')
print(json.dumps(r,indent=2))
