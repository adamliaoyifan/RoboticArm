"""Frozen baseline/candidate full-placement request campaign, without motion."""
import itertools
import json
from pathlib import Path
import subprocess
import sys
import time

from occ_query_runtime_campaign import stop_owner
from occ_query_placement_comparison import compare_in_environment
from occ_query_runtime_environment import installed_environment


def planning(args,common):
    revision=common.git('rev-parse','HEAD')
    if common.git('status','--porcelain'):
        raise RuntimeError('commit campaign code before freezing')
    if common.git('rev-parse',args.baseline_revision)!=common.BASELINE:
        raise RuntimeError('unregistered baseline revision')
    formal=(not args.rehearsal and args.samples==30 and args.warmup==3
        and set(args.resolutions)==common.formal_resolution_set(getattr(args,'matrix_era','g6_full'))
        and set(args.boxes)=={0,5,20}
        and set(args.paths)=={'hit','fallback'})
    if not formal and not args.rehearsal:
        raise RuntimeError('partial matrix requires --rehearsal')
    out=Path(args.out).resolve()
    if not out.is_relative_to(common.EVIDENCE):
        raise RuntimeError('evidence outside task root')
    out.mkdir(parents=True,exist_ok=False)
    sources={};hashes={}
    for label,ref in [('baseline',common.BASELINE),('candidate',revision)]:
        sources[label]=out/(label+'_source');common.archive(ref,sources[label])
        hashes[label]=common.source_hashes(sources[label])
    common.write_json(out/'source_hashes.json',hashes)
    manifest=dict(kind='planning',revision=revision,baseline_revision=common.BASELINE,
        dirty='',formal_matrix=formal,groups=[],started_ns=time.time_ns(),
        executor='installed production main',overall_g2='not_evaluated')
    script=sources['candidate']/'src/luggage_perception/test/occ_query_placement_runtime.py'
    installs={'baseline':args.baseline_install,'candidate':args.install}
    environments={k:installed_environment(v) for k,v in installs.items()}
    for group_index,(resolution,boxes,path) in enumerate(itertools.product(args.resolutions,args.boxes,args.paths)):
        group=out/('r%s_b%d_%s'%(resolution,boxes,path));group.mkdir()
        entry=dict(resolution=resolution,boxes=boxes,path=path,runs={})
        manifest['groups'].append(entry)
        order=['baseline','candidate'] if group_index%2==0 else ['candidate','baseline']
        entry['order']=order
        for label in order:
            run=group/label;run.mkdir()
            shared=['--source',str(sources[label]),'--install',installs[label],
                '--domain',str(args.domain),'--resolution',str(resolution),'--boxes',str(boxes)]
            command=[sys.executable,str(script),'owner','--out',str(run/'owner'),*shared]
            record=dict(owner_command=command,phase='startup')
            entry['runs'][label]=record;common.write_json(out/'manifest.json',manifest)
            with (run/'owner.log').open('w') as log:
                owner=subprocess.Popen(command,env=environments[label],cwd='/tmp',stdout=log,stderr=subprocess.STDOUT)
                try:
                    client=[sys.executable,str(script),'client','--out',str(run/'client'),
                        '--owner',str(run/'owner'),*shared,'--samples',str(args.samples),'--warmup',str(args.warmup)]
                    if path=='fallback':client.append('--fallback')
                    record['client_command']=client
                    with (run/'client.log').open('w') as client_log:
                        result=subprocess.run(client,env=environments[label],cwd='/tmp',stdout=client_log,
                            stderr=subprocess.STDOUT,timeout=900)
                    record.update(phase='complete',client_exit_code=result.returncode)
                    result_path=run/'client/result.json'
                    record['result']=json.loads(result_path.read_text()) if result_path.exists() else {
                        'outcome':'fail','capture_complete':False,'reason':'no client result'}
                except Exception as error:
                    record.update(phase='failed',error=repr(error))
                finally:
                    stop_owner(owner,wake_script=sources['candidate']/'scripts/g3/occ_query_runtime_wake.py',
                        domain=args.domain,node='placement_planner',env=environments[label],
                        wake_log=run/'control-wake.jsonl')
                    record['owner_exit_code']=owner.returncode
                    common.write_json(out/'manifest.json',manifest)
            common.check_evidence_budget(out)
        # Compare all response fields. Undefined CDR alignment padding can
        # differ for equal messages, so retain raw hashes separately.
        left=group/'baseline/client';right=group/'candidate/client'
        files=sorted(left.glob('*.response.cdr'))
        comparisons={p.name:(right/p.name).exists() and p.read_bytes()==(right/p.name).read_bytes() for p in files}
        entry['wire_comparison']=dict(samples=len(files),required=args.samples+args.warmup+1,
            equal=bool(files) and all(comparisons.values()),differences=[k for k,v in comparisons.items() if not v])
        entry['response_comparison']=compare_in_environment(left,right,
            group/'response_comparison.json',
            script=sources['candidate']/'scripts/g3/occ_query_placement_comparison.py',
            environment=environments['candidate'])
        common.write_json(out/'manifest.json',manifest)
    manifest.update(phase='complete',ended_ns=time.time_ns(),
        source_hashes_unchanged={k:common.source_hashes(v)==hashes[k] for k,v in sources.items()})
    common.write_json(out/'manifest.json',manifest)
    failed=any(not g['response_comparison']['equal'] or
        g['response_comparison']['samples']!=args.samples+args.warmup+1 or
        any(r.get('result',{}).get('outcome')!='pass' for r in g['runs'].values()) for g in manifest['groups'])
    return int(failed)
