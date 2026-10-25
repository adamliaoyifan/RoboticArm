"""Frozen fixed-load runtime campaign; failure never lowers rates or lease."""
import itertools
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

from occ_query_runtime_campaign import stop_owner,wake_owner
from occ_query_runtime_environment import installed_environment


def concurrency(args,common):
    revision=common.git('rev-parse','HEAD')
    if common.git('status','--porcelain'):raise RuntimeError('commit campaign code before freezing')
    if common.git('rev-parse',args.baseline_revision)!=common.BASELINE:raise RuntimeError('unregistered baseline revision')
    formal=not args.rehearsal and args.duration_wall==120 and args.repeat==3 and set(args.boxes)=={0,5,20}
    if not formal and not args.rehearsal:raise RuntimeError('partial matrix requires --rehearsal')
    out=Path(args.out).resolve()
    if not out.is_relative_to(common.EVIDENCE):raise RuntimeError('evidence outside task root')
    out.mkdir(parents=True,exist_ok=False)
    sources={};hashes={}
    for label,ref in [('baseline',common.BASELINE),('candidate',revision)]:
        sources[label]=out/(label+'_source');common.archive(ref,sources[label])
        hashes[label]=common.source_hashes(sources[label])
    common.write_json(out/'source_hashes.json',hashes)
    manifest=dict(kind='concurrency',revision=revision,baseline_revision=common.BASELINE,
        dirty='',formal_matrix=formal,started_ns=time.time_ns(),groups=[],overall_g2='not_evaluated',
        rates={'integrate':2,'summary':10,'roi':5,'distance256':2,'surface':1},
        heartbeat_period_s=.25,lease_s=2.,points=30000,resolution=.01)
    install={'candidate':args.install,'baseline':args.baseline_install}
    environments={k:installed_environment(v) for k,v in install.items()}
    owner_script=sources['candidate']/'src/luggage_perception/test/occ_query_concurrency_owner.py'
    client_script=sources['candidate']/'src/luggage_perception/test/occ_query_concurrency_client.py'
    for boxes,index in itertools.product(args.boxes,range(args.repeat)):
        group=out/('b%d_repeat%d'%(boxes,index));group.mkdir()
        entry=dict(boxes=boxes,repetition=index,runs={});manifest['groups'].append(entry)
        order=['baseline','candidate'] if index%2==0 else ['candidate','baseline']
        for label in order:
            run=group/label;run.mkdir()
            command=[sys.executable,str(owner_script),'--out',str(run/'owner'),
                '--source',str(sources[label]),'--install',install[label],'--domain',str(args.domain),
                '--boxes',str(boxes)]
            record=dict(owner_command=command,phase='startup',started_ns=time.time_ns())
            entry['runs'][label]=record;common.write_json(out/'manifest.json',manifest)
            with (run/'owner.log').open('w') as log:
                owner=subprocess.Popen(command,env=environments[label],cwd='/tmp',stdout=log,stderr=subprocess.STDOUT)
                try:
                    deadline=time.monotonic()+90.
                    while not (run/'owner/ready.json').exists():
                        if owner.poll() is not None:raise RuntimeError('owner constructor failure')
                        if time.monotonic()>deadline:raise RuntimeError('owner startup deadline')
                        time.sleep(.05)
                    client=[sys.executable,str(client_script),'--out',str(run/'client'),
                        '--owner',str(run/'owner'),'--install',install[label],'--domain',str(args.domain),
                        '--duration',str(args.duration_wall)]
                    if label=='baseline':client.append('--baseline')
                    record['client_command']=client;record['phase']='client'
                    common.write_json(out/'manifest.json',manifest)
                    with (run/'client.log').open('w') as client_log:
                        result=subprocess.run(client,env=environments[label],cwd='/tmp',stdout=client_log,
                            stderr=subprocess.STDOUT,timeout=args.duration_wall+180.)
                    record['client_exit_code']=result.returncode
                    path=run/'client/result.json'
                    record['result']=json.loads(path.read_text()) if path.exists() else {
                        'outcome':'fail','capture_complete':False,'reason':'client missing result'}
                    record['phase']='complete'
                except Exception as error:
                    record.update(phase='failed',error=repr(error))
                finally:
                    if owner.poll() is None:
                        owner.send_signal(signal.SIGUSR1)
                        wake_owner(sources['candidate']/'scripts/g3/occ_query_runtime_wake.py',args.domain,
                            'cargo_volume_mapper',env=environments[label],log=run/'control-wake.jsonl')
                        deadline=time.monotonic()+45.
                        while not (run/'owner/capture.json').exists() and owner.poll() is None and time.monotonic()<deadline:
                            time.sleep(.05)
                    record['capture_complete']=(run/'owner/capture.json').exists()
                    stop_owner(owner,wake_script=sources['candidate']/'scripts/g3/occ_query_runtime_wake.py',
                        domain=args.domain,env=environments[label],wake_log=run/'control-wake.jsonl')
                    record['owner_exit_code']=owner.returncode
                    record['ended_ns']=time.time_ns()
                    common.write_json(out/'manifest.json',manifest)
            # Decode/rebuild only after both live processes have stopped. The
            # verifier pins baseline primary modules in a fresh process while
            # using candidate IDL, never mixed product implementations.
            replay_command=[sys.executable,str(sources['candidate']/'scripts/g3/occ_query_concurrency_replay.py'),
                '--case',str(run),'--out',str(run/'replay')]
            record['replay_command']=replay_command
            with (run/'replay.log').open('w') as replay_log:
                try:
                    replay_result=subprocess.run(replay_command,env=environments['candidate'],cwd='/tmp',
                        stdout=replay_log,stderr=subprocess.STDOUT,timeout=1800.)
                    record['replay_exit_code']=replay_result.returncode
                    for name in ('result','gates'):
                        path=run/'replay'/(name+'.json')
                        if path.exists():record['replay_'+name]=json.loads(path.read_text())
                    replay_manifest=run/'replay/manifest.json'
                    record['capture_complete']=record['capture_complete'] and replay_manifest.exists() and (
                        json.loads(replay_manifest.read_text()).get('capture_complete',False))
                except Exception as error:
                    record.update(replay_error=repr(error),capture_complete=False)
            common.write_json(out/'manifest.json',manifest)
            common.check_evidence_budget(out)
            if not record['capture_complete']:
                manifest.update(phase='stopped',stop_reason='INCOMPLETE_EVIDENCE')
                common.write_json(out/'manifest.json',manifest)
                return 1
    manifest.update(phase='complete',ended_ns=time.time_ns(),
        source_hashes_unchanged={k:common.source_hashes(v)==hashes[k] for k,v in sources.items()})
    common.write_json(out/'manifest.json',manifest)
    return int(any(r.get('replay_gates',{}).get('outcome')!='pass' for g in manifest['groups'] for r in g['runs'].values()))
