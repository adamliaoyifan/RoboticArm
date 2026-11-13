#!/usr/bin/env python3
"""Replay user-supplied joint endpoints against a source fixture; never execute.

The output is a development boundary bundle, not upper-placement physics evidence
or an OMPL speed comparison. Run twice in one scene to expose context reuse.
"""
import argparse
from dataclasses import asdict,replace
import json
from pathlib import Path
import subprocess
import time
import traceback
from assets import load,sha256
from full_arm import FullArmScene
from ideal_snapshot import capture_fixture
from planning_types import UpperConfig
from curobo_scene import make_request
from path_planning import joint_path,time_path,certify_path


def save(path,data):path.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo',type=Path,required=True)
    parser.add_argument('--scene-dir',type=Path,required=True)
    parser.add_argument('--scene-config',type=Path,required=True)
    parser.add_argument('--endpoints',type=Path,required=True,help='JSON start/goal arrays, radians, six source joints')
    parser.add_argument('--gpu-python',required=True)
    parser.add_argument('--e2e-root',required=True)
    parser.add_argument('--assets-manifest',required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--budget-s',type=float,default=300.)
    parser.add_argument('--config',type=Path)
    args=parser.parse_args();out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    config=replace(UpperConfig.load(args.config),free_motion_backend='curobo_fast',
                   curobo_python=args.gpu_python,curobo_e2e_root=args.e2e_root,
                   curobo_assets_manifest=args.assets_manifest,path_attempt_budget_s=args.budget_s)
    assets=load(args.repo,args.scene_config)
    snapshot,state=capture_fixture(args.scene_dir,assets,config)
    scene=FullArmScene(assets,snapshot,config)
    endpoints=json.loads(args.endpoints.read_text());start=endpoints['start'];goal=endpoints['goal']
    # Optional explicit attachment uses exactly the declared current grasp.
    contact=endpoints.get('contact_tool',False)
    if 'payload' in endpoints:
        p=endpoints['payload']
        scene.set_payload(p['size'],p['mode'],p['world_pose'],p['box_T_tool'],p.get('allow_support',False))
    request=make_request(scene,start,goal,config,contact);save(out/'request.json',request)
    save(out/'environment.json',dict(revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=args.repo,text=True).strip(),
        dirty=subprocess.check_output(['git','status','--porcelain'],cwd=args.repo,text=True).splitlines(),
        config=asdict(config),snapshot_id=snapshot.snapshot_id,asset_hashes=assets['hashes'],
        source_hashes={p.name:sha256(p) for p in Path(__file__).parent.glob('curobo_*.py')},
        scope='offline_free_motion_only',upper_placement_physics='not_evaluated',speedup='not_evaluated'))
    results=[]
    try:
        for index in range(2):
            before=time.monotonic();path,detail=joint_path(scene,start,goal,config,contact)
            row=dict(index=index,accepted=path is not None,detail=detail,wall_s=time.monotonic()-before)
            save(out/f'query-{index}.json',row)
            if path is not None:
                timed=time_path(scene,path,config)
                save(out/f'controller-{index}.json',timed)
                proof=certify_path(scene,timed,config,contact,deadline=time.monotonic()+args.budget_s,interval_bounds=True)
                row['controller_certificate']=proof;row['accepted']=bool(proof['valid'])
            results.append(row);save(out/'results.json',results)
            if not row['accepted']:break
    except BaseException:
        save(out/'exception.json',dict(traceback=traceback.format_exc()));raise
    finally:
        files={p.name:dict(bytes=p.stat().st_size,sha256=sha256(p)) for p in out.iterdir() if p.is_file()}
        save(out/'manifest.json',dict(case='free_joint_source_smoke',stage='gpu_proposal_and_source_admission',
            first_failure=next((r['detail'].get('reason') for r in results if not r['accepted']),None),
            request_identity=request['identity'],artifacts=files,window='two_queries_stop_on_failure',
            capture_complete=bool(results or (out/'exception.json').exists()),replay_possible=False,
            replay_prerequisites=['matching external assets manifest/full bundle','pinned external Python runtime'],
            scored=False,config_revision=config.revision))
        client=getattr(scene,'_curobo_client',None)
        if client is not None:client.close()
    return 0 if len(results)==2 and all(r['accepted'] for r in results) else 1


if __name__=='__main__':raise SystemExit(main())
