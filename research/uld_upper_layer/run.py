#!/usr/bin/env python3
"""Reproducible offline experiment; writes evidence, never moves a robot."""
import argparse
from collections import Counter, defaultdict
from datetime import datetime
import json
from importlib.metadata import version, PackageNotFoundError
import platform
from pathlib import Path
import subprocess
import time
import traceback
import numpy as np
from scipy.spatial.transform import Rotation
from assets import load
from action import SETTINGS, build, cases, record, validate


def save(path, value):
    Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def git(repo, *args):
    return subprocess.check_output(['git','-C',str(repo),*args],text=True).strip()


def stage_ik(assets, candidate, path):
    from luggage_planning.offline_ik import refine_to_pose
    urdf=assets['urdf']; limits=np.array(urdf.joint_limits())
    rng=np.random.default_rng(61110)
    seeds=[np.clip(np.zeros(6),limits[:,0],limits[:,1])]
    seeds += list(rng.uniform(limits[:,0],limits[:,1],size=(15,6)))
    previous=None; stages=[]
    names={'approach_up','handoff','withdraw','push_start','push_end'}
    with path.open('w') as stream:
        for stage in candidate['stages']:
            if stage['name'] not in names:
                continue
            target=assets['world_container'] @ stage['tcp']
            quat=Rotation.from_matrix(target[:3,:3]).as_quat()
            attempts=[]; solution=None
            for seed_index, seed in enumerate(([previous] if previous is not None else [])+seeds):
                started=time.monotonic()
                joints,info=refine_to_pose(urdf,urdf.path_to('suction_contact_frame'),
                                         target[:3,3],quat,seed,pos_tol=.0005,rot_tol=.005)
                row=dict(case_id=candidate['case']['id'],stage=stage['name'],
                         target_world=target.tolist(),seed_index=seed_index,
                         seed=np.asarray(seed).tolist(),joints=joints,
                         elapsed_s=time.monotonic()-started,**info)
                stream.write(json.dumps(row)+'\n');stream.flush();attempts.append(info)
                if joints is not None:
                    solution=joints;previous=joints;break
            stages.append(dict(name=stage['name'],outcome='pose_ik_found' if solution else 'search_exhausted',
                               attempts=len(attempts),joints=solution,
                               target_world=target.tolist(),best_position_error_m=min(x['pos_err_m'] for x in attempts)))
    return dict(case_id=candidate['case']['id'],stages=stages,
                all_stage_poses_found=all(x['outcome']=='pose_ik_found' for x in stages),
                full_arm_collision='not_evaluated',connected_path='not_evaluated')


def draw(assets, candidate, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Rectangle
    from scipy.spatial import ConvexHull
    hull=assets['hull']; front=candidate['support_x_m']; top=candidate['support_z_m']
    available={s['name']:s for s in candidate['stages']}
    selected=[available[n] for n in ('handoff','withdraw','push_start','push_end') if n in available]
    fig,axes=plt.subplots(2,2,figsize=(12,8),sharex=True,sharey=True)
    for ax,stage in zip(axes.flat,selected):
        ax.add_patch(Rectangle((front,hull.floor_z),hull.half_x-front,top-hull.floor_z,
                               facecolor='#cbd5e1',label='filled support'))
        ax.plot([-hull.half_x,hull.half_x],[hull.ceiling_z]*2,'k-',linewidth=3,label='ceiling')
        boxes=[(stage['payload'],'#fbbf24')]+[(b.moved(stage['tcp']),'#2563eb') for b in assets['tool']]
        for box,color in boxes:
            vertices=box.corners()[:,[0,2]]
            boundary=vertices[ConvexHull(vertices).vertices]
            ax.add_patch(Polygon(boundary,facecolor=color,edgecolor=color,alpha=.65))
        ax.scatter(*stage['tcp'][[0,2],3],c='red',s=20,zorder=5)
        ax.set_title(stage['name']+' / '+stage['contact_mode'])
        ax.set_xlim(-1.5,.8);ax.set_ylim(.65,2.15);ax.set_aspect('equal');ax.grid(alpha=.2)
        ax.set_xlabel('container X (m)');ax.set_ylabel('container Z (m)')
    case=candidate['case']
    fig.suptitle(f"{case['box']}, axis {case['insertion_axis']}, {case['grasp']} grasp, "
                 f"tool yaw {case['tool_yaw_deg']} deg\n"
                 'Fixed-tool geometry only; arm, turning path and physics NOT validated')
    fig.tight_layout();fig.savefig(output/'stages.png',dpi=150);fig.savefig(output/'stages.svg');plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();repo=args.repo.resolve();output=args.output.resolve()
    output.mkdir(parents=True,exist_ok=False)
    started=time.monotonic()
    manifest=dict(started_at=datetime.now().astimezone().isoformat(),
                  command=['python3','research/uld_upper_layer/run.py','--repo',str(repo),'--output',str(output)],
                  revision=git(repo,'rev-parse','HEAD'),dirty=git(repo,'status','--porcelain'),
                  python=platform.python_version(),settings=SETTINGS,repetitions=1,
                  seed=61110,simulation_started=False,capture_complete=False,
                  limitations=['synthetic rigid support and catalog boxes; not online perception',
                               'conservative local mesh bounding boxes',
                               'no whole-arm collision checking or connected trajectory',
                               'outside pickup/inversion and force/contact physics not evaluated',
                               'no statistical reliability claim'])
    manifest['packages']={}
    for package in ('numpy','scipy','trimesh','PyYAML','matplotlib','xacro'):
        try:
            manifest['packages'][package]=version(package)
        except PackageNotFoundError:
            manifest['packages'][package]='system_package_version_unavailable'
    save(output/'manifest.json',manifest)
    try:
        assets=load(repo)
        manifest['input_hashes']=assets['hashes']
        manifest['geometry_hash']=assets['hull'].geometry_hash
        save(output/'manifest.json',manifest)
        (output/'robot.urdf').write_text(assets['xml'])
        save(output/'geometry.json',dict(scene=assets['scene'],catalog=assets['catalog'],
                                         contact=assets['contact'],tool=[b.record() for b in assets['tool']]))
        results=[]
        with (output/'cases.jsonl').open('w') as stream:
            for case in cases(assets):
                result=validate(assets,build(assets,case));results.append(result)
                stream.write(json.dumps(record(result))+'\n');stream.flush()
        assert len(results)==864
        counts=Counter(r['reason'] for r in results)
        accepted=[r for r in results if r['outcome']=='geometry_candidate']
        groups=defaultdict(list)
        for r in accepted:
            groups[(r['case']['box'],r['case']['insertion_axis'])].append(r)
        selected=[]
        for group in sorted(groups):
            # Deterministic largest window first, then case ID; no search tuning
            # based on IK outcomes. Keep the selection independent of IK.
            selected.append(sorted(groups[group],key=lambda r:(-r['interval']['width_m'],r['case']['id']))[0])
        print(json.dumps({'geometry_cases':len(results),'geometry_candidates':len(accepted),'reasons':counts}),flush=True)
        ik=[]
        for candidate in selected:
            result=stage_ik(assets,candidate,output/(candidate['case']['id']+'_ik.jsonl'))
            ik.append(result);save(output/'ik_summary.json',ik)
            print(json.dumps({'ik_case':result['case_id'],'all_stage_poses_found':result['all_stage_poses_found'],
                              'stages':{s['name']:s['outcome'] for s in result['stages']}}),flush=True)
        save(output/'ik_summary.json',ik)
        breakdown=[]
        for box in ('carryon','standard','large'):
            for grasp in ('center','rear'):
                subset=[r for r in results if r['case']['box']==box and r['case']['grasp']==grasp]
                breakdown.append(dict(box=box,grasp=grasp,total=len(subset),
                                      candidates=sum(r['outcome']=='geometry_candidate' for r in subset),
                                      reasons=dict(Counter(r['reason'] for r in subset))))
        summary=dict(total_cases=len(results),geometry_candidates=len(accepted),reason_counts=dict(counts),
                     breakdown=breakdown,ik_candidates=len(ik),
                     ik_all_stage_pose_cases=sum(r['all_stage_poses_found'] for r in ik),
                     ik_outcome='evaluated' if ik else 'not_evaluated:no_geometry_candidates',
                     full_action_outcome='not_evaluated',limitations=manifest['limitations'])
        save(output/'summary.json',summary)
        diagram_case=selected[0] if selected else next((r for r in results if r['stages']),None)
        if diagram_case:
            save(output/'diagram_case.json',record(diagram_case));draw(assets,diagram_case,output)
        artifact_bytes=sum(p.stat().st_size for p in output.iterdir() if p.is_file())
        if artifact_bytes>100*1024*1024:
            raise ValueError('artifact size exceeds frozen 100 MB bound')
        manifest.update(completed_at=datetime.now().astimezone().isoformat(),elapsed_s=time.monotonic()-started,
                        capture_complete=True,case_records=len(results),artifact_bytes=artifact_bytes,
                        artifacts=sorted(p.name for p in output.iterdir()))
        save(output/'manifest.json',manifest)
        print('Saved',output,flush=True)
    except Exception:
        (output/'failure.txt').write_text(traceback.format_exc())
        manifest.update(completed_at=datetime.now().astimezone().isoformat(),capture_complete=False,
                        failure='failure.txt',elapsed_s=time.monotonic()-started)
        save(output/'manifest.json',manifest)
        raise


if __name__=='__main__':
    main()
