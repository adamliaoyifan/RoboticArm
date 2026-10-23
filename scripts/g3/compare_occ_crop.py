#!/usr/bin/env python3
"""Frozen-input OCC crop comparison. Does not launch ROS or move the robot.

This measures the in-process Store transaction, not ROS transport latency.
Real-frame scoring is only available when supplied measured inputs exist.
"""
import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import platform
import resource
import subprocess
import sys
import time
import numpy as np
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception import cargo_view_integration as integration
from luggage_perception import occupancy_map_store as store_module
from luggage_description.container_geometry import normalize_descriptor, contains_point
from luggage_description.container_geometry_batch import contains_points

ROOT=Path(__file__).resolve().parents[2]


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);sys.modules[name]=module
    spec.loader.exec_module(module)
    return module


def baseline(path):
    mapper=load('crop_baseline_mapper',path/'mapper.py')
    integ=load('crop_baseline_integration',path/'integration.py')
    owner=load('crop_baseline_store',path/'store.py')
    owner.apply_integrate=integ.apply_integrate
    return mapper.CargoVolumeMapper,owner


def metadata():
    return dict(revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
                dirty=subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).splitlines(),
                environment=platform.platform(),python=sys.version,seed=20261023,
                scope='in-process Store apply_update; ROS transport/decode/TF not timed',
                capture_complete=True,replay_possible=True)


def mapper(cls,resolution,case):
    return cls(case['inner_size'],case['center_base'],case['yaw'],resolution,
               max_raycast_points=2000,geometry_descriptor=case.get('geometry_descriptor'))


def capture(args):
    args.out.mkdir(parents=True,exist_ok=True)
    names={'mapper.py':'cargo_volume_mapper.py','integration.py':'cargo_view_integration.py',
           'store.py':'occupancy_map_store.py'}
    hashes={}
    for name,source in names.items():
        data=subprocess.check_output(['git','show',args.revision+':src/luggage_perception/luggage_perception/'+source],cwd=ROOT)
        (args.out/name).write_bytes(data);hashes[name]=hashlib.sha256(data).hexdigest()
    (args.out/'manifest.json').write_text(json.dumps(dict(revision=args.revision,hashes=hashes),indent=2))


def correctness(args):
    old,_=baseline(args.baseline);case=json.loads(args.case.read_text());rows=[]
    for res in (.01,.02,.05):
        m=mapper(CargoVolumeMapper,res,case);b=mapper(old,res,case)
        rng=np.random.default_rng(20261023)
        # Long exterior segments, interior origins and all hull faces.
        starts=rng.uniform([-2.,-1.2,-.2],[2.,1.2,1.8],(10000,3))
        ends=rng.uniform([-2.,-1.2,-.2],[2.,1.2,1.8],(10000,3))
        np.savez_compressed(args.out/('rays_%g.npz'%res),starts=starts,ends=ends)
        misses=0;t=time.perf_counter()
        for index,(start,end) in enumerate(zip(starts,ends)):
            a=m._raycast_indices(tuple(start),tuple(end));expected=b._raycast_indices(tuple(start),tuple(end))
            if a!=expected:
                np.savez_compressed(args.out/('failure_%g_%d.npz'%(res,index)),start=start,end=end,
                                    actual=np.array(sorted(a)),expected=np.array(sorted(expected)))
                misses+=1
        rows.append(dict(resolution=res,rays=len(starts),different_sets=misses,elapsed_s=time.perf_counter()-t))
        print(json.dumps(rows[-1]),flush=True)
    result=dict(metadata(),rows=rows,pass_=all(r['different_sets']==0 for r in rows))
    (args.out/'correctness.json').write_text(json.dumps(result,indent=2))
    return result['pass_']


def workload(n,outside,crossing):
    rng=np.random.default_rng(20261023)
    inside_n=int(round(n*(1-outside)))
    inside=rng.uniform([-.60,-.6,.3],[.60,.4,1.3],(inside_n,3))
    if crossing:
        external=rng.uniform([1.1,-.6,.3],[1.6,.4,1.3],(n-inside_n,3))
    else:
        external=rng.uniform([-2.5,-.6,.3],[-1.5,.4,1.3],(n-inside_n,3))
    points=np.concatenate([inside,external]);rng.shuffle(points)
    return points,(-1.2,0.,.8)


def benchmark(args):
    old,old_owner=baseline(args.baseline);case=json.loads(args.case.read_text());rows=[]
    if args.revision_only_control:
        # Explicit ablation: no crop or ray change, only avoid two unnecessary
        # full-frontier stats scans when reading the mapper revision.
        source=(args.baseline/'integration.py').read_text()
        source=source.replace('int(mapper.stats()["map_revision"])','int(mapper.map_revision)')
        namespace=dict(old_owner.apply_integrate.__globals__)
        exec(compile(source,'revision_only_control','exec'),namespace)
        old_owner.apply_integrate=namespace['apply_integrate']
    workloads=[(100000,.9,False,.01)] if args.target_only else [
        (n,f,c,r) for r in (.01,.02,.05) for n in (10000,100000)
        for f in (0.,.5,.9) for c in ([False] if f==0 else [False,True])]
    for n,f,c,res in workloads:
        key='n%d_out%g_cross%d_r%g'%(n,f,c,res);folder=args.out/key;folder.mkdir(exist_ok=True)
        points,origin=workload(n,f,c)
        np.savez_compressed(folder/'input.npz',points=points,origin=origin)
        b=mapper(old,res,case);m=mapper(CargoVolumeMapper,res,case)
        stores=[old_owner.OccupancyMapStore(b,epoch='baseline'),store_module.OccupancyMapStore(m,epoch='candidate')]
        modules=[old_owner,store_module];samples=[[],[]];details=None
        for rep in range(args.warmup+args.repeat):
            for j in ([0,1] if rep%2==0 else [1,0]):
                owner=stores[j];mod=modules[j]
                owner.apply_update(mod.MapUpdate('reset%d'%rep,mod.UpdateKind.RESET))
                update=mod.MapUpdate('view%d'%rep,mod.UpdateKind.INTEGRATE_VIEW,
                    expected_snapshot=owner.identity,geometry_hash=owner._mapper.geometry_hash,
                    points=points,origin=origin,stamp_key=(rep+1,0),acquisition_stamp_ns=(rep+1)*1000000000,
                    settled=True,schema_version=1)
                t=time.perf_counter();result=owner.apply_update(update);elapsed=time.perf_counter()-t
                if result.status.value!='accepted':
                    raise RuntimeError('rejected workload '+key+': '+result.reason)
                if rep>=args.warmup:samples[j].append(elapsed)
                if j==1:details=result.details
        bp,cp=[float(np.percentile(s,95)) for s in samples]
        target=n==100000 and f==.9 and not c and res==.01
        row=dict(workload=key,n=n,outside_fraction=f,crossing=c,resolution=res,
                 samples=samples,baseline_p95_s=bp,candidate_p95_s=cp,ratio=cp/bp,
                 threshold_s=bp*.8 if target else bp*1.1+.001,
                 peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                 candidate_details=details,target=target)
        row['pass']=len(samples[0])>=30 and cp<=row['threshold_s'] and row['peak_rss_kib']<2*1024*1024
        rows.append(row);(folder/'result.json').write_text(json.dumps(row,indent=2,default=str))
        report=dict(metadata(),rows=rows,warmup=args.warmup,repeat=args.repeat,
                    comparator='revision-only ablation' if args.revision_only_control else 'original 86565812',
                    measured_frames='not_evaluated: no complete measured input bundle found',
                    rss_ratio='not_evaluated: shared process peak is not per-variant RSS',
                    pass_=all(r['pass'] for r in rows))
        (args.out/'benchmark.json').write_text(json.dumps(report,indent=2,default=str))
        print(json.dumps({k:v for k,v in row.items() if k not in ('samples','candidate_details')}),flush=True)
        del stores,b,m
    return report['pass_']


def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['capture','correctness','benchmark'])
    p.add_argument('--out',type=Path,required=True);p.add_argument('--baseline',type=Path)
    p.add_argument('--case',type=Path);p.add_argument('--revision',default='86565812')
    p.add_argument('--warmup',type=int,default=5);p.add_argument('--repeat',type=int,default=30)
    p.add_argument('--target-only',action='store_true')
    p.add_argument('--revision-only-control',action='store_true')
    args=p.parse_args();args.out.mkdir(parents=True,exist_ok=True)
    result=globals()[args.mode](args)
    return 1 if result is False else 0

if __name__=='__main__':raise SystemExit(main())
