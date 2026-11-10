#!/usr/bin/env python3
"""Bounded owner transaction benchmark and raw-point sampling comparison."""
import argparse
import json
import platform
import resource
import time
from pathlib import Path
import numpy as np
from compare_cargo_grid import make_mapper,commit
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.cargo_volume_snapshot import copy_volume
from luggage_perception.occupancy_map_store import OccupancyMapStore,MapUpdate,UpdateKind


def sampling_cases(out):
    rows=[]
    for step in (.05,.01):
        points=[(float(x),float(y),.199) for x in np.arange(-.1+step/2,.1,step)
                for y in np.arange(-.1+step/2,.1,step)]
        for r in (.05,.02,.01):
            m=CargoVolumeMapper((.6,.6,.4),(0.,0.,.2),0.,r)
            # Direct sensor hits isolate sampling, without a geometry commit or
            # free-space ray. Never infer solid fill from sparse top points.
            for point in points:m.mark_occupied_world(*point)
            s=m.surface_map_2d(); region=occupied=0
            for ix in range(m.nx):
                for iy in range(m.ny):
                    x=-.3+(ix+.5)*r;y=-.3+(iy+.5)*r
                    if -.1<x<.1 and -.1<y<.1:
                        region+=1;occupied+=s['state'][ix][iy]=='occupied'
            row=dict(point_spacing_m=step,point_count=len(points),resolution_m=r,
                     box_footprint_columns=region,observed_occupied_columns=occupied,
                     unobserved_top_columns=region-occupied,
                     interpretation='sampling deficit, not committed-solid raster holes')
            rows.append(row)
            (out/('sensor_%g_%g.json'%(step,r))).write_text(json.dumps(dict(
                input_points=points,result=row,surface=s)))
    (out/'sensor_sampling.json').write_text(json.dumps(rows,indent=2))
    return rows


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--case',type=Path,required=True);parser.add_argument('--repeat',type=int,default=30)
    args=parser.parse_args();args.out.mkdir(parents=True,exist_ok=True)
    case=json.loads(args.case.read_text());results=[]
    for resolution in (.05,.02,.01):
        t=time.perf_counter();m=make_mapper(case,resolution);store=OccupancyMapStore(m,epoch='benchmark')
        construction=time.perf_counter()-t;samples=[];single=[]
        for rep in range(args.repeat+3):
            store.apply_update(MapUpdate('reset%d'%rep,UpdateKind.RESET))
            t=time.perf_counter();per=[]
            for i,b in enumerate(case['boxes']):
                start=time.perf_counter();commit(store,b,'%d_%d'%(rep,i));per.append(time.perf_counter()-start)
            elapsed=time.perf_counter()-t
            if rep>=3:samples.append(elapsed);single.extend(per)
        start=time.perf_counter();volume=copy_volume(m);export=time.perf_counter()-start
        row=dict(resolution_m=resolution,construction_and_snapshot_s=construction,
                 repetitions=len(samples),warmup=3,five_commit_samples_s=samples,
                 five_commit_p95_s=float(np.percentile(samples,95)),five_commit_max_s=max(samples),
                 single_commit_p95_s=float(np.percentile(single,95)),export_s=export,
                 exported_voxels=len(volume['states']),peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
        row['pass']=(len(samples)>=30 and construction<=30 and row['five_commit_p95_s']<=1
                     and row['five_commit_max_s']<=2 and row['peak_rss_kib']<2*1024*1024)
        results.append(row);print(json.dumps({k:v for k,v in row.items() if k!='five_commit_samples_s'}),flush=True)
        del m,store,volume
    sensor=sampling_cases(args.out)
    report=dict(environment=platform.platform(),clock='perf_counter',rows=results,
                sensor_sampling=sensor,capture_complete=True,replay_possible=True)
    (args.out/'benchmark.json').write_text(json.dumps(report,indent=2))
    return 0 if all(r['pass'] for r in results) else 1

if __name__=='__main__':raise SystemExit(main())
