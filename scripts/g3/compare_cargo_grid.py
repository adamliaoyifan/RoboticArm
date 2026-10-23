#!/usr/bin/env python3
"""Offline measured-geometry raster oracle; never publishes a planning map.

The independent oracle uses GEOS polygon/cell intersection, not mapper clipping.
Empty means unoccupied (usually unknown), not evidence of observed free space.
"""
import argparse
import hashlib
import json
import math
import platform
import resource
import subprocess
import time
from pathlib import Path

import numpy as np
from shapely.geometry import Polygon, box as rectangle
from luggage_description.container_geometry import contains_point
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import OccupancyMapStore, MapUpdate, UpdateKind


def footprint(box):
    c, s, a = box['center'], box['size'], box['yaw']
    co, si = math.cos(a), math.sin(a)
    return Polygon([(c[0]+co*x-si*y, c[1]+si*x+co*y)
                    for x, y in [(-s[0]/2,-s[1]/2),(s[0]/2,-s[1]/2),
                                 (s[0]/2,s[1]/2),(-s[0]/2,s[1]/2)]])


def make_mapper(case, resolution):
    return CargoVolumeMapper(**{k: case[k] for k in
        ('inner_size','center_base','yaw','geometry_descriptor') if k in case}, resolution=resolution)


def commit(store, box, operation):
    result = store.apply_update(MapUpdate(
        operation_id=operation, kind=UpdateKind.ADD_BOX_V2,
        expected_snapshot=store.identity, geometry_hash=store.mapper.geometry_hash,
        box=(box['center'],box['size'],box['yaw']), measured_source='verify_placed_box',
        verified_center=box['center'], verify_tolerance_m=.005,
        verify_source='offline_measured_fixture'))
    if result.status.value != 'accepted':
        raise RuntimeError(result.status.value + ": " + result.reason)


def oracle(mapper, boxes):
    r=mapper.resolution; nx,ny,nz=mapper.nx,mapper.ny,mapper.nz
    expected=np.zeros((nz,ny,nx),dtype=bool)
    tops=np.zeros((ny,nx)); x0=-mapper.inner_l/2; y0=-mapper.inner_w/2
    floor=mapper.center[2]-mapper.inner_h/2
    for b in boxes:
        polygon=footprint(b); lo=polygon.bounds
        bottom=b['center'][2]-b['size'][2]/2; top=b['center'][2]+b['size'][2]/2
        zs=np.flatnonzero(np.minimum(floor+(np.arange(nz)+1)*r,top)
                         -np.maximum(floor+np.arange(nz)*r,bottom)>1e-12)
        for iy in range(ny):
            y=y0+iy*r
            if y+r<lo[1] or y>lo[3]: continue
            for ix in range(nx):
                x=x0+ix*r
                if x+r<lo[0] or x>lo[2]: continue
                if polygon.intersection(rectangle(x,y,x+r,y+r)).area>1e-12:
                    expected[zs,iy,ix]=True
                    tops[iy,ix]=max(tops[iy,ix],top-floor)
    active=np.frombuffer(mapper._active,dtype=np.uint8).reshape(expected.shape).astype(bool)
    return expected & active,tops


def check_case(case,resolution,out=None):
    begin=time.perf_counter(); m=make_mapper(case,resolution); store=OccupancyMapStore(m,epoch='comparison')
    for i,b in enumerate(case['boxes']): commit(store,b,str(i))
    elapsed=time.perf_counter()-begin
    surface=store.v1_surface_dict(); actual=np.frombuffer(m._grid,dtype=np.uint8).reshape(m.nz,m.ny,m.nx)==2
    expected,tops=oracle(m,case['boxes'])
    projected=np.array(surface['height']).T
    has=expected.any(axis=0); expected_top=np.where(has,(m.nz-expected[::-1].argmax(axis=0))*resolution,0.)
    expected_top=np.minimum(expected_top,m.inner_h)
    missed=int((expected & ~actual).sum()); excess=int((actual & ~expected).sum())
    mismatch=int((np.abs(projected-expected_top)>1e-9).sum())
    active=np.frombuffer(m._active,dtype=np.uint8).reshape(actual.shape).astype(bool)
    # Full cell-centre hull oracle, independent of the optimized separability.
    # Full-sized scene masks are compared to the preserved legacy mask at5cm;
    # small synthetic cases exercise all centres at each resolution.
    mask_mismatch=0
    if len(m._grid)<150000:
        for iz,iy,ix in np.ndindex(actual.shape):
            point=(-m.inner_l/2+(ix+.5)*resolution,-m.inner_w/2+(iy+.5)*resolution,
                   m._hull.floor_z+(iz+.5)*resolution)
            mask_mismatch+=bool(active[iz,iy,ix]) != contains_point(m._hull,point)
    error=projected[has]-tops[has]
    row=dict(case=case['name'],resolution_m=resolution,voxels=len(m._grid),
             missed_occupied_voxels=missed,excess_occupied_voxels=excess,
             surface_mismatch_cells=mismatch,active_mask_mismatches=mask_mismatch,
             unknown_columns=sum(v=='unknown' for line in surface['state'] for v in line),
             occupied_columns=int(has.sum()),max_top_error_m=float(error.max()) if len(error) else 0.,
             min_top_error_m=float(error.min()) if len(error) else 0.,elapsed_s=elapsed)
    row['pass']=not(missed or excess or mismatch or mask_mismatch) and (
        row['min_top_error_m']>=-1e-9 and row['max_top_error_m']<=resolution+1e-9)
    if 'gap' in case:
        g=case['gap']; r=resolution
        gap_cells=retained=0
        for iy in range(m.ny):
            y=-m.inner_w/2+(iy+.5)*r
            for ix in range(m.nx):
                x=-m.inner_l/2+(ix+.5)*r
                if g[0]<x<g[1] and g[2]<y<g[3]:
                    gap_cells+=1;retained+=not actual[:,iy,ix].any()
        row.update(gap_center_cells=gap_cells,gap_preserved_cells=retained,
                   gap_preserved_fraction=retained/gap_cells if gap_cells else None)
    if out is not None:
        out.mkdir(parents=True,exist_ok=True)
        (out/'input.json').write_text(json.dumps(case,indent=2))
        (out/'surface.json').write_text(json.dumps(surface))
        np.savez_compressed(out/'arrays.npz',actual=actual,expected=expected,active=active,height=projected)
        (out/'result.json').write_text(json.dumps(row,indent=2))
    return row,m,store


def fixtures():
    base=dict(inner_size=[.6,.6,.4],center_base=[0.,0.,.2],yaw=0.)
    for yaw in (0,5,45,90):
        for phase in (0,.25,.5,.75):
            shift=phase*.05
            yield dict(base,name='yaw%d_phase%g'%(yaw,phase),boxes=[dict(
                center=[shift,shift,.1],size=[.24,.18,.2],yaw=math.radians(yaw))])
    for phase in (0,.25,.5,.75):
        shift=phase*.05
        yield dict(base,name='gap_2cm_phase%g'%phase,boxes=[
            dict(center=[-.11+shift,0.,.1],size=[.2,.2,.2],yaw=0.),
            dict(center=[.11+shift,0.,.1],size=[.2,.2,.2],yaw=0.)],gap=[-.01+shift,.01+shift,-.1,.1])


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--measured-case',type=Path,required=True);parser.add_argument('--baseline',type=Path,required=True)
    args=parser.parse_args();args.out.mkdir(parents=True,exist_ok=True)
    rows=[];images=[]
    cases=list(fixtures());measured=json.loads(args.measured_case.read_text());measured['name']='measured_five_boxes';cases.append(measured)
    for case in cases:
        for r in (.05,.02,.01):
            row,m,store=check_case(case,r,args.out/(case['name']+'_%g'%r))
            if case['name']=='measured_five_boxes' and r==.05:
                old=json.loads((args.baseline/'surface.json').read_text())
                # Store adds identity/provenance; compare the legacy projection keys.
                current=m.surface_map_2d()
                equal=(bytes(m._grid)==(args.baseline/'states.bin').read_bytes()
                       and bytes(m._active)==(args.baseline/'active.bin').read_bytes() and old==current)
                row['baseline_5cm_exact_match']=equal;row['pass'] &= equal
            rows.append(row)
            if case['name'] in ('measured_five_boxes','gap_2cm_phase0.5') and r in (.05,.01):
                images.append((case,r,np.array(store.snapshot().height).T))
            print(json.dumps(row),flush=True)
            del m,store
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(11,9),layout='constrained')
    for ax,(case,r,height) in zip(axes.flat,images):
        ax.imshow(height,origin='lower',extent=[-case['inner_size'][0]/2,-case['inner_size'][0]/2+height.shape[1]*r,
                   -case['inner_size'][1]/2,-case['inner_size'][1]/2+height.shape[0]*r],interpolation='nearest',vmin=0,vmax=.3)
        for b in case['boxes']:
            xy=np.array(footprint(b).exterior.coords);ax.plot(xy[:,0],xy[:,1],color='white',lw=.8)
        ax.set_title('%s / %g cm'%(case['name'],r*100));ax.set_xlabel('container X (m)');ax.set_ylabel('container Y (m)')
    fig.suptitle('Heightmap comparison (dark = no occupied top, not observed-free)')
    fig.savefig(args.out/'comparison.png',dpi=150)
    report=dict(revision=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
                dirty=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines(),
                environment=platform.platform(),cases=len(rows),passed=sum(x['pass'] for x in rows),
                capture_complete=True,replay_possible=True,rows=rows)
    (args.out/'summary.json').write_text(json.dumps(report,indent=2))
    return 0 if all(x['pass'] for x in rows) else 1

if __name__=='__main__': raise SystemExit(main())
