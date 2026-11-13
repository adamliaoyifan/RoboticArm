"""One deterministic timing observation; no reliability or hardware claim."""
import json,time,subprocess
from pathlib import Path
from luggage_description.container_geometry import cuboid_from_inner_size
from luggage_packing.placement_query import placement_query
from luggage_packing.placement_volume_query import KnownFreeVolume
r=.01;nx,ny,nz=149,197,148
surface=dict(resolution=r,nx=nx,ny=ny,inner_size=[1.49,1.97,1.48],floor_z=0.,
 center_base=[0.,0.,1.27],yaw=0.,state=[['free']*ny for _ in range(nx)],
 height=[[0.]*ny for _ in range(nx)],confidence=[['sensor']*ny for _ in range(nx)],
 geometry_descriptor=cuboid_from_inner_size((1.49,1.97,1.48),floor_z=.53).descriptor())
evidence=dict(resolution=r,nx=nx,ny=ny,nz=nz,origin_xy=[-.745,-.985],floor_z=.53,
 states=bytes([1])*(nx*ny*nz))
t=time.perf_counter();verifier=KnownFreeVolume(evidence);prefix_s=time.perf_counter()-t
baseline={};exec(compile(subprocess.check_output(['git','show','ed2c47f4:src/luggage_packing/luggage_packing/placement_query.py']),'<baseline>','exec'),baseline)
rows=[]
for name,query in [('baseline',baseline['placement_query']),('continuous_admission',placement_query)]:
 t=time.perf_counter();out=query(surface,(.55,.4,.25),params={'top_n':32,'keep_rejected':0},volume_verifier=verifier)
 rows.append(dict(name=name,elapsed_s=time.perf_counter()-t,success=out['success'],
  enumerated=out.get('n_enumerated'),retained=len(out['candidates'])))
result=dict(scope=__doc__,prefix_s=prefix_s,dimensions=[nx,ny,nz],resolution_m=r,box_size=[.55,.4,.25],rows=rows)
Path(__file__).with_suffix('.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result))
