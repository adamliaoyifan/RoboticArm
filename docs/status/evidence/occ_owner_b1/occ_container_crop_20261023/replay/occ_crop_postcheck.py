"""Post-score supplemental behavior/RSS probes; never run beside timing."""
import sys,json,math,resource,hashlib,subprocess
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path.cwd()/'scripts/g3'))
import compare_occ_crop as c
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.cargo_view_integration import apply_integrate
root=Path('docs/status/evidence/occ_owner_b1/occ_container_crop_20261023')
case=json.loads(Path('/tmp/elfin-grid-1cm-g1/docs/status/evidence/occ_owner_b1/grid_1cm_s1_20261023/measured_five_boxes.json').read_text())
old,owner=c.baseline(root/'baseline')
if len(sys.argv)>1:
 variant=sys.argv[1];cls=old if variant=='old' else CargoVolumeMapper
 m=c.mapper(cls,.01,case);store=(owner if variant=='old' else c.store_module).OccupancyMapStore(m,epoch='rss')
 mod=owner if variant=='old' else c.store_module
 points,origin=c.workload(100000,.9,False)
 result=store.apply_update(mod.MapUpdate('rss',mod.UpdateKind.INTEGRATE_VIEW,
  expected_snapshot=store.identity,geometry_hash=m.geometry_hash,points=points,origin=origin,
  stamp_key=(1,0),acquisition_stamp_ns=1000000000,settled=True,schema_version=1))
 assert result.accepted
 print(json.dumps(dict(variant=variant,peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)))
 sys.exit(0)
rows=[]
for yaw in (math.radians(37),math.pi/2):
 for res in (.01,.02,.05):
  kwargs=dict(inner_size=(1.,1.,1.),center_base=(.2,-.3,.5),yaw=yaw,resolution=res,max_raycast_points=2000)
  m=CargoVolumeMapper(**kwargs);b=old(**kwargs);rng=np.random.default_rng(20261023)
  starts=rng.uniform([-1,-1,-.2],[1,1,1.2],(10000,3));ends=rng.uniform([-1,-1,-.2],[1,1,1.2],(10000,3))
  different=sum(m._raycast_indices(tuple(a),tuple(z))!=b._raycast_indices(tuple(a),tuple(z)) for a,z in zip(starts,ends))
  # Same valid interior inputs, full state/logodds/shadow equivalence through
  # update order, with independent old ray stepping on a reference mapper.
  ref=CargoVolumeMapper(**kwargs)
  ref._raycast_indices=lambda a,z,diagnostics=None:b._raycast_indices(a,z)
  points=np.array([m._local_to_world(*p) for p in rng.uniform(-.4,.4,(300,3))])
  for _ in range(3):apply_integrate(m,points,(-1,0,.6));apply_integrate(ref,points,(-1,0,.6))
  checks={name:getattr(m,name)==getattr(ref,name) for name in ['_grid','_source','_sensor_seen','_sensor_values','_sensor_labels','_sensor_instance_ids']}
  row=dict(yaw=yaw,resolution=res,rays=10000,different_sets=different,states=checks)
  rows.append(row)
  assert different==0 and all(checks.values()),row
(root/'supplemental_oracle.json').write_text(json.dumps(dict(revision=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),rows=rows),indent=2))
print('additional yaw and state oracles passed',len(rows))
