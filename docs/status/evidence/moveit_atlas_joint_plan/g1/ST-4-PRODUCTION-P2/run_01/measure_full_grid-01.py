"""Synthetic all-free workload; no scene observation or motion certificate."""
import json, math, resource, time
from pathlib import Path
from luggage_description.scene_tf_config_utils import load_scene_tf_config
from luggage_description.container_geometry import descriptor_from_scene_config, normalize_descriptor
from luggage_packing.placement_volume_query import KnownFreeVolume
from luggage_packing.placement_query import placement_query
scene=Path("src/luggage_description/config/scene_tf_deploy_update.yaml")
hull=normalize_descriptor(descriptor_from_scene_config(load_scene_tf_config(str(scene))))
r=.01
size=[2*hull.half_x,2*hull.half_y,hull.ceiling_z-hull.floor_z]
nx,ny,nz=[math.ceil(v/r-1e-9) for v in size]
surface=dict(resolution=r,nx=nx,ny=ny,inner_size=size,floor_z=0.,
 center_base=[0.,0.,hull.floor_z+size[2]/2],yaw=0.,
 state=[["free"]*ny for _ in range(nx)],height=[[0.]*ny for _ in range(nx)],
 confidence=[["sensor"]*ny for _ in range(nx)],geometry_descriptor=hull.descriptor())
evidence=dict(resolution=r,nx=nx,ny=ny,nz=nz,origin_xy=[-size[0]/2,-size[1]/2],
 floor_z=hull.floor_z,states=bytes([1])*(nx*ny*nz))
began=time.perf_counter();verify=KnownFreeVolume(evidence);prefix_s=time.perf_counter()-began
began=time.perf_counter();result=placement_query(surface,(.55,.4,.25),volume_verifier=verify)
query_s=time.perf_counter()-began
record=dict(input_type="synthetic_all_free_timing_only",geometry_hash=hull.geometry_hash,
 dimensions=[nx,ny,nz],cells=nx*ny*nz,resolution_m=r,box_wdh=[.55,.4,.25],
 prefix_build_s=prefix_s,prefix_bytes=verify.unknown.nbytes+verify.occupied.nbytes,
 query_s=query_s,candidates_enumerated=result["candidates_enumerated"],
 retained=sum(c["feasible"] for c in result["candidates"]),
 retained_headings=sorted(set(round(c["heading_theta"],8) for c in result["candidates"] if c["feasible"])),
 max_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
 motion_attempts=0,certificate="not_evaluated")
Path(__file__).with_name("full-grid-timing.json").write_text(json.dumps(record,indent=2)+"\n")
print(json.dumps(record))
