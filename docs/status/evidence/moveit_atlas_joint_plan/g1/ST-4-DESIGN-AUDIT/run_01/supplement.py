"""Supplement source provenance and recheck existing frozen/bag evidence."""
import hashlib,json,math,sys,time
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[7];OUT=Path(__file__).resolve().parent
sha=lambda b:hashlib.sha256(b).hexdigest()
files=[
'src/uld_occ_observer/config/observer.yaml','src/uld_occ_observer/launch/standalone.launch.py',
'src/uld_occ_observer/scripts/occ_surface_bridge.py',
'src/luggage_planning/launch/hardware_pick.launch.py',
'src/luggage_planning/luggage_planning/motion_executor.py',
'src/luggage_planning/luggage_planning/waypoint_generator.py',
'src/luggage_perception/luggage_perception/cargo_observation_partition.py',
'src/luggage_perception/luggage_perception/cargo_observation_service.py',
'src/luggage_description/luggage_description/cargo_grid_contract.py',
'docs/status/evidence/moveit_atlas_joint_plan/g1/ST-0/deploy_freeze_01/manifest.json',
'docs/status/evidence/moveit_atlas_joint_plan/g1/ST-0/deploy_freeze_01/native_qualification/result.json',
'docs/status/evidence/moveit_atlas_joint_plan/g1/ST-0/deploy_freeze_01/pair_replay/result.json',
]
m=json.loads((OUT/'manifest.json').read_text());seen={r['path'] for r in m['inputs']}
for rel in files:
 if rel in seen:continue
 p=ROOT/rel
 if not p.is_file():continue
 data=p.read_bytes();saved=OUT/'snapshots'/rel;saved.parent.mkdir(parents=True,exist_ok=True);saved.write_bytes(data)
 m['inputs'].append(dict(path=rel,sha256=sha(data),bytes=len(data),snapshot=str(saved.relative_to(OUT)),supplement_time_ns=time.time_ns()))
(OUT/'manifest.json').write_text(json.dumps(m,indent=2)+'\n')
freeze=json.loads((ROOT/files[9]).read_text())
frozen=[]
for name,row in freeze['inputs'].items():
 p=Path(name);current=sha(p.read_bytes()) if p.is_file() else None
 frozen.append(dict(path=name,expected=row['sha256'],current=current,match=current==row['sha256']))
inv=json.loads((ROOT/'docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-OBS-DATA-AUDIT/run_02/inventory.json').read_text())
bags=[]
for row in inv:
 p=Path(row['path']);files_ok=[]
 for f in row['files']:
  q=Path(f['path']);s=q.stat() if q.exists() else None
  files_ok.append(dict(path=str(q),exists=s is not None,size_equal=s is not None and s.st_size==f['bytes'],mtime_equal=s is not None and s.st_mtime_ns==f['mtime_ns']))
 bags.append(dict(metadata=str(p),metadata_hash_equal=p.is_file() and sha(p.read_bytes())==row['metadata_sha256'],files=files_ok,topics=len(row['topics'])))
geom=yaml.safe_load((ROOT/'src/luggage_description/config/scene_tf_deploy_update.yaml').read_text())['container']['inner']
resolution=yaml.safe_load((ROOT/'src/uld_occ_observer/config/observer.yaml').read_text())['resolution_m']
dims=[math.ceil(x/resolution) for x in [geom['length'],geom['width'],geom['ceiling_z']-geom['floor_z']]]
count=math.prod(dims)
parity=json.loads((OUT/'install_parity.json').read_text())
for r in parity:
 if r['exists'] and r['path'].endswith('.yaml'):
  r['parsed_yaml_equal']=yaml.safe_load((ROOT/r['path']).read_text())==yaml.safe_load((ROOT/r['source']).read_text())
(OUT/'install_parity.json').write_text(json.dumps(parity,indent=2)+'\n')
result=dict(frozen_inputs=frozen,bag_inventory_recheck=bags,observer_source_resolution_m=resolution,full_grid_dimensions=dims,full_grid_cells=count,read_cells_limit=65536,required_pages=math.ceil(count/65536),compute_source_page_cap=64,calibration='not evaluated; bag metadata/stat consistency is not MCAP content hashing or sensor accuracy')
(OUT/'data_recheck.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({'frozen_matches':sum(r['match'] for r in frozen),'frozen_count':len(frozen),'bag_metadata_matches':sum(r['metadata_hash_equal'] for r in bags),'bags':len(bags),'dimensions':dims,'cells':count,'pages':result['required_pages']}))
