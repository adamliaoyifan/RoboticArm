"""Read-only bounded audit snapshot; finish rechecks the same captured inputs."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[7]
OUT = Path(__file__).resolve().parent

def sha(data): return hashlib.sha256(data).hexdigest()
def git(*args): return subprocess.check_output(['git', *args], cwd=ROOT, text=True).strip()
def write(name, value): (OUT/name).write_text(json.dumps(value, indent=2, ensure_ascii=False)+'\n')

if len(sys.argv)>1 and sys.argv[1]=='finish':
    manifest=json.loads((OUT/'manifest.json').read_text())
    changed=[]
    for row in manifest['inputs']:
        path=ROOT/row['path']
        digest=sha(path.read_bytes()) if path.is_file() else None
        if digest!=row['sha256']:changed.append({'path':row['path'],'before':row['sha256'],'after':digest})
    manifest['concurrent_changes']=changed
    manifest['ended_ns']=time.time_ns()
    manifest['retained_bytes']=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file())
    manifest['capture_complete']=manifest['retained_bytes']<25*1024*1024
    manifest['replay_possible']=True
    manifest['limits']='Offline source/data audit only; no live parameter overrides, controller calls or physical validation.'
    write('manifest.json',manifest)
    print(json.dumps({'inputs':len(manifest['inputs']),'concurrent_changes':changed,'retained_bytes':manifest['retained_bytes'],'capture_complete':manifest['capture_complete']}))
    raise SystemExit()

patterns=[
'src/luggage_planning/luggage_planning/placement_prior*.py',
'src/luggage_planning/luggage_planning/joint_operation*.py',
'src/luggage_planning/luggage_planning/{name}.py',
'src/luggage_packing/luggage_packing/placement*.py',
'src/luggage_planning/config/*.yaml',
'src/luggage_planning/launch/*operation*.py',
'src/luggage_planning/launch/placement_prior.launch.py',
'src/luggage_planning/scripts/*planner*node.py',
'src/luggage_planning/scripts/hardware_pick_driver.py',
'src/luggage_packing/scripts/placement_planner_node.py',
'src/luggage_description/config/scene_tf_deploy_update.yaml',
'src/luggage_description/config/suction_contact_model.yaml',
'src/luggage_description/luggage_description/cargo_grid_parameters.py',
'src/luggage_perception/config/*observer*.yaml',
'src/luggage_msgs/msg/JointOperation*.msg',
'src/luggage_msgs/srv/*Operation*.srv',
'st09_contracts/payload_envelope.py',
'src/luggage_planning/CMakeLists.txt',
'docs/agents/reviews/2026-11-12_1145_joint-baseline-next-todo.md',
'docs/plans/PLACEMENT_CLEARANCE_REDESIGN_DISCUSSION_20261112.md',
]
extra=['route_producer','route_producer_factory','atlas_runtime','atlas_proposals','moveit_edge_adapter','moveit_state_validity','occupancy_place_paths','attachment_geometry_policy','hardware_placement_launch','joint_operation_runtime_ports']
paths=set()
for pattern in patterns:
 if '{name}' in pattern:
  for name in extra:paths.update(ROOT.glob(pattern.format(name=name)))
 else:paths.update(ROOT.glob(pattern))
base=ROOT/'docs/status/evidence/moveit_atlas_joint_plan/g1'
for relative in ['ST-4-OBS-DATA-AUDIT/run_02','ST-4-INTEGRATION/g2/release_resolution_02','ST-4-INTEGRATION/g2/release_collision_root_04']:
 for name in ['manifest.json','summary.json','inventory.json','coverage_assessment.json','contact_source_analysis.json','result.json']:
  path=base/relative/name
  if path.is_file():paths.add(path)
manifest={'started_ns':time.time_ns(),'revision':git('rev-parse','HEAD'),'dirty':git('status','--porcelain=v1','--untracked-files=normal').splitlines(),'controller_goals':0,'scene_writes':0,'gazebo':False,'hardware':False,'inputs':[],'capture_complete':False,'replay_possible':False}
for path in sorted(paths):
 if not path.is_file():continue
 data=path.read_bytes()
 if len(data)>2*1024*1024:raise ValueError('per-input bound exceeded: '+str(path))
 rel=path.relative_to(ROOT)
 saved=OUT/'snapshots'/rel;saved.parent.mkdir(parents=True,exist_ok=True);saved.write_bytes(data)
 manifest['inputs'].append({'path':str(rel),'sha256':sha(data),'bytes':len(data),'snapshot':str(saved.relative_to(OUT))})
# Named standard prefixes only; this does not assert that a running process uses them.
parity=[]
entries={
 'luggage_planning':[
 ('lib/luggage_planning/joint_operation_prepare_node.py','src/luggage_planning/scripts/joint_operation_prepare_node.py'),
 ('lib/luggage_planning/motion_planner_node.py','src/luggage_planning/scripts/motion_planner_node.py'),
 ('local/lib/python3.10/dist-packages/luggage_planning/joint_operation_prepare_node.py','src/luggage_planning/luggage_planning/joint_operation_prepare_node.py'),
 ('local/lib/python3.10/dist-packages/luggage_planning/joint_operation_runtime_ports.py','src/luggage_planning/luggage_planning/joint_operation_runtime_ports.py'),
 ('share/luggage_planning/config/attachment_geometry.yaml','src/luggage_planning/config/attachment_geometry.yaml')],
 'luggage_packing':[('lib/luggage_packing/placement_planner_node.py','src/luggage_packing/scripts/placement_planner_node.py')],
 'luggage_description':[('share/luggage_description/config/scene_tf_deploy_update.yaml','src/luggage_description/config/scene_tf_deploy_update.yaml')],
}
for prefix in ['install','install-mo']:
 for pkg,items in entries.items():
  for suffix,source in items:
   installed=ROOT/prefix/pkg/suffix;src=ROOT/source
   row={'prefix':prefix,'path':str(installed.relative_to(ROOT)),'source':source,'exists':installed.is_file()}
   if installed.is_file():
    data=installed.read_bytes();row.update(sha256=sha(data),resolved_path=str(installed.resolve()),same_as_source=src.is_file() and sha(data)==sha(src.read_bytes()))
   parity.append(row)
write('install_parity.json',parity)
write('manifest.json',manifest)
print(json.dumps({'inputs':len(manifest['inputs']),'source_bytes':sum(r['bytes'] for r in manifest['inputs']),'install_entries':len(parity),'revision':manifest['revision']}))
