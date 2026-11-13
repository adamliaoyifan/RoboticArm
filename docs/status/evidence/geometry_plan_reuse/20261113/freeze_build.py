"""Freeze effective source/install correspondence before one simulation run."""
import hashlib,importlib,json,subprocess,time
from pathlib import Path
root=Path.cwd();out=Path(__file__).parent/'build_01';out.mkdir(exist_ok=False)
packages=('luggage_description','luggage_msgs','luggage_planning','luggage_gazebo','luggage_perception','luggage_packing','luggage_mapping','uld_occ_observer')
hashes={};matches=[];mismatches=[]
for package in packages:
 source=root/'src'/package
 for path in source.rglob('*'):
  if path.is_file() and '__pycache__' not in path.parts and path.suffix!='.pyc':
   hashes[str(path.relative_to(root))]=hashlib.sha256(path.read_bytes()).hexdigest()
 if package in ('luggage_msgs','uld_occ_observer'):continue
 module=importlib.import_module(package);directory=Path(module.__file__).resolve().parent
 assert str(directory).startswith(str(root/'install-production')),str(directory)
 for path in (source/package).rglob('*.py'):
  if '__pycache__' in path.parts:continue
  target=directory/path.relative_to(source/package)
  record=dict(source=str(path.relative_to(root)),install=str(target.relative_to(root)))
  (matches if target.is_file() and path.read_bytes()==target.read_bytes() else mismatches).append(record)
 # Verify the entry scripts and launch files used by installed ROS invocations.
 for kind,targetbase in [('scripts',root/'install-production'/package/'lib'/package),
                          ('launch',root/'install-production'/package/'share'/package/'launch'),
                          ('config',root/'install-production'/package/'share'/package/'config')]:
  for path in (source/kind).glob('*'):
   if not path.is_file():continue
   target=targetbase/path.name
   if target.is_file():
    record=dict(source=str(path.relative_to(root)),install=str(target.relative_to(root)))
    (matches if path.read_bytes()==target.read_bytes() else mismatches).append(record)
manifest=dict(revision=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
 dirty=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines(),
 frozen_wall=time.time(),source_after=hashes,matched=matches,mismatched=mismatches)
(out/'manifest.json').write_text(json.dumps(manifest,indent=2))
print(json.dumps(dict(source_files=len(hashes),matched=len(matches),mismatched=mismatches)))
assert not mismatches,'stale installed files'
