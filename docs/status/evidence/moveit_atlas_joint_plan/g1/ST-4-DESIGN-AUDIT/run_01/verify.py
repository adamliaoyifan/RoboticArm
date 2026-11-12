"""Verify retained audit provenance, finite probes and cited artifacts."""
import hashlib,json,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[7];OUT=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
m=json.loads((OUT/'manifest.json').read_text());findings=json.loads((OUT/'findings.json').read_text())
errors=[];changed=[]
for row in m['inputs']:
 p=OUT/row['snapshot'];current=ROOT/row['path']
 if not p.is_file() or sha(p)!=row['sha256']:errors.append('snapshot corrupt '+row['path'])
 if not current.is_file() or sha(current)!=row['sha256']:changed.append(row['path'])
for f in findings:
 for r in f['sources']:
  rows=[row for row in m['inputs'] if row['path']==r['path']]
  if len(rows)!=1:errors.append('missing provenance '+f['id']);continue
  lines=(OUT/rows[0]['snapshot']).read_text().splitlines()
  if r['anchor'] not in lines[r['line']-1] or r['sha256']!=rows[0]['sha256']:errors.append('source anchor mismatch '+f['id'])
index={};base=ROOT/'docs/status/evidence/moveit_atlas_joint_plan/g1'
for f in findings:
 for name in f['artifacts']:
  rel,_,selector=name.partition(':')
  candidates=[OUT/rel,base/rel,base/'ST-4-INTEGRATION/g2'/rel]
  resolved=next((p for p in candidates if p.is_file()),None)
  if resolved is None:errors.append('missing artifact '+name);continue
  if selector:
   data=json.loads(resolved.read_text())
   if not any(row.get('case')==selector for row in data):errors.append('missing case '+name)
  index[name]=dict(path=str(resolved.relative_to(ROOT)),sha256=sha(resolved),selector=selector)
probes=json.loads((OUT/'probes.json').read_text())
if len(probes)!=3 or not all(p['passed'] for p in probes):errors.append('finite probes failed')
# Recheck atlas output bytes against the source qualification receipt.
a=ROOT/'src/luggage_planning/data/reachability_atlas_deploy_update'
receipt=json.loads((a/'operation_atlas_qualification.json').read_text());h=hashlib.sha256()
for name in ['s20_container_collision_aware.npz','s20_container_collision_aware.yaml']:
 data=(a/name).read_bytes();h.update(len(data).to_bytes(8,'big'));h.update(data)
atlas=dict(artifact_sha256=h.hexdigest(),receipt_artifact_sha256=receipt['qualification']['artifact_sha256'],match=h.hexdigest()==receipt['qualification']['artifact_sha256'],receipt_sha256=sha(a/'operation_atlas_qualification.json'))
if not atlas['match']:errors.append('current atlas receipt mismatch')
(OUT/'atlas_recheck.json').write_text(json.dumps(atlas,indent=2)+'\n')
(OUT/'artifact_index.json').write_text(json.dumps(index,indent=2)+'\n')
verification=dict(verified_ns=time.time_ns(),inputs=len(m['inputs']),findings=len(findings),source_references=sum(len(f['sources']) for f in findings),probes=3,probe_passes=3,errors=errors,concurrent_changes=changed,retained_bytes=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file()),scope='offline design audit only; no product performance, controller or physical pass')
verification['capture_health_pass']=not errors and verification['retained_bytes']<25*1024*1024
(OUT/'verification.json').write_text(json.dumps(verification,indent=2)+'\n')
m.update(capture_complete=verification['capture_health_pass'],concurrent_changes=changed,retained_bytes=verification['retained_bytes'],review_verification_ns=verification['verified_ns'],audit_gate='pass' if verification['capture_health_pass'] else 'incomplete',report_path='docs/reviews/PLACEMENT_PLANNER_DESIGN_AUDIT_20261112.md')
(OUT/'manifest.json').write_text(json.dumps(m,indent=2)+'\n')
print(json.dumps(verification))
if errors:raise SystemExit(1)
