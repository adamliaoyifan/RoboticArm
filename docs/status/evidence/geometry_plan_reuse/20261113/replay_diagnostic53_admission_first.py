"""Replay recorded same-map dimension refresh at original32 selected poses."""
import gzip,hashlib,json,math,time
from pathlib import Path
from rclpy.serialization import deserialize_message
from uld_occ_observer.srv import ReadCells,ReadSurface
from luggage_mapping.observer import ObserverConfiguration,assemble_snapshot,identity_record
from luggage_planning.placement_prior.contracts import Transform
from luggage_planning.placement_prior.requests import admit_placement_targets
root=Path('docs/status/evidence/three_box_observed/g2/diagnostic_53')
review=json.loads((root/'requested_slot_admission_review.json').read_text())
launch=json.loads((root/'run/launch.json').read_text());owner=launch['map_owner']
cfg=ObserverConfiguration.load(owner['geometry_file'],owner['config_file']);captured=None
rows=json.loads((root/'cycle/manifest.json').read_text())['records']
for row in rows:
 name=row['file']
 if 'observer_get_snapshot_response' in name:states=[];columns=[];identity=None;generation=None
 elif 'observer_read_cells_response' in name or 'observer_read_surface_response' in name:
  raw=(root/'cycle'/name).read_bytes();raw=gzip.decompress(raw) if name.endswith('.gz') else raw
  assert hashlib.sha256(raw).hexdigest()==row['sha256']
  if 'read_cells' in name:
   msg=deserialize_message(raw,ReadCells.Response);states.extend(msg.states)
  else:
   msg=deserialize_message(raw,ReadSurface.Response)
   columns.extend(dict(x=x,y=y,active_count=n,known_count=k,has_occupied=o,top_z=z)
      for x,y,n,k,o,z in zip(msg.x,msg.y,msg.active_count,msg.known_count,msg.has_occupied,msg.top_z))
  current=identity_record(msg.identity)
  assert identity is None or identity==current
  identity=current;generation=msg.validity.generation
 elif ('observer_release_request' in name and identity is not None
       and len(states)==math.prod(cfg.dimensions_xyz) and len(columns)==math.prod(cfg.dimensions_xyz[:2])):
  if all(identity[key]==review['source'][key] for key in ('epoch','map_revision','observed_revision','config_digest')):
   captured=assemble_snapshot(cfg,identity,generation,columns,states,
      scene_revision=launch['scene_revision'],request_id='saved-dimension-refresh')
   break
assert captured is not None,'exact saved snapshot not found'
targets=tuple(Transform(tuple(row['translation']),tuple(row['quaternion_xyzw'])) for row in review['slots'])
began=time.perf_counter()
accepted=admit_placement_targets(captured.surface,review['dimensions'],targets,volume=captured.volume)
result=dict(scope=__doc__,input_root=str(root),source=review['source'],dimensions=review['dimensions'],
 old_accepted=review['admitted'],new_accepted=accepted,unchanged_slots=review['slots'],elapsed_s=time.perf_counter()-began)
Path(__file__).with_suffix('.json').write_text(json.dumps(result,indent=2))
print(json.dumps(dict(old_accepted=sum(review['admitted']),new_accepted=sum(accepted),total=len(accepted),elapsed_s=result['elapsed_s'])))
assert all(accepted)
