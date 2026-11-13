"""Export the latest complete archived native O snapshot; no scene/GT insertion."""
import gzip,hashlib,json,math,sys
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from uld_occ_observer.srv import ReadCells,ReadSurface
from luggage_mapping.observer import ObserverConfiguration,assemble_snapshot,identity_record
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
root=Path(sys.argv[1]);launch=json.loads((root/'run/launch.json').read_text());owner=launch['map_owner'];cfg=ObserverConfiguration.load(owner['geometry_file'],owner['config_file']);best=None;examined=0
folders=[root/'cycle',*sorted((root/'run/execution').glob('*'))]
for folder in folders:
 manifest=folder/'manifest.json'
 if not manifest.exists():continue
 states=[];columns=[];identity=None;consistent=True
 for row in json.loads(manifest.read_text()).get('records',[]):
  name=row['file']
  if 'observer_get_snapshot_response' in name:states=[];columns=[];identity=None;consistent=True
  elif 'observer_read_cells_response' in name or 'observer_read_surface_response' in name:
   raw=(folder/name).read_bytes();raw=gzip.decompress(raw) if name.endswith('.gz') else raw
   assert hashlib.sha256(raw).hexdigest()==row['sha256'],name
   if 'read_cells' in name:
    msg=deserialize_message(raw,ReadCells.Response);states.extend(msg.states)
   else:
    msg=deserialize_message(raw,ReadSurface.Response);columns.extend(dict(x=x,y=y,active_count=n,known_count=k,has_occupied=o,top_z=z) for x,y,n,k,o,z in zip(msg.x,msg.y,msg.active_count,msg.known_count,msg.has_occupied,msg.top_z))
   page=identity_record(msg.identity)
   if identity is None:identity=page;generation=msg.validity.generation
   consistent=consistent and identity==page and generation==msg.validity.generation
  if ('observer_release_request' in name and consistent and identity is not None
      and len(states)==math.prod(cfg.dimensions_xyz) and len(columns)==math.prod(cfg.dimensions_xyz[:2])):
   examined+=1;key=(identity['epoch'],identity['last_integrated_stamp_ns'],identity['map_revision'],identity['observed_revision'])
   if best is None or key>best[0]:best=(key,states.copy(),columns.copy(),identity,generation,str(folder.relative_to(root)),name)
assert best is not None,'NO_COMPLETE_ARCHIVED_SNAPSHOT'
_,states,columns,identity,generation,folder,boundary=best
captured=assemble_snapshot(cfg,identity,generation,columns,states,scene_revision=launch['scene_revision'],request_id='final-o-export');record=captured.surface.record();surface=np.asarray(record['state']);height=np.asarray(record['surface_z_container'],dtype=float);r=cfg.resolution_m;nx,ny,nz=cfg.dimensions_xyz;origin=np.asarray(captured.volume.origin_container);extent=[origin[0],origin[0]+nx*r,origin[1],origin[1]+ny*r]
np.savez_compressed(root/'final_o_native.npz',states_xyz=np.asarray(states,dtype=np.uint8).reshape((nz,ny,nx)).transpose(2,1,0),surface_classification=surface,surface_z_container=height,origin_container=origin,resolution_m=r)
summary=dict(scope=__doc__,source=captured.source.__dict__,resolution_m=r,dimensions_xyz=cfg.dimensions_xyz,archived_complete_snapshots_examined=examined,selected_folder=folder,selected_release_boundary=boundary,cell_counts={str(k):states.count(k) for k in range(4)},surface_counts={k:int(np.sum(surface==k)) for k in ['unknown','free','occupied','inactive']})
(root/'final_o_export.json').write_text(json.dumps(summary,indent=2)+'\n')
codes=np.zeros(surface.shape)
for i,k in enumerate(['unknown','free','occupied','inactive']):codes[surface==k]=i
fig,axes=plt.subplots(1,2,figsize=(11,6),layout='constrained')
axes[0].imshow(codes.T,origin='lower',extent=extent,cmap=ListedColormap(['#666666','#edf6ec','#d85839','#222222']),vmin=0,vmax=3,interpolation='nearest');axes[0].set_title('Actual O columns: gray unknown / pale free / red occupied')
height=np.where(surface=='occupied',height,np.nan);descriptor=cfg.descriptor();im=axes[1].imshow(height.T,origin='lower',extent=extent,cmap='viridis',vmin=descriptor['floor_z'],vmax=descriptor['ceiling_z'],interpolation='nearest');axes[1].set_title('Actual occupied top, container-absolute z');fig.colorbar(im,ax=axes[1],label='z (m)')
review=root/'three_box_review.json'
if review.exists():
 for i,box in enumerate(json.loads(review.read_text()).get('boxes',[]),1):
  # Overlay only: requested footprints are never inserted into sensor O.
  centre=np.asarray(box['target_xyz_container'])
  # Exact orientation is not in review: show requested centre label only.
  for ax in axes:ax.plot(centre[0],centre[1],'+',color='black');ax.text(centre[0],centre[1],str(i),fontsize=11,color='black')
for ax in axes:ax.set_xlabel('container x (m)');ax.set_ylabel('container y (m)');ax.set_aspect('equal')
fig.suptitle(f'{root.name}: actual sensor O, {r*100:.0f} cm native cells; numbers = requested centres only\nepoch {identity["epoch"]}, revision {identity["map_revision"]}; no GT or box geometry inserted',fontsize=11)
fig.savefig(root/'final_o_native.png',dpi=160);plt.close(fig);print(json.dumps(summary,ensure_ascii=False))
