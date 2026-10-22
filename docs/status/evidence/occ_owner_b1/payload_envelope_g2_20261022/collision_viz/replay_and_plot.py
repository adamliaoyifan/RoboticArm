"""Saved-state diagnostic ablations and figures; never updates online margins."""
import json, sys, subprocess, csv
from pathlib import Path
from itertools import product
from collections import Counter
# Use matching distro matplotlib/mplot3d; retain user-site trimesh.
sys.path.insert(0, '/usr/lib/python3/dist-packages')
import numpy as np
from scipy.spatial.transform import Rotation
import trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
OUT=Path(__file__).resolve().parent
ROOT=next(p for p in OUT.parents if (p/'st09_contracts').exists())
for p in [ROOT,ROOT/'scripts',ROOT/'scripts/g3',ROOT/'src/luggage_planning']:
    sys.path.insert(0,str(p))
from replay_envelope_anchor_collisions import pose
from replay_collision_attribution import Urdf,_origin_of,_resolve_package_path
from st09_contracts.payload_envelope import build_envelope,EnvelopeLimits
RUN=OUT.parent/'live_03'
b=json.loads(next((RUN/'selection_boundary').glob('*/request.json')).read_text())
rows=json.loads((RUN/'selection_response.json').read_text())['rows']
rows=[r for r in rows if 'slot_index' in r and r.get('reason')!='ATTEMPT_SUMMARY']
states={}; mappings=[]
for r in rows:
    ids=[]
    for e in r.get('staging_endpoints',[]):
        q=tuple(e['q'])
        if q not in states: states[q]={'id':len(states)+1,'node_id':e['node_id'],'q':list(q)}
        ids.append(states[q]['id'])
    mappings.append({'slot_index':r['slot_index'],'reason':r.get('reason'),'state_ids':sorted(set(ids)), 'slot_pose':r.get('slot_pose')})
urdf=Urdf((RUN/'robot_expanded.urdf').read_text())
g=b['grasp']; nominal=np.linalg.inv(pose(g['position'],g['quaternion_xyzw']))
configs={'full':(5,(.05,.05,.05),(.1,.1,.005)),
         'no_position_z':(5,(.05,.05,.05),(.1,.1,0)),
         'no_z':(5,(.05,.05,0),(.1,.1,0)),
         'only_z':(0,(0,0,.05),(0,0,.005))}
payloads={'nominal':(b['box_wdh'],nominal)}
for name,args in configs.items():
    e=build_envelope(b['box_wdh'],nominal[:3,3],Rotation.from_matrix(nominal[:3,:3]).as_quat(),EnvelopeLimits(*args))
    payloads[name]=(e.dimensions_wdh,pose(e.t_tcp_box_translation,e.t_tcp_box_quaternion_xyzw))
# Remove only the upward Z inflation (55mm), retaining the lower face.
size,offset=payloads['full']; size=np.array(size).copy(); offset=offset.copy()
# Tool +Z points down in these saved tool-down poses; verify below.
size[2]-=.055; offset[2,3]+=.0275
payloads['no_upper_z']=(size,offset)
transforms={s['id']:urdf.link_transforms(s['q']) for s in states.values()}
assert all(tf['suction_contact_frame'][2,2]<-.999 for tf in transforms.values())
meshes=[]
for link in ['elfin_link%d'%i for i in range(1,7)]:
    for c in urdf.link_els[link].findall('collision'):
        element=c.find('geometry/mesh')
        if element is None: continue
        mesh=trimesh.load(_resolve_package_path(element.get('filename')),force='mesh',process=False)
        mesh.vertices*=np.array([float(x) for x in element.get('scale','1 1 1').split()])
        xyz,rot=_origin_of(c); origin=np.eye(4); origin[:3,:3]=rot;origin[:3,3]=xyz
        meshes.append((link,mesh,origin))
hits=[]
for link,mesh,origin in meshes:
    if link in ('elfin_link5','elfin_link6'): continue
    lines=[f'{len(mesh.vertices)} {len(mesh.faces)}']
    lines+=[' '.join(map(str,v)) for v in mesh.vertices]
    lines+=[' '.join(map(str,f)) for f in mesh.faces]
    cases=[]
    for sid,tf in transforms.items():
        for kind,(size,offset) in payloads.items():
            cases.append(' '.join(map(str,[f'{sid}:{kind}',*(tf[link]@origin).flat,*size,*(tf['suction_contact_frame']@offset).flat])))
    lines.append(str(len(cases)));lines+=cases
    result=subprocess.run(['/tmp/payload_contact_probe'],input='\n'.join(lines)+'\n',text=True,capture_output=True,check=True)
    for line in result.stdout.splitlines():
        label,hit,distance,n,*raw=line.split();sid,kind=label.split(':')
        if hit=='1':
            contacts=np.array(raw,dtype=float).reshape(-1,4).tolist()
            hits.append(dict(state_id=int(sid),kind=kind,link=link,contacts=contacts))
counts={k:{'states':len({h['state_id'] for h in hits if h['kind']==k}), 'by_link':dict(Counter(h['link'] for h in hits if h['kind']==k))} for k in payloads}
old=json.loads((RUN/'offline_anchor_payload_collisions.json').read_text())
assert counts['nominal']['states']==old['unique_hit_states']['nominal']==16
assert counts['full']['states']==old['unique_hit_states']['envelope']==20
result={'scope':'20 saved staging states; URDF root coordinates metres; no world or trajectory validity claim; FCL contacts are representative capped at 16 per mesh pair', 'counts':counts,'states':list(states.values()),'slot_mapping':mappings,'hits':hits}
(OUT/'ablation_contacts.json').write_text(json.dumps(result,indent=2)+'\n')
with (OUT/'slot_state_mapping.csv').open('w') as f:
    w=csv.writer(f);w.writerow(['slot_index','reason','saved_state_ids'])
    for r in mappings:w.writerow([r['slot_index'],r['reason'],' '.join(map(str,r['state_ids']))])
# Heatmap distinguishes repeated candidates from distinct configurations.
fig,axes=plt.subplots(1,2,figsize=(15,6),gridspec_kw={'width_ratios':[1,1.3]})
ks=list(payloads)
a=np.array([[sum(h['state_id']==i and h['kind']==k for h in hits) for k in ks] for i in range(1,21)])
im=axes[0].imshow(a,aspect='auto',vmin=0,vmax=2,cmap='YlOrRd');axes[0].set_xticks(range(len(ks)),ks,rotation=35,ha='right');axes[0].set_yticks(range(20),range(1,21));axes[0].set_ylabel('Saved staging state ID');axes[0].set_title('Colliding links among link1-4 (0, 1, 2)')
for (y,x),v in np.ndenumerate(a):axes[0].text(x,y,str(v),ha='center',va='center',fontsize=8)
for reason,color in [('EDGE_BUILD_FAILED','crimson'),('NO_FEASIBLE_ROUTE','royalblue')]:
    rs=[r for r in mappings if r['reason']==reason and r['slot_pose']]
    axes[1].scatter([r['slot_pose']['position'][0] for r in rs],[r['slot_pose']['position'][1] for r in rs],s=18,c=color,label=f'{reason}: {len(rs)}')
axes[1].set(xlabel='Slot X in container frame (m)',ylabel='Slot Y in container frame (m)',title=f'{len(mappings)} attempted slots; not contact locations')
axes[1].legend(fontsize=8);axes[1].set_aspect('equal');fig.tight_layout();fig.savefig(OUT/'ablation_slots.png',dpi=170);plt.close(fig)
def draw(ax,sid):
    tf=transforms[sid]; points=[]
    for link,mesh,origin in meshes:
        t=tf[link]@origin;verts=mesh.vertices@t[:3,:3].T+t[:3,3]
        # Full collision mesh drawn; opacity prevents hiding payload surfaces.
        color={'elfin_link1':'#d95319','elfin_link2':'#d49c00'}.get(link,'#8c98a5')
        ax.add_collection3d(Poly3DCollection(verts[mesh.faces],facecolor=color,edgecolor='none',alpha=.32))
        points.extend([verts.min(0),verts.max(0)])
    for kind,color in [('full','red'),('nominal','blue')]:
        size,off=payloads[kind]; t=tf['suction_contact_frame']@off
        verts=np.array(list(product((-1,1),repeat=3)))*np.array(size)/2
        verts=verts@t[:3,:3].T+t[:3,3]; points.extend(verts)
        for i in range(8):
            for j in range(i):
                if bin(i^j).count('1')==1:ax.plot(*verts[[i,j]].T,color=color,lw=.7)
    contacts=[c[:3] for h in hits if h['state_id']==sid and h['kind']=='full' for c in h['contacts']]
    if contacts:ax.scatter(*np.array(contacts).T,c='magenta',s=7,depthshade=False)
    p=np.array(points);center=(p.max(0)+p.min(0))/2;radius=max(p.max(0)-p.min(0))/2*1.05
    ax.set(xlim=(center[0]-radius,center[0]+radius),ylim=(center[1]-radius,center[1]+radius),zlim=(center[2]-radius,center[2]+radius))
    ax.set_box_aspect((1,1,1));ax.view_init(22,-55)
    nh=any(h['state_id']==sid and h['kind']=='nominal' for h in hits)
    ax.set_title(f'S{sid:02d}: nominal {"HIT" if nh else "clear"}, full HIT',fontsize=9)
    ax.tick_params(labelsize=6);ax.set_xlabel('X m',fontsize=7);ax.set_ylabel('Y m',fontsize=7);ax.set_zlabel('Z m',fontsize=7)
fig=plt.figure(figsize=(20,16))
for sid in range(1,21):draw(fig.add_subplot(4,5,sid,projection='3d'),sid)
fig.suptitle('Saved staging poses | blue: measured box | red: envelope | magenta: FCL contact samples\nOrange: link1; yellow: link2; grey: remaining arm. URDF root frame; offline poses, not executed motion.',fontsize=14)
fig.tight_layout(rect=(0,0,1,.95));fig.savefig(OUT/'all_20_staging.png',dpi=150);plt.close(fig)
clear=[i for i in range(1,21) if not any(h['state_id']==i and h['kind']=='nominal' for h in hits)]
fig=plt.figure(figsize=(13,6))
for pos,sid in enumerate([1,clear[0]],1):draw(fig.add_subplot(1,2,pos,projection='3d'),sid)
fig.suptitle('Blue: measured box | Red: full envelope | Magenta: sampled FCL contacts',fontsize=12)
fig.tight_layout();fig.savefig(OUT/'collision_examples.png',dpi=180);plt.close(fig)
print(json.dumps({'counts':counts,'attempted_slots':len(mappings),'states':len(states),'rows_without_states':sum(not r['state_ids'] for r in mappings)},indent=2))
