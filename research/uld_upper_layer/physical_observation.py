"""Read-only native simulator observations for the ideal-input experiment.

All public poses/points/normals below are container-absolute. A payload and
its six measured joints share the native physics-step stamp. Lower-cargo and
contact streams carry their own checked stamps; receipt time is never used.
"""
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation


def pose_matrix(row):
    result=np.eye(4);p=row['position'];q=row['orientation']
    result[:3,3]=[p.get(k,0.) for k in ('x','y','z')]
    result[:3,:3]=Rotation.from_quat([q.get(k,0.) for k in ('x','y','z')]+[q.get('w',1.)]).as_matrix()
    if not np.isfinite(result).all():raise ValueError('NONFINITE_OBSERVED_POSE')
    return result


@dataclass(frozen=True)
class ObservedBody:
    name: str
    dimensions: tuple
    container_T_body: tuple
    stamp_s: float


@dataclass(frozen=True)
class ObservedContact:
    other_collision: str
    point_container: tuple
    normal_toward_payload_container: tuple
    depth_m: float


@dataclass(frozen=True)
class PhysicalSample:
    stamp_s: float
    q: tuple
    container_T_box: tuple
    container_T_tool: tuple
    lower: tuple
    contacts: tuple
    contact_stamp_s: float
    attached: bool
    pose_command_count: int
    processed_commands: int
    payload_pose_command_component_present: bool | None = None
    tool_T_box_at_attach: tuple | None = None
    attachment_joint_component_present: bool | None = None


@dataclass(frozen=True)
class CollisionContact:
    other_collision: str
    point_container: tuple
    normal_toward_observed_container: tuple
    depth_m: float


def _decode_collision_contacts(message,identity,container_T_world,*,exact):
    contacts=[]
    for contact in message.get('contact',[]):
        a=contact['collision1']['name'];b=contact['collision2']['name']
        first=a==identity if exact else a.startswith(identity+'::')
        second=b==identity if exact else b.startswith(identity+'::')
        if first==second:
            raise ValueError('CONTACT_COLLISION_IDENTITY_MISMATCH' if exact else 'CONTACT_PAYLOAD_IDENTITY_MISMATCH')
        points=contact.get('position',[]);normals=contact.get('normal',[]);depths=contact.get('depth',[])
        if not points or not len(points)==len(normals)==len(depths):
            raise ValueError('NATIVE_CONTACT_GEOMETRY_INCOMPLETE')
        for point,normal,depth in zip(points,normals,depths):
            point=np.array([point.get(k,0.) for k in ('x','y','z')])
            normal=np.array([normal.get(k,0.) for k in ('x','y','z')])*(1 if first else -1)
            if not np.isfinite(np.r_[point,normal,depth]).all() or abs(np.linalg.norm(normal)-1)>1e-4:
                raise ValueError('INVALID_NATIVE_CONTACT_GEOMETRY')
            point=container_T_world@np.r_[point,1.]
            normal=container_T_world[:3,:3]@normal
            contacts.append(CollisionContact(b if first else a,tuple(point[:3]),tuple(normal),float(depth)))
    return tuple(contacts)


def read_payload_contacts(message,box_id,container_T_world):
    """Decode signed native contact geometry once for arm and carriage readers."""
    return tuple(ObservedContact(c.other_collision,c.point_container,c.normal_toward_observed_container,c.depth_m)
        for c in _decode_collision_contacts(message,box_id,container_T_world,exact=False))


def read_exact_collision_contacts(message,collision_name,container_T_world):
    """Decode one exact tool collision identity, with signed inward normals."""
    return _decode_collision_contacts(message,collision_name,container_T_world,exact=True)



class ObservationReader:
    def __init__(self,assets,snapshot,box_id,max_age_s):
        if not np.isfinite(max_age_s) or max_age_s<=0:raise ValueError('INVALID_OBSERVATION_MAX_AGE')
        self._container_T_world=np.linalg.inv(assets['world_container'])
        urdf=assets['urdf'];zero=[0.]*6
        self._link_T_tool=np.linalg.inv(urdf.tcp(zero,urdf.path_to('elfin_link6')))@urdf.tcp(zero,urdf.path_to('suction_contact_frame'))
        self._lower=tuple((body.name,body.size) for body in snapshot.occupied)
        self._box_id=box_id;self._max_age_s=max_age_s

    def read(self,row):
        if row.get('capture_error'):raise ValueError('PHYSICAL_CAPTURE_ERROR:'+row['capture_error'])
        native=row.get('vacuum')
        if not native or not native.get('ready') or not native.get('joint_positions_complete'):
            raise ValueError('NATIVE_PHYSICAL_STATE_INCOMPLETE')
        stamp=float(native['sim_time_s']);q=tuple(native['q'])
        if len(q)!=6 or not np.isfinite(q).all() or not np.isfinite(stamp):
            raise ValueError('INVALID_NATIVE_JOINTS_OR_STAMP')
        def age(other,name):
            if other is None or not np.isfinite(other) or abs(stamp-other)>self._max_age_s:
                raise ValueError('STALE_PHYSICAL_STREAM:'+name)
        contact_message=row.get('contacts') or {};contact_stamp=row.get('contact_stamp')
        if 'contacts_component_available' in native:
            if not native['contacts_component_available'] or 'payload_contacts' not in native:
                raise ValueError('NATIVE_PHYSICAL_CONTACT_COMPONENT_UNAVAILABLE')
            contact_message=native['payload_contacts'];contact_stamp=stamp
        if 'native_latest_stamp_s' in row:age(row['native_latest_stamp_s'],'native_processing_backlog')
        age(row.get('clock'),'clock');age(contact_stamp,'contacts')
        lower=[]
        for name,dimensions in self._lower:
            age(row.get('pose_stamps',{}).get(name),name)
            if name not in row['poses']:raise ValueError('MISSING_LOWER_BODY:'+name)
            lower.append(ObservedBody(name,dimensions,tuple((self._container_T_world@pose_matrix(row['poses'][name])).ravel()),row['pose_stamps'][name]))
        contacts=read_payload_contacts(contact_message,self._box_id,self._container_T_world)
        box=self._container_T_world@pose_matrix(native['payload_pose_world'])
        tool=self._container_T_world@pose_matrix(native['link_pose_world'])@self._link_T_tool
        relation=None
        if 'link_T_payload_xyz' in native and 'link_T_payload_quat_xyzw' in native:
            link_T_box=np.eye(4);link_T_box[:3,3]=native['link_T_payload_xyz']
            link_T_box[:3,:3]=Rotation.from_quat(native['link_T_payload_quat_xyzw']).as_matrix()
            relation=tuple((np.linalg.inv(self._link_T_tool)@link_T_box).ravel())
        return PhysicalSample(stamp,q,tuple(box.ravel()),tuple(tool.ravel()),tuple(lower),tuple(contacts),
                              float(contact_stamp),bool(native['attached']),
                              int(native['pose_command_count']),int(native['processed_commands']),
                              native.get('payload_pose_command_component_present'),relation,
                              native.get('attachment_joint_component_present'))
