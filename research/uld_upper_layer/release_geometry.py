"""Source-derived initial geometry for independent release calibration fixtures.

Below-policy support ratios are deliberately diagnostic workload inputs. This
module never asserts physical stability or generates executable arm plans.
Unavailable source-tool/support geometry is distinct from physical failure.
"""
from dataclasses import dataclass,asdict
import numpy as np
from scipy.spatial.transform import Rotation
from shapely.geometry import box
from shapely.ops import unary_union
from candidates import flat,tool_collision
from geometry import pose,Box
from action import boundary_failure
from planning_types import digest,SupportMetrics
from support_geometry import measure_support


@dataclass(frozen=True)
class ReleaseGeometryCase:
    case_id: str
    requested_support_ratio: float
    box_yaw_deg: float
    repetition: int
    geometry_available: bool
    reason: str
    container_T_box: tuple | None
    container_T_tool: tuple | None
    expected_support: SupportMetrics
    source_geometry_id: str


def release_geometry(assets,snapshot,state,config,ratio,yaw,repetition):
    if not 0<ratio<=1 or isinstance(repetition,bool) or not isinstance(repetition,int) or repetition<0:
        raise ValueError('INVALID_RELEASE_WORKLOAD')
    if not np.isfinite(yaw) or not np.isclose(yaw%90.,0.,atol=1e-8):
        raise ValueError('RELEASE_FIXTURE_YAW_MUST_ALIGN_SUPPORT_GRID')
    source_id=digest(dict(assets=assets['hashes'],snapshot=snapshot.snapshot_id,config=config.revision,box=asdict(state)))
    identity=digest(dict(source=source_id,ratio=ratio,yaw=yaw,repetition=repetition))[:20]
    top=max(p.surface_z_container for p in snapshot.surfaces)
    patches=tuple(p for p in snapshot.surfaces if abs(p.surface_z_container-top)<=config.contact_tolerance_m)
    support=unary_union([box(*p.bounds_xy) for p in patches])
    edge=support.bounds[0]
    # Keep the footprint in one actual row where possible, so seams do not
    # silently substitute bounding-rectangle support for real support area.
    patch=min(patches,key=lambda p:(-box(*p.bounds_xy).area,p.source_id))
    lateral=(patch.bounds_xy[1]+patch.bounds_xy[3])/2
    rz=Rotation.from_euler('z',yaw,degrees=True).as_matrix()
    length,width=np.abs(rz[:2,:2])@np.array(state.dimensions[:2])
    front=edge+ratio*length
    box_pose=pose([front-length/2,lateral,top+state.dimensions[2]/2],
                  rz@Rotation.from_euler('x',np.pi).as_matrix())
    metrics=measure_support(box_pose,state.dimensions,patches,config,state.com_local)
    failure=boundary_failure(Box('release_payload',box_pose[:3,3],box_pose[:3,:3],
        np.array(state.dimensions)/2),assets,config.collision_margin_m)
    if failure:
        return ReleaseGeometryCase(identity,ratio,yaw,repetition,False,
            'PAYLOAD_HULL:'+failure['reason'],flat(box_pose),None,metrics,source_id)
    footprint=np.array(assets['contact']['footprint_size_xy_m'])
    boundary=assets['contact']['boundary_margin_m'];options=[]
    for tool_yaw in config.tool_yaws_deg:
        rotation=Rotation.from_euler('z',tool_yaw,degrees=True).as_matrix()
        projected=np.abs(rotation[:2,:2])@footprint
        if width<projected[1]+2*boundary or length<projected[0]+2*boundary:continue
        front_to_tool_center=length-projected[0]/2-boundary
        tcp=pose([front-front_to_tool_center,lateral,top],rotation)
        failure=tool_collision(assets,snapshot,tcp,config.collision_margin_m)
        if failure:continue
        actual_forward=max(body.moved(tcp).corners()[:,0].max() for body in assets['tool'])
        gap=edge-actual_forward
        if gap<config.collision_margin_m-1e-9:continue
        options.append((gap,float(tool_yaw),tcp))
    if not options:
        return ReleaseGeometryCase(identity,ratio,yaw,repetition,False,
            'NO_SOURCE_TOOL_CLEARANCE_FOR_LEGAL_FOOTPRINT',flat(box_pose),None,metrics,source_id)
    _,_,tcp=max(options,key=lambda row:(row[0],-row[1]))
    return ReleaseGeometryCase(identity,ratio,yaw,repetition,True,
        'GEOMETRY_ONLY_PHYSICS_NOT_EVALUATED',flat(box_pose),flat(tcp),metrics,source_id)


def release_matrix(assets,snapshot,state,config,ratios,yaws,repetitions):
    return tuple(release_geometry(assets,snapshot,state,config,float(ratio),float(yaw),repeat)
                 for ratio in ratios for yaw in yaws for repeat in range(repetitions))
