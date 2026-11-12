"""Measured top-column coverage and pinhole ranking, without ROS or map writes.

All coordinates here are container-absolute or explicitly world qualified.
Projected FOV coverage is a ranking proxy; only actual O columns pass adoption.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def release_coverage(surface,container_from_box,dimensions):
    rotation=Rotation.from_quat(container_from_box.quaternion_xyzw).as_matrix()
    if np.max(abs(rotation[:,2]-np.array((0.,0.,1.))))>1e-9:
        raise ValueError('OBSERVED_RELEASE_TARGET_MUST_BE_UPRIGHT_IN_CONTAINER')
    width,depth,height=dimensions
    top=container_from_box.translation[2]+height/2
    resolution=surface['resolution_m']
    states=np.asarray(surface['state'])
    nx,ny=states.shape
    x=surface['origin_xy_container'][0]+(np.arange(nx)+.5)*resolution
    y=surface['origin_xy_container'][1]+(np.arange(ny)+.5)*resolution
    xx,yy=np.meshgrid(x,y,indexing='ij')
    points=np.stack((xx,yy,np.full_like(xx,top)),axis=-1)
    local=(points-np.asarray(container_from_box.translation))@rotation
    footprint=(abs(local[:,:,0])<=width/2)&(abs(local[:,:,1])<=depth/2)
    observed=np.asarray(surface['surface_z_container'],dtype=float)
    matches=footprint&(states=='occupied')&np.isfinite(observed)&(abs(observed-top)<=resolution+1e-9)
    count=int(footprint.sum());matched=int(matches.sum())
    return dict(accepted=count>0 and count==matched,expected_columns=count,matched_columns=matched,
        missing_points_container=points[footprint&~matches].tolist(),
        expected_top_z_container=top,matched_area_m2=matched*resolution**2,
        state_counts={str(state):int(np.count_nonzero(footprint&(states==state)))
                      for state in np.unique(states[footprint])})


def projected_missing_columns(points_container,world_from_container,world_from_camera,intrinsics):
    """Count source-cell centres projected inside the calibrated camera image.

    This does not assume unobserved space is empty or authorize a motion.
    Occlusion and actual sensor success are resolved after a checked view move.
    """
    points=np.asarray(points_container,dtype=float).reshape(-1,3)
    if not len(points):return 0
    def matrix(transform):
        result=np.eye(4);result[:3,:3]=Rotation.from_quat(transform.quaternion_xyzw).as_matrix()
        result[:3,3]=transform.translation;return result
    camera_from_container=np.linalg.inv(matrix(world_from_camera))@matrix(world_from_container)
    camera=points@camera_from_container[:3,:3].T+camera_from_container[:3,3]
    forward=camera[:,2]>0
    camera=camera[forward]
    if not len(camera):return 0
    u=intrinsics.fx*camera[:,0]/camera[:,2]+intrinsics.cx
    v=intrinsics.fy*camera[:,1]/camera[:,2]+intrinsics.cy
    return int(np.count_nonzero((u>=0)&(u<intrinsics.width)&(v>=0)&(v<intrinsics.height)))


def camera_aimed_at_missing(world_camera_position,points_container,world_from_container):
    """Aim a proposed optical +Z axis at the measured missing-footprint centre."""
    from luggage_planning.container_aim_utils import look_at_quaternion
    from luggage_planning.placement_prior.contracts import Transform
    points=np.asarray(points_container,dtype=float).reshape(-1,3)
    if not len(points):raise ValueError('MISSING_FOOTPRINT_TARGET_REQUIRED')
    centre=world_from_container.compose(Transform(tuple(points.mean(axis=0)),(0.,0.,0.,1.))).translation
    return Transform(tuple(world_camera_position),tuple(look_at_quaternion(world_camera_position,centre)))
