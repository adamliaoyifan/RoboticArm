"""ROS bindings for the observed top-domain contract; no scene/map writes."""
import numpy as np

from luggage_perception.observed_suction_domain import ObservedSuctionDomain


def acquisition_metadata_from_message(message):
    """Identity only, also available when this frame has no fitted domain."""
    return dict(box_id=message.box_id, instance_id=message.instance_id,
        generation=int(message.generation),
        stamp_ns=int(message.header.stamp.sec)*10**9+int(message.header.stamp.nanosec),
        frame_id=message.header.frame_id, contact_model_hash=message.contact_model_hash)


def domain_to_message(record):
    from luggage_msgs.msg import SuctionTopDomainV2
    result = SuctionTopDomainV2()
    result.header.frame_id = record.frame_id
    result.header.stamp.sec,result.header.stamp.nanosec = divmod(record.stamp_ns,10**9)
    result.box_id,result.instance_id,result.generation = record.box_id,record.instance_id,record.generation
    result.contact_model_version,result.contact_model_hash = record.contact_model_version,record.contact_model_hash
    result.valid,result.reason,result.domain_sha256 = True,'DIRECTLY_OBSERVED',record.digest()
    p,q = result.box_pose_world.position,result.box_pose_world.orientation
    p.x,p.y,p.z = record.box_xyz_world
    q.x,q.y,q.z,q.w = record.box_quaternion_xyzw
    for name in ('box_wdh','u_axis_world','v_axis_world','normal_world','origin_uv_top_m'):
        setattr(result,name,list(getattr(record,name)))
    result.plane_offset_world_m,result.cell_size_m = record.plane_offset_world_m,record.cell_size_m
    result.size_u,result.size_v = record.shape
    result.support = list(record.support)
    return result


def domain_from_message(message):
    if message.valid is not True or message.reason != 'DIRECTLY_OBSERVED':
        raise ValueError('valid directly observed top domain required')
    p,q = message.box_pose_world.position,message.box_pose_world.orientation
    record = ObservedSuctionDomain(message.box_id,message.instance_id,int(message.generation),
        int(message.header.stamp.sec)*10**9+int(message.header.stamp.nanosec),
        message.header.frame_id,int(message.contact_model_version),message.contact_model_hash,
        (p.x,p.y,p.z),(q.x,q.y,q.z,q.w),tuple(message.box_wdh),tuple(message.u_axis_world),
        tuple(message.v_axis_world),tuple(message.normal_world),message.plane_offset_world_m,
        tuple(message.origin_uv_top_m),message.cell_size_m,(int(message.size_u),int(message.size_v)),
        bytes(message.support))
    if message.domain_sha256 != record.digest():
        raise ValueError('observed top domain digest mismatch')
    return record


def observed_domain_for_detection(detection, evaluation):
    """Bind the exported domain to this exact measured DetectedLuggage.

    A centre-only evaluation or absent/full-height prior cannot create support.
    Float stamps below only compare algorithm provenance; the record retains
    the original integer nanoseconds from the acquisition header.
    """
    domain = getattr(evaluation,'observed_top_domain',None)
    stamp_ns = int(detection.header.stamp.sec)*10**9+int(detection.header.stamp.nanosec)
    stamp = float(detection.header.stamp.sec)+float(detection.header.stamp.nanosec)*1e-9
    if (evaluation is None or not evaluation.ok or domain is None
            or not detection.top_surface_valid or not detection.height_valid
            or detection.height_source != detection.HEIGHT_SOURCE_MEASURED_SUPPORT
            or detection.header.frame_id != 'world' or evaluation.frame != 'world'
            or domain.frame_id != 'world' or evaluation.stamp != stamp or domain.stamp != stamp):
        raise ValueError('exact-acquisition measured geometry and observed domain required')
    cells = np.asarray(domain.valid_cells)
    if cells.dtype != np.bool_ or cells.ndim != 2:
        raise ValueError('boolean whole-cell observed support required')
    p,q = detection.pose.position,detection.pose.orientation
    return ObservedSuctionDomain(detection.id,evaluation.instance_id,int(evaluation.generation),
        stamp_ns,'world',int(evaluation.model_version),evaluation.model_hash,
        (p.x,p.y,p.z),(q.x,q.y,q.z,q.w),(detection.width,detection.depth,detection.height),
        domain.u_axis_world,domain.v_axis_world,domain.normal_world,domain.plane_offset_world_m,
        domain.origin_uv,domain.cell_size_m,tuple(int(v) for v in cells.shape),
        cells.astype(np.uint8,copy=False).tobytes())


def require_matching_detection(record, detection, *, generation, contact_model_hash):
    """Consumer binds this domain to an independently supplied observation."""
    p,q = detection.pose.position,detection.pose.orientation
    stamp_ns = int(detection.header.stamp.sec)*10**9+int(detection.header.stamp.nanosec)
    if (not detection.top_surface_valid or not detection.height_valid
            or detection.height_source != detection.HEIGHT_SOURCE_MEASURED_SUPPORT
            or (record.box_id,record.generation,record.stamp_ns,record.frame_id,record.contact_model_hash) !=
               (detection.id,generation,stamp_ns,detection.header.frame_id,contact_model_hash)
            or record.box_xyz_world != (p.x,p.y,p.z)
            or record.box_quaternion_xyzw != (q.x,q.y,q.z,q.w)
            or record.box_wdh != (detection.width,detection.depth,detection.height)):
        raise ValueError('observed domain and independent detection disagree')
