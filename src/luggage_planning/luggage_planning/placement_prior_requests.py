"""ROS request decoding and choice encoding, separate from background execution."""
import json
import math

from luggage_planning.placement_prior_contracts import (
    Transform, PlacementCandidate, GraspCandidate, PriorRequest,
)
from luggage_planning.placement_prior_pick import candidates_for_prior, transform_from_pose


def measured_detection_view(box):
    from luggage_planning.pickup_collision import pickup_collision_aabb
    from luggage_planning.ros_message_adapters import (
        detected_observation_identity, suction_candidates_from_detected,
    )
    from luggage_planning.suction_pick_session import DetectionView
    xyz, quat, size = pickup_collision_aabb(box)  # measured-support admission
    stamp, frame = detected_observation_identity(box)
    x,y,z,w = quat
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return DetectionView(stamp=stamp, frame=frame, candidates=suction_candidates_from_detected(box),
        top_surface_valid=bool(box.top_surface_valid), detection_yaw=yaw,
        yaw_valid=bool(box.yaw_valid), box_xyz=tuple(xyz), box_quat=tuple(quat),
        box_size=tuple(size), stamp_ns=int(box.header.stamp.sec)*1000000000+int(box.header.stamp.nanosec))


def enumerate_placements(confirmed, box_wdh, max_slots):
    from luggage_packing.placement_solver import generate_candidates
    from luggage_planning.occupancy_place_paths import (
        OccupancySnapshot, _legacy_view_of_surface_contract,
    )
    record = confirmed.record()
    metadata = json.loads(confirmed.confirmation_json)
    inner_size = metadata.get('inner_size')
    descriptor = metadata.get('geometry_descriptor')
    # Admit the exact hull identity before using its metadata. Grid values and
    # height datum come exclusively from the admitted typed publication.
    OccupancySnapshot.from_surface_2d(
        record, expected_hash=record['geometry_hash'],
        expected_revision=record['map_revision'], inner_size=inner_size,
        geometry_descriptor=descriptor)
    surface = _legacy_view_of_surface_contract(record, inner_size, descriptor)
    # The current packing solver indexes a grid starting at -inner_size/2.
    # Refuse other origins instead of silently shifting every candidate.
    if any(abs(float(origin) + float(size) / 2) > 1e-9
           for origin, size in zip(record['origin_xy_container'], inner_size[:2])):
        raise ValueError('placement solver requires a centered container grid')
    # Existing solver performs support/hull/clearance checks. No catalogue or
    # perfect geometry enters the production ComputePlacement service.
    candidates = generate_candidates(surface, box_wdh, params={'top_n':max_slots})
    result = []
    for candidate in candidates:
        if not candidate['feasible']:
            continue
        yaw=float(candidate['heading_theta'])
        result.append(PlacementCandidate(str(candidate['candidate_id']),
            Transform(candidate['center_container'],(0,0,math.sin(yaw/2),math.cos(yaw/2)))))
        if len(result)>=max_slots:
            break
    return tuple(result)


def decode_request(message, snapshot, confirmed, *, fallback_yaw=0.0, prefetched=None):
    from luggage_description.frame_diagnostics import BoundaryContext
    from luggage_description.ros_frame_bindings.poses import pose_from_message
    if not message.request_id or not 1 <= message.max_slots <= 32:
        raise ValueError('request ID and max_slots in [1,32] required')
    if message.geometry_hash != snapshot.geometry_hash or message.expected_map_revision != snapshot.map_revision:
        raise ValueError('snapshot identity mismatch')
    if message.measured:
        detection=measured_detection_view(message.detection)
        dimensions=detection.box_size
        grasps=candidates_for_prior(detection,fallback_yaw=fallback_yaw)
    else:
        dimensions=tuple(message.template_box_wdh)
        grasps=tuple(GraspCandidate('template-%d'%i,transform_from_pose(p),
                    'hypothesis:'+message.request_id,'hypothetical',i)
                    for i,p in enumerate(message.template_box_from_tcp))
        if not grasps:
            raise ValueError('explicit template grasps required')
    placements=[]
    for i,slot in enumerate(message.slots):
        if len(placements)>=message.max_slots:
            break
        context=BoundaryContext('PRIOR_SLOT','caller','placement_prior',slot.identity.request_id,
            snapshot.scene_revision,'container_link','box_center','scene')
        record=pose_from_message(slot,'slot',context)
        placements.append(PlacementCandidate('requested-%d'%i,
            Transform(record['position'],record['quaternion_xyzw'])))
    if not placements:
        from luggage_planning.placement_prior_prefetch import reusable_placements
        placements = reusable_placements(prefetched, snapshot, dimensions, message.max_slots)
    if not placements:
        placements=enumerate_placements(confirmed,dimensions,message.max_slots)
    if not placements:
        raise ValueError('no geometric placement candidate')
    return PriorRequest(message.request_id,snapshot,dimensions,tuple(placements),grasps,
                        bool(message.measured),int(message.candidate_limit),float(message.budget_s))


def encode_choices(result, request):
    from geometry_msgs.msg import Pose
    from luggage_msgs.msg import PlacementPriorChoiceV2
    from luggage_planning.place_service import pose_message
    def record(transform,reference,semantics,frame):
        return dict(contract_version=2,frame_id=frame,length_unit='m',stamp_ns=0,
                    time_basis='scene',scene_revision=request.snapshot.scene_revision,
                    request_id=request.request_id,reference=reference,
                    orientation_semantics=semantics,position=transform.translation,
                    quaternion_xyzw=transform.quaternion_xyzw)
    choices=[]
    for choice in result.choices:
        message=PlacementPriorChoiceV2();message.slot_id=choice.placement.slot_id
        message.slot=pose_message(record(choice.placement.container_from_box,'box_center','box_axis_pi','container_link'))
        message.candidate_id=choice.grasp.candidate_id
        message.observation_id=choice.grasp.observation_id
        message.contact_model_hash=choice.grasp.contact_model_hash
        message.grasp=pose_message(record(choice.grasp.box_from_tcp,'suction_contact_frame','box','box'))
        for transform in choice.tcp_references:
            pose=Pose();pose.position.x,pose.position.y,pose.position.z=transform.translation
            pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=transform.quaternion_xyzw
            message.tcp_references_world.append(pose)
        choices.append(message)
    return choices
