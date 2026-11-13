"""ROS request decoding and choice encoding, separate from background execution."""
import json
import math

from luggage_planning.placement_prior.contracts import (
    Transform, PlacementCandidate, GraspCandidate, PriorRequest,
)
from luggage_planning.placement_prior.pick import candidates_for_prior, transform_from_pose


def measured_detection_view(box):
    from luggage_planning.pick.pickup_collision import pickup_collision_aabb
    from luggage_planning.adapters.ros_message_adapters import (
        detected_observation_identity, suction_candidates_from_detected,
    )
    from luggage_planning.pick.suction_pick_session import DetectionView
    xyz, quat, size = pickup_collision_aabb(box)  # measured-support admission
    stamp, frame = detected_observation_identity(box)
    x,y,z,w = quat
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return DetectionView(stamp=stamp, frame=frame, candidates=suction_candidates_from_detected(box),
        top_surface_valid=bool(box.top_surface_valid), detection_yaw=yaw,
        yaw_valid=bool(box.yaw_valid), box_xyz=tuple(xyz), box_quat=tuple(quat),
        box_size=tuple(size), stamp_ns=int(box.header.stamp.sec)*1000000000+int(box.header.stamp.nanosec))


def _placement_surface(confirmed):
    """Validate the typed surface before converting its height datum."""
    from luggage_planning.place.occupancy_place_paths import (
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
    return surface, descriptor


def admit_placement_targets(confirmed, box_wdh, targets, *, volume, on_verdict=None):
    """Admit actual poses; optionally emit the same predicate's bounded verdict.

    ``on_verdict`` is synchronous evidence capture, never another permission.
    Capture failures propagate. The existing bool tuple remains the authority.
    """
    from luggage_packing.selected_pose_admission import SelectedPoseAdmission
    from luggage_packing.placement_volume_query import KnownFreeVolume, evidence_from_volume
    surface, _ = _placement_surface(confirmed)
    admission = SelectedPoseAdmission(surface, box_wdh,
        volume_verifier=KnownFreeVolume(evidence_from_volume(volume)))
    accepted = []
    for index, target in enumerate(targets):
        detail = dict(schema='selected_pose_admission_v1', target_index=index,
            center_container=list(target.translation),
            nominal_dimensions_wdh=list(box_wdh),
            quaternion_xyzw_container=list(target.quaternion_xyzw)) if on_verdict is not None else None
        x,y,z,w = target.quaternion_xyzw
        if abs(x) > 1e-8 or abs(y) > 1e-8:
            reason = 'nonhorizontal_placement_target'
        else:
            try:
                candidate = admission.candidate(target.translation, 2*math.atan2(z,w))
                if detail is not None:
                    detail.update(admission.explain(candidate))
                    reason = detail['reason']
                else:
                    reason = admission(candidate)
            except (ValueError, TypeError) as error:
                reason = 'invalid_placement_target'
                if detail is not None:
                    detail['error_type'] = type(error).__name__
        accepted.append(reason is None)
        if detail is not None:
            detail.update(accepted=reason is None, reason=reason)
            on_verdict(detail)
    return tuple(accepted)


def enumerate_placements(confirmed, box_wdh, max_slots, *, volume=None, margin=.05,
                         ranking_geometry=None, release_gap_m=0., vertical_margin=None,
                         approach_clearance_m=0.):
    from luggage_packing.placement_query import placement_query
    from luggage_packing.placement_solver import retain_directed_candidates
    from luggage_packing.placement_volume_query import KnownFreeVolume, evidence_from_volume
    surface, descriptor = _placement_surface(confirmed)
    result = placement_query(surface, box_wdh,
        volume_verifier=KnownFreeVolume(evidence_from_volume(volume)))
    candidates = [candidate for candidate in result['candidates'] if candidate['feasible']]
    if volume is not None:
        from luggage_planning.placement_prior.ranking import rank_payload_clearance
        from luggage_description.container_geometry import normalize_descriptor
        candidates = rank_payload_clearance(candidates, box_wdh, volume, margin,
            hull=normalize_descriptor(descriptor) if ranking_geometry is not None else None,
            geometry=ranking_geometry,release_gap_m=release_gap_m,vertical_margin=vertical_margin,
            approach_clearance_m=approach_clearance_m)
    candidates = retain_directed_candidates(candidates, max_slots)
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


def decode_request(message, snapshot, confirmed, *, fallback_yaw=0.0, prefetched=None,
                   volume=None, margin=.05):
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
        from luggage_planning.placement_prior.prefetch import reusable_placements
        placements = reusable_placements(prefetched, snapshot, dimensions, message.max_slots)
    if not placements:
        placements=enumerate_placements(confirmed,dimensions,message.max_slots,
                                       volume=volume,margin=margin)
        from luggage_planning.placement_prior.prefetch import rank_regenerated_placements
        placements=rank_regenerated_placements(prefetched,snapshot,placements)
    if not placements:
        raise ValueError('no geometric placement candidate')
    return PriorRequest(message.request_id,snapshot,dimensions,tuple(placements),grasps,
                        bool(message.measured),int(message.candidate_limit),float(message.budget_s),
                        Transform(detection.box_xyz,detection.box_quat) if message.measured else None)


def encode_choices(result, request):
    from geometry_msgs.msg import Pose
    from luggage_msgs.msg import PlacementPriorChoiceV2
    from luggage_planning.place.place_service import pose_message
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
