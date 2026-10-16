"""Connect prior choices to the existing measured suction-candidate pick path.

Candidate contact orientation describes an OUTWARD surface normal, not TCP
orientation. Always derive G from the actual attach segment built by the same
waypoint builder used by the hardware driver. Never reuse the raw contact
quaternion as a tool quaternion, or rewrite a validated candidate in place.
"""
from dataclasses import replace
import hashlib
import json

from luggage_planning.placement_prior_contracts import GraspCandidate, Transform
from luggage_planning.suction_candidate_waypoints import build_candidate_pick_segments


def transform_from_pose(pose):
    p, q = pose.position, pose.orientation
    return Transform((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))


def candidates_for_prior(detection, *, fallback_yaw=0.0):
    if detection.frame != 'world' or not detection.top_surface_valid:
        raise ValueError('world-frame measured suction observation required')
    if detection.stamp_ns is None or detection.stamp_ns < 0:
        raise ValueError('exact observation stamp required')
    if not detection.candidates or min(detection.box_size) <= 0:
        raise ValueError('measured geometry and suction candidates required')
    world_box = Transform(detection.box_xyz, detection.box_quat)
    result = []
    first = detection.candidates[0]
    identity = (first.instance_id, first.generation, first.model_hash, first.model_version)
    for candidate in detection.candidates:
        current = (candidate.instance_id, candidate.generation,
                   candidate.model_hash, candidate.model_version)
        if current != identity or candidate.frame != detection.frame or candidate.stamp != detection.stamp:
            raise ValueError('suction observation identity mismatch')
        segments = build_candidate_pick_segments(
            candidate, detection_yaw=detection.detection_yaw,
            yaw_valid=detection.yaw_valid, fallback_yaw=fallback_yaw)
        attach = next(segment for segment in segments if segment.name == 'attach')
        world_tcp = transform_from_pose(attach.target_pose)
        grasp = world_box.inverse().compose(world_tcp)
        # Pin directed box orientation and exact observation identity. Same ID
        # with changed contact geometry cannot accidentally reuse a previous G.
        material = (identity, detection.stamp_ns, detection.frame,
                    detection.box_xyz, detection.box_quat, detection.box_size,
                    detection.detection_yaw, detection.yaw_valid, fallback_yaw)
        observation_id = hashlib.sha256(json.dumps(material, sort_keys=True).encode()).hexdigest()
        result.append(GraspCandidate(candidate.candidate_id, grasp, observation_id,
                                     candidate.model_hash, int(candidate.rank)))
    return tuple(result)


def bind_choice_to_pick(choice, detection, *, fallback_yaw=0.0):
    """Restrict this pick to the checked candidate without changing its metadata.

    No generic fallback is allowed to keep this same slot/grasp binding after a
    different candidate is picked. A retry on another candidate must select its
    own prior choice or return to the ordinary planning path.
    """
    current = candidates_for_prior(detection, fallback_yaw=fallback_yaw)
    if choice.grasp not in current:
        raise ValueError('prior grasp no longer matches observation')
    selected = tuple(c for c in detection.candidates if c.candidate_id == choice.grasp.candidate_id)
    if len(selected) != 1:
        raise ValueError('ambiguous selected suction candidate')
    return replace(detection, candidates=selected)
