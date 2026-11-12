"""Bounded explanations of native route refusals, with no admission authority.

Object names identify model geometry only. They never establish observed cargo
ownership, physical contact, or permission to remove an obstacle. Complete CDR
requests/responses remain the replay evidence; this summary omits sample arrays.
"""
import math

from luggage_planning.joint_operation.contracts import OPERATION_JOINTS


# The six-leg operation contract and native contact response bound.
MAX_LEGS = 6
MAX_CONTACTS = 32


def _finite(value):
    number = float(value)
    return number if math.isfinite(number) else None


def _pose(pose):
    return dict(position=[_finite(getattr(pose.position, axis)) for axis in 'xyz'],
                quaternion_xyzw=[_finite(getattr(pose.orientation, axis)) for axis in 'xyzw'])


def _objects(scene, name):
    matches = []
    for obj in scene.world.collision_objects:
        if obj.id == name:
            matches.append(dict(representation='world_object', frame_id=obj.header.frame_id,
                primitives=[dict(type=shape.type,dimensions=list(shape.dimensions),pose=_pose(pose))
                    for shape,pose in zip(obj.primitives[:8],obj.primitive_poses[:8])],
                primitive_count=len(obj.primitives),mesh_count=len(obj.meshes)))
    for attached in scene.robot_state.attached_collision_objects:
        if attached.object.id == name:
            matches.append(dict(representation='attached_object',link_name=attached.link_name,
                                frame_id=attached.object.header.frame_id))
    # A missing world/attached match may be a robot link or an unknown body.
    # Do not reinterpret the native body_type integers as our message enum.
    return matches


def route_refusal_summary(request, response, *, model):
    summary = dict(scope='native model refusal, not measured physical collision',
        request_id=request.request_id,scene_digest=request.scene_digest,
        route_digest=request.route_digest,model_sha256=response.model_sha256,
        response_identity_matches=(response.request_id == request.request_id
            and response.scene_digest == request.scene_digest
            and response.route_digest == request.route_digest
            and response.model_sha256 == model.model_sha256),
        result=response.result,all_valid=response.all_valid,legs=[])
    for index, verdict in enumerate(response.leg_results[:MAX_LEGS]):
        if verdict.valid:
            continue
        leg = request.legs[index] if index < len(request.legs) else None
        matched = leg is not None and leg.edge_id == verdict.edge_id
        scene = (request.load_state_scenes[leg.load_state_index] if matched
                 and leg.load_state_index < len(request.load_state_scenes) else None)
        row = dict(edge_id=verdict.edge_id,reason=verdict.reason,detail=verdict.detail,
            first_invalid_sample=verdict.first_invalid_sample,n_samples=verdict.n_samples,
            load_state_index=leg.load_state_index if matched else None,
            scene_name=scene.name if scene is not None else None,
            support_contact_count=len(verdict.support_contacts),contacts=[],
            contact_count=len(verdict.contacts),contacts_truncated=len(verdict.contacts)>MAX_CONTACTS)
        if (matched and tuple(leg.joint_names) == OPERATION_JOINTS
                and verdict.first_invalid_sample < leg.n_samples):
            begin = verdict.first_invalid_sample * 6
            joints = tuple(leg.sample_positions_flat[begin:begin+6])
            if len(joints) == 6 and all(math.isfinite(q) for q in joints):
                row['sample_joints'] = list(joints)
                row['tcp_world_xyz'] = model.urdf.tcp(joints,model.path)[:3,3].tolist()
        for contact in verdict.contacts[:MAX_CONTACTS]:
            row['contacts'].append(dict(body_1=contact.body_1,body_2=contact.body_2,
                native_body_type_1=contact.body_type_1,native_body_type_2=contact.body_type_2,
                sample_index=contact.sample_index,depth_m=_finite(contact.depth),
                position=[_finite(getattr(contact.position,a)) for a in 'xyz'],
                body_1_scene_matches=[] if scene is None else _objects(scene,contact.body_1),
                body_2_scene_matches=[] if scene is None else _objects(scene,contact.body_2)))
        summary['legs'].append(row)
    summary['legs_truncated'] = len(response.leg_results)>MAX_LEGS
    return summary
