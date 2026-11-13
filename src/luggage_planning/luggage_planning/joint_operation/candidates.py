"""Build exact joint-operation candidates from observed directed contacts.

Target hints select where to ask perception, never where to command a wrist.
Only the returned actual contact pose defines G and the frozen payload.
"""
from dataclasses import dataclass
import hashlib
import math

import numpy as np

from luggage_perception.observed_contact_footprint import (
    check_observed_contact_footprint, project_contact_footprint,
)
from luggage_planning.joint_operation.contacts import grasp_for_contact
from luggage_planning.joint_operation.planner import OperationCandidate
from luggage_planning.joint_operation.scenes import build_operation_scenes
from luggage_planning.placement_prior.contracts import PlacementCandidate, Transform
from luggage_description.frame_transforms import rotate_vector
from luggage_planning.place.waypoint_generator import (
    DEFAULT_PLACE_CLEARANCE_Z, EXIT_EXTRA_LIFT_M, PLACE_RELEASE_GAP_M, corridor_clearance,
)
from luggage_planning.planning_contract.attachment_geometry import freeze_geometry


@dataclass(frozen=True)
class ContactProposal:
    query_id: str
    center_uv_top_m: tuple
    yaw_rad: float
    checked_centres: int
    reason: str

    def to_message(self):
        from luggage_msgs.msg import DirectedSuctionQueryV2
        if self.reason != 'OBSERVED_FOOTPRINT_PROPOSAL':
            raise ValueError('complete geometric proposal required')
        return DirectedSuctionQueryV2(query_id=self.query_id,
            center_uv_top_m=list(self.center_uv_top_m),yaw_rad=self.yaw_rad)


def _contact_matrix(domain, uv, yaw):
    normal = np.asarray(domain.normal_world)
    x = math.cos(yaw)*np.asarray(domain.u_axis_world)+math.sin(yaw)*np.asarray(domain.v_axis_world)
    matrix = np.eye(4)
    matrix[:3,:3] = np.column_stack((x,-np.cross(normal,x),-normal))
    matrix[:3,3] = domain.world_point(uv)
    return matrix


def centered_ranking_geometry(domain, limits):
    """Ranking proposal with the observed normal; never suction permission.

    The eventual offset and full footprint still come from perception's directed
    contact evaluation. Tiny measured tilt must not be replaced by a perfect
    vertical relation when proposing the existing conservative envelope.
    """
    box = Transform(domain.box_xyz_world, domain.box_quaternion_xyzw)
    uv = np.asarray((domain.u_axis_world, domain.v_axis_world)) @ np.asarray(domain.box_xyz_world)
    contact = Transform.from_matrix(_contact_matrix(domain, uv, 0.))
    tool_box = box.inverse().compose(contact).inverse()
    return freeze_geometry(domain.box_wdh,tool_box.translation,tool_box.quaternion_xyzw,limits)


def propose_contact(domain, model, *, query_id, world_from_place_box,
                    desired_world_from_tool, max_center_checks):
    """Back-project a target-side tool hint into an actual observed pick face.

    max_center_checks bounds geometric work, not permitted missing depth.
    No quality score or IK permission is fabricated, including on success.
    """
    if (not query_id or type(max_center_checks) is not int or not 1 <= max_center_checks <= 4096
            or type(world_from_place_box) is not Transform or type(desired_world_from_tool) is not Transform):
        raise ValueError('explicit target/hint and bounded centre search required')
    world_pick_box = Transform(domain.box_xyz_world,domain.box_quaternion_xyzw)
    predicted_pick_tool = world_pick_box.compose(world_from_place_box.inverse()).compose(desired_world_from_tool)
    basis = np.asarray((domain.u_axis_world,domain.v_axis_world))
    desired_uv = basis @ predicted_pick_tool.translation
    direction = basis @ np.asarray(_rotation_x(predicted_pick_tool))
    if np.linalg.norm(direction) <= 1e-12:
        return ContactProposal(query_id,(),0.,0,'TOOL_HINT_DIRECTION_DEGENERATE')
    yaw = math.atan2(direction[1],direction[0])
    # Relative footprint extents in the observed plane. This includes the
    # configured physical margin, not any fitted or inferred missing support.
    vertices = project_contact_footprint(domain,model,_contact_matrix(domain,(0.,0.),yaw))
    if vertices is None:
        return ContactProposal(query_id,(),yaw,0,'FOOTPRINT_PROJECTION_DEGENERATE')
    lower = np.asarray(domain.origin_uv_top_m)-vertices.min(axis=0)
    upper = (np.asarray(domain.origin_uv_top_m)+np.asarray(domain.shape)*domain.cell_size_m
             -vertices.max(axis=0))
    if np.any(lower > upper):
        return ContactProposal(query_id,(),yaw,0,'FOOTPRINT_LARGER_THAN_OBSERVED_GRID')
    preferred = np.clip(desired_uv,lower,upper)
    centres = np.argwhere(domain.cells())
    uv = np.asarray(domain.origin_uv_top_m)+(centres+.5)*domain.cell_size_m
    # Do not spend the bounded containment budget on centres whose physical
    # footprint cannot fit even the observed grid bounds. The exact observed
    # cell gate below still rejects holes and irregular boundaries.
    uv = uv[np.all((uv >= lower) & (uv <= upper), axis=1)]
    distance = np.sum((uv-desired_uv)**2,axis=1)
    count = min(len(uv),max_center_checks)
    if count:
        nearest = np.argpartition(distance,count-1)[:count]
        nearest = nearest[np.argsort(distance[nearest],kind='stable')]
        proposals = [preferred]+list(uv[nearest])
    else:
        proposals = [preferred]
    checked = 0
    for point in proposals[:max_center_checks]:
        checked += 1
        if check_observed_contact_footprint(domain,model,_contact_matrix(domain,point,yaw)).contained:
            return ContactProposal(query_id,tuple(float(v) for v in point),yaw,checked,
                                   'OBSERVED_FOOTPRINT_PROPOSAL')
    return ContactProposal(query_id,(),yaw,checked,'CONTACT_CENTRE_SEARCH_INCOMPLETE')


def _rotation_x(transform):
    from luggage_description.frame_transforms import rotate_vector
    return rotate_vector(transform.quaternion_xyzw,(1.,0.,0.))


def admit_pickup_association(snapshot, domain, pickup_object_id):
    """Absent private target is OK; never erase an unrelated same-ID obstacle."""
    from moveit_msgs.msg import CollisionObject, PlanningScene
    from rclpy.serialization import deserialize_message
    from shape_msgs.msg import SolidPrimitive
    if not pickup_object_id or not pickup_object_id.strip():
        raise ValueError('explicit pickup object identity required')
    scene = deserialize_message(snapshot.scene_cdr,PlanningScene)
    objects = [obj for obj in scene.world.collision_objects if obj.id == pickup_object_id]
    if len(objects) > 1:
        raise ValueError('PICKUP_OBJECT_ASSOCIATION_CONFLICT')
    if not objects:
        return
    obj = objects[0]
    if (obj.header.frame_id != 'world' or obj.operation != CollisionObject.ADD
            or len(obj.primitives) != 1 or len(obj.primitive_poses) != 1
            or obj.primitives[0].type != SolidPrimitive.BOX or obj.meshes or obj.mesh_poses
            or obj.planes or obj.plane_poses or obj.subframe_names or obj.subframe_poses
            or len(obj.primitives[0].dimensions) != 3):
        raise ValueError('PICKUP_OBJECT_ASSOCIATION_CONFLICT')
    pose = obj.primitive_poses[0]
    actual = Transform((pose.position.x,pose.position.y,pose.position.z),
        (pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w))
    if (not np.allclose(obj.primitives[0].dimensions,domain.box_wdh,atol=1e-8,rtol=0)
            or not np.allclose(actual.translation,domain.box_xyz_world,atol=1e-8,rtol=0)
            or abs(sum(a*b for a,b in zip(actual.quaternion_xyzw,domain.box_quaternion_xyzw))) < 1.-1e-12):
        raise ValueError('PICKUP_OBJECT_ASSOCIATION_CONFLICT')


def candidate_for_contact(snapshot, domain, contact, placement, limits, *,
                          pickup_object_id, observation_id, rank,
                          pick_clearances=None, corridor_surface_max=None,
                          pick_entry_seeds=(), place_entry_seeds=()):
    """Compose a candidate, not a motion permission or an OCC slot admission.

    placement must already have passed the caller's OCC/hull/stacking admission.
    Exact box targets are preserved; native checks see the frozen full envelope.
    """
    if type(placement) is not PlacementCandidate:
        raise ValueError('exact admitted placement required')
    admit_pickup_association(snapshot,domain,pickup_object_id)
    grasp = grasp_for_contact(domain,contact,observation_id=observation_id,rank=rank,
                              pick_clearances=pick_clearances)
    tool_box = grasp.box_from_tcp.inverse()
    geometry = freeze_geometry(domain.box_wdh,tool_box.translation,tool_box.quaternion_xyzw,limits)
    pick_box = Transform(domain.box_xyz_world,domain.box_quaternion_xyzw)
    settled = snapshot.world_from_container.compose(placement.container_from_box)
    released_container = Transform(tuple(v+d for v,d in zip(placement.container_from_box.translation,
        (0.,0.,PLACE_RELEASE_GAP_M))),placement.container_from_box.quaternion_xyzw)
    released = snapshot.world_from_container.compose(released_container)
    release_tool = released.compose(grasp.box_from_tcp)
    clearance = corridor_clearance(corridor_surface_max,domain.box_wdh[2],
        release_tool.translation[2],DEFAULT_PLACE_CLEARANCE_Z)
    up = rotate_vector(snapshot.world_from_container.quaternion_xyzw,(0.,0.,1.))
    def raised(distance):
        return Transform(tuple(p+distance*n for p,n in zip(release_tool.translation,up)),
                         release_tool.quaternion_xyzw)
    entry, retreat = raised(clearance), raised(clearance+EXIT_EXTRA_LIFT_M)
    scenes = build_operation_scenes(snapshot,pickup_object_id=pickup_object_id,geometry=geometry,
        world_from_pick_box=pick_box,world_from_settled_box=settled)
    material = (snapshot.identity,placement,contact.quality.evidence_sha256,geometry.digest(),
                grasp.pick_references,entry,retreat,pickup_object_id)
    identifier = placement.slot_id+':'+contact.query_id+':'+hashlib.sha256(repr(material).encode()).hexdigest()[:16]
    return OperationCandidate(identifier,pick_box,contact.world_from_contact,released,release_tool,settled,
        geometry,scenes,grasp.pick_references[0],grasp.pick_references[3],entry,retreat,
        pick_entry_seeds,place_entry_seeds)
