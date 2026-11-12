"""Driver-facing optional prior client; normal planning remains the fallback."""
from dataclasses import replace
import threading

from luggage_planning.placement_prior.contracts import PlacementCandidate
from luggage_planning.placement_prior.core import PriorChoice
from luggage_planning.placement_prior.pick import (
    candidates_for_prior, transform_from_pose, bind_choice_to_pick,
)


def bind_response_to_pick(response, detection, *, scene_revision, map_revision, geometry_hash):
    if not response or not response.success or not response.measured:
        raise ValueError('no measured prior')
    if response.map_revision!=map_revision or response.geometry_hash!=geometry_hash:
        raise ValueError('stale prior map')
    available={g.candidate_id:g for g in candidates_for_prior(detection)}
    for wire in response.choices:
        candidate=available.get(wire.candidate_id)
        if candidate is None:
            continue
        if (wire.slot.identity.scene_revision!=scene_revision or
                wire.slot.identity.frame_id!='container_link' or
                wire.slot.reference!='box_center' or
                wire.grasp.identity.scene_revision!=scene_revision or
                wire.grasp.reference!='suction_contact_frame' or
                wire.grasp.orientation_semantics!='box'):
            raise ValueError('prior frame identity mismatch')
        if (wire.observation_id!=candidate.observation_id or
                wire.contact_model_hash!=candidate.contact_model_hash or
                transform_from_pose(wire.grasp.pose)!=candidate.box_from_tcp):
            raise ValueError('prior grasp provenance mismatch')
        if tuple(response.box_wdh)!=tuple(detection.box_size):
            raise ValueError('prior measured box changed')
        choice=PriorChoice(PlacementCandidate(wire.slot_id,transform_from_pose(wire.slot.pose)),
                           candidate,transform_from_pose(wire.tcp_references_world[-1]),
                           tuple(transform_from_pose(p) for p in wire.tcp_references_world))
        return bind_choice_to_pick(choice,detection),wire
    raise ValueError('no matching prior suction candidate')


class PlacementPriorClient:
    def __init__(self,node,*,group=None):
        from luggage_msgs.srv import PreparePlacementPriorV2
        from luggage_msgs.msg import SurfaceContractV2
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
        self._type=PreparePlacementPriorV2
        self._client=node.create_client(PreparePlacementPriorV2,'/placement_prior/prepare',callback_group=group)
        self._surface=None
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._subscription=node.create_subscription(SurfaceContractV2,'/luggage/cargo_map/surface_v2',
                                                   self._on_surface,qos,callback_group=group)

    def _on_surface(self,message):
        self._surface=message

    def prepare(self,box,request_id,*,slots=(),budget_s=10.):
        surface=self._surface
        if surface is None or not self._client.service_is_ready():
            return None
        request=self._type.Request();request.request_id=request_id
        request.measured=True;request.detection=box;request.slots=list(slots)
        request.geometry_hash=surface.geometry_hash;request.expected_map_revision=surface.map_revision
        request.max_slots=8;request.candidate_limit=32;request.budget_s=float(budget_s)
        future=self._client.call_async(request);ready=threading.Event()
        future.add_done_callback(lambda f:ready.set())
        if not ready.wait(budget_s+5.):
            return None
        result=future.result();current=self._surface
        if current is None or (current.geometry_hash,current.map_revision,current.identity.scene_revision)!=(
                surface.geometry_hash,surface.map_revision,surface.identity.scene_revision):
            return None
        return result

    def bind(self,response,detection,scene_revision):
        surface=self._surface
        if surface is None:raise ValueError('OCC snapshot unavailable')
        return bind_response_to_pick(response,detection,scene_revision=scene_revision,
                                     map_revision=surface.map_revision,geometry_hash=surface.geometry_hash)
