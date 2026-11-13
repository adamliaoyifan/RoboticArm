"""Task intent over the SAME shared observed placement admission.

No requested pose comes from a catalog, prior placed list or simulator truth.
Filter the full finite query before retaining32 proposals so floor-first output
caps cannot accidentally hide sensor-observed stacking support.
"""
import math
import json
from luggage_description.container_geometry import normalize_descriptor
from luggage_packing.placement_query import placement_query,DIRECTED_PLACEMENT_YAWS
from luggage_packing.placement_volume_query import KnownFreeVolume,evidence_from_volume
from luggage_packing.placement_solver import retain_directed_candidates
from luggage_planning.placement_prior.requests import _placement_surface
from luggage_planning.placement_prior.ranking import rank_payload_clearance
from luggage_planning.placement_prior.contracts import Transform
from luggage_planning.planning_contract.attachment_geometry import freeze_geometry
from luggage_planning.place.waypoint_generator import PLACE_RELEASE_GAP_M,DEFAULT_PLACE_CLEARANCE_Z


def observed_placement_slots(confirmed,volume,dimensions,mode,*,limits,margin,vertical_margin):
    if mode not in ('floor','stack'):raise ValueError('EXPLICIT_PLACEMENT_TASK_REQUIRED')
    surface,descriptor=_placement_surface(confirmed)
    capacity=len(surface['state'])*len(surface['state'][0])*len(DIRECTED_PLACEMENT_YAWS)
    queried=placement_query(surface,dimensions,params={'top_n':capacity,'keep_rejected':0},
        volume_verifier=KnownFreeVolume(evidence_from_volume(volume)))
    floor=float(descriptor['floor_z']);resolution=float(confirmed.record()['resolution_m'])
    candidates=[row for row in queried['candidates'] if row['feasible']]
    count=len(candidates)
    def requested_layer(row):
        bottom=row['center_container'][2]-dimensions[2]/2
        return abs(bottom-floor)<=resolution+1e-9 if mode=='floor' else bottom>floor+resolution+1e-9
    candidates=[row for row in candidates if requested_layer(row)]
    matching=len(candidates)
    geometry=freeze_geometry(dimensions,(0.,0.,dimensions[2]/2),(1.,0.,0.,0.),limits)
    candidates=rank_payload_clearance(candidates,dimensions,volume,margin,
        hull=normalize_descriptor(descriptor),geometry=geometry,release_gap_m=PLACE_RELEASE_GAP_M,
        vertical_margin=vertical_margin,approach_clearance_m=DEFAULT_PLACE_CLEARANCE_Z)
    retained=retain_directed_candidates(candidates,32)
    slots=tuple(Transform(row['center_container'],(0.,0.,math.sin(row['heading_theta']/2),
        math.cos(row['heading_theta']/2))) for row in retained)
    metadata=json.loads(confirmed.confirmation_json)
    source=metadata.get("observer_source") or {key:confirmed.record()[key] for key in
        ("geometry_hash","map_revision","scene_revision","stamp_ns")}
    return slots,dict(task_mode=mode,finite_capacity=capacity,feasible_total=count,
        feasible_requested_layer=matching,retained=len(slots),source=source)
