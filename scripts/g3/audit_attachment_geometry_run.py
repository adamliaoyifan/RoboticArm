#!/usr/bin/env python3
"""Inspect a frozen G4 selection boundary without running motion or planning."""
import json
from pathlib import Path
import sys
from collections import Counter
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.set_message import set_message_fields
from luggage_msgs.srv import SelectPlacePathV2
from luggage_msgs.msg import PoseContractV2, SurfaceContractV2
from moveit_msgs.srv import GetPlanningScene
from st09_contracts.attachment_geometry import AttachmentGeometry
from st09_contracts.payload_envelope import PayloadEnvelope, EnvelopeLimits
from luggage_planning.typed_driver_adapter import TypedDriverAdapter
from luggage_planning.place_service import request_dict


def audit(run):
    boundary = next((run/'selection_boundary').glob('*/request.json'))
    data = json.loads(boundary.read_text())
    raw = data['conservative_geometry']
    env = raw['conservative']
    geometry = AttachmentGeometry(tuple(raw['measured_dimensions_wdh']),
        tuple(raw['t_tool_box_translation']), tuple(raw['t_tool_box_quaternion_xyzw']),
        PayloadEnvelope(tuple(env['dimensions_wdh']), tuple(env['t_tcp_box_translation']),
            tuple(env['t_tcp_box_quaternion_xyzw']), EnvelopeLimits(**env['limits']),env['nominal_digest']))
    assert geometry.digest() == raw['digest']
    slots = []
    for value in data['slots']:
        slot = PoseContractV2()
        set_message_fields(slot, value)
        slots.append(slot)
    surface = SurfaceContractV2()
    set_message_fields(surface, data['surface'])
    adapter = object.__new__(TypedDriverAdapter)
    adapter._select = object()
    requests = []
    adapter._call = lambda client, request, timeout: requests.append(request)
    adapter.select_place_path(request_id=data['request_id'], box_wdh=data['box_wdh'],
        slots=slots,slot_scores=data['slot_scores'], surface_message=surface,
        inner_size=data['inner_size'],geometry_hash=data['geometry_hash'],
        map_revision=data['map_revision'],start_tcp=data['start_tcp'],
        start_joints=data['start_joints'],grasp=data['grasp'],
        container_transform=data['container_transform'],box_id=data['box_id'],
        generation=data['generation'], conservative_geometry=geometry)
    admitted = request_dict(requests[0])
    assert admitted['conservative_geometry'].digest() == geometry.digest()
    response = deserialize_message((boundary.parent/'response.cdr').read_bytes(),SelectPlacePathV2.Response)
    rows = json.loads(response.rows_json)
    summary = next(row for row in rows if row.get('reason')=='ATTEMPT_SUMMARY')
    suite = json.loads((run/'driver_out/suite.json').read_text())
    trial_path = next((run/'driver_out/dumps').glob('*/trial.json'))
    trial = json.loads(trial_path.read_text())
    replay = json.loads((trial_path.parent/'replay/manifest.json').read_text())
    scene_path = boundary.parent/'planning_scene.cdr'
    scene_match = None
    if scene_path.exists():
        scene = deserialize_message(scene_path.read_bytes(),GetPlanningScene.Response).scene
        attached = scene.robot_state.attached_collision_objects
        assert len(attached) == 1 and len(attached[0].object.primitives) == 1
        scene_match = tuple(attached[0].object.primitives[0].dimensions)==geometry.conservative.dimensions_wdh
        assert scene_match
        assert len(scene.allowed_collision_matrix.entry_values) == len(scene.allowed_collision_matrix.entry_names)
        assert any(obj.meshes for obj in scene.world.collision_objects)
    manifest = json.loads((run/'manifest.json').read_text())
    result = dict(revision=manifest['revision'], dirty=manifest['dirty'],
        boxes_attempted=suite['boxes_attempted'], boxes_picked=int(trial['vac_attach']),
        boxes_packed=suite['boxes_packed'], boxes_verified=0, boxes_committed=0,
        end_reason=suite['termination_reason'], wall_s=suite['wall_total_sec'],
        stage_timings_ms=json.loads(response.stage_timings_json),
        candidates=summary['n_ordered'], tried=summary['n_tried'],untried=summary['n_untried'],
        rejection_histogram=dict(Counter(row.get('reason') for row in rows if row.get('reason')!='ATTEMPT_SUMMARY')),
        route_builds=sum(row.get('route_build_count',0) for row in rows),
        certified_routes=int(response.success),carrying_segments=len(response.motion_segments),
        measured_dimensions=geometry.measured_dimensions_wdh,
        conservative_dimensions=geometry.conservative.dimensions_wdh,
        attachment_geometry_digest=geometry.digest(),
        request_reencoded_and_admitted=True, scene_dimensions_match=scene_match,
        full_scene_captured=scene_path.exists(), replay_manifest=replay,
        functional_suite_pass=False, closed_loop_cycle_latency=dict(n=0,status='not_evaluated'),
        startup=json.loads((run/'startup_preflight.json').read_text()))
    (run/'metrics.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__ == '__main__':
    result=audit(Path(sys.argv[1]))
    print(json.dumps({key:result[key] for key in ('end_reason','boxes_attempted','boxes_packed',
        'tried','candidates','request_reencoded_and_admitted','scene_dimensions_match')},indent=2))
