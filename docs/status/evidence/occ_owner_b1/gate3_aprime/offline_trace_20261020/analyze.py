"""Offline boundary comparison; creates no ROS nodes or action clients.

Run with the workspace ROS message environment sourced. Reconstructed outputs
are conditional mechanism checks, never captured controller goals.
"""
import base64
import hashlib
import gzip
import json
from pathlib import Path
import subprocess
import threading
from types import SimpleNamespace

from rclpy.serialization import deserialize_message, serialize_message
from luggage_msgs.srv import ValidateRouteV2, SelectPlacePathV2
from luggage_planning.route_producer import _leg_record
from luggage_planning.place_service import _route_motion_messages, _route_boundary
from luggage_planning.motion_frame_adapters import record_from_message
from luggage_planning.motion_frame_contracts import ExecutionContext, TARGET_LINK
from luggage_planning.motion_executor import MotionExecutor

OUT = Path(__file__).resolve().parent
WS = OUT.parents[5]
MAIN = Path('/home/adamliao/work/elfin_humble_ws')
R10 = MAIN / 'docs/status/evidence/master_pack_recovery/2026-10-14_g3/gate3_aprime_r10_prior'
FAIL = 'dumps/box_00_HANDOFF_SETTLE_TIMEOUT'


def read(path):
    return json.loads(path.read_text())


def sha(data):
    return hashlib.sha256(data).hexdigest()


def trajectory_record(trajectory):
    return dict(
        header=dict(sec=trajectory.header.stamp.sec,
                    nanosec=trajectory.header.stamp.nanosec,
                    frame_id=trajectory.header.frame_id),
        joint_names=list(trajectory.joint_names),
        points=[dict(positions=list(p.positions), velocities=list(p.velocities),
                     accelerations=list(p.accelerations), effort=list(p.effort),
                     time_from_start_ns=p.time_from_start.sec * 10**9 + p.time_from_start.nanosec)
                for p in trajectory.points])


def summary(trajectory):
    record = trajectory_record(trajectory)
    return dict(n_points=len(record['points']), joint_names=record['joint_names'],
                header=record['header'], first=record['points'][0], last=record['points'][-1],
                fields_sha256=sha(json.dumps(record,sort_keys=True,separators=(',', ':')).encode()))


def compare_driver(trajectory, boundary):
    driver = boundary['trajectory']
    assert not driver['truncated']
    assert driver['joint_names'] == list(trajectory.joint_names)
    assert len(driver['points']) == len(trajectory.points)
    q_error = v_error = t_error = 0.0
    for source, saved in zip(trajectory.points, driver['points']):
        q_error = max(q_error, max(abs(a-b) for a, b in zip(source.positions, saved['q'])))
        v_error = max(v_error, max(abs(a-b) for a, b in zip(source.velocities, saved['v'])))
        t = source.time_from_start.sec + 1e-9 * source.time_from_start.nanosec
        t_error = max(t_error, abs(t-saved['t']))
    assert q_error == 0.0 and v_error == 0.0
    assert t_error <= 0.5e-6
    return dict(max_position_error=q_error, max_velocity_error=v_error,
                max_time_error_seconds=t_error, accelerations='missing',
                effort='missing', header='missing',
                execution_window_seconds=boundary['result_sim_stamp']-boundary['submit_sim_stamp'])


def conditional_rebuild(request, leg, selected):
    """Exercise actual producer/encoding/admission/executor code up to submit hook.

    The exact live start and occupancy context were not captured. Assume live
    start equals the first knot, so wrap shift is zero. No occupancy claim.
    """
    points = leg.knots.points
    trajectory = SimpleNamespace(
        joint_names=leg.knots.joint_names,
        positions=[p.positions for p in points], velocities=[p.velocities for p in points],
        accelerations=[p.accelerations for p in points],
        time_from_start_ns=[p.time_from_start.sec*10**9+p.time_from_start.nanosec for p in points])
    request_id = selected.request_id
    scene_revision = selected.start_tcp.identity.scene_revision
    stamp_ns = selected.start_tcp.identity.stamp_ns
    record = _leg_record('transit', trajectory, request_id, scene_revision, stamp_ns)
    messages = _route_motion_messages([record], dict(request_id=request_id,
                                                  scene_revision=scene_revision))
    context = ExecutionContext(_route_boundary(request_id, scene_revision,
                                              'robot_state'), TARGET_LINK, 'world')
    decoded = record_from_message(messages[0])
    executor = object.__new__(MotionExecutor)
    executor._frame, executor._link = 'world', TARGET_LINK
    executor._v2_lock = threading.Lock()
    captured = []
    executor._timed_submit = lambda t: captured.append(t) or True
    executor.execute_segment_v2(decoded, context, current_joints=list(points[0].positions))
    rebuilt = captured[-1].joint_trajectory
    assert trajectory_record(rebuilt) == trajectory_record(leg.knots)
    (OUT/'r10_conditional_reconstruction.cdr').write_bytes(serialize_message(captured[-1]))
    (OUT/'r10_conditional_reconstruction.json').write_text(
        json.dumps(trajectory_record(rebuilt), indent=2)+'\n')
    # A separate mechanism check exercises the existing integer-turn branch.
    import math
    live = list(points[0].positions)
    live[0] += 2*math.pi
    executor.execute_segment_v2(decoded, context, current_joints=live)
    shifted = captured[-1].joint_trajectory
    for before, after in zip(rebuilt.points, shifted.points):
        assert abs(after.positions[0]-before.positions[0]-2*math.pi) < 1e-12
        assert list(after.positions[1:]) == list(before.positions[1:])
        assert after.velocities == before.velocities
        assert after.accelerations == before.accelerations
        assert after.time_from_start == before.time_from_start
    return dict(exact_equal_with_zero_wrap=True, integer_turn_branch_verified=True,
                summary=summary(rebuilt), actual_goal_reconstructed=False,
                assumptions=['live start equals first knot', 'occupancy context omitted',
                             'typed identity from same-run Select request; response not captured'])


def main():
    result = dict(base_revision='bb238bb7', simulations_started=0, cases={})
    inputs = []
    boundary_path = R10/FAIL/'replay/last_boundary.json'
    boundary = read(boundary_path)
    inputs.append(boundary_path)
    requests = []
    for path in sorted((R10/'cert_capture').glob('cert_request_*.json')):
        saved = read(path)
        request = deserialize_message(base64.b64decode(saved['request_b64']), ValidateRouteV2.Request)
        requests.append(request)
        inputs.append(path)
        leg = request.legs[0]
        item = dict(index=saved['index'], route_digest=request.route_digest,
                    scene_revision=request.scene_revision, edge_id=leg.edge_id,
                    first_leg=summary(leg.knots))
        if len(leg.knots.points) == boundary['trajectory']['n_points']:
            item['driver_comparison'] = compare_driver(leg.knots, boundary)
        result['cases'][path.name] = item
    assert trajectory_record(requests[1].legs[0].knots) == trajectory_record(requests[2].legs[0].knots)
    select_path = R10/'select_capture/select_place_path_v2_00.bin'
    selected = deserialize_message(select_path.read_bytes(), SelectPlacePathV2.Request)
    inputs.append(select_path)
    result['conditional_reconstruction'] = conditional_rebuild(requests[2], requests[2].legs[0], selected)
    snapshot = OUT/'inputs_2c'
    c2_boundary_path = snapshot/'last_boundary.json'
    handoff_path = snapshot/'handoff.json'
    joint_path = snapshot/'joint_ring.jsonl.gz'
    watcher_path = snapshot/'controller_state.jsonl.gz'
    inputs += [c2_boundary_path, handoff_path, joint_path, watcher_path, R10/'manifest.json']
    c2 = read(c2_boundary_path)
    cs = read(handoff_path)['controller_state']['rows']
    js = [json.loads(line) for line in gzip.open(joint_path, 'rt')]
    watcher = [json.loads(line) for line in gzip.open(watcher_path, 'rt')]
    end = c2['trajectory']['points'][-1]['q']
    post = [row for row in cs if row['stamp'] >= c2['result_sim_stamp']]
    matches = []
    for row in [watcher[-30], watcher[-1]]:
        matched = min(js, key=lambda j: max(abs(a-b) for a,b in zip(j['position'],row['feedback.positions'])))
        matches.append(dict(watcher_wall=row['t'], watcher_cached_sim=row['sim'],
                            joint_state_stamp=matched['stamp'],
                            feedback_match_error=max(abs(a-b) for a,b in zip(matched['position'],row['feedback.positions'])),
                            apparent_age_s=row['sim']-matched['stamp']))
    result['2c'] = dict(driver_n_points=c2['trajectory']['n_points'],
        driver_last=c2['trajectory']['points'][-1],
        execution_window_s=c2['result_sim_stamp']-c2['submit_sim_stamp'],
        response_wait_s=c2['response_wait_s'],
        driver_controller_range_s=[cs[0]['stamp'],cs[-1]['stamp']],
        post_result_controller_samples=len(post),
        post_result_max_reference_error=max(max(abs(a-b) for a,b in zip(r['reference'],end)) for r in post),
        post_result_reference_distinct=len({tuple(r['reference']) for r in post}),
        post_result_reference_equals_output=all(r['reference']==r['output'] for r in post),
        watcher_matches=matches,
        actual_fjt_goal='missing', certification='missing',
        driver_capture_missing_fields=['accelerations','effort','header','integer_nanosecond_times'])
    source_paths = ['src/luggage_planning/luggage_planning/'+name+'.py' for name in
                    ['motion_executor','motion_frame_adapters','motion_frame_contracts','route_producer','typed_driver_adapter','place_service']]
    source_paths += ['src/luggage_gazebo/scripts/place_smoke_driver.py',
                     'src/elfin_control/config/elfin_controllers_sim.yaml',
                     'src/elfin_moveit_config/config/moveit_controllers.yaml']
    provenance = []
    for name in source_paths:
        now = (WS/name).read_bytes()
        then = subprocess.check_output(['git','show','5a80e9b4:'+name],cwd=WS)
        provenance.append(dict(path=name, sha256=sha(now), equals_r10_committed_source=now==then))
    assert all(row['equals_r10_committed_source'] for row in provenance)
    result['source_provenance'] = provenance
    result['inputs'] = [dict(path=str(p),bytes=p.stat().st_size,sha256=sha(p.read_bytes())) for p in inputs]
    (OUT/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(result=str(OUT/'result.json'),
                         r10_comparison=result['cases']['cert_request_2.json']['driver_comparison'],
                         conditional_reconstruction_equal=True, c2=result['2c']),indent=2))


if __name__ == '__main__':
    main()
