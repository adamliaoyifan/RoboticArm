"""One non-scored independent release calibration with actual tool controllers.

Original full tool and dynamic cargo stay physical. No box commands, force
feedback, side-contact waiver, arm-admission or formal full-action score.
"""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import shutil
import subprocess
import time
import traceback
import xml.etree.ElementTree as ET

import numpy as np

from assets import load, sha256
from carriage_commands import CarriageCommandPublisher
from execution_policy import ExecutionPolicy
from release_collision import ReleaseCollisionScene
from release_calibration import CalibrationEnrollment
from release_execution import ReleaseExecutionMonitor, ReleaseMotionPolicy, motion_reference
from release_fixture import ReleaseFixturePolicy
from release_observation import ReleaseObservationReader
from release_readiness import CalibrationReadyWindow, inspect_release_start
from release_rehearsal import save, ROOT, COORD, assert_sim_slot_free
from release_runtime import ReleaseRuntime
from release_trajectory import make_release_withdrawal
from planning_types import UpperConfig


def record_event(trace, event_type, **values):
    """Keep the event type distinct from a motion's name payload."""
    trace.write(json.dumps(dict(event=event_type, wall=time.monotonic(), **values), allow_nan=False)+'\n')
    trace.flush()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-dir', type=Path, required=True)
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--joint-bridge', type=Path, required=True)
    parser.add_argument('--policy', type=Path)
    parser.add_argument('--motion-policy', type=Path)
    parser.add_argument('--calibration-enrollment', type=Path)
    args = parser.parse_args()
    out = args.evidence.resolve()
    if not out.is_relative_to(COORD/'docs/status/evidence'):
        raise ValueError('PRIMARY_EVIDENCE_REQUIRED')
    policy = ExecutionPolicy.load(args.policy); motion_policy = ReleaseMotionPolicy.load(args.motion_policy)
    spec = json.loads((args.scene_dir/'release_scene.json').read_text())
    fixture = json.loads((args.scene_dir/'source_fixture.json').read_text())
    world = args.scene_dir/'release.sdf'
    for path, expected in ((world, spec['world_sha256']), (Path(spec['native_plugin']), spec['native_plugin_sha256'])):
        if sha256(path) != expected:
            raise ValueError('RELEASE_SCENE_CHANGED')
    if not spec.get('tool_collision_names'):
        raise ValueError('COMPLETE_TOOL_CONTACTS_REQUIRED')
    if sha256(args.scene_dir/'source_fixture.json') != spec['source_fixture_sha256']:
        raise ValueError('RELEASE_SOURCE_FIXTURE_CHANGED')
    enrollment = None
    if args.calibration_enrollment:
        enrollment = CalibrationEnrollment(**json.loads(args.calibration_enrollment.read_text()))
        if enrollment.case_id != spec['case']['case_id'] or enrollment.scene_sha256 != spec['world_sha256']:
            raise ValueError('CALIBRATION_ENROLLMENT_SCENE_MISMATCH')
    config = UpperConfig.load(**spec['configuration']); carriage = ReleaseFixturePolicy(**spec['fixture_policy'])
    if fixture['physics_step_s'] > motion_policy.native_max_gap_s:
        raise ValueError('RELEASE_NATIVE_STEP_EXCEEDS_POLICY')
    assets = load(ROOT); size = tuple(fixture['pickup_payload']['size_m'])
    reader = ReleaseObservationReader(assets['world_container'],
        {b['name']: b['size_m'] for b in fixture['boxes']}, 'upper_layer_payload',
        tool_collision_names=spec['tool_collision_names'])
    assert_sim_slot_free(); out.mkdir(parents=True, exist_ok=False)
    source = out/'source'; source.mkdir()
    for path in list(Path(__file__).parent.glob('*.py'))+list(Path(__file__).parent.glob('*.yaml')):
        shutil.copyfile(path, source/path.name)
    for name in ('release.sdf', 'release_scene.json', 'source_fixture.json'):
        shutil.copyfile(args.scene_dir/name, out/name)
    if enrollment:
        save(out/'calibration_enrollment.json', asdict(enrollment))
    # Preserve failure inputs, native binaries and original geometry before
    # physics starts; teardown must not be the first opportunity to save them.
    asset_dir = out/'assets'; asset_dir.mkdir(); asset_index = {}
    asset_paths = {str(ROOT/name) if not Path(name).is_absolute() else name for name in assets['hashes']}
    for name, expected in fixture['source_hashes'].items():
        path = Path(name) if Path(name).is_absolute() else ROOT/name
        if sha256(path) != expected:
            raise ValueError('RELEASE_SOURCE_ASSET_CHANGED:' + name)
        asset_paths.add(str(path))
    asset_paths.update(n.text for n in ET.parse(world).findall('.//geometry/mesh/uri'))
    asset_paths.update((str(args.joint_bridge.resolve()), str(Path(spec['native_plugin']).resolve())))
    for name in sorted(asset_paths):
        path = Path(name); checksum = sha256(path); target = asset_dir/(checksum[:16]+'_'+path.name)
        shutil.copyfile(path, target); asset_index[name] = dict(path=str(target.relative_to(out)), sha256=checksum)
    save(out/'asset_index.json', asset_index)
    native_source = out/'native_source'; native_source.mkdir()
    for path in Path(__file__).with_name('native').iterdir():
        if path.is_file():
            shutil.copyfile(path, native_source/path.name)
    manifest = dict(profile='unscored_independent_release_calibration_rehearsal', scored_units=0,
        revision=subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip(),
        dirty_paths=subprocess.check_output(['git', '-C', str(ROOT), 'status', '--porcelain', '--', 'research/uld_upper_layer'], text=True).splitlines(),
        started_wall=time.time(), configuration=asdict(config), policy=asdict(policy), motion_policy=asdict(motion_policy),
        scene_sha256=sha256(world), plugin_sha256=sha256(spec['native_plugin']), joint_bridge_sha256=sha256(args.joint_bridge),
        assets_hashes=assets['hashes'], source_hashes={p.name: sha256(p) for p in source.iterdir()},
        command=__import__('sys').argv, environment=dict(ROS_DOMAIN_ID='7', IGN_PARTITION='uld_upper_release_trial'),
        calibration_enrollment=asdict(enrollment) if enrollment else None,
        capture_complete=False, replay_possible=False, teardown_complete=False,
        execution_admitted=False, formal_C_success=False, arm_reachability='not_evaluated')
    save(out/'manifest.json', manifest)
    runtime = ReleaseRuntime(world, out, policy, fixture['physics_step_s'])
    commands = CarriageCommandPublisher(args.joint_bridge, ['/upper_layer/release/x', '/upper_layer/release/z'],
        runtime.env, out, carriage, policy)
    trace = (out/'decisions.jsonl').open('w'); monitor = None; latest = None; service = None
    started = time.monotonic(); failure = None; result = None; last_command_stamp = None
    def next_sample():
        nonlocal latest
        if time.monotonic()-started > policy.wall_budget_s:
            raise TimeoutError('RELEASE_WALL_BUDGET')
        row = runtime.next_native()
        if row is None:
            time.sleep(.001); return None
        latest = reader.read(row)
        if latest.stamp_s > policy.sim_budget_s:
            raise TimeoutError('RELEASE_SIM_BUDGET')
        return latest
    def settled(name):
        gate = CalibrationReadyWindow(size, (0., 0., 0.), config, policy)
        pacing = spec.get('scheduling_real_time_factor') or 1.
        deadline = time.monotonic()+policy.condition_wait_budget_s+policy.stability_dwell_s/pacing
        while time.monotonic() < deadline:
            sample = next_sample()
            if sample is None:
                continue
            if sample.tool_contacts:
                raise RuntimeError('RELEASE_TOOL_CONTACT_DURING_READINESS')
            decision = gate.observe(sample)
            record_event(trace, name, stamp_s=sample.stamp_s, decision=decision)
            if decision.get('calibration_ready'):
                return sample, decision
        raise TimeoutError('RELEASE_SETTLED_READINESS_TIMEOUT')
    try:
        runtime.start()
        # Clock transport discovery is startup; no actuator command yet.
        deadline = time.monotonic()+policy.condition_wait_budget_s
        while time.monotonic() < deadline:
            if runtime.discovered:
                break
            time.sleep(.01)
        else:
            raise TimeoutError('RELEASE_NATIVE_STARTUP_TIMEOUT')
        publishers = runtime.publishers(); commands.start()
        initial, _ = settled('initial_readiness')
        # Hold the measured tool position while still attached, then require a
        # fresh full stable dwell. This avoids an uncontrolled unloaded jump
        # toward the controller's old zero target when vacuum is removed.
        commands.publish(initial.joint_displacements_xz_m, initial.stamp_s)
        initial, ready = settled('controlled_hold_readiness')
        save(out/'initial_sample.json', asdict(initial)); save(out/'initial_readiness.json', ready)
        motions = make_release_withdrawal(initial, config, policy, carriage)
        scene = ReleaseCollisionScene(assets, world, initial, size, config)
        certificate = scene.certify(motions, time.monotonic()+policy.condition_wait_budget_s)
        save(out/'collision_certificate.json', certificate)
        save(out/'motions.json', [asdict(m) for m in motions])
        save(out/'geometry_hashes.json', scene.model_hashes)
        if not certificate['valid']:
            raise RuntimeError('RELEASE_NOMINAL_COLLISION_CHECK_FAILED')
        # Verify current physical preconditions against the frozen settled input
        # after offline certification; never call its false admission flag true.
        current = reader.read(runtime.current_native())
        current_check = inspect_release_start(current, size, (0., 0., 0.), config, policy, initial)
        if not current_check['physical_preconditions_satisfied']:
            raise RuntimeError('RELEASE_INITIAL_CHANGED_DURING_PLANNING')
        save(out/'pre_detach_check.json', current_check)
        monitor = ReleaseExecutionMonitor(initial, size, config, policy, motion_policy, scene)
        runtime.arm()
        service = runtime.detach(); record_event(trace, 'detach_requested', stamp_s=current.stamp_s)
        deadline = time.monotonic()+policy.condition_wait_budget_s
        detached_sample = None
        while time.monotonic() < deadline:
            sample = next_sample()
            if sample is None:
                continue
            detached = not sample.attached and not sample.constraint_present and sample.attachment_detach_count == 1
            error = monitor.observe(sample, detached=detached, reference_tool=initial.container_T_tool, separating_interface=True)
            if error:
                raise RuntimeError(error['reason'])
            if detached and sample.processed_commands == initial.processed_commands+1 and service.poll() is not None:
                stdout, stderr = service.communicate()
                save(out/'detach_response.json', dict(returncode=service.returncode, stdout=stdout, stderr=stderr))
                if service.returncode != 0 or 'true' not in stdout.lower():
                    raise RuntimeError('RELEASE_DETACH_SERVICE_REJECTED')
                detached_sample = sample; break
        if detached_sample is None:
            raise TimeoutError('RELEASE_NATIVE_DETACH_NOT_CONFIRMED')
        record_event(trace, 'detach_confirmed', stamp_s=detached_sample.stamp_s)
        for index, motion in enumerate(motions):
            stage_start = latest.stamp_s; endpoint_deadline = None
            record_event(trace, 'motion_started', name=motion.name, stamp_s=stage_start)
            while True:
                sample = next_sample()
                if sample is None:
                    continue
                elapsed = sample.stamp_s-stage_start
                q, tool = motion_reference(motion, elapsed)
                error = monitor.observe(sample, detached=True, reference_tool=tool, separating_interface=index == 0)
                if error:
                    raise RuntimeError(error['reason'])
                if last_command_stamp is None or sample.stamp_s-last_command_stamp >= policy.monitor_period_s-1e-9:
                    commands.publish(q, sample.stamp_s); last_command_stamp = sample.stamp_s
                    record_event(trace, 'motion_feedback', stamp_s=sample.stamp_s, name=motion.name, metrics=monitor.metrics())
                if elapsed >= motion.time_from_start_s[-1]:
                    if np.max(np.abs(np.array(sample.joint_displacements_xz_m)-q)) <= motion_policy.endpoint_joint_error_m:
                        break
                    if endpoint_deadline is None:
                        endpoint_deadline = time.monotonic()+policy.condition_wait_budget_s
                    if time.monotonic() > endpoint_deadline:
                        raise TimeoutError('RELEASE_ENDPOINT_NOT_REACHED')
            record_event(trace, 'motion_completed', name=motion.name, stamp_s=sample.stamp_s)
        stable_start = latest.stamp_s
        record_event(trace, 'independent_stability_started', stamp_s=stable_start)
        q, tool = motion_reference(motions[-1], motions[-1].time_from_start_s[-1])
        while latest.stamp_s-stable_start < policy.stability_dwell_s:
            sample = next_sample()
            if sample is None:
                continue
            error = monitor.observe(sample, detached=True, reference_tool=tool, separating_interface=False)
            if error:
                raise RuntimeError(error['reason'])
        save(out/'final_sample.json', asdict(latest))
        result = dict(valid=True, independent_release_stability_observed=True, native_detach_confirmed=True,
            withdrawal_completed=True, stability_window_s=latest.stamp_s-stable_start, metrics=monitor.metrics(),
            initial_execution_policy_satisfied=ready['execution_policy_satisfied'],
            execution_admitted=False, formal_C_success=False, publishers=publishers)
    except Exception as exc:
        failure = dict(error=str(exc), traceback=traceback.format_exc(),
            stamp_s=latest.stamp_s if latest else None, metrics=monitor.metrics() if monitor else None)
        save(out/'failure.json', failure)
        if latest:
            save(out/'failed_sample.json', asdict(latest))
            try:
                commands.publish(latest.joint_displacements_xz_m, latest.stamp_s)
                record_event(trace, 'stop_hold_requested', stamp_s=latest.stamp_s)
            except Exception as stop_error:
                record_event(trace, 'stop_hold_failed', error=str(stop_error))
        # Continue observation-only capture for the configured failure postwindow.
        if runtime.process is not None and runtime.process.poll() is None:
            wall_end = time.monotonic()+policy.condition_wait_budget_s
            first_stamp = latest.stamp_s if latest else None
            while time.monotonic() < wall_end:
                try:
                    row = runtime.next_native(check_fresh=False)
                except Exception:
                    break
                if row:
                    if first_stamp is None:
                        first_stamp = row['sim_time_s']
                    if row['sim_time_s']-first_stamp >= policy.trigger_post_s:
                        break
                else:
                    time.sleep(.001)
    finally:
        if service is not None and service.poll() is None:
            service.terminate()
            try:
                service.communicate(timeout=3)
            except subprocess.TimeoutExpired:
                service.kill(); service.communicate(timeout=3)
        try:
            commands.close()
        except Exception as close_error:
            record_event(trace, 'command_close_failed', error=str(close_error))
        capture = runtime.stop_capture(); save(out/'capture_metrics.json', capture)
        if result:
            save(out/'result.json', result)
        manifest.update(capture_complete=capture['native_samples'] > 0 and not capture['native_error'] and not capture['clock_error'],
            replay_possible=capture['native_samples'] > 0 and not capture['native_error'] and not capture['clock_error'],
            failure=failure, ended_wall=time.time())
        save(out/'manifest.json', manifest)
        # Raw observations and failure inputs are durable before owned teardown.
        manifest['teardown_complete'] = runtime.teardown()
        save(out/'manifest.json', manifest); trace.close()
    if failure:
        raise RuntimeError(failure['error'])
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
