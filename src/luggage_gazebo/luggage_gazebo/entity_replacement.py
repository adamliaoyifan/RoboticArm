"""Bounded observed replacement of a simulation entity; no planning inputs."""
import math
import subprocess
import time

from luggage_gazebo.place_metrics import parse_ign_model_pose


def probe_model(model, timeout):
    """Fresh Gazebo query; transport failure never means entity absence."""
    command = ['ign', 'model', '-m', model, '--pose']
    try:
        process = subprocess.run(command, capture_output=True, text=True,
                                 timeout=timeout, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return dict(status='unavailable', detail=str(exc))
    output = process.stdout + process.stderr
    result = dict(returncode=process.returncode, output=output[-2000:])
    if process.returncode != 0:
        return dict(result, status='unavailable')
    pose = parse_ign_model_pose(output)
    if pose is not None and len(pose) == 6 and all(map(math.isfinite, pose)):
        return dict(result, status='present', pose=pose)
    if 'No model named <%s> was found' % model in output:
        return dict(result, status='absent')
    return dict(result, status='unavailable')


def replace_entity(*, probe, delete, create, pose_matches, timeout=10.,
                   persistence=.4, clock=time.monotonic, sleep=time.sleep):
    """Observe disappearance before create, and persistent pose before success.

    Callbacks receive their remaining wall-time allowance. Service callbacks
    must bound their entire call, not just one of discovery/response waits.
    No retry issues another delete/create command after ambiguous completion.
    """
    started = clock()
    deadline = started + timeout
    trace = []

    def remaining():
        return max(0., deadline - clock())

    def finish(reason):
        return dict(success=reason == 'READY', reason=reason,
                    elapsed_s=clock() - started, trace=trace)

    def observe(stage):
        value = probe(min(2., remaining()))
        trace.append(dict(value, stage=stage, elapsed_s=clock() - started))
        return value

    initial = observe('before_delete')
    if initial['status'] != 'present':
        return finish('INITIAL_ENTITY_UNCONFIRMED')
    expected = initial['pose']
    for stage, command, wanted in (
            ('delete', delete, 'absent'),
            ('create', lambda budget: create(expected, budget), 'present')):
        if remaining() <= 0:
            return finish('REPLACEMENT_TIMEOUT')
        error = command(remaining())
        trace.append(dict(stage=stage + '_ack', error=error,
                          elapsed_s=clock() - started))
        if error:
            return finish(stage.upper() + '_REFUSED')
        ready_since = None
        while remaining() > 0:
            observed = observe('after_' + stage)
            if remaining() <= 0:
                return finish(stage.upper() + '_UNCONFIRMED')
            ready = observed['status'] == wanted
            if ready and wanted == 'present':
                ready = pose_matches(observed['pose'], expected)
            if ready:
                if wanted == 'absent':
                    break
                if ready_since is None:
                    ready_since = clock()
                if clock() - ready_since >= persistence:
                    return finish('READY')
            else:
                ready_since = None
            sleep(min(.1, remaining()))
        else:
            return finish(stage.upper() + '_UNCONFIRMED')
    return finish('REPLACEMENT_TIMEOUT')
