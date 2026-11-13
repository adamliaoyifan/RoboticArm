"""Same observed operation for single-cycle and persistent three-box diagnostics."""
from luggage_msgs.srv import SpawnNextBox
import uuid


def can_reprepare_before_pick(result):
    return (not result.success and result.outcome=='STOPPED'
            and result.motion_terminal and not result.may_have_payload
            and result.reason in ('CURRENT_SOURCE_REFUSED','BOUND_NATIVE_ADMISSION_REQUIRED'))


def run_observed_cycle(driver, *, task_mode=None, prefix=''):
    def stage(name,call):return driver.stage(prefix+name,call)
    spawned=stage('physics_spawn',lambda:driver.call(driver.spawn,SpawnNextBox.Request(),'physics_spawn',30.))
    if not spawned.success:raise RuntimeError('SPAWN_FAILED')
    # One task per externally authorized box cycle, including prepare retries.
    # A changed detector ID never allocates a fresh recovery budget.
    driver.pickup_task_id = 'pickup-'+uuid.uuid4().hex
    # Spawn response is actuator/eval data, never measured placement input.
    prepare=lambda:driver.plan(task_mode=task_mode)
    plan,detection,domain=stage('prepare',prepare)
    result=stage('execute',lambda:driver.execute_prepared(plan,detection,domain,'execute'))
    original_instance=domain.instance_id
    for attempt in range(getattr(driver.args,'prepick_replans',0)):
        if not can_reprepare_before_pick(result.result):break
        suffix=str(attempt+1)
        stage('return_pick_retry_'+suffix,lambda:driver.return_to_pick_observation(plan))
        plan,detection,domain=stage('prepare_retry_'+suffix,prepare)
        if not original_instance or domain.instance_id!=original_instance:
            raise RuntimeError('PREPICK_REPLAN_INSTANCE_CHANGED')
        result=stage('execute_retry_'+suffix,lambda:driver.execute_prepared(plan,detection,domain,'execute_retry_'+suffix))
    outcome=result.result
    if not outcome.success:raise RuntimeError('EXECUTE:'+outcome.outcome+':'+outcome.reason)
    if not outcome.motion_terminal or outcome.may_have_payload:
        raise RuntimeError('COMPLETED_OPERATION_NOT_TERMINAL_AND_EMPTY')
    return plan,outcome
