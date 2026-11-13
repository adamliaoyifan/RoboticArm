"""Freeze only the owned simulation clock during offline computation.

No pose or controller command is changed; every executed motion uses live
physics. Resume runs before returning a plan and on all exception paths.
"""
from contextlib import contextmanager


@contextmanager
def paused_for_planning(runtime,monitor=None):
    try:
        if monitor is None:runtime.set_physics_paused(True)
        else:runtime.set_physics_paused(True,monitor=monitor)
        yield
    finally:
        if monitor is None:runtime.set_physics_paused(False)
        else:runtime.set_physics_paused(False,monitor=monitor)


def rehearse_pause(runtime,monitor,observe,policy):
    """Exercise real clock control and native-step continuity without placement."""
    import time
    import rclpy
    def pump():
        rclpy.spin_once(runtime.node,timeout_sec=policy.monitor_period_s)
        reason=observe(runtime.snapshot())
        if reason:raise RuntimeError(reason)
    monitor.begin_phase('planning_hold',True,True,False)
    with paused_for_planning(runtime,observe):
        initial=runtime.snapshot()['vacuum']['sim_time_s']
        deadline=time.monotonic()+2*policy.observation_max_age_s
        while time.monotonic()<deadline:pump()
        final=runtime.snapshot()['vacuum']['sim_time_s']
        if final!=initial:raise RuntimeError('PLANNING_CLOCK_DID_NOT_FREEZE')
    deadline=time.monotonic()+policy.condition_wait_budget_s
    while monitor.sample.stamp_s<final+policy.trigger_post_s:
        pump()
        if time.monotonic()>deadline:raise TimeoutError('RESUMED_NATIVE_STREAM_NOT_ADVANCING')
    return dict(status='planning_pause_rehearsal_pass',paused_initial_stamp_s=initial,
        paused_final_stamp_s=final,resumed_stamp_s=monitor.sample.stamp_s,
        scope='owned_clock_control_and_monitor_continuity_only_not_upper_placement')
