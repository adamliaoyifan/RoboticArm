"""Best-effort post-trigger capture without losing the original failure result."""
import time


def collect_failure_tail(runtime,policy,stamp):
    import rclpy
    tail=[]
    if stamp is None:return tail,'NATIVE_STAMP_UNAVAILABLE'
    deadline=time.monotonic()+policy.condition_wait_budget_s
    try:
        while time.monotonic()<deadline:
            if not rclpy.ok():return tail,'ROS_CONTEXT_CLOSED'
            rclpy.spin_once(runtime.node,timeout_sec=policy.monitor_period_s)
            row=runtime.snapshot();native=row.get('vacuum')
            if native:
                tail.append((native['sim_time_s'],row))
                if native['sim_time_s']-stamp>=policy.trigger_post_s:return tail,None
    except Exception as exc:
        return tail,type(exc).__name__+':'+str(exc)
    return tail,'FAILURE_TAIL_DEADLINE'
