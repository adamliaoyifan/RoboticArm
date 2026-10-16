"""Transport wait for an already-certified trajectory, not a planning limit."""
import math


def timed_execution_wait(points, overhead_s):
    overhead_s=float(overhead_s)
    if not math.isfinite(overhead_s) or overhead_s<=0:
        raise ValueError('positive finite execution response overhead required')
    if not points:
        raise ValueError('certified trajectory is empty')
    last_ns=-1
    for point in points:
        duration=point.time_from_start
        if duration.sec<0 or not 0<=duration.nanosec<1_000_000_000:
            raise ValueError('invalid trajectory duration')
        current_ns=duration.sec*1_000_000_000+duration.nanosec
        if current_ns<=last_ns:
            raise ValueError('trajectory time must increase')
        last_ns=current_ns
    return last_ns/1_000_000_000+overhead_s


