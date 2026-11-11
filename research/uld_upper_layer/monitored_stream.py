"""Drain native physics samples instead of discarding states between ROS polls.

The runtime's bounded history is observations only. Native gaps and processing
backlog remain hard failures. Age is checked against the newest native stamp,
in addition to per-stream stamp checks inside ObservationReader.
"""
import time
class NativeSampleMonitor:
    def __init__(self,runtime,monitor,policy):
        self._runtime=runtime;self._monitor=monitor;self._policy=policy
        self._max_backlog_s=0.
        self._calls=0;self._selected_samples=0;self._max_callback_wall_s=0.

    def __call__(self,row):
        started=time.perf_counter();self._calls+=1
        try:return self._drain(row)
        finally:self._max_callback_wall_s=max(self._max_callback_wall_s,time.perf_counter()-started)

    def _drain(self,row):
        newest=row.get('vacuum')
        if newest is None:return self._monitor.observe(row)
        samples=self._runtime.native_samples_after(self._monitor.sample.stamp_s)
        for native in samples:
            # The capture thread may append after the caller took `row`.
            # Leave those later samples for the next snapshot, preserving a
            # single upper timestamp bound for the whole observation batch.
            if native['sim_time_s']>newest['sim_time_s']:break
            previous=self._monitor.sample
            state_changed=(native['attached']!=previous.attached or native['processed_commands']!=previous.processed_commands)
            if native['sim_time_s']-previous.stamp_s<self._policy.monitor_period_s-1e-9 and not state_changed:continue
            backlog=newest['sim_time_s']-native['sim_time_s']
            self._max_backlog_s=max(self._max_backlog_s,backlog)
            # Feed the actual historical physics state with current separately
            # stamped lower/contact streams; reader rejects any age > policy.
            observation=dict(row,vacuum=native,native_latest_stamp_s=newest['sim_time_s'])
            self._selected_samples+=1
            reason=self._monitor.observe(observation)
            if reason:return reason
        # Validate the newest streams even when native time did not advance;
        # a stopped capture or clock cannot hide behind an empty history.
        return self._monitor.observe(row)

    def metrics(self):return dict(max_native_processing_backlog_s=self._max_backlog_s,
        callbacks=self._calls,selected_samples=self._selected_samples,max_callback_wall_s=self._max_callback_wall_s)
