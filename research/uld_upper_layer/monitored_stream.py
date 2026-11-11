"""Drain native physics samples instead of discarding states between ROS polls.

The runtime's bounded history is observations only. Native gaps and processing
backlog remain hard failures. Age is checked against the newest native stamp,
in addition to per-stream stamp checks inside ObservationReader.
"""
import time
import math
class NativeSampleMonitor:
    def __init__(self,runtime,monitor,policy):
        self._runtime=runtime;self._monitor=monitor;self._policy=policy
        self._max_backlog_s=0.
        self._calls=0;self._selected_samples=0;self._max_callback_wall_s=0.
        self._native_step_s=runtime.native_step_s
        if self._native_step_s is None or not math.isfinite(self._native_step_s) or self._native_step_s<=0:
            raise ValueError('NATIVE_CONTACT_STEP_REQUIRED')
        self._contact_stamp=monitor.sample.stamp_s
        self._contact_samples=0;self._max_native_gap_s=0.

    def __call__(self,row):
        started=time.perf_counter();self._calls+=1
        try:return self._drain(row)
        finally:self._max_callback_wall_s=max(self._max_callback_wall_s,time.perf_counter()-started)

    def _drain(self,row):
        if self._monitor.failure:return self._monitor.failure['reason']
        newest=row.get('vacuum')
        if newest is None:return self._monitor.observe(row)
        samples=self._runtime.native_samples_after(self._contact_stamp)
        for native in samples:
            # The capture thread may append after the caller took `row`.
            # Leave those later samples for the next snapshot, preserving a
            # single upper timestamp bound for the whole observation batch.
            if native['sim_time_s']>newest['sim_time_s']:break
            gap=native['sim_time_s']-self._contact_stamp
            self._max_native_gap_s=max(self._max_native_gap_s,gap)
            observation=dict(row,vacuum=native,native_latest_stamp_s=newest['sim_time_s'])
            if gap<=0 or gap>self._native_step_s+1e-9:
                return self._monitor.fail_execution('NATIVE_CONTACT_STREAM_GAP',
                    offending_stamp_s=native['sim_time_s'],previous_stamp_s=self._contact_stamp,
                    gap_s=gap,expected_step_s=self._native_step_s,invalid_observation=observation)
            backlog=newest['sim_time_s']-native['sim_time_s']
            self._max_backlog_s=max(self._max_backlog_s,backlog)
            self._contact_samples+=1
            reason=self._monitor.observe_native_contacts(observation)
            if reason:return reason
            self._contact_stamp=native['sim_time_s']
            previous=self._monitor.sample
            state_changed=(native['attached']!=previous.attached or native['processed_commands']!=previous.processed_commands)
            if native['sim_time_s']-previous.stamp_s<self._policy.monitor_period_s-1e-9 and not state_changed:continue
            # Feed the actual historical physics state with current separately
            # stamped lower/contact streams; reader rejects any age > policy.
            self._selected_samples+=1
            reason=self._monitor.observe(observation)
            if reason:return reason
        if newest['sim_time_s']>self._contact_stamp+1e-9:
            return self._monitor.fail_execution('NATIVE_CONTACT_STREAM_MISSING',
                offending_stamp_s=newest['sim_time_s'],previous_stamp_s=self._contact_stamp,
                invalid_observation=row)
        # Validate the newest streams even when native time did not advance;
        # a stopped capture or clock cannot hide behind an empty history.
        return self._monitor.observe(row)

    def metrics(self):return dict(max_native_processing_backlog_s=self._max_backlog_s,
        callbacks=self._calls,selected_samples=self._selected_samples,max_callback_wall_s=self._max_callback_wall_s,
        native_contact_samples=self._contact_samples,native_contact_last_stamp_s=self._contact_stamp,
        native_contact_step_s=self._native_step_s,max_native_contact_gap_s=self._max_native_gap_s,
        native_contact_scope='declared_lower_and_panel_rules_every_native_step_full_scene_at_monitor_period')
