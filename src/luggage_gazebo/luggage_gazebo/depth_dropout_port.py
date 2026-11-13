"""ROS adapter for an explicit simulation fixture, outside perception owners."""
from collections import deque
import json
import time

import numpy as np
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from luggage_gazebo.depth_dropout_evidence import DepthDropoutEvidence
from luggage_gazebo.depth_dropout_fixture import DepthDropoutConfig, apply_dropout, optical_axis_world


class DepthDropoutPort:
    def __init__(self, node, config_path, evidence_root, selection_file, publish):
        with open(config_path) as stream:
            self.config = DepthDropoutConfig.from_record(json.load(stream))
        self.evidence = DepthDropoutEvidence(evidence_root, selection_file)
        self.publish = publish
        self.buffer = Buffer(node=node)
        self.listener = TransformListener(self.buffer, node)
        self.pending = deque()
        self.timer = node.create_timer(.02, self.flush, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.errors = []
        self.error_count = 0
        self.closed = False

    def _capture_error(self, error):
        self.error_count += 1
        if len(self.errors) < 16:
            self.errors.append(type(error).__name__+':'+str(error))

    def submit(self, message):
        if len(self.pending) >= 5:
            old, _ = self.pending.popleft()
            self._emit(old, None, 'TF_QUEUE_OVERFLOW')
        self.pending.append((message, time.monotonic()))
        self.flush()

    def flush(self):
        while self.pending:
            message, begin = self.pending[0]
            if message.header.frame_id != self.config.optical_frame:
                self.pending.popleft()
                self._emit(message, None, 'OPTICAL_FRAME_MISMATCH')
                continue
            stamp = Time.from_msg(message.header.stamp)
            if stamp.nanoseconds == 0:
                self.pending.popleft()
                self._emit(message, None, 'ZERO_STAMP_NOT_EXACT')
                continue
            try:
                transform = self.buffer.lookup_transform(self.config.world_frame,
                    self.config.optical_frame, stamp, timeout=Duration())
            except TransformException:
                if time.monotonic()-begin < .5:
                    break
                self.pending.popleft()
                self._emit(message, None, 'EXACT_TF_TIMEOUT')
                continue
            self.pending.popleft()
            self._emit(message, transform, None)
        try:
            self.evidence.poll_selections()
        except Exception as error:
            self._capture_error(error)

    def _emit(self, message, transform, reason, *, publish=True):
        before = np.frombuffer(bytes(message.data), dtype='<u2').reshape(message.height, message.width)
        after = before
        effect = dict(active=False, reason=reason, changed_pixels=0)
        if transform is not None:
            q = transform.transform.rotation
            try:
                after, effect = apply_dropout(before, self.config, optical_axis_world((q.x, q.y, q.z, q.w)))
                t = transform.transform.translation
                effect['exact_transform'] = dict(translation=[t.x, t.y, t.z], rotation=[q.x, q.y, q.z, q.w],
                    stamp_ns=Time.from_msg(transform.header.stamp).nanoseconds)
            except ValueError as error:
                effect['reason'] = str(error)
        message.data = after.tobytes()
        effect['published'] = publish
        if publish:
            self.publish(message)
        # Capture failure cannot turn a diagnostic failure into a different sensor workload.
        try:
            self.evidence.record(Time.from_msg(message.header.stamp).nanoseconds,
                                 message.header.frame_id, before, after, effect)
        except Exception as error:
            self._capture_error(error)

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.timer.cancel()
        while self.pending:
            # rclpy's signal handler may already have shut down this context.
            # Preserve the unsubmitted frame as such; publishing cannot recover it.
            self._emit(self.pending.popleft()[0], None, 'TEARDOWN_PENDING_TF', publish=False)
        if self.errors:
            self.evidence.missing.extend(self.errors[:16])
        self.evidence.close()
