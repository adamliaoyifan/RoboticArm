"""Own an eval driver's ROS executor and preserve failure chronology.

Runtime callback exceptions remain fatal. Stopping and joining executors
before destroying their nodes prevents teardown from racing live callbacks.
"""
import json
from pathlib import Path
import threading
import time
import traceback

from luggage_gazebo.eval_lifecycle import close_driver_workers


class DriverExecutor:
    def __init__(self, executor, driver, out):
        self.executor = executor
        self.driver = driver
        self.out = Path(out)
        self.phase = "created"
        self.failure = None
        self._lock = threading.Lock()
        self.thread = threading.Thread(target=self._spin, daemon=True)

    def _record(self, event, **fields):
        with self._lock:
            row = dict(event=event, phase=self.phase,
                       mono=time.monotonic(), wall=time.time())
            row.update(fields)
            with (self.out / "executor_lifecycle.jsonl").open("a") as handle:
                handle.write(json.dumps(row, sort_keys=True) + "\n")

    def start(self):
        self.phase = "running"
        self._record("start")
        self.thread.start()

    def _spin(self):
        try:
            self.executor.spin()
        except BaseException as exc:
            failure_type = type(exc).__name__
            failure_phase = self.phase
            text = traceback.format_exc()
            (self.out / "executor_crash.txt").write_text(text)
            self._record("exception", exception_type=failure_type,
                         phase=failure_phase,
                         message=str(exc),
                         joint_state=getattr(self.driver, "_last_joint_state", None),
                         controller_messages=getattr(
                             self.driver, "_controller_state_count", None))
            # Publish failure only after its traceback and phase are durable.
            self.failure = failure_type
        finally:
            self._record("spin_return", failed=self.failure is not None)

    def close(self):
        self.phase = "stopping"
        self._record("stop_requested")
        close_driver_workers(self.driver, self.executor, self.thread)
        self.phase = "closed"
        self._record("nodes_destroyed")
