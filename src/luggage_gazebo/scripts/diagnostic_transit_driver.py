#!/usr/bin/env python3
"""Explicit single-trial entry; wiring tests do not invoke main or any motion.

Requires diagnostic_capture launch routing plus a separately running capture
node. The normal pack startup uses an explicit first-trial exit hook. Outcomes
retain the box for the existing carrying recovery flow, including failure.
"""
import argparse
import json
import os
from pathlib import Path
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from pack_eval_driver import PackEvalDriver, merge_args
from luggage_gazebo.transit_capture_exit import TransitCaptureExit
from luggage_gazebo.place_metrics import PlaceTrial
from luggage_planning.capture.async_boundary_capture import AsyncBoundaryCapture


def install_capture_exit(driver, directory, run_id):
    fault_publisher = driver.create_publisher(String, '/diagnostic_capture/fault', 10)

    def report_fault(name, detail):
        driver.get_logger().error(name + ': ' + detail)
        fault_publisher.publish(String(data=json.dumps(dict(
            run_id=run_id, name=name, detail=detail))))

    recorder = AsyncBoundaryCapture(directory, run_id=run_id, on_error=report_fault)
    driver._diagnostic_fault_publisher = fault_publisher
    last_health = {}

    def health(message):
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict):
                raise ValueError('capture health must be an object')
            last_health.update(data=data, received=time.monotonic())
        except (ValueError, TypeError) as error:
            recorder.fault('CAPTURE_HEALTH_INVALID', error)

    driver._diagnostic_health_subscription = driver.create_subscription(
        String, '/diagnostic_capture/health', health, 10)

    def healthy():
        status = last_health.get('data', {})
        return (recorder.healthy and bool(status) and status.get('run_id') == run_id
                and not status.get('errors')
                and not status.get('closed') and status.get('worker_alive')
                and status.get('targets_ready')
                and time.monotonic() - last_health.get('received', 0.) < 2.)

    typed = driver._typed
    previous = getattr(typed, '_boundary_recorder', None)

    def boundary(name, message):
        if not recorder.record('driver:' + name, message):
            raise RuntimeError('DIAGNOSTIC_CAPTURE_UNHEALTHY: ' + name)
        if previous is not None:
            previous(name, message)

    typed.set_boundary_recorder(boundary)
    driver._transit_capture_exit = TransitCaptureExit(
        healthy=healthy, now_ns=lambda: driver.get_clock().now().nanoseconds)

    def trial_exit(record, fail, index):
        if record is None:
            record = PlaceTrial(index=index)
            record.extras = {'diagnostic_exception': fail}
            record = driver._abort(record, fail or 'DIAGNOSTIC_NO_TRIAL', True)
        driver._diagnostic_trial_result = record
        return ('DIAGNOSTIC_COMPLETE' if record.extras.get('diagnostic_complete')
                else 'DIAGNOSTIC_ABORT')

    driver._diagnostic_trial_exit = trial_exit
    return recorder, healthy


def main(argv=None):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--capture-directory', required=True)
    parser.add_argument('--capture-run-id', required=True)
    capture_args, remaining = parser.parse_known_args(argv)
    args = merge_args(remaining)
    if args.plan_only or not args.frame_runtime:
        raise SystemExit('Diagnostic requires unchanged production frame-runtime motion mode')
    os.makedirs(args.out, exist_ok=True)
    os.makedirs(args.dump_dir, exist_ok=True)
    rclpy.init(args=['--ros-args', '-r',
                     '/validate_route_v2:=/diagnostic_capture/validate_route_v2'])
    driver = PackEvalDriver(args)
    recorder, healthy = install_capture_exit(driver, capture_args.capture_directory,
                                              capture_args.capture_run_id)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(driver)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    result = None
    try:
        deadline = time.monotonic() + 5.
        while not healthy() and time.monotonic() < deadline:
            time.sleep(.02)
        if not healthy():
            raise RuntimeError('DIAGNOSTIC_CAPTURE_NOT_READY: no trial started')
        # Preserve production startup, home/spawn ordering and readiness gates.
        # The installed hook returns before any commit, cleanup or next cycle.
        driver.run()
        result = getattr(driver, '_diagnostic_trial_result', None)
    finally:
        capture_health = recorder.close()
        Path(args.out, 'diagnostic_capture_result.json').write_text(json.dumps(dict(
            fail_code=getattr(result, 'fail_code', 'DIAGNOSTIC_EXCEPTION'),
            place_state=getattr(result, 'place_state', ''),
            driver_capture_health=capture_health, automatic_retry=False,
            box_retained=True, dispatch_ready=False), indent=2))
        executor.shutdown()
        spinner.join(timeout=5.)
        driver.destroy_node()
        rclpy.shutdown()
    return 0 if (getattr(result, 'extras', {}).get('diagnostic_complete')
                  and capture_health['recorder_io_complete']) else 1


if __name__ == '__main__':
    raise SystemExit(main())
