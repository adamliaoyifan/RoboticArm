#!/usr/bin/env python3
"""Bounded watchdog for the gz_ros2_control controller_manager.

The installed ``gz_ros2_control`` 0.7.20 fetches the model URDF with one
``AsyncParametersClient::get_parameters`` call and then waits indefinitely on
the returned future. In one of twenty measured launches that future did not
complete: the plugin logged ``connected to service!!`` and then neither
``Received URDF from param server`` nor its own retry message, and the
controller manager was never constructed. The transport/executor cause below
that observed boundary was not isolated (see
``docs/status/evidence/platform_free_height/2026-09-16_startup_rehearsal/``).

The plugin cannot be made to retry from here, so this watchdog makes the hang
*fast and named* instead of silent. Without it the two controller spawners sit
on their 60 s ``--controller-manager-timeout`` and the whole launch looks alive
for minutes while nothing will ever happen.

Exits 0 only after an actual ``ListControllers`` response, 1 on timeout.
The global graph also includes client-only service names, so advertisement is
not readiness. One persistent node; no ``ros2`` CLI subprocess, because per-poll node
construction adds discovery churn to the very window the plugin is racing in.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time

SERVICE_SUFFIX = "/list_controllers"

READY_MARKER = "wait_controller_manager: ok"
FAIL_MARKER = "startup_failed: controller_manager_not_ready"


def wait_for_controller_response(node, service_name, timeout_sec, *, recorder=None):
    """One typed read-only RPC within one total discovery/response deadline.

    This proves a responding manager, not active controllers, sensor readiness
    or the cause of an unavailable manager. Caller owns node/context teardown.
    """
    from rclpy.executors import SingleThreadedExecutor
    from controller_manager_msgs.srv import ListControllers
    if not math.isfinite(timeout_sec) or timeout_sec <= 0:
        raise ValueError('positive finite controller readiness timeout required')
    started=time.monotonic()
    deadline=started+timeout_sec
    client=node.create_client(ListControllers,service_name)
    executor=SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    future=None
    def result(ok,reason):
        return dict(ok=ok,reason=reason,elapsed_s=time.monotonic()-started,service=service_name)
    try:
        while not client.service_is_ready():
            remaining=deadline-time.monotonic()
            if remaining<=0:return result(False,'CONTROLLER_MANAGER_SERVICE_UNAVAILABLE')
            executor.spin_once(timeout_sec=min(.05,remaining))
        request=ListControllers.Request()
        if recorder:recorder('request',request)
        future=client.call_async(request)
        executor.spin_until_future_complete(future,timeout_sec=max(0.,deadline-time.monotonic()))
        if not future.done():return result(False,'CONTROLLER_MANAGER_RESPONSE_TIMEOUT')
        response=future.result()
        if response is None:return result(False,'CONTROLLER_MANAGER_RESPONSE_MISSING')
        if recorder:recorder('response',response)
        if time.monotonic()>deadline:return result(False,'CONTROLLER_MANAGER_RESPONSE_TIMEOUT')
        return result(True,'CONTROLLER_MANAGER_RESPONDED')
    finally:
        if future is not None and not future.done():
            client.remove_pending_request(future)
            future.cancel()
        node.destroy_client(client)
        executor.remove_node(node)
        executor.shutdown(timeout_sec=1.)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller-manager", default="/controller_manager")
    parser.add_argument("--timeout-sec", type=float, default=45.0)
    args = parser.parse_args(argv)

    service_name = args.controller_manager.rstrip("/") + SERVICE_SUFFIX

    import rclpy

    rclpy.init(args=None)
    node = rclpy.create_node("wait_controller_manager")
    evidence=os.environ.get('ELFIN_STARTUP_EVIDENCE_DIR','')
    folder=Path(evidence)/'controller_manager' if evidence else None
    records=[]
    if folder:folder.mkdir(parents=True,exist_ok=False)
    def record(name,message):
        from rclpy.serialization import serialize_message
        if folder:
            raw=serialize_message(message)
            filename=name+'.cdr'
            (folder/filename).write_bytes(raw)
            records.append(dict(file=filename,bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest()))
    outcome=None
    try:
        outcome=wait_for_controller_response(node,service_name,args.timeout_sec,recorder=record)
    finally:
        if folder:
            (folder/'result.json').write_text(json.dumps(dict(outcome=outcome,records=records,
                scope='Read-only manager response, not active-controller or sensor qualification'),indent=2))
        node.destroy_node()
        rclpy.try_shutdown()

    if outcome['ok']:
        print("%s %s after %.2fs" % (READY_MARKER, service_name, outcome['elapsed_s']))
        return 0
    print("%s: %s %s after %.2fs" % (FAIL_MARKER,service_name,outcome['reason'],outcome['elapsed_s']),
          file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
