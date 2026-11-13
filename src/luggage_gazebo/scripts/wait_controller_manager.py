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


def wait_for_controller_response(node, service_name, timeout_sec, *, recorder=None,
                                 retry_interval_sec=1.0):
    """Typed read-only probes within one total discovery/response deadline.

    This proves a responding manager, not active controllers, sensor readiness
    or the cause of an unavailable manager. Caller owns node/context teardown.
    A pending request alone does not prove manager absence. Additional queries
    share the original deadline and client; an earlier delayed reply stays
    eligible. The finite deadline/interval bound the number of pending queries.
    """
    from rclpy.executors import SingleThreadedExecutor
    from controller_manager_msgs.srv import ListControllers
    if not math.isfinite(timeout_sec) or timeout_sec <= 0:
        raise ValueError('positive finite controller readiness timeout required')
    if not math.isfinite(retry_interval_sec) or retry_interval_sec <= 0:
        raise ValueError('positive finite controller probe retry interval required')
    started=time.monotonic()
    deadline=started+timeout_sec
    client=node.create_client(ListControllers,service_name)
    executor=SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    futures=[]
    next_request=started
    def result(ok,reason,response_attempt=None):
        return dict(ok=ok,reason=reason,elapsed_s=time.monotonic()-started,service=service_name,
                    attempt_count=len(futures),response_attempt=response_attempt,
                    timeout_sec=timeout_sec,retry_interval_sec=retry_interval_sec)
    try:
        while time.monotonic()<deadline:
            for index,future in enumerate(futures,1):
                if not future.done():continue
                response=future.result()
                if response is None:return result(False,'CONTROLLER_MANAGER_RESPONSE_MISSING')
                if recorder:recorder('response' if index==1 else 'response_'+str(index),response)
                if time.monotonic()>deadline:return result(False,'CONTROLLER_MANAGER_RESPONSE_TIMEOUT')
                return result(True,'CONTROLLER_MANAGER_RESPONDED',index)
            now=time.monotonic()
            if now>=deadline:break
            if client.service_is_ready() and now>=next_request:
                request=ListControllers.Request()
                index=len(futures)+1
                if recorder:recorder('request' if index==1 else 'request_'+str(index),request)
                futures.append(client.call_async(request))
                next_request=now+retry_interval_sec
            # One executor/node; no CLI discovery polling or renewed deadline.
            remaining=deadline-time.monotonic()
            if remaining>0:executor.spin_once(timeout_sec=min(.05,remaining))
        return result(False,'CONTROLLER_MANAGER_RESPONSE_TIMEOUT' if futures else
                      'CONTROLLER_MANAGER_SERVICE_UNAVAILABLE')
    finally:
        for future in futures:
            if not future.done():
                client.remove_pending_request(future)
                future.cancel()
        node.destroy_client(client)
        executor.remove_node(node)
        executor.shutdown(timeout_sec=1.)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller-manager", default="/controller_manager")
    parser.add_argument("--timeout-sec", type=float, default=45.0)
    parser.add_argument("--retry-interval-sec", type=float, default=1.0)
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
        outcome=wait_for_controller_response(node,service_name,args.timeout_sec,recorder=record,
            retry_interval_sec=args.retry_interval_sec)
    finally:
        if folder:
            temporary=folder/'result.json.tmp'
            temporary.write_text(json.dumps(dict(outcome=outcome,records=records,
                scope='Read-only manager response, not active-controller or sensor qualification'),indent=2))
            temporary.replace(folder/'result.json')
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
