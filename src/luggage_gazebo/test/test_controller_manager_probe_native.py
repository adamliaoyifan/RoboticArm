"""Actual ROS service readiness: client-only graphs must never pass."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import threading
import time

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.serialization import serialize_message
from controller_manager_msgs.srv import ListControllers


SCRIPT=Path(__file__).resolve().parents[1]/'scripts/wait_controller_manager.py'
SPEC=importlib.util.spec_from_file_location('tested_controller_manager_watchdog',SCRIPT)
WATCHDOG=importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(WATCHDOG)


@pytest.mark.parametrize('case',['client_only','responding','unresponsive'])
def test_actual_response_required_and_bounded(case,tmp_path):
    directory=Path(os.environ.get('LOADING_STARTUP_TEST_EVIDENCE',str(tmp_path)))/case
    directory.mkdir(parents=True,exist_ok=False)
    context=Context()
    rclpy.init(context=context,domain_id=178)
    probe=rclpy.create_node('controller_probe_'+case,context=context)
    owner=rclpy.create_node('controller_probe_fixture_'+case,context=context)
    name='/controller_probe_'+case+'/list_controllers'
    executor=SingleThreadedExecutor(context=context)
    worker=None
    calls=[]
    records=[]
    outcome=None
    def callback(request,response):
        calls.append(serialize_message(request))
        return response  # Empty list is responsive; active-state gate is separate.
    if case!='client_only':
        owner.create_service(ListControllers,name,callback)
    # This client alone is enough for the old graph-name predicate to pass.
    ghost=owner.create_client(ListControllers,name)
    try:
        deadline=time.monotonic()+2.
        while not any(n==name for n,_ in probe.get_service_names_and_types()):
            assert time.monotonic()<deadline,'fixture service name not discovered'
            time.sleep(.01)
        if case=='responding':
            executor.add_node(owner)
            worker=threading.Thread(target=executor.spin)
            worker.start()
        def record(label,message):
            raw=serialize_message(message)
            filename=label+'.cdr'
            (directory/filename).write_bytes(raw)
            records.append(dict(file=filename,bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest()))
        outcome=WATCHDOG.wait_for_controller_response(probe,name,1.,recorder=record)
        assert outcome['elapsed_s']<1.5
        if case=='responding':
            assert outcome['ok'] and outcome['reason']=='CONTROLLER_MANAGER_RESPONDED'
            assert len(calls)==1
            assert [r['file'] for r in records]==['request.cdr','response.cdr']
        elif case=='client_only':
            assert not outcome['ok'] and outcome['reason']=='CONTROLLER_MANAGER_SERVICE_UNAVAILABLE'
            assert not records and not calls
            assert not ghost.service_is_ready()
        else:
            assert ghost.service_is_ready()
            assert not outcome['ok'] and outcome['reason']=='CONTROLLER_MANAGER_RESPONSE_TIMEOUT'
            assert [r['file'] for r in records]==['request.cdr']
            assert not calls
    finally:
        executor.shutdown(timeout_sec=1.)
        if worker:worker.join(timeout=1.)
        probe.destroy_node()
        owner.destroy_node()
        context.try_shutdown()
        (directory/'result.json').write_text(json.dumps(dict(case=case,outcome=outcome,
            records=records,worker_stopped=worker is None or not worker.is_alive(),
            scope='Real generated service conformance; no Gazebo, no reliability claim'),indent=2))
    assert worker is None or not worker.is_alive()
