"""Real ROS held-response cases; reproduce a boundary, not a DDS root cause."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import threading
import time

import pytest
import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.serialization import serialize_message


def load_watchdog(implementation):
    path = (Path(os.environ['BASELINE_WATCHDOG_PATH']) if implementation == 'reference' else
            Path(__file__).resolve().parents[1]/'scripts/wait_controller_manager.py')
    spec = importlib.util.spec_from_file_location('recovery_watchdog_'+implementation, path)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('implementation', ['reference','current'])
@pytest.mark.parametrize('scenario', ['held_first','late_first'])
def test_actual_held_response_and_independent_manager(implementation,scenario,tmp_path):
    if implementation == 'reference' and scenario == 'late_first':
        pytest.skip('reference reproducer only needs held_first')
    if implementation == 'reference' and not os.environ.get('BASELINE_WATCHDOG_PATH'):
        pytest.skip('requires exact archived reference watchdog')
    directory=Path(os.environ.get('LOADING_STARTUP_RECOVERY_EVIDENCE',str(tmp_path)))/(implementation+'_'+scenario)
    directory.mkdir(parents=True,exist_ok=False)
    context=rclpy.context.Context();rclpy.init(context=context,domain_id=177)
    owner=rclpy.create_node('probe_recovery_owner',context=context)
    probe=rclpy.create_node('probe_recovery_client',context=context)
    observer=rclpy.create_node('probe_independent_client',context=context)
    name='/probe_recovery_'+implementation+'_'+scenario+'/list_controllers'
    release=threading.Event();first_seen=threading.Event();retry_seen=threading.Event()
    lock=threading.Lock();calls=[];records=[];outcome={};errors=[]
    def callback(request,response):
        with lock:
            index=len(calls)+1;calls.append(dict(index=index,monotonic=time.monotonic()))
        if index == 1:
            first_seen.set()
            (retry_seen if scenario == 'late_first' else release).wait(timeout=5.)
        elif scenario == 'late_first':
            retry_seen.set();release.wait(timeout=5.)
        return response
    service=owner.create_service(ListControllers,name,callback,callback_group=ReentrantCallbackGroup())
    independent=observer.create_client(ListControllers,name)
    owner_executor=MultiThreadedExecutor(num_threads=4,context=context);owner_executor.add_node(owner)
    observer_executor=SingleThreadedExecutor(context=context);observer_executor.add_node(observer)
    server_worker=threading.Thread(target=owner_executor.spin);server_worker.start()
    watchdog=load_watchdog(implementation)
    def record(label,message):
        raw=serialize_message(message);filename=label+'.cdr'
        (directory/filename).write_bytes(raw)
        records.append(dict(file=filename,sha256=hashlib.sha256(raw).hexdigest(),bytes=len(raw)))
    def run_probe():
        try:outcome.update(watchdog.wait_for_controller_response(probe,name,2.2,recorder=record))
        except BaseException as error:errors.append(repr(error))
    worker=threading.Thread(target=run_probe)
    try:
        assert independent.wait_for_service(timeout_sec=3.)
        worker.start();assert first_seen.wait(timeout=1.5)
        if scenario == 'held_first':
            query=ListControllers.Request();record('independent_request',query)
            future=independent.call_async(query)
            observer_executor.spin_until_future_complete(future,timeout_sec=1.)
            assert future.done(),'independent client must demonstrate responding manager'
            record('independent_response',future.result())
        worker.join(timeout=3.)
        assert not worker.is_alive() and not errors
        assert outcome['elapsed_s'] < 2.7
        if implementation == 'reference':
            assert not outcome['ok'] and outcome['reason'] == 'CONTROLLER_MANAGER_RESPONSE_TIMEOUT'
            assert len(calls) == 2  # Held probe plus actually answered independent query.
        else:
            assert outcome['ok'] and outcome['reason'] == 'CONTROLLER_MANAGER_RESPONDED'
            assert outcome['attempt_count'] == 2
            assert outcome['response_attempt'] == (1 if scenario == 'late_first' else 2)
            assert len(calls) == (2 if scenario == 'late_first' else 3)
    finally:
        release.set();retry_seen.set()
        if worker.ident is not None:worker.join(timeout=3.)
        observer_executor.shutdown(timeout_sec=1.)
        owner_executor.shutdown(timeout_sec=2.);server_worker.join(timeout=2.)
        observer.destroy_client(independent);owner.destroy_service(service)
        probe.destroy_node();observer.destroy_node();owner.destroy_node();context.try_shutdown()
        (directory/'result.json').write_text(json.dumps(dict(implementation=implementation,scenario=scenario,
            outcome=outcome,calls=calls,records=records,errors=errors,
            worker_stopped=not worker.is_alive() and not server_worker.is_alive(),
            scope='Programmed held response; no claim about actual DDS loss mechanism'),indent=2)+'\n')
    assert not worker.is_alive() and not server_worker.is_alive()


def test_never_responding_retries_share_one_budget(tmp_path):
    module=load_watchdog('current')
    context=rclpy.context.Context();rclpy.init(context=context,domain_id=177)
    owner=rclpy.create_node('silent_probe_owner',context=context)
    probe=rclpy.create_node('silent_probe_client',context=context)
    name='/silent_recovery/list_controllers'
    owner.create_service(ListControllers,name,lambda request,response:response)
    records=[];outcome=None;clients=[]
    create=probe.create_client
    def capture_client(*args,**kwargs):
        client=create(*args,**kwargs);clients.append(client);return client
    probe.create_client=capture_client
    try:
        until=time.monotonic()+3.
        while not any(n==name for n,_ in probe.get_service_names_and_types()):
            assert time.monotonic()<until;time.sleep(.01)
        outcome=module.wait_for_controller_response(probe,name,.6,retry_interval_sec=.1,
            recorder=lambda label,message:records.append(label))
        assert not outcome['ok'] and outcome['reason']=='CONTROLLER_MANAGER_RESPONSE_TIMEOUT'
        assert outcome['elapsed_s']<1.1 and 1<=outcome['attempt_count']<=6
        assert len(records)==outcome['attempt_count']
        assert all(not c._pending_requests for c in clients)
    finally:
        probe.destroy_node();owner.destroy_node();context.try_shutdown()
        directory=Path(os.environ.get('LOADING_STARTUP_RECOVERY_EVIDENCE',str(tmp_path)))/'never_responding'
        directory.mkdir(parents=True,exist_ok=False)
        (directory/'result.json').write_text(json.dumps(dict(outcome=outcome,labels=records,worker_stopped=True),indent=2)+'\n')


@pytest.mark.parametrize('interval',[0.,-1.,float('inf'),float('nan')])
def test_invalid_retry_interval_never_creates_client(interval):
    with pytest.raises(ValueError):
        load_watchdog('current').wait_for_controller_response(None,'/unused',1.,retry_interval_sec=interval)
