"""Admit a runner-owned controller-probe receipt before starting a cargo job.

This does not authorize motion or replace active-controller/sensor/scene checks.
Only the explicit-start caller supplies an operation callback. The evidence
folder must be fresh and owned by that caller's current launch.
"""
import hashlib
import json
import time


def _read_probe_receipt(folder):
    from controller_manager_msgs.srv import ListControllers
    from rclpy.serialization import deserialize_message
    try:
        receipt=json.loads((folder/'result.json').read_text())
    except (FileNotFoundError,json.JSONDecodeError):
        return None
    if not isinstance(receipt,dict) or not isinstance(receipt.get('outcome'),dict):
        return dict(ok=False,reason='STARTUP_CONTROLLER_PROBE_EVIDENCE_INVALID',detail='invalid receipt structure')
    outcome=receipt['outcome']
    if not outcome.get('ok') or outcome.get('reason')!='CONTROLLER_MANAGER_RESPONDED':
        return dict(ok=False,reason='STARTUP_CONTROLLER_PROBE_REFUSED',probe=outcome)
    requests=[];responses=[]
    try:
        for row in receipt['records']:
            name=row['file']
            if '/' in name or not name.endswith('.cdr'):
                raise ValueError('unexpected probe artifact path')
            raw=(folder/name).read_bytes()
            if len(raw)!=row['bytes'] or hashlib.sha256(raw).hexdigest()!=row['sha256']:
                raise ValueError('probe artifact hash or size mismatch')
            if name=='request.cdr' or name.startswith('request_'):
                deserialize_message(raw,ListControllers.Request);requests.append(name)
            elif name=='response.cdr' or name.startswith('response_'):
                deserialize_message(raw,ListControllers.Response);responses.append(name)
            else:raise ValueError('unexpected probe artifact label')
        if not requests or len(responses)!=1:
            raise ValueError('actual request and exactly one accepted response required')
        accepted=outcome['response_attempt'];attempts=outcome['attempt_count']
        if type(accepted) is not int or type(attempts) is not int or not 1<=accepted<=attempts:
            raise ValueError('accepted response attempt missing')
        expected_request='request.cdr' if accepted==1 else 'request_'+str(accepted)+'.cdr'
        expected_response='response.cdr' if accepted==1 else 'response_'+str(accepted)+'.cdr'
        if len(requests)!=attempts or expected_request not in requests or responses!=[expected_response]:
            raise ValueError('accepted response does not match a captured request')
    except (KeyError,TypeError,ValueError,OSError,RuntimeError) as error:
        return dict(ok=False,reason='STARTUP_CONTROLLER_PROBE_EVIDENCE_INVALID',detail=str(error))
    return dict(ok=True,reason='STARTUP_CONTROLLER_PROBE_VERIFIED',probe=outcome,
                request_files=requests,response_file=responses[0])


def run_after_controller_probe(folder,launch,*,deadline,on_ready,on_admission=None,
                               clock=time.monotonic,sleep=time.sleep):
    """Wait under the caller's deadline; call on_ready exactly once on admission.

    Returns (work_exit_code, admission). A failed/terminal launch, bad receipt or
    deadline invokes no operation. on_admission records the result BEFORE any
    operation callback so a later interrupted job retains the startup decision.
    """
    started=clock()
    while True:
        if launch.poll() is not None:
            admission=dict(ok=False,reason='STARTUP_LAUNCH_TERMINAL')
            break
        if clock()>=deadline:
            admission=dict(ok=False,reason='STARTUP_CONTROLLER_PROBE_DEADLINE')
            break
        admission=_read_probe_receipt(folder)
        if admission is not None:break
        sleep(min(.05,max(0.,deadline-clock())))
    if admission['ok'] and (clock()>=deadline or launch.poll() is not None):
        admission=dict(ok=False,reason='STARTUP_REVOKED_BEFORE_DISPATCH')
    admission['elapsed_s']=clock()-started
    if on_admission:on_admission(admission)
    return (on_ready() if admission['ok'] else 1),admission
