#!/usr/bin/env python3
"""Native service functional rehearsal; no MoveIt/Gazebo or motion action.

This checks the native service boundary only, not complete F1/F2 acceptance.
The capture provides exact box geometry, not measured suction-patch authority.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--native-probe', type=Path, required=True)
    parser.add_argument('--checker', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    import yaml
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.serialization import deserialize_message, serialize_message
    from moveit_msgs.srv import GetCartesianPath
    from moveit_msgs.msg import PlanningScene
    from geometry_msgs.msg import Pose
    from luggage_msgs.srv import CheckPlacementToolV2
    import copy
    model_text = (args.capture/'robot.urdf').read_text()
    semantic = (args.capture/'robot.srdf').read_text()
    model_hash = hashlib.sha256((model_text+semantic).encode()).hexdigest()
    parameters = args.output/'parameters.yaml'
    parameters.write_text(yaml.safe_dump({'placement_tool_check':{'ros__parameters':{
        'robot_description':model_text,'robot_description_semantic':semantic}}}))
    material = subprocess.check_output([str(args.native_probe), str(args.capture)], text=True)
    (args.output/'native_probe.log').write_text(material)
    rows = material.splitlines()
    box_row = next(row.split() for row in rows if row.startswith('BOX size '))
    quat = next(row.split()[1:] for row in rows if row.startswith('BOX_QUAT '))
    dims = list(map(float,box_row[2:5]));xyz=list(map(float,box_row[6:9]))
    original = deserialize_message((args.capture/'req-1a/edge-0000.request.cdr').read_bytes(),GetCartesianPath.Request)
    scene_bytes = (args.capture/'req-1a/edge-0000.scene.cdr').read_bytes()
    scene = deserialize_message(scene_bytes,PlanningScene)
    box=Pose();box.position.x,box.position.y,box.position.z=xyz
    box.orientation.x,box.orientation.y,box.orientation.z,box.orientation.w=map(float,quat)
    request=CheckPlacementToolV2.Request()
    request.request_id='historical-endpoint-service-replay'
    request.snapshot_id=hashlib.sha256(scene_bytes).hexdigest()
    request.model_sha256=model_hash;request.frame_id='world'
    request.tcp_link=original.link_name;request.payload_id=scene.robot_state.attached_collision_objects[0].object.id
    request.scene=scene;request.box_wdh=dims;request.budget_s=10.
    request.tcp_poses=[copy.deepcopy(original.waypoints[-1]) for _ in range(2)]
    request.tcp_poses[1].position.x+=.02
    request.box_poses=[copy.deepcopy(box),copy.deepcopy(box)]
    (args.output/'request.cdr').write_bytes(serialize_message(request))
    log=(args.output/'checker.log').open('w')
    proc=subprocess.Popen([str(args.checker),'--ros-args','--params-file',str(parameters)],
                          stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    node=None;executor=None;spin=None
    report={'completed':False,'simulation':False,'motion':False,'pid':proc.pid,
            'model_sha256':model_hash,'scene_sha256':request.snapshot_id,
            'worktree_revision':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
            'worktree_dirty':subprocess.check_output(['git','status','--porcelain'],text=True),
            'started':time.time()}
    try:
        rclpy.init();node=rclpy.create_node('prior_tool_service_replay')
        executor=MultiThreadedExecutor(num_threads=2);executor.add_node(node)
        spin=threading.Thread(target=executor.spin,daemon=True);spin.start()
        client=node.create_client(CheckPlacementToolV2,'/placement_prior/check_tool')
        if not client.wait_for_service(timeout_sec=15.):raise RuntimeError('checker startup timeout')
        future=client.call_async(request);ready=threading.Event();future.add_done_callback(lambda f:ready.set())
        if not ready.wait(15.):raise RuntimeError('checker response timeout')
        response=future.result()
        (args.output/'response.cdr').write_bytes(serialize_message(response))
        report.update(complete=response.complete,reason=response.reason,valid=list(response.valid),
                      contacts=[{'sample':c.sample_index,'a':c.body_1,'b':c.body_2} for c in response.contacts],
                      service_time_s=response.service_time_s)
        assert response.request_id==request.request_id and response.snapshot_id==request.snapshot_id
        assert response.model_sha256==model_hash
        assert response.complete and list(response.valid)==[False,True]
        assert any({c.body_1,c.body_2}=={'airport_container_real','mid360_mount_frame'} and c.sample_index==0 for c in response.contacts)
        report['completed']=True
    finally:
        if executor:executor.shutdown(timeout_sec=2.)
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if spin:spin.join(2.)
        os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=5.)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=5.)
        report['returncode']=proc.returncode;report['ended']=time.time();log.close()
        (args.output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
        inventory=[]
        for p in sorted(args.output.iterdir()):
            if p.is_file():inventory.append({'path':p.name,'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'bytes':p.stat().st_size})
        (args.output/'inventory.json').write_text(json.dumps(inventory,indent=2)+'\n')
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
