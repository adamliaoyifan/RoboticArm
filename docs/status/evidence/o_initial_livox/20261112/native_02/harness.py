"""Actual O services and stamped TF; isolated process, no Gazebo or motion."""
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import pytest


def test_actual_owner_initialization_lidar_and_shared_placement_source(tmp_path):
    import rclpy
    import shutil
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
    from std_msgs.msg import Header
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster
    from ament_index_python.packages import get_package_prefix
    from uld_occ_observer.msg import UpdateReceipt
    from uld_occ_observer.srv import (InitializeEmpty, GetSnapshot, ReadCells,
        ReadSurface, ReleaseSnapshot, GetValidity, ResetObservation, IntegrateObservation)
    from test_joint_operation_observer import configuration
    from luggage_mapping.observer import identity_record, assemble_snapshot
    directory=Path(os.environ['O_INITIAL_NATIVE_EVIDENCE']);directory.mkdir(parents=True,exist_ok=False)
    config,geometry,config_file=configuration(directory)
    executable=Path(get_package_prefix('uld_occ_observer'))/'lib/uld_occ_observer/observer_node'
    args=[str(executable),'--ros-args','-p','geometry_file:='+str(geometry),'-p',
        'config_file:='+str(config_file),'-p','input_mode:=subscription',
        '-p','optical_frame:=test_optical','-p','cloud_topic:=/test/depth',
        '-p','lidar_cloud_topic:=/test/lidar','-p','lidar_frame:=test_lidar']
    manifest=dict(args=args,domain=os.environ.get('ROS_DOMAIN_ID'),
        binary_sha256=hashlib.sha256(executable.read_bytes()).hexdigest(),capture_complete=False)
    shutil.copyfile(__file__,directory/'harness.py')
    (directory/'manifest.json').write_text(json.dumps(manifest,indent=2))
    log=(directory/'observer.log').open('wb');process=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT)
    rclpy.init();node=rclpy.create_node('initial_lidar_probe')
    executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    clients={name:node.create_client(service,'/uld_occ_observer/'+name) for name,service in
        [('initialize_empty',InitializeEmpty),('get_snapshot',GetSnapshot),('read_cells',ReadCells),
         ('read_surface',ReadSurface),('release_snapshot',ReleaseSnapshot),('get_validity',GetValidity),
         ('reset_observation',ResetObservation),('integrate_observation',IntegrateObservation)]}
    count=0
    def record(name,msg):
        nonlocal count
        count+=1;(directory/('%03d_%s.cdr'%(count,name))).write_bytes(serialize_message(msg))
    def call(name,request):
        record(name+'_request',request);future=clients[name].call_async(request)
        deadline=time.monotonic()+10.
        while not future.done() and time.monotonic()<deadline:time.sleep(.01)
        assert future.done(),name+' timeout'
        response=future.result();record(name+'_response',response);return response
    receipts=[]
    sub=node.create_subscription(UpdateReceipt,'/uld_occ_observer/receipts',lambda r:(receipts.append(r),record('receipt',r)),100)
    pub=node.create_publisher(PointCloud2,'/test/lidar',10)
    depth_pub=node.create_publisher(PointCloud2,'/test/depth',10)
    broadcaster=StaticTransformBroadcaster(node)
    try:
        for client in clients.values():assert client.wait_for_service(timeout_sec=15.)
        current=call('get_validity',GetValidity.Request())
        request=InitializeEmpty.Request(expected_epoch=current.epoch,geometry_hash=config.descriptor()['geometry_hash'],
            config_digest=config.config_digest,declaration_id='user-declared-empty',declared_by='test_operator')
        declared=call('initialize_empty',request)
        assert declared.status=='OK' and declared.validity.state=='VALID',str(declared)
        assert declared.identity.observed_revision==0 and declared.identity.last_integrated_stamp_ns==0
        assert declared.identity.initialization_stamp_ns>0
        replay=call('initialize_empty',request)
        assert replay.identity==declared.identity and replay.validity.generation==declared.validity.generation
        lease=call('get_snapshot',GetSnapshot.Request(max_age_ms=1000));assert lease.status=='OK'
        states=call('read_cells',ReadCells.Request(handle=lease.handle,begin=[0,0,0],end=[4,4,4],limit=64))
        assert set(states.states)=={1} and set(states.evidence_sources)=={2}
        surface=call('read_surface',ReadSurface.Request(handle=lease.handle,cursor=0,limit=16))
        columns=[dict(x=x,y=y,active_count=a,known_count=k,has_occupied=o,top_z=z)
                 for x,y,a,k,o,z in zip(surface.x,surface.y,surface.active_count,surface.known_count,surface.has_occupied,surface.top_z)]
        captured=assemble_snapshot(config,identity_record(lease.identity),lease.validity.generation,
            columns,states.states,scene_revision='source-fixture',request_id='initial')
        assert captured.source.observed_revision==0 and captured.source.initialization_id=='user-declared-empty'
        assert b'initial_empty' in captured.surface.record_json
        # Configured physical frame without acquisition TF must not mutate O.
        deadline=time.monotonic()+5.
        while pub.get_subscription_count()==0 and time.monotonic()<deadline:time.sleep(.01)
        assert pub.get_subscription_count()>0
        missing=create_cloud_xyz32(Header(stamp=node.get_clock().now().to_msg(),frame_id='test_lidar'),[(.05,.05,-.15)])
        record('missing_tf_input',missing);pub.publish(missing)
        deadline=time.monotonic()+5.
        while not receipts and time.monotonic()<deadline:time.sleep(.01)
        assert receipts and not any(r.status=='INTEGRATED' for r in receipts)
        unchanged=call('get_snapshot',GetSnapshot.Request(max_age_ms=1000))
        assert unchanged.status=='OK' and unchanged.identity.map_revision==lease.identity.map_revision
        assert call('release_snapshot',ReleaseSnapshot.Request(handle=unchanged.handle)).status=='OK'
        # Physical laser frame: yaw90deg, translated0.9m above container datum.
        tf=TransformStamped();tf.header.frame_id='container_link';tf.child_frame_id='test_lidar'
        tf.transform.translation.z=.9;tf.transform.rotation.z=math.sin(math.pi/4);tf.transform.rotation.w=math.cos(math.pi/4)
        record('lidar_static_tf',tf);broadcaster.sendTransform(tf)
        deadline=time.monotonic()+5.
        while pub.get_subscription_count()==0 and time.monotonic()<deadline:time.sleep(.01)
        assert pub.get_subscription_count()>0
        time.sleep(.3)
        stamp=node.get_clock().now().to_msg()
        cloud=create_cloud_xyz32(Header(stamp=stamp,frame_id='test_lidar'),[(.05,.05,-.15)])
        record('lidar_input',cloud);pub.publish(cloud)
        deadline=time.monotonic()+8.
        while not any(r.sensor_id=='livox' and r.status=='INTEGRATED' for r in receipts) and time.monotonic()<deadline:time.sleep(.01)
        assert any(r.sensor_id=='livox' and r.status=='INTEGRATED' and r.hit_keys==1 for r in receipts),[r.status for r in receipts]
        latest=call('get_snapshot',GetSnapshot.Request(max_age_ms=1000));assert latest.status=='OK',str(latest)
        output=call('read_cells',ReadCells.Request(handle=latest.handle,begin=[0,0,0],end=[4,4,4],limit=64))
        expected=1+4*(2+4*1)
        assert output.states[expected]==2 and output.evidence_sources[expected]==1
        assert latest.identity.observed_revision==1 and latest.identity.last_integrated_stamp_ns==stamp.sec*10**9+stamp.nanosec
        assert latest.identity.initialization_id==declared.identity.initialization_id
        # Wrong input frame cannot be silently relabelled into a valid scan.
        cloud.header.frame_id='wrong_lidar_frame';cloud.header.stamp=node.get_clock().now().to_msg()
        record('wrong_frame_input',cloud);pub.publish(cloud);time.sleep(.2)
        wrong=call('get_snapshot',GetSnapshot.Request(max_age_ms=1000))
        assert wrong.status=='OK' and wrong.identity.map_revision==latest.identity.map_revision
        # Camera subscription shares the writer and its own physical optical frame.
        optical=TransformStamped();optical.header.frame_id='container_link';optical.child_frame_id='test_optical'
        optical.transform.translation.z=.6;optical.transform.rotation.w=1.
        record('camera_static_tf',optical);broadcaster.sendTransform(optical)
        time.sleep(.2)
        depth=create_cloud_xyz32(Header(stamp=node.get_clock().now().to_msg(),frame_id='test_optical'),[(.05,.05,.15)])
        record('camera_input',depth);depth_pub.publish(depth)
        deadline=time.monotonic()+8.
        while not any(r.sensor_id=='d555' and r.status=='INTEGRATED' for r in receipts) and time.monotonic()<deadline:time.sleep(.01)
        assert any(r.sensor_id=='d555' and r.status=='INTEGRATED' and r.identity.observed_revision==2 for r in receipts),[r.status for r in receipts]
        bad=InitializeEmpty.Request(expected_epoch=latest.identity.epoch,geometry_hash='wrong',
            config_digest=config.config_digest,declaration_id='bad',declared_by='operator')
        assert call('initialize_empty',bad).status=='INITIALIZATION_IDENTITY_MISMATCH'
        for handle in (lease.handle,latest.handle,wrong.handle):
            assert call('release_snapshot',ReleaseSnapshot.Request(handle=handle)).status=='OK'
        reset=call('reset_observation',ResetObservation.Request(expected_epoch=latest.identity.epoch,reset_id='post-cargo'))
        assert reset.status=='OK'
        request.expected_epoch=reset.epoch
        assert call('initialize_empty',request).status=='INITIALIZATION_ALREADY_USED'
        manifest.update(passed=True,capture_complete=True,replay_possible=True,
            expected_occupied_index_xyz=[1,2,1],scope='actual O owner and TF, no simulator')
    except Exception as error:
        manifest.update(passed=False,failure_reason=type(error).__name__+': '+str(error),
                        capture_complete=True,replay_possible=True)
        raise
    finally:
        executor.shutdown(timeout_sec=5.);thread.join(timeout=5.);node.destroy_node();rclpy.shutdown()
        if process.poll() is None:process.send_signal(signal.SIGINT)
        try:process.wait(timeout=8.)
        except subprocess.TimeoutExpired:process.terminate();process.wait(timeout=5.)
        log.close();manifest['exit_code']=process.returncode;manifest['executor_joined']=not thread.is_alive()
        (directory/'manifest.json').write_text(json.dumps(manifest,indent=2))
        assert process.returncode==0 and not thread.is_alive()
