"""Behavioral wire test of the installed MoveIt self-mask adapter."""
import os
from pathlib import Path
import subprocess
import time

import pytest


@pytest.mark.skipif(not os.environ.get('OBSERVER_SELF_MASK_EXECUTABLE'),
                    reason='explicit installed native self-mask gate')
def test_exact_shape_origin_preserves_external_points_and_refuses_missing_tf(tmp_path):
    import rclpy
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import TransformStamped
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import PointField, PointCloud2
    from sensor_msgs_py.point_cloud2 import create_cloud, read_points
    from std_msgs.msg import Header, String
    from tf2_msgs.msg import TFMessage
    import yaml

    # Two actual URDF collision bodies, including a nonzero collision origin.
    xml='''<robot name="mask"><link name="world"/><link name="tool"><collision>
    <origin xyz="0.2 0 0"/><geometry><box size="0.2 0.2 0.2"/></geometry>
    </collision></link><joint name="a" type="fixed"><parent link="world"/>
    <child link="tool"/></joint><link name="aux"><collision><geometry>
    <sphere radius="0.05"/></geometry></collision></link><joint name="b" type="fixed">
    <parent link="world"/><child link="aux"/></joint></robot>'''
    params=tmp_path/'params.yaml';params.write_text(yaml.safe_dump({'/**':{'ros__parameters':
        dict(robot_description=xml,robot_description_semantic='<robot name="mask"/>')}}))
    exe=Path(os.environ['OBSERVER_SELF_MASK_EXECUTABLE']);log=(tmp_path/'node.log').open('wb')
    env=dict(os.environ,ROS_DOMAIN_ID='180',ROS_LOCALHOST_ONLY='1')
    process=subprocess.Popen([str(exe),'--ros-args','--params-file',str(params)],env=env,
                             stdout=log,stderr=subprocess.STDOUT)
    old={key:os.environ.get(key) for key in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY')}
    os.environ.update(ROS_DOMAIN_ID='180',ROS_LOCALHOST_ONLY='1')
    context=rclpy.context.Context();rclpy.init(context=context)
    node=rclpy.create_node('self_mask_behavior',context=context)
    executor=rclpy.executors.SingleThreadedExecutor(context=context);executor.add_node(node)
    statuses=[];outputs=[]
    status_sub=node.create_subscription(String,'/luggage/observation/self_mask_status',
                                        lambda m:statuses.append(m.data),10)
    output_sub=node.create_subscription(PointCloud2,'/luggage/observation/lidar_points',
                                        outputs.append,10)
    pub=node.create_publisher(PointCloud2,'/luggage/observation/lidar_points_unmasked',10)
    tf_pub=node.create_publisher(TFMessage,'/tf_static',QoSProfile(depth=10,
        durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def until(predicate,seconds=5):
        deadline=time.monotonic()+seconds
        while not predicate() and time.monotonic()<deadline:executor.spin_once(timeout_sec=.02)
        assert predicate(),statuses
    def tf(child,x):
        msg=TransformStamped(header=Header(stamp=Time(sec=1),frame_id='sensor'),child_frame_id=child)
        msg.transform.translation.x=x;msg.transform.rotation.w=1.;return msg
    fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1)
            for i,name in enumerate(('x','y','z','intensity'))]
    points=[(1.2,0.,0.,11.),(-1.,0.,0.,12.),(1.301,0.,0.,13.),(2.,0.,0.,14.)]
    cloud=create_cloud(Header(stamp=Time(sec=1),frame_id='sensor'),fields,points)
    try:
        until(lambda:pub.get_subscription_count()>0)
        tf_pub.publish(TFMessage(transforms=[tf('tool',1.)]));time.sleep(.3)
        pub.publish(cloud)
        until(lambda:any('REFUSED' in s for s in statuses))
        assert not outputs,'missing one body transform must refuse the entire cloud'
        tf_pub.publish(TFMessage(transforms=[tf('tool',1.),tf('aux',-1.)]));time.sleep(.3)
        pub.publish(cloud);until(lambda:bool(outputs))
        result=outputs[-1]
        assert result.header==cloud.header and result.fields==cloud.fields
        assert [int(p['intensity']) for p in read_points(result)]==[13,14]
        assert any('robot_removed=2' in s and 'clipped=0' in s for s in statuses)
        assert result.width==2 and result.height==1 and len(result.data)==32
        from luggage_msgs.srv import SetObserverPayloadMask
        service=node.create_client(SetObserverPayloadMask,'/luggage/observation/set_payload_mask')
        until(lambda:service.service_is_ready())
        def transition(mode, operation='box1', frame='tool'):
            request=SetObserverPayloadMask.Request(state=mode,operation_id=operation,tool_frame=frame)
            request.geometry.geometry_digest='confirmed-measured-fixture'
            request.geometry.tool_conservative_translation=[.35,0.,0.]
            request.geometry.tool_conservative_quaternion_xyzw=[0.,0.,0.,1.]
            request.geometry.conservative_dimensions_wdh=[.1,.1,.1]
            future=service.call_async(request);until(future.done)
            return future.result()
        assert not transition(2).accepted,'cannot invent attachment without pending barrier'
        assert transition(1).accepted
        before=len(outputs);count=len(statuses)
        cloud.header.stamp=node.get_clock().now().to_msg();pub.publish(cloud)
        until(lambda:len(statuses)>count)
        assert len(outputs)==before and 'payload transition unresolved' in statuses[-1]
        assert not transition(2,operation='other-box').accepted
        assert transition(2).accepted
        cloud.header.stamp=node.get_clock().now().to_msg();attached_stamp=cloud.header.stamp
        pub.publish(cloud);until(lambda:len(outputs)>before)
        assert [int(p['intensity']) for p in read_points(outputs[-1])]==[14]
        assert any('payload_removed=1' in s for s in statuses)
        assert transition(1,operation='release-box1').accepted
        assert transition(0,operation='release-box1').accepted
        before=len(outputs);cloud.header.stamp=node.get_clock().now().to_msg();pub.publish(cloud)
        until(lambda:len(outputs)>before)
        assert [int(p['intensity']) for p in read_points(outputs[-1])]==[13,14]
        # Out-of-order delivery after release retains acquisition-time ownership.
        before=len(outputs);cloud.header.stamp=attached_stamp;pub.publish(cloud)
        until(lambda:len(outputs)>before)
        assert [int(p['intensity']) for p in read_points(outputs[-1])]==[14]
    finally:
        executor.shutdown();node.destroy_node();context.shutdown();process.terminate()
        try:process.wait(timeout=8.)
        except subprocess.TimeoutExpired:process.kill();process.wait()
        log.close()
        for key,value in old.items():
            if value is None:os.environ.pop(key,None)
            else:os.environ[key]=value
