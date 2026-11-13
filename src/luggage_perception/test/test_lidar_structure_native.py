"""Installed O subscription consumes additive structure-terminal labels."""
import os
from pathlib import Path
import pytest


@pytest.mark.skipif(not os.environ.get('STRUCTURE_OBSERVER_EXECUTABLE'),reason='explicit native wire gate')
def test_installed_writer_preserves_ray_and_ordinary_hit(tmp_path):
    import hashlib,json,subprocess,time
    import rclpy
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from rclpy.serialization import serialize_message
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import PointCloud2,PointField
    from sensor_msgs_py.point_cloud2 import create_cloud
    from std_msgs.msg import Header
    from tf2_msgs.msg import TFMessage
    from uld_occ_observer.msg import UpdateReceipt
    import yaml
    from luggage_description.container_geometry import cuboid_from_inner_size
    descriptor=json.dumps(cuboid_from_inner_size((.4,.4,.4),floor_z=.53).descriptor(),sort_keys=True,separators=(',',':'))
    geometry=tmp_path/'geometry.yaml';geometry.write_text(yaml.safe_dump(dict(schema='uld_geometry_export_v1',
        descriptor_json=descriptor,descriptor_sha256=hashlib.sha256(descriptor.encode()).hexdigest())))
    config=tmp_path/'observer.yaml';config.write_text(yaml.safe_dump(dict(resolution_m=.02,
        depth_z_min_m=.1,depth_z_max_m=6.,clock_domain='ros')))
    log=(tmp_path/'node.log').open('wb')
    cmd=[os.environ['STRUCTURE_OBSERVER_EXECUTABLE'],'--ros-args','-p','geometry_file:='+str(geometry),
        '-p','config_file:='+str(config),'-p','input_mode:=subscription','-p','lidar_cloud_topic:=/structure_scan',
        '-p','lidar_frame:=lidar']
    process=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,env=dict(os.environ))
    context=rclpy.context.Context();rclpy.init(context=context)
    node=rclpy.create_node('structure_wire_probe',context=context)
    executor=rclpy.executors.SingleThreadedExecutor(context=context);executor.add_node(node)
    receipts=[]
    subscription=node.create_subscription(UpdateReceipt,'/uld_occ_observer/receipts',receipts.append,10)
    publisher=node.create_publisher(PointCloud2,'/structure_scan',10)
    tfpub=node.create_publisher(TFMessage,'/tf_static',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def until(predicate,timeout=15.):
        limit=time.monotonic()+timeout
        while not predicate():
            if process.poll() is not None:raise AssertionError('observer exited; see '+str(tmp_path/'node.log'))
            assert time.monotonic()<limit,'wire deadline; receipts='+str([(r.status,r.hit_keys) for r in receipts])
            executor.spin_once(timeout_sec=.05)
    try:
        until(lambda:publisher.get_subscription_count()>0)
        tf=TransformStamped();tf.header.frame_id='container_link';tf.child_frame_id='lidar'
        tf.transform.translation.z=.9;tf.transform.rotation.w=1.
        tfpub.publish(TFMessage(transforms=[tf]))
        for _ in range(10):executor.spin_once(timeout_sec=.05)
        fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z'))]
        fields.append(PointField(name='structure_terminal',offset=12,datatype=PointField.UINT8,count=1))
        for number,labels,expected_hits in ((1,(1,0),1),(2,(0,0),2)):
            stamp=node.get_clock().now().to_msg()
            cloud=create_cloud(Header(stamp=stamp,frame_id='lidar'),fields,
                [(0.,0.,-.36997,labels[0]),(.1,0.,-.2,labels[1])])
            (tmp_path/f'input_{number}.cdr').write_bytes(serialize_message(cloud))
            before=len(receipts);publisher.publish(cloud)
            until(lambda:any(r.raw_points==2 for r in receipts[before:]))
            receipt=next(r for r in receipts[before:] if r.raw_points==2)
            (tmp_path/f'receipt_{number}.cdr').write_bytes(serialize_message(receipt))
            assert receipt.hit_keys==expected_hits
            assert receipt.free_keys>0 and receipt.accepted_points==2
        (tmp_path/'result.json').write_text(json.dumps(dict(command=cmd,outcome='pass',
            labelled_hit_keys=1,ordinary_hit_keys=2,raw_endpoints=2,teardown='finally')))
    finally:
        executor.shutdown();node.destroy_node();context.shutdown();process.terminate()
        try:process.wait(timeout=8.)
        except subprocess.TimeoutExpired:process.kill();process.wait()
        log.close()
