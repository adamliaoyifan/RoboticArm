"""Actual DDS delivery into the real prepare subscription, without motion.

Only transport is exercised. Geometry/admission/certification are covered by
their own tests; this callback double cannot authorize a plan.
"""
import time
from pathlib import Path

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from luggage_msgs.msg import SuctionTopDomainV2
from luggage_mapping.configuration import owner_configuration
from luggage_planning.joint_operation.prepare_node import JointOperationPrepareNode
from test_snapshot_joint_path_native import robot_fixture


def test_sensor_domain_reaches_actual_prepare_subscription(tmp_path,monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID','87')
    context=Context();rclpy.init(context=context)
    model,semantic=robot_fixture()
    owner=owner_configuration(backend='observer',artifact_root=tmp_path/'owner',
        scene=str(Path(get_package_share_directory('luggage_description'))/'config/scene_tf_deploy_update.yaml'),
        observer_config=str(Path(get_package_share_directory('uld_occ_observer'))/'config/observer.yaml'))
    class TransportProbe(JointOperationPrepareNode):
        received=None
        def _domain(self,message):self.received=message
    parameters=dict(frame_scene_revision='transport-fixture',robot_description=model,
        robot_description_semantic=semantic,container_translation=[0.,0.,0.],
        container_quaternion_xyzw=[0.,0.,0.,1.],observer_geometry_file=owner.geometry_file,
        observer_config_file=owner.observer_config_file)
    root=TransportProbe(context=context,parameter_overrides=[Parameter(name,value=value) for name,value in parameters.items()])
    source=Node('domain_sensor_fixture',context=context)
    publisher=source.create_publisher(SuctionTopDomainV2,'/luggage/suction_top_domain_v2',qos_profile_sensor_data)
    executor=SingleThreadedExecutor(context=context);executor.add_node(root);executor.add_node(source)
    message=SuctionTopDomainV2(reason='actual-best-effort-transport-fixture')
    try:
        deadline=time.monotonic()+5.
        while root.received is None and time.monotonic()<deadline:
            publisher.publish(message);executor.spin_once(timeout_sec=.05)
        assert root.received is not None
        assert root.received.reason==message.reason
    finally:
        executor.shutdown();root.destroy_node();source.destroy_node();context.shutdown()
