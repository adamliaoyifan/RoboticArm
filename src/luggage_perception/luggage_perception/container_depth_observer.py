"""Explicit settled-depth acquisition client; never moves the robot.

An orchestrator calls observe_once after reaching an observation pose. This
client joins exact image/intrinsic stamps, checks the existing motion stability
gate, publishes measured optical points, and asks the map owner to integrate.
It neither labels a map confirmed nor creates geometry from a template.
"""
from collections import OrderedDict, deque
import math
import threading
import time
from types import SimpleNamespace

from luggage_perception.motion_stability_filter import MotionStabilityGate


def stamp_ns(message):
    return int(message.header.stamp.sec)*1_000_000_000+int(message.header.stamp.nanosec)


def depth_cloud(image, info, *, stride=4):
    from luggage_perception.depth_deprojection import deproject_stride
    from luggage_perception.ros_message_adapters import depth_array_from_msg, cloud_msg_from_points
    if stamp_ns(image)<=0 or stamp_ns(image)!=stamp_ns(info):
        raise ValueError('DEPTH_INTRINSICS_STAMP_MISMATCH')
    if not image.header.frame_id or image.header.frame_id!=info.header.frame_id:
        raise ValueError('DEPTH_INTRINSICS_FRAME_MISMATCH')
    if (image.width,image.height)!=(info.width,info.height):
        raise ValueError('DEPTH_INTRINSICS_SIZE_MISMATCH')
    values=(info.k[0],info.k[4],info.k[2],info.k[5])
    if not all(math.isfinite(v) for v in values) or min(values[:2])<=0:
        raise ValueError('DEPTH_INTRINSICS_INVALID')
    if any(not math.isfinite(v) or abs(v)>1e-12 for v in info.d):
        raise ValueError('DISTORTED_DEPTH_UNSUPPORTED')
    depth=depth_array_from_msg(image)
    if depth is None:raise ValueError('DEPTH_ENCODING_UNSUPPORTED')
    intrinsics=SimpleNamespace(fx=values[0],fy=values[1],cx=values[2],cy=values[3])
    points=deproject_stride(depth,intrinsics,stride=stride)
    if isinstance(points,tuple):points=points[0]
    if not len(points):raise ValueError('DEPTH_VIEW_EMPTY')
    return cloud_msg_from_points(points,image.header.stamp,image.header.frame_id)


class ContainerDepthObserver:
    def __init__(self,node,*,group=None,recorder=None):
        from sensor_msgs.msg import Image, CameraInfo, JointState, PointCloud2
        from luggage_msgs.srv import IntegrateCargoView, GetCargoMapStats
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        self._node=node;self._recorder=recorder;self._lock=threading.Lock()
        self._images=OrderedDict();self._infos=OrderedDict();self._joints=deque(maxlen=64)
        self._gate=MotionStabilityGate(joint_names=['elfin_joint%d'%i for i in range(1,7)])
        self._integrate_type=IntegrateCargoView;self._stats_type=GetCargoMapStats
        qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
        self._subscriptions=[
            node.create_subscription(Image,'/luggage/preprocessed/camera/depth/image',
                lambda msg:self._store(self._images,msg),qos,callback_group=group),
            node.create_subscription(CameraInfo,'/luggage/preprocessed/camera/depth/camera_info',
                lambda msg:self._store(self._infos,msg),qos,callback_group=group),
            node.create_subscription(JointState,'/joint_states',self._on_joints,qos,callback_group=group)]
        self._publisher=node.create_publisher(PointCloud2,'/luggage/perception/container_depth_points',10)
        self._integrate=node.create_client(IntegrateCargoView,'/cargo_map/integrate_container_depth_view',callback_group=group)
        self._stats=node.create_client(GetCargoMapStats,'/cargo_map/get_stats',callback_group=group)

    def _now(self):return self._node.get_clock().now().nanoseconds/1e9

    def _store(self,buffer,message):
        with self._lock:
            buffer[stamp_ns(message)]=message
            while len(buffer)>8:buffer.popitem(last=False)

    def _on_joints(self,message):
        with self._lock:
            self._gate.update(message.name,message.position,message.velocity,
                              stamp=stamp_ns(message)/1e9,now=stamp_ns(message)/1e9)
            self._joints.append(message)

    def _record(self,boundary,message):
        if self._recorder:self._recorder(boundary,message)

    def _call(self,client,request,timeout=5.):
        if not client.wait_for_service(timeout_sec=2.):raise RuntimeError('OBSERVATION_SERVICE_UNAVAILABLE')
        self._record(client.srv_name+':request',request)
        future=client.call_async(request);done=threading.Event()
        future.add_done_callback(lambda _:done.set())
        if not done.wait(timeout):raise TimeoutError('OBSERVATION_SERVICE_TIMEOUT')
        response=future.result();self._record(client.srv_name+':response',response)
        return response

    def observe_once(self,view_id,*,timeout=5.):
        deadline=time.monotonic()+timeout;chosen=None
        while time.monotonic()<deadline:
            with self._lock:
                now=self._now()
                common=sorted(set(self._images)&set(self._infos),reverse=True)
                for stamp in common:
                    if stamp>0 and 0<=now-stamp/1e9<=1. and self._gate.accepts_cloud(stamp/1e9,now):
                        chosen=(self._images[stamp],self._infos[stamp],tuple(self._joints))
                        break
            if chosen:break
            time.sleep(.02)
        if chosen is None:raise TimeoutError('NO_SETTLED_EXACT_DEPTH_PAIR')
        image,info,joints=chosen
        cloud=depth_cloud(image,info)
        for name,message in [('depth_image',image),('depth_intrinsics',info),('optical_cloud',cloud)]:
            self._record(name,message)
        for message in joints:self._record('joint_stability_window',message)
        stats=self._call(self._stats,self._stats_type.Request())
        request=self._integrate_type.Request()
        request.schema_version=1;request.view_request_id=str(view_id);request.candidate_id=str(view_id)
        request.source_acquisition_stamp=image.header.stamp
        request.expected_map_revision=stats.map_revision;request.geometry_hash=stats.geometry_hash
        request.settled=True;request.frame_count=1
        for _ in range(3):
            self._publisher.publish(cloud)
            time.sleep(.05)
            response=self._call(self._integrate,request)
            if response.success or response.reason_code!='STAMP_STALE':break
        if not response.success:raise RuntimeError(response.reason_code+': '+response.message)
        if (response.schema_version != request.schema_version
                or response.integrated_acquisition_stamp != request.source_acquisition_stamp
                or response.resulting_map_revision <= request.expected_map_revision):
            raise RuntimeError('DEPTH_INTEGRATION_RESPONSE_MISMATCH')
        return dict(map_revision=response.resulting_map_revision,
                    acquisition_stamp_ns=stamp_ns(image),unknown_ratio=response.unknown_ratio,
                    occupancy_ratio=response.occupancy_ratio)
