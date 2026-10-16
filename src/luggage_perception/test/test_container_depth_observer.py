from collections import OrderedDict, deque
from copy import deepcopy
import threading
from types import SimpleNamespace
import numpy as np
import pytest
from sensor_msgs.msg import Image, CameraInfo
from luggage_msgs.srv import IntegrateCargoView, GetCargoMapStats
from luggage_perception.container_depth_observer import depth_cloud, ContainerDepthObserver
from luggage_perception.motion_stability_filter import MotionStabilityGate
from luggage_perception.ros_message_adapters import cloud_points_from_msg


def pair():
    image=Image();image.header.stamp.sec=1;image.header.frame_id='optical'
    image.height=2;image.width=2;image.encoding='16UC1';image.step=4
    image.data=np.array([[1000,0],[2000,1000]],dtype=np.uint16).tobytes()
    info=CameraInfo();info.header=deepcopy(image.header);info.height=2;info.width=2
    info.k=[2.,0.,0.,0.,2.,0.,0.,0.,1.]
    return image,info


def test_exact_optical_points_and_stamp():
    image,info=pair();cloud=depth_cloud(image,info,stride=1)
    assert cloud.header==image.header
    np.testing.assert_allclose(cloud_points_from_msg(cloud),[[0,0,1],[0,1,2],[.5,.5,1]])


@pytest.mark.parametrize('change',[
    lambda i,k:setattr(k.header.stamp,'nanosec',1),
    lambda i,k:setattr(k.header,'frame_id','other'),
    lambda i,k:setattr(k,'width',3),
    lambda i,k:setattr(k,'d',[.1]),
    lambda i,k:setattr(k,'d',[float('nan')]),
    lambda i,k:setattr(i.header.stamp,'sec',0),
])
def test_invalid_pair_never_produces_cloud(change):
    image,info=pair();change(image,info)
    with pytest.raises(ValueError):depth_cloud(image,info)


def observer(stable=True):
    obj=object.__new__(ContainerDepthObserver)
    image,info=pair();obj._images=OrderedDict({1_000_000_000:image});obj._infos=OrderedDict({1_000_000_000:info})
    obj._lock=threading.Lock();obj._joints=deque();obj._now=lambda:1.
    obj._gate=MotionStabilityGate(joint_names=['joint'])
    for t in np.linspace(.1,1.,10):
        obj._gate.update(['joint'],[0. if stable else t],[0.],stamp=t,now=t)
    obj._recorder=None;obj._integrate_type=IntegrateCargoView;obj._stats_type=GetCargoMapStats
    obj._stats='stats';obj._integrate='integrate';published=[];requests=[]
    obj._publisher=SimpleNamespace(publish=published.append)
    def call(client,request):
        requests.append((client,request))
        if client=='stats':return SimpleNamespace(map_revision=4,geometry_hash='hull')
        return SimpleNamespace(success=True,schema_version=1,integrated_acquisition_stamp=request.source_acquisition_stamp,
            resulting_map_revision=5,unknown_ratio=.9,occupancy_ratio=.0)
    obj._call=call
    return obj,published,requests


def test_stable_input_uses_current_owner_revision_without_faking_confirmation():
    obj,published,requests=observer()
    result=obj.observe_once('view',timeout=.1)
    assert len(published)==1
    req=requests[-1][1]
    assert req.expected_map_revision==4 and req.geometry_hash=='hull' and req.settled
    assert req.source_acquisition_stamp==published[0].header.stamp
    assert result['map_revision']==5 and result['unknown_ratio']==.9


def test_moving_input_causes_no_publish_or_service_mutation():
    obj,published,requests=observer(stable=False)
    with pytest.raises(TimeoutError):obj.observe_once('view',timeout=.025)
    assert not published and not requests


def test_old_joint_samples_cannot_authorize_fresh_depth():
    obj,published,requests=observer();obj._now=lambda:3.
    obj._images[1_000_000_000].header.stamp.sec=3
    obj._infos[1_000_000_000].header.stamp.sec=3
    obj._images[3_000_000_000]=obj._images.pop(1_000_000_000)
    obj._infos[3_000_000_000]=obj._infos.pop(1_000_000_000)
    with pytest.raises(TimeoutError):obj.observe_once('view',timeout=.025)
    assert not published and not requests


def test_mismatched_owner_echo_is_not_admitted():
    obj,published,requests=observer()
    original=obj._call
    def mismatch(client,request):
        response=original(client,request)
        if client=='integrate':response.schema_version=99
        return response
    obj._call=mismatch
    with pytest.raises(RuntimeError,match='RESPONSE_MISMATCH'):
        obj.observe_once('view',timeout=.1)
