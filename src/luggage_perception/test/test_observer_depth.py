from copy import deepcopy
import numpy as np
import pytest
from sensor_msgs.msg import JointState
from test_container_depth_observer import pair
from luggage_perception.observer_depth import SettledDepthRays
from luggage_perception.ros_message_adapters import cloud_points_from_msg


def rays(*, moving=False):
    output = SettledDepthRays(stride=1)
    image, info = pair()
    output.store('image',image);output.store('info',info)
    for stamp in np.linspace(.1,1.,10):
        joints = JointState(name=['elfin_joint%d'%i for i in range(1,7)],
                            position=[float(stamp) if moving else 0.]*6,velocity=[0.]*6)
        joints.header.stamp.sec=int(stamp)
        joints.header.stamp.nanosec=round((stamp-int(stamp))*1e9)
        output.joints(joints)
    return output


def test_optical_rays_keep_all_observed_points_and_original_acquisition():
    output = rays();cloud,reason = output.next_cloud(1.)
    assert reason=='OBSERVED_OPTICAL_RAYS'
    assert cloud.header==output.images[10**9].header
    np.testing.assert_allclose(cloud_points_from_msg(cloud),[[0,0,1],[0,1,2],[.5,.5,1]])
    assert output.next_cloud(1.)[0] is None


@pytest.mark.parametrize('moving,now',[(True,1.),(False,3.),(False,.5)])
def test_moving_stale_future_views_publish_nothing(moving,now):
    assert rays(moving=moving).next_cloud(now)[0] is None


def test_missing_stamp_pair_does_not_use_nearest_intrinsics():
    output = rays(); info=output.infos.pop(10**9)
    info.header.stamp.nanosec=1;output.store('info',info)
    assert output.next_cloud(1.)[0] is None


def test_bad_frame_refuses_without_consuming_stamp():
    output = rays();output.infos[10**9]=deepcopy(output.infos[10**9])
    output.infos[10**9].header.frame_id='other'
    with pytest.raises(ValueError,match='FRAME_MISMATCH'):output.next_cloud(1.)
    assert output.last_stamp==0


def test_bounded_latest_input_storage():
    output = rays()
    for stamp in range(2,20):
        image,info=pair();image.header.stamp.sec=stamp;info.header.stamp.sec=stamp
        output.store('image',image);output.store('info',info)
    assert len(output.images)==len(output.infos)==8
