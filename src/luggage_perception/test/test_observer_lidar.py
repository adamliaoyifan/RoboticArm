from types import SimpleNamespace as S
from luggage_perception.observer_lidar import SettledLidarClouds


def cloud(frame='lidar', sec=2):
    return S(header=S(frame_id=frame,stamp=S(sec=sec,nanosec=0)),fields=[S(name=n) for n in ('x','y','z')],data=b'preserved')


def test_keeps_original_xyz_scan_and_acquisition_header():
    gate=SettledLidarClouds(frame='lidar')
    gate.gate.accepts_cloud=lambda *a:True
    original=cloud()
    assert gate.admit(original,2.1)==(original,'OBSERVED_LIDAR_XYZ')
    assert gate.admit(original,2.1)[0] is None


def test_wrong_frame_missing_xyz_stale_and_moving_never_publish():
    gate=SettledLidarClouds(frame='lidar')
    assert gate.admit(cloud('wrong'),2.)[1]=='LIDAR_FRAME_MISMATCH'
    malformed=cloud();malformed.fields=[]
    assert gate.admit(malformed,2.)[1]=='LIDAR_XYZ_REQUIRED'
    assert gate.admit(cloud(),4.)[1]=='NO_FRESH_LIDAR'
    assert gate.admit(cloud(),1.)[0] is None
    assert gate.admit(cloud(),2.)[1]=='ROBOT_NOT_SETTLED'
    assert gate.last_stamp_ns==0
