"""Simulation release is a physical model handover, never a planned placement."""
import xml.etree.ElementTree as ET
from luggage_gazebo.simulation_release import dynamic_release_sdf,release_model

SDF='<sdf><model name="box"><static>true</static><link name="body"><collision name="c"><geometry><box><size>1 2 3</size></box></geometry><surface><contact><collide_bitmask>1</collide_bitmask></contact></surface></collision><visual name="v"><geometry><mesh><uri>model://actual_mesh</uri></mesh></geometry></visual></link></model></sdf>'


def test_dynamic_handover_preserves_original_geometry_and_visuals():
    before=ET.fromstring(SDF);after=ET.fromstring(dynamic_release_sdf(SDF,'box'))
    assert after.find('model/static').text=='false'
    assert ET.tostring(before.find('.//collision/geometry'))==ET.tostring(after.find('.//collision/geometry'))
    assert ET.tostring(before.find('.//visual'))==ET.tostring(after.find('.//visual'))
    assert after.find('.//collide_bitmask').text!='1'


def test_release_uses_actual_pose_and_requires_disappearance_before_recreate():
    calls=[];poses=iter([dict(status='present',pose=[1,2,3,0,0,0]),dict(status='absent'),
                      dict(status='present',pose=[1,2,2.95,0,0,0])])
    def create(pose,sdf,budget):
        calls.append(('create',pose));assert ET.fromstring(sdf).find('model/static').text=='false'
    result=release_model(model='box',generation=4,identity=lambda:('box',4),sdf=SDF,
        probe=lambda budget:next(poses),delete=lambda budget:calls.append(('delete',)),create=create)
    assert result['success'] and calls==[('delete',),('create',[1,2,3,0,0,0])]
    assert result['observation']=='dynamic_presence_only_not_settled_support'


def test_mismatched_instance_never_probes_or_deletes():
    def forbidden(*a):raise AssertionError('unexpected physical mutation')
    result=release_model(model='box',generation=4,identity=lambda:('box',5),sdf=SDF,
        probe=forbidden,delete=forbidden,create=forbidden)
    assert not result['success'] and result['reason']=='RELEASE_INSTANCE_MISMATCH'
