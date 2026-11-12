"""Dynamics change preserves every existing collision and visual element."""
import xml.etree.ElementTree as ET
from luggage_gazebo.rigid_attachment import dynamic_carry_sdf


def test_dynamic_carry_retains_all_geometry_and_contact_masks():
    original='''<sdf><model name="measured-identity"><static>true</static><link name="body">
      <visual name="skin"><geometry><box><size>1 2 3</size></box></geometry></visual>
      <collision name="rim"><geometry><box><size>1 2 3</size></box></geometry>
      <surface><contact><collide_bitmask>254</collide_bitmask></contact></surface></collision>
      </link></model></sdf>'''
    before=ET.fromstring(original);after=ET.fromstring(dynamic_carry_sdf(original))
    assert after.findtext('model/static')=='false'
    assert after.findtext('model/link/gravity')=='false'
    for kind in ['collision','visual']:
        assert ET.tostring(before.find('model/link/'+kind))==ET.tostring(after.find('model/link/'+kind))
