"""Release the simulation actuator's existing body; never provide planning data."""
import hashlib
import xml.etree.ElementTree as ET

from luggage_description.gazebo_spawn_collision import BOX_FULL_MASK
from luggage_gazebo.entity_replacement import replace_entity


def dynamic_release_sdf(sdf,model):
    """Only change dynamics and the carry mask; retain all existing geometry."""
    root=ET.fromstring(sdf)
    body=root.find('model')
    if body is None or body.get('name')!=model:
        raise ValueError('RELEASE_MODEL_SDF_IDENTITY_MISMATCH')
    static=body.find('static')
    if static is None:static=ET.SubElement(body,'static')
    static.text='false'
    for mask in body.findall('.//collision/surface/contact/collide_bitmask'):
        mask.text=str(BOX_FULL_MASK)
    return ET.tostring(root,encoding='unicode')


def release_model(*,model,generation,identity,sdf,probe,delete,create):
    """One bounded replacement, no duplicate destructive command on uncertainty.

    Presence of the dynamic body confirms actuator handover only. Gravity and
    settled support must be independently observed; no target pose is accepted.
    """
    expected=(model,generation)
    if identity()!=expected:
        return dict(success=False,reason='RELEASE_INSTANCE_MISMATCH',trace=[])
    dynamic=dynamic_release_sdf(sdf,model)
    def guarded(command):
        def call(*args):
            if identity()!=expected:return 'RELEASE_INSTANCE_CHANGED'
            return command(*args)
        return call
    report=replace_entity(probe=probe,delete=guarded(delete),
        create=guarded(lambda pose,budget:create(pose,dynamic,budget)),
        pose_matches=lambda actual,expected:True,persistence=0.)
    if identity()!=expected:
        report.update(success=False,reason='RELEASE_INSTANCE_CHANGED')
    report.update(model=model,generation=generation,
        sdf_sha256=hashlib.sha256(dynamic.encode()).hexdigest(),dynamic_sdf=dynamic,
        observation='dynamic_presence_only_not_settled_support')
    return report
