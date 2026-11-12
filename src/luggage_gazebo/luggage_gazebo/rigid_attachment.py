"""Fortress built-in fixed-joint actuation; never returns planning geometry.

One DetachableJoint instance belongs to each actual simulated cargo identity.
Detach is acknowledged before the existing physical release replacement.
"""
import json
import re
import subprocess
import time
import xml.etree.ElementTree as ET


def dynamic_carry_sdf(text):
    root = ET.fromstring(text)
    root.find('model/static').text = 'false'
    for link in root.findall('model/link'):
        gravity = link.find('gravity')
        if gravity is None:
            gravity = ET.SubElement(link, 'gravity')
        gravity.text = 'false'
    return ET.tostring(root, encoding='unicode')


class FortressRigidAttachment:
    def __init__(self, world, robot_model, parent_link='elfin_link6'):
        self.world, self.robot_model, self.parent_link = world, robot_model, parent_link
        self.attached_model = None

    def _run(self, command, timeout=3.):
        result = subprocess.run(command, capture_output=True, text=True, timeout=timeout)
        if result.returncode:
            raise RuntimeError('RIGID_TRANSPORT_REFUSED:' + result.stderr[-300:])
        return result.stdout

    def _transition(self, topic, expected):
        # Subscribe before publishing. The native joint reports its state when
        # its PreUpdate actually creates/removes the fixed joint component.
        reader = subprocess.Popen(['ign','topic','-e','-t',topic+'/state','--num','1'],
                                  stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            self._run(['ign','topic','-t',topic+'/'+('attach' if expected=='attached' else 'detach'),
                       '-m','ignition.msgs.Empty','-p','unused: true'])
            output, _ = reader.communicate(timeout=5.)
            if 'data: "'+expected+'"' not in output:
                raise RuntimeError('RIGID_STATE_UNCONFIRMED:'+output[-300:])
        finally:
            if reader.poll() is None:
                reader.kill(); reader.communicate()

    def attach(self, model):
        if self.attached_model is not None:
            raise RuntimeError('RIGID_ATTACHMENT_ALREADY_OWNED')
        output=self._run(['ign','model','-m',self.robot_model,'--pose'])
        match=re.search(r'Model:\s*\[(\d+)\]',output)
        if match is None or '- Name: '+self.robot_model+'\n' not in output:
            raise RuntimeError('RIGID_PARENT_IDENTITY_UNAVAILABLE:'+output[-300:])
        topic='/luggage/physical_attachment/'+model
        plugin=ET.Element('parameters')
        for key,value in dict(parent_link=self.parent_link,child_model=model,
                child_link='suitcase_link',initial_attach='false',
                attach_topic=topic+'/attach',detach_topic=topic+'/detach',
                output_topic=topic+'/state').items():
            ET.SubElement(plugin,key).text=value
        inner=''.join(ET.tostring(e,encoding='unicode') for e in plugin)
        request='entity {id: %s} plugins {name: "ignition::gazebo::systems::DetachableJoint" filename: "ignition-gazebo-detachable-joint-system" innerxml: %s}' % (match[1],json.dumps(inner))
        output=self._run(['ign','service','-s','/world/'+self.world+'/entity/system/add',
            '--reqtype','ignition.msgs.EntityPlugin_V','--reptype','ignition.msgs.Boolean',
            '--timeout','2000','--req',request])
        if not re.search(r'\bdata:\s*true\b',output):
            raise RuntimeError('RIGID_PLUGIN_LOAD_UNCONFIRMED:'+output[-300:])
        # Ensure the queued plugin is configured before its attach command.
        deadline=time.monotonic()+3.
        while True:
            info=self._run(['ign','topic','-i','-t',topic+'/state'])
            if 'ignition.msgs.StringMsg' in info:break
            if time.monotonic()>=deadline:raise RuntimeError('RIGID_PLUGIN_NOT_READY')
        self._transition(topic,'attached')
        self.attached_model=model

    def detach(self, model):
        if self.attached_model!=model:
            raise RuntimeError('RIGID_RELEASE_IDENTITY_MISMATCH')
        self._transition('/luggage/physical_attachment/'+model,'detached')
        self.attached_model=None
