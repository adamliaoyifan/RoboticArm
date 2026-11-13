"""Instantiate static box models from scene dimensions, preserving materials.

The repository SDF is a visual/material template. pedestal.size and
pickup_platform.size are authoritative XYZ extents in each model's local frame;
the link origin stays at bottom centre. Never resize the container mesh here.
"""
from copy import deepcopy
import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET


def configured_model(template, scene, model_name):
    root=ET.parse(template).getroot()
    model=deepcopy(root.find('model'))
    key={'robot_pedestal':'pedestal','pickup_platform':'pickup_platform'}.get(model_name)
    if key is None:return model
    size=tuple(float(v) for v in scene[key]['size'])
    if len(size)!=3 or not all(math.isfinite(v) and v>0 for v in size):
        raise ValueError('INVALID_SCENE_BOX_SIZE:'+model_name)
    links=model.findall('link')
    if len(links)!=1:raise ValueError('SINGLE_BOX_MODEL_REQUIRED:'+model_name)
    for kind in ('collision','visual'):
        parts=links[0].findall(kind)
        if len(parts)!=1 or parts[0].find('geometry/box/size') is None:
            raise ValueError('SINGLE_BOX_GEOMETRY_REQUIRED:'+model_name)
        parts[0].find('geometry/box/size').text=' '.join(format(v,'.17g') for v in size)
        pose=parts[0].find('pose')
        if pose is None:pose=ET.SubElement(parts[0],'pose')
        pose.text='0 0 '+format(size[2]/2,'.17g')+' 0 0 0'
    return model


def write_configured_model(template,scene,model_name):
    """Launch-owned immutable temporary SDF; original model assets remain intact."""
    model=configured_model(template,scene,model_name)
    root=ET.Element('sdf',version='1.6');root.append(model)
    directory=Path(tempfile.mkdtemp(prefix='luggage-scene-model-'))
    path=directory/(model_name+'.sdf')
    ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
    return str(path)
