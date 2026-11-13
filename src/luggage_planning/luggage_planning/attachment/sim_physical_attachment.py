"""Simulation-only model pose observation, never measured planning geometry."""
import math
import re
import subprocess

from luggage_description.scene_tf_publisher import rpy_to_quaternion
from luggage_planning.attachment.vacuum_backend import relative_offset


def observe_model_pose(model, *, run=subprocess.run):
    response = run(['ign', 'model', '-m', model, '--pose'],
                   capture_output=True, text=True, timeout=2., check=False)
    if response.returncode:
        raise ValueError('physical model query failed')
    text = response.stdout
    if '- Name: '+model+'\n' not in text:
        raise ValueError('physical model identity unavailable')
    matched = re.search(r'Pose \[ XYZ \(m\) \] \[ RPY \(rad\) \]:\s*'
                        r'\[([^\]]+)\]\s*\[([^\]]+)\]', text)
    if not matched:
        raise ValueError('physical model pose unavailable')
    values = [float(v) for group in matched.groups() for v in group.split()]
    if len(values) != 6 or not all(map(math.isfinite, values)):
        raise ValueError('physical model pose invalid')
    return values[:3], list(rpy_to_quaternion(values[3:]))


def freeze_physical_relation(model, generation, tool, *, read_identity, observe=None):
    """Pin the physical model epoch around a fresh pose query.

    Identity provider exposes only model name/generation. No catalog dimensions
    or measured box axes participate in this simulation actuator relation.
    """
    expected = (str(model), int(generation))
    if read_identity() != expected:
        raise ValueError('physical model generation mismatch')
    xyz, quat = (observe or observe_model_pose)(model)
    if read_identity() != expected:
        raise ValueError('physical model changed during observation')
    offset, orientation = relative_offset(tool['position'], tool['quaternion_xyzw'], xyz, quat)
    return dict(t_tool_box_translation=list(offset),
                t_tool_box_quaternion=list(orientation),
                model_name=model, generation=generation,
                physical_pose=dict(position=list(xyz), quaternion_xyzw=list(quat)))
