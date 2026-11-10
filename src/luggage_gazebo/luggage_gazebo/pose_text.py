"""Decode `ign topic -e` Pose_V text for release checks and exact-byte replay.

The private descriptor mirrors ignition-msgs8's Pose_V/Pose/Header/Time/
Vector3d/Quaternion field names and numbers. Protobuf handles numeric syntax,
field ordering and omitted proto3 scalar zeros; this module owns framing and
validation. It is observation-only and never supplies planning geometry.
"""
from collections import Counter, deque
from dataclasses import dataclass
import math
import re

from google.protobuf import descriptor_pb2, message_factory, text_format


def _pose_vector_type():
    file = descriptor_pb2.FileDescriptorProto(
        name='release_pose_text.proto', package='release_observation', syntax='proto3')
    # Types follow FieldDescriptorProto: double=1, int64=3, int32=5,
    # uint32=13, string=9; a string type here denotes a message reference.
    schema = {
        'Time': [('sec', 1, 3), ('nsec', 2, 5)],
        'HeaderData': [('key', 1, 9), ('value', 2, 9, True)],
        'Header': [('stamp', 1, 'Time'), ('data', 2, 'HeaderData', True)],
        'Vector3d': [('header', 1, 'Header'), ('x', 2, 1), ('y', 3, 1), ('z', 4, 1)],
        'Quaternion': [('header', 1, 'Header'), ('x', 2, 1), ('y', 3, 1),
                       ('z', 4, 1), ('w', 5, 1)],
        'Pose': [('header', 1, 'Header'), ('name', 2, 9), ('id', 3, 13),
                 ('position', 4, 'Vector3d'), ('orientation', 5, 'Quaternion')],
        'Pose_V': [('header', 1, 'Header'), ('pose', 2, 'Pose', True)],
    }
    for name, fields in schema.items():
        message = file.message_type.add(name=name)
        for item in fields:
            field_name, number, kind, *repeated = item
            field = message.field.add(name=field_name, number=number,
                                      label=3 if repeated else 1)
            if isinstance(kind, str):
                field.type = 11
                field.type_name = '.release_observation.' + kind
            else:
                field.type = kind
    return message_factory.GetMessages([file])['release_observation.Pose_V']


PoseVector = _pose_vector_type()


@dataclass(frozen=True)
class PoseSample:
    model: str
    entity_id: int
    stamp_ns: int
    pose7: tuple  # x,y,z,qx,qy,qz,qw; world frame, metres


class PoseTextDecoder:
    """Incremental top-level header/pose blocks from the CLI's text stream.

    One header establishes the stamp for the following poses. Every new header
    resets all stamp components. EOF with an open block is a named rejection.
    Blocks are bounded; diagnostics retain counters and at most 32 details.
    """
    MAX_BLOCK_CHARS = 1024 * 1024

    def __init__(self):
        self.errors = deque(maxlen=32)
        self.counts = Counter()
        self.last_rejections = {}
        self._stamp_ns = None
        self._lines = []
        self._depth = 0
        self._size = 0

    def _reject(self, reason, model=None):
        self.counts[reason] += 1
        self.errors.append({'reason': reason, 'model': model})
        if model:
            # Only the latest diagnostic per entity; cap adversarial input.
            if len(self.last_rejections) >= 1024:
                self.last_rejections.clear()
            self.last_rejections[model] = reason

    def diagnostics(self):
        return {'counts': dict(self.counts), 'recent_errors': list(self.errors)}

    def feed_line(self, line):
        stripped = line.strip()
        if not stripped and not self._lines:
            return []
        if not self._lines and re.match(r'^(header|pose)\s*\{', stripped) is None:
            self._reject('unexpected_top_level')
            self._stamp_ns = None
            return []
        # Braces in quoted entity names/header values do not frame messages.
        structural = re.sub(r'"(?:\\.|[^"\\])*"', '""', stripped)
        self._lines.append(line)
        self._size += len(line)
        self._depth += structural.count('{') - structural.count('}')
        if self._size > self.MAX_BLOCK_CHARS or self._depth < 0:
            self._reject('invalid_framing')
            self._reset_block()
            self._stamp_ns = None
            return []
        if self._depth:
            return []
        block = '\n'.join(self._lines)
        self._reset_block()
        return self._decode_block(block)

    def _reset_block(self):
        self._lines = []
        self._size = self._depth = 0

    def finish(self):
        if self._lines:
            self._reject('truncated_message')
            self._reset_block()
        self._stamp_ns = None

    def _decode_block(self, block):
        is_header = block.lstrip().startswith('header')
        if is_header:
            self._stamp_ns = None
        message = PoseVector()
        try:
            text_format.Parse(block, message)
        except text_format.ParseError:
            self._reject('malformed_protobuf')
            self._stamp_ns = None
            return []
        if is_header:
            self.counts['headers'] += 1
            if not message.header.HasField('stamp'):
                self._reject('missing_stamp')
                return []
            stamp = message.header.stamp
            if stamp.sec < 0 or not 0 <= stamp.nsec < 1_000_000_000:
                self._reject('invalid_stamp')
                return []
            self._stamp_ns = stamp.sec * 1_000_000_000 + stamp.nsec
            return []
        samples = []
        for pose in message.pose:
            self.counts['poses_seen'] += 1
            reason = None
            position, q = pose.position, pose.orientation
            values = (position.x, position.y, position.z, q.x, q.y, q.z, q.w)
            if not pose.name:
                reason = 'missing_model_name'
            elif self._stamp_ns is None:
                reason = 'missing_stamp'
            elif not pose.HasField('position') or not pose.HasField('orientation'):
                reason = 'missing_pose_structure'
            elif not all(math.isfinite(value) for value in values):
                reason = 'nonfinite_pose'
            elif abs(sum(value * value for value in values[3:]) - 1.0) > 1e-3:
                reason = 'invalid_quaternion'
            if reason:
                self._reject(reason, pose.name or None)
                continue
            self.last_rejections.pop(pose.name, None)
            self.counts['accepted'] += 1
            samples.append(PoseSample(pose.name, pose.id, self._stamp_ns, values))
        return samples


def parse_pose_samples(text, model):
    decoder = PoseTextDecoder()
    result = []
    for line in str(text).splitlines():
        result.extend((sample.stamp_ns, list(sample.pose7))
                      for sample in decoder.feed_line(line) if sample.model == str(model))
    decoder.finish()
    return result
