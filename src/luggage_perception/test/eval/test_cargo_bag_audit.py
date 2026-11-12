"""Finite offline evidence contract, independent from ROS-generated bindings."""
import json
import struct

from mcap_ros2.writer import Writer
import pytest

from luggage_perception.eval.cargo_acquisition_audit import AcquisitionIndex, cdr_header
from luggage_perception.eval.cargo_bag_audit import SOURCES, DETECTION_TOPIC, audit_mcap, _fields


def header_bytes(endian='<', sec=10, nsec=2, frame='optical'):
    frame = frame.encode() + b'\0'
    return (b'\0\1\0\0' if endian == '<' else b'\0\0\0\0') + struct.pack(
        endian + 'iII', sec, nsec, len(frame)) + frame


@pytest.mark.parametrize('endian', ['<', '>'])
def test_header_stamp_and_frame(endian):
    assert cdr_header(header_bytes(endian)) == (10_000_000_002, 'optical')


@pytest.mark.parametrize('data', [b'', header_bytes()[:12], b'\0\3' + header_bytes()[2:],
                                  header_bytes(nsec=1_000_000_000), header_bytes()[:-1],
                                  header_bytes()[:-1] + b'x'])
def test_bad_header_refuses(data):
    with pytest.raises(ValueError):
        cdr_header(data)


@pytest.mark.parametrize('mismatch', ['none', 'stamp', 'frame', 'conflict', 'duplicate'])
def test_exact_join_and_conflicts(mismatch):
    index = AcquisitionIndex()
    index.add('rgb', 100, 'optical', 200, 'a')
    index.add('depth', 101 if mismatch == 'stamp' else 100,
              'other' if mismatch == 'frame' else 'optical', 220, 'b')
    if mismatch in ('conflict', 'duplicate'):
        index.add('depth', 100, 'optical', 230, 'c' if mismatch == 'conflict' else 'b')
    assert bool(index.exact_keys(['rgb', 'depth'])) == (mismatch in ('none', 'duplicate'))
    result = index.summary()['depth']
    assert result['conflicting_keys'] == int(mismatch == 'conflict')
    assert result['duplicate_keys'] == int(mismatch in ('conflict', 'duplicate'))


def test_native_slot_fields_do_not_disappear():
    # Matches native decoder: an empty __dict__ coexists with real fields in slots.
    class NativeRecord:
        __slots__ = ('header', 'pca_valid', '__dict__')
    record = NativeRecord()
    record.header = dict(stamp=10)
    record.pca_valid = True
    assert vars(record) == {}
    result = _fields(record)
    assert result['pca_valid'] is True
    assert result['header'] == {'stamp': 10}


HEADER_SCHEMA = '''
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
'''
IMAGE_SCHEMA = '''std_msgs/Header header
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
''' + HEADER_SCHEMA
INFO_SCHEMA = '''std_msgs/Header header
uint32 height
uint32 width
string distortion_model
float64[] d
float64[9] k
''' + HEADER_SCHEMA


def test_native_mcap_counts_exact_joins_and_lossless_examples(tmp_path):
    path = tmp_path / 'fixture.mcap'
    with path.open('wb') as handle:
        writer = Writer(handle)
        image_schema = writer.register_msgdef('sensor_msgs/msg/Image', IMAGE_SCHEMA)
        info_schema = writer.register_msgdef('sensor_msgs/msg/CameraInfo', INFO_SCHEMA)
        for nsec in (1, 2):
            h = dict(stamp=dict(sec=10, nanosec=nsec), frame_id='optical')
            for i, topic in enumerate(SOURCES):
                if i == 3:
                    message = dict(header=h, height=1, width=2, distortion_model='plumb_bob',
                                   d=[], k=[1., 0., 0., 0., 1., 0., 0., 0., 1.])
                else:
                    message = dict(header=h, height=1, width=2,
                                   encoding=['rgb8', '16UC1', 'mono16'][i],
                                   is_bigendian=0, step=6 if i == 0 else 4,
                                   data=bytes([1, 2, 3, 4, 5, 6]) if i == 0 else bytes([1, 0, 2, 0]))
                writer.write_message(topic, info_schema if i == 3 else image_schema,
                                     message, log_time=10_000_000_000 + nsec)
        detection_schema = writer.register_msgdef('luggage_msgs/msg/DetectionFrame',
                                                  'std_msgs/Header header\nbool pca_valid\n' + HEADER_SCHEMA)
        writer.write_message(DETECTION_TOPIC, detection_schema,
                             dict(header=h, pca_valid=True), log_time=10_000_000_003)
        writer.finish()
    output = tmp_path / 'output'
    expected = {t: 2 for t in SOURCES}
    expected[DETECTION_TOPIC] = 1
    result = audit_mcap(path, output, expected)
    assert result['count_mismatches'] == {}
    assert result['exact_four_source_join_count'] == 2
    assert result['topic_counts'] == expected
    assert all(not e['missing_topics'] for e in result['examples'])
    assert len(list(output.glob('frame_*/*.cdr'))) == 8
    assert len(list(output.glob('frame_*/*.png'))) == 6
    assert json.loads((output / 'summary.json').read_text())['absolute_pose_error'].startswith('not_evaluated')
    measured = json.loads((output / 'detection_outputs.json').read_text())
    assert measured[0]['measured_output']['pca_valid'] is True
