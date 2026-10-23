"""Behavioral gates for the real Gazebo CLI's proto3 text conventions."""
import pytest
from luggage_gazebo.pose_text import PoseTextDecoder, parse_pose_samples

MODEL = 'pickup_box_0001_inch20'
HEADER = 'header { stamp { sec: 7 nsec: 250000000 } }'
POSE = 'pose { name: "%s" id: 42 position { z: 0.6 } orientation { w: 1 } }' % MODEL


def decode(text):
    decoder = PoseTextDecoder()
    samples = [sample for line in text.splitlines() for sample in decoder.feed_line(line)]
    decoder.finish()
    return samples, decoder


def test_omitted_scalar_zeros_and_empty_position():
    samples, _ = decode(HEADER+'\n'+POSE)
    assert samples[0].pose7 == (0, 0, .6, 0, 0, 0, 1)
    assert samples[0].entity_id == 42
    samples, _ = decode(HEADER+'\n'+POSE.replace('z: 0.6', ''))
    assert samples[0].pose7[:3] == (0, 0, 0)


def test_field_order_numeric_syntax_and_nested_headers():
    text = HEADER+'\npose { orientation { w: 1 } name: "%s" position { y: +2E-1 x: -2.03e+0 header { data {key: "x" value: "{ignored}"} } z: .6 } }' % MODEL
    samples, _ = decode(text)
    assert samples[0].pose7 == (-2.03, .2, .6, 0, 0, 0, 1)


def test_each_header_resets_zero_stamp_components():
    text = HEADER+'\n'+POSE+'\nheader { stamp { sec: 8 } }\n'+POSE+'\nheader { stamp { nsec: 5 } }\n'+POSE+'\nheader { stamp {} }\n'+POSE
    samples, _ = decode(text)
    assert [sample.stamp_ns for sample in samples] == [7_250_000_000, 8_000_000_000, 5, 0]


@pytest.mark.parametrize('header,pose,reason', [
    ('header {}', POSE, 'missing_stamp'),
    ('header { stamp { nsec: 1000000000 } }', POSE, 'invalid_stamp'),
    (HEADER, POSE.replace('position { z: 0.6 }', ''), 'missing_pose_structure'),
    (HEADER, POSE.replace('w: 1', ''), 'invalid_quaternion'),
    (HEADER, POSE.replace('w: 1', 'w: 2'), 'invalid_quaternion'),
    (HEADER, POSE.replace('z: 0.6', 'z: nan'), 'nonfinite_pose'),
    (HEADER, POSE.replace('w: 1', 'w: inf'), 'nonfinite_pose'),
    (HEADER, POSE.replace('name: "%s"' % MODEL, ''), 'missing_model_name'),
    (HEADER, POSE.replace('z: 0.6', 'z: broken'), 'malformed_protobuf'),
])
def test_bad_input_refused_by_name(header, pose, reason):
    samples, decoder = decode(header+'\n'+pose)
    assert not samples
    assert decoder.counts[reason]


def test_missing_header_and_truncated_eof_are_not_valid_defaults():
    samples, decoder = decode(POSE)
    assert not samples and decoder.counts['missing_stamp'] == 1
    samples, decoder = decode(HEADER+'\npose { name: "x"\nposition {')
    assert not samples and decoder.counts['truncated_message'] == 1


def test_multi_entity_and_model_identity():
    samples, _ = decode(HEADER+'\n'+POSE.replace(MODEL, 'other')+'\n'+POSE)
    assert [sample.model for sample in samples] == ['other', MODEL]
    assert parse_pose_samples(HEADER+'\n'+POSE, MODEL) == [(7_250_000_000, [0, 0, .6, 0, 0, 0, 1])]


def test_real_captured_message_fixture():
    # Exact shape observed in raw_pose.txt, including default fields omitted.
    text = '''header {
  stamp {
    sec: 3
    nsec: 449000000
  }
}
pose {
  name: "_preload_suitcase_loafbrr_small"
  id: 11
  position {
    z: -100
  }
  orientation {
    w: 1
  }
}
'''
    samples, decoder = decode(text)
    assert not decoder.errors
    assert samples[0].stamp_ns == 3_449_000_000
    assert samples[0].pose7 == (0, 0, -100, 0, 0, 0, 1)


def test_line_stream_reconstructed_from_arbitrary_record_chunks():
    text = HEADER+'\n'+POSE+'\n'
    for width in (1, 3, 17, 65536):
        decoder = PoseTextDecoder()
        pending = ''
        result = []
        for offset in range(0, len(text), width):
            pending += text[offset:offset+width]
            while '\n' in pending:
                line, pending = pending.split('\n', 1)
                result.extend(decoder.feed_line(line))
        decoder.finish()
        assert len(result) == 1 and result[0].model == MODEL


def test_diagnostics_are_bounded_and_framing_has_size_limit():
    decoder = PoseTextDecoder()
    for _ in range(100):
        decoder.feed_line('garbage')
    assert decoder.counts['unexpected_top_level'] == 100
    assert len(decoder.errors) == 32
    decoder.feed_line('pose {' + ' ' * decoder.MAX_BLOCK_CHARS)
    assert decoder.counts['invalid_framing'] == 1
