"""Read-only MCAP coverage audit. Never imports robot runtime or opens a ROS graph."""
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import time

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
import numpy as np
import yaml

from .cargo_acquisition_audit import AcquisitionIndex, cdr_header, distribution

SOURCES = (
    '/luggage/preprocessed/camera/color/image',
    '/luggage/preprocessed/camera/depth/image',
    '/luggage/semantic/instance_mask',
    '/luggage/preprocessed/camera/depth/camera_info',
)
HEADER_TOPICS = SOURCES + ('/joint_states', '/livox/lidar',
                          '/luggage/semantic/cargo_points')
EVENT_TOPICS = ('/vacuum/events_json', '/trajectory_executor/events',
                '/luggage/perception/detection/latest', '/luggage_detector/diagnostics_json')
DETECTION_TOPIC = '/luggage/perception/detection_frame'
OTHER_TOPICS = ('/tf', '/tf_static', '/robot_description', DETECTION_TOPIC) + EVENT_TOPICS


def sha(data):
    return hashlib.sha256(data).hexdigest()


def _fields(value):
    # MCAP's native dynamic decoder uses slots even though vars(obj) exists and is empty.
    if hasattr(value, '__slots__'):
        return {k: _fields(getattr(value, k)) for k in value.__slots__ if not k.startswith('_')}
    if hasattr(value, '__dict__'):
        return {k: _fields(v) for k, v in vars(value).items() if not k.isupper()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [_fields(v) for v in value]
    return value


def inventory(root):
    result = []
    for path in sorted(root.rglob('metadata.yaml')):
        data = path.read_bytes()
        info = yaml.safe_load(data)['rosbag2_bagfile_information']
        files = []
        for rel in info.get('relative_file_paths', []):
            source = path.parent / rel
            stat = source.stat()
            files.append(dict(path=str(source), bytes=stat.st_size, mtime_ns=stat.st_mtime_ns))
        result.append(dict(path=str(path), metadata_sha256=sha(data), files=files,
                           storage=info['storage_identifier'],
                           duration_ns=info['duration']['nanoseconds'],
                           topics=[dict(name=t['topic_metadata']['name'],
                                        type=t['topic_metadata']['type'],
                                        count=t['message_count'])
                                   for t in info['topics_with_message_count']]))
    return result


def _decode_image(msg):
    types = {'rgb8': ('u1', 3), 'bgr8': ('u1', 3), 'mono16': ('u2', 1),
             '16UC1': ('u2', 1), '32FC1': ('f4', 1)}
    if msg.encoding not in types:
        raise ValueError('unsupported example encoding ' + msg.encoding)
    kind, channels = types[msg.encoding]
    dtype = np.dtype(('>' if msg.is_bigendian else '<') + kind)
    row = int(msg.width) * channels * dtype.itemsize
    if msg.step < row or len(msg.data) != msg.step * msg.height:
        raise ValueError('malformed image payload')
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
    array = raw[:, :row].copy().view(dtype).reshape(msg.height, msg.width, channels)
    return array[..., 0] if channels == 1 else array


def audit_mcap(path, output, expected_counts):
    output.mkdir(parents=True, exist_ok=False)
    start = time.monotonic()
    index = AcquisitionIndex()
    decoder = DecoderFactory()
    counts, tf_edges = Counter(), defaultdict(list)
    events, joint_deltas, detections = [], [], []
    previous_joint = None
    identical_joint_pairs = 0
    source_schema = {}
    trace_path = output / 'acquisition_headers.jsonl'
    with path.open('rb') as handle, trace_path.open('w') as trace:
        reader = make_reader(handle)
        for schema, channel, record in reader.iter_messages(topics=HEADER_TOPICS + OTHER_TOPICS):
            if time.monotonic() - start > 15 * 60:
                raise TimeoutError('audit wall clock budget exhausted')
            topic = channel.topic
            counts[topic] += 1
            source_schema[topic] = dict(name=schema.name, encoding=schema.encoding,
                                        schema_sha256=sha(schema.data))
            if topic in HEADER_TOPICS:
                stamp, frame = cdr_header(record.data)
                # Source hashes are exact; counts for other streams do not certify identity.
                digest = sha(record.data) if topic in SOURCES else ''
                index.add(topic, stamp, frame, record.log_time, digest)
                trace.write(json.dumps(dict(topic=topic, stamp_ns=stamp, frame=frame,
                                            log_time_ns=record.log_time, cdr_sha256=digest)) + '\n')
                if topic != '/joint_states':
                    continue
            decode = decoder.decoder_for(channel.message_encoding, schema)
            if decode is None:
                raise ValueError('no native schema decoder for ' + topic)
            msg = decode(record.data)
            if topic == '/joint_states':
                joints = (tuple(msg.name), np.asarray(msg.position))
                if previous_joint is not None and joints[0] == previous_joint[0]:
                    delta = np.max(np.abs(joints[1] - previous_joint[1]))
                    joint_deltas.append(float(delta))
                    identical_joint_pairs += int(delta == 0)
                previous_joint = joints
            elif topic in ('/tf', '/tf_static'):
                for tf in msg.transforms:
                    t, q = tf.transform.translation, tf.transform.rotation
                    tf_edges[(topic, tf.header.frame_id, tf.child_frame_id)].append(
                        (tf.header.stamp.sec * 10**9 + tf.header.stamp.nanosec,
                         (t.x, t.y, t.z, q.x, q.y, q.z, q.w)))
            elif topic in EVENT_TOPICS:
                events.append(dict(topic=topic, log_time_ns=record.log_time, data=msg.data))
            elif topic == DETECTION_TOPIC:
                measured = _fields(msg)
                if not measured or 'header' not in measured:
                    raise ValueError('empty/malformed decoded DetectionFrame')
                detections.append(dict(log_time_ns=record.log_time, measured_output=measured))
            elif topic == '/robot_description':
                (output / 'recorded_robot_description.xml').write_text(msg.data)
    mismatch = {t: dict(expected=expected_counts.get(t), actual=n)
                for t, n in counts.items() if n != expected_counts.get(t)}
    # Required streams absent from the MCAP also fail count parity, never vanish from the gate.
    for t in HEADER_TOPICS + OTHER_TOPICS:
        if expected_counts.get(t, 0) and not counts[t]:
            mismatch[t] = dict(expected=expected_counts[t], actual=0)
    keys = index.exact_keys(SOURCES)
    mask_keys = sorted(index.hashes[SOURCES[2]])
    candidates = keys or mask_keys
    selected = [candidates[i] for i in sorted(set(np.linspace(
        0, len(candidates) - 1, min(3, len(candidates)), dtype=int)))] if candidates else []
    examples = _capture_examples(path, output, selected)
    tf_summary = [dict(topic=k[0], parent=k[1], child=k[2], count=len(v),
                       first_stamp_ns=min(x[0] for x in v), last_stamp_ns=max(x[0] for x in v),
                       unique_transforms=len(set(x[1] for x in v)),
                       static_transform=list(v[0][1]) if k[0] == '/tf_static' else None)
                  for k, v in sorted(tf_edges.items())]
    result = dict(source=str(path), topic_counts=dict(counts), count_mismatches=mismatch,
                  sources=source_schema, acquisitions=index.summary(),
                  exact_four_source_join_count=len(keys),
                  exact_join_definition='same positive acquisition stamp and optical frame; no conflicting source',
                  tf_edges=tf_summary, events=events, examples=examples,
                  joint_delta_max_abs_rad=distribution(joint_deltas),
                  identical_consecutive_joint_pairs=identical_joint_pairs,
                  joint_static_classification='not_evaluated: no validated motion threshold or scene labels',
                  absolute_pose_error='not_evaluated: no independent reference supplied',
                  physical_false_association='not_evaluated: no labelled competing-box benchmark',
                  current_deployment_transfer='not_evaluated: historical sensor/extrinsic qualification required',
                  elapsed_s=time.monotonic() - start)
    _verify_examples(output, examples)
    (output / 'detection_outputs.json').write_text(json.dumps(detections, indent=2))
    (output / 'summary.json').write_text(json.dumps(result, indent=2))
    if mismatch:
        raise ValueError('selected topic count mismatch; see durable summary.json')
    if not examples:
        raise ValueError('no real source example captured')
    return result


def _verify_examples(output, examples):
    """Exercise retained boundary decoding; existence alone is not capture health."""
    from mcap.records import Schema
    import cv2
    for example in examples:
        folder = output / ('frame_' + str(example['stamp_ns']))
        for records in example['exact_sources'].values():
            for item in records:
                prefix = folder / item['files']
                data = prefix.with_suffix('.cdr').read_bytes()
                schema_data = prefix.with_suffix('.ros2msg').read_bytes()
                if sha(data) != item['cdr_sha256'] or sha(schema_data) != item['schema_sha256']:
                    raise ValueError('retained source hash mismatch')
                schema = Schema(id=1, name=item['schema'], encoding='ros2msg', data=schema_data)
                # Each standalone retained schema owns a fresh decoder cache.
                DecoderFactory().decoder_for('cdr', schema)(data)
                png = prefix.with_suffix('.png')
                if png.exists() and cv2.imread(str(png), cv2.IMREAD_UNCHANGED) is None:
                    raise ValueError('retained image cannot be decoded')
                npy = prefix.with_suffix('.npy')
                if npy.exists():
                    np.load(npy, allow_pickle=False)


def _capture_examples(path, output, selected):
    """At most3 source boundaries. Missing exact partners remain explicitly missing."""
    import cv2
    decoder = DecoderFactory()
    examples = {key: {} for key in selected}
    with path.open('rb') as handle:
        for schema, channel, record in make_reader(handle).iter_messages(topics=SOURCES):
            key = cdr_header(record.data)
            if key not in examples:
                continue
            if sum(p.stat().st_size for p in output.rglob('*') if p.is_file()) + len(record.data) > 112 * 1024**2:
                raise ValueError('example evidence budget exhausted')
            decode = decoder.decoder_for(channel.message_encoding, schema)
            msg = decode(record.data)
            folder = output / ('frame_' + str(key[0]))
            folder.mkdir(exist_ok=True)
            role = str(SOURCES.index(channel.topic))
            # Preserve conflicting bytes separately, never overwrite a source by key alone.
            prefix = role + '_' + sha(record.data)
            (folder / (prefix + '.cdr')).write_bytes(record.data)
            (folder / (prefix + '.ros2msg')).write_bytes(schema.data)
            item = dict(topic=channel.topic, cdr_sha256=sha(record.data),
                        schema=schema.name, schema_sha256=sha(schema.data), files=prefix)
            if channel.topic != SOURCES[3]:
                array = _decode_image(msg)
                item.update(encoding=msg.encoding, shape=list(array.shape))
                if channel.topic == SOURCES[2]:
                    labels, pixels = np.unique(array, return_counts=True)
                    item['label_pixel_counts'] = dict(zip(map(str, labels.tolist()), pixels.tolist()))
                if array.dtype.kind == 'f':
                    np.save(folder / (prefix + '.npy'), array)
                else:
                    visible = array[..., ::-1] if msg.encoding == 'rgb8' else array
                    if not cv2.imwrite(str(folder / (prefix + '.png')), visible):
                        raise ValueError('example image encode failed')
            else:
                item.update(width=msg.width, height=msg.height, k=list(msg.k), d=list(msg.d),
                            distortion_model=msg.distortion_model)
            examples[key].setdefault(channel.topic, []).append(item)
    return [dict(stamp_ns=k[0], frame=k[1], exact_sources=v,
                 missing_topics=[t for t in SOURCES if t not in v]) for k, v in examples.items()]
