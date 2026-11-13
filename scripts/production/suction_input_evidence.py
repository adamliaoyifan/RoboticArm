"""Optional exact consumed-input evidence for simulation; no sensing effects.

SuctionInputCapture owns staging/pinning in the detector. This module only binds
its existing diagnostic environment switch and audits already-written inputs.
It never reconstructs a missing transform or authorizes a grasp or trajectory.
"""
import gzip
import hashlib
import importlib
import json
from pathlib import Path


def configure_capture(environment, folder, *, enabled):
    """Change only the diagnostic root when explicitly requested by the caller."""
    if not enabled:
        return dict(managed=False, enabled=bool(environment.get('ELFIN_SUCTION_INPUT_CAPTURE')))
    directory = Path(folder).resolve() / 'suction_inputs'
    environment['ELFIN_SUCTION_INPUT_CAPTURE'] = str(directory)
    return dict(managed=True, enabled=True, directory=str(directory),
                scope='exact selected detector inputs; no extra observation or motion')


def _hash(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _selected_message(directory, manifest, topic):
    from rclpy.serialization import deserialize_message
    rows = [row for row in manifest['records'] if row['topic'] == topic
            and row['acquisition_stamp_ns'] == manifest['acquisition_stamp_ns']]
    if len(rows) != 1:
        raise ValueError('SELECTED_INPUT_NOT_UNIQUE:' + topic)
    row = rows[0]
    raw = (directory / row['file']).read_bytes()
    if row['file'].endswith('.gz'):
        raw = gzip.decompress(raw)
    if hashlib.sha256(raw).hexdigest() != row['sha256']:
        raise ValueError('SELECTED_CDR_HASH_MISMATCH:' + topic)
    module, name = row['ros_type'].rsplit('.', 1)
    return deserialize_message(raw, getattr(importlib.import_module(module), name))


def audit_selected_inputs(folder):
    """Match each selected acquisition by instance/generation/stamp, never nearest.

    A run without any selected pickup has no measured capture sample. It is not
    an incomplete selection inventory, and must be reported as NOT_EVALUATED.
    """
    import numpy as np
    from luggage_perception.ros_message_adapters import depth_array_from_msg

    folder = Path(folder)
    captured, invalid = [], []
    for record_path in sorted((folder / 'suction_inputs').glob('case-*/record.json')):
        try:
            captured.append((record_path, json.loads(record_path.read_text())))
        except (OSError, ValueError) as error:
            invalid.append(dict(file=str(record_path.relative_to(folder)), reason=str(error)))
    selected = []
    for manifest_path in sorted((folder / 'driver').glob('pickup_acquisition_*/manifest.json')):
        result = dict(acquisition=str(manifest_path.parent.relative_to(folder)), complete=False)
        try:
            manifest = json.loads(manifest_path.read_text())
            if not manifest['selected_acquisition_complete']:
                raise ValueError('SELECTED_ACQUISITION_INCOMPLETE')
            frame = _selected_message(manifest_path.parent, manifest, '/luggage/perception/detection_frame')
            stamp = manifest['acquisition_stamp_ns']
            if int(frame.header.stamp.sec)*10**9+int(frame.header.stamp.nanosec) != stamp:
                raise ValueError('SELECTED_FRAME_STAMP_MISMATCH')
            identity = (frame.instance_id, int(frame.generation), stamp)
            result.update(instance_id=identity[0], generation=identity[1], stamp_ns=stamp)
            matches = [(path, record) for path, record in captured
                       if tuple(record.get('identity', ())) == identity[:2]
                       and record.get('stamp_ns') == stamp]
            if len(matches) != 1:
                raise ValueError('EXACT_CONSUMED_INPUT_NOT_UNIQUE')
            path, record = matches[0]
            if record['schema'] != 'suction-input-capture/1' or record['frame'] != frame.header.frame_id:
                raise ValueError('CONSUMED_INPUT_METADATA_MISMATCH')
            depth_msg = _selected_message(manifest_path.parent, manifest, '/luggage/preprocessed/camera/depth/image')
            if int(depth_msg.header.stamp.sec)*10**9+int(depth_msg.header.stamp.nanosec) != stamp:
                raise ValueError('SELECTED_DEPTH_STAMP_MISMATCH')
            depth = depth_array_from_msg(depth_msg)
            arrays_path = path.with_name('inputs.npz')
            with np.load(arrays_path, allow_pickle=False) as arrays:
                same_depth = depth is not None and np.array_equal(arrays['depth_mm'], depth)
                matrix = arrays['optical_to_world']
                if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
                    raise ValueError('CAPTURED_TRANSFORM_INVALID')
                if depth is None or arrays['plane_labels'].shape != depth.shape or 'instance_region' not in arrays:
                    raise ValueError('CAPTURED_EVALUATOR_INPUT_MISSING')
            if not same_depth:
                raise ValueError('SELECTED_DEPTH_DIFFERS_FROM_CONSUMED_INPUT')
            result.update(complete=True, depth_equal=True,
                case=str(path.parent.relative_to(folder)), record_sha256=_hash(path),
                inputs_sha256=_hash(arrays_path), inputs_bytes=arrays_path.stat().st_size)
        except Exception as error:
            # Evidence errors cannot escape to skip the simulation owner's teardown.
            result['reason'] = type(error).__name__ + ':' + str(error)
        selected.append(result)
    complete = not invalid and all(row['complete'] for row in selected)
    status = 'INCOMPLETE' if not complete else ('COMPLETE' if selected else 'NOT_EVALUATED')
    return dict(status=status, complete=complete, selected_count=len(selected),
                captured_count=len(captured), selected=selected, invalid=invalid,
                scope='selected input identity and depth parity; not domain replay or motion qualification')
