"""Freeze and inspect opt-in Gazebo sensor faults; never a motion authority."""
import hashlib
import json
from pathlib import Path

import numpy as np

from luggage_gazebo.depth_dropout_fixture import DepthDropoutConfig


def validate_fixture_options(*, enabled, start, capture_suction_inputs):
    if enabled and not (start and capture_suction_inputs):
        raise ValueError('--depth-dropout-fixture requires --start and --capture-suction-inputs')


def configure_fixture(env, folder, source):
    env.pop('ELFIN_SIM_DEPTH_DROPOUT_CONFIG', None)
    env.pop('ELFIN_SIM_DEPTH_DROPOUT_EVIDENCE', None)
    if source is None:
        return None
    raw = Path(source).read_bytes()
    config = DepthDropoutConfig.from_record(json.loads(raw))
    frozen = Path(folder)/'input_depth_dropout.json'
    with frozen.open('xb') as stream:
        stream.write(raw)
    env['ELFIN_SIM_DEPTH_DROPOUT_CONFIG'] = str(frozen)
    env['ELFIN_SIM_DEPTH_DROPOUT_EVIDENCE'] = str(Path(folder)/'depth_dropout')
    return dict(file=frozen.name, sha256=hashlib.sha256(raw).hexdigest(),
                model_hash=config.model_hash, scene_revision=config.scene_revision)


def validate_fixture_binding(receipt, resolved):
    for key, launch_key in (('model_hash', 'model_sha256'), ('scene_revision', 'scene_revision')):
        if receipt[key] != resolved.get(launch_key):
            raise ValueError('DEPTH_DROPOUT_LAUNCH_IDENTITY:'+key)


def audit_fixture(folder):
    root = Path(folder)/'depth_dropout'
    summary = json.loads((root/'summary.json').read_text())
    missing = list(summary['missing'])
    if summary['trace_truncated']:
        missing.append('FIXTURE_TRACE_TRUNCATED')
    if not summary['selected']:
        missing.append('NO_EXACT_SELECTED_FIXTURE_INPUT')
    for row in summary['selected']:
        raw = (root/row['file']).read_bytes()
        if hashlib.sha256(raw).hexdigest() != row['file_sha256']:
            missing.append('FIXTURE_SELECTED_FILE_HASH')
        with np.load(root/row['file']) as arrays:
            for name, field in (('before_mm','input_sha256'), ('after_mm','output_sha256')):
                if hashlib.sha256(arrays[name].tobytes()).hexdigest() != row[field]:
                    missing.append('FIXTURE_SELECTED_ARRAY_HASH:'+name)
        suction = json.loads((Path(folder)/'suction_inputs'/row['suction_case']/'record.json').read_text())
        if row['identity'] != suction['identity'] or row['stamp_ns'] != suction['stamp_ns']:
            missing.append('FIXTURE_SUCTION_SELECTION_MISMATCH')
    return dict(complete=summary['complete'] and not missing, missing=missing,
                selected_count=len(summary['selected']), counts=summary['counts'],
                changed_selected_count=sum(row['active'] and row['changed_pixels'] > 0
                                           for row in summary['selected']))


def record_fixture_audit(manifest, folder, audit=audit_fixture):
    """Called after owned teardown; all audit errors affect evidence only."""
    if manifest.get('depth_dropout_fixture') is None:
        return
    try:
        result = audit(folder)
        manifest['depth_dropout_fixture']['audit'] = result
        if result['complete']:
            return
    except Exception as error:
        manifest['depth_dropout_fixture']['audit_error'] = type(error).__name__+':'+str(error)
    manifest['capture_complete'] = False
    manifest['missing'].append('exact depth dropout fixture inputs')
