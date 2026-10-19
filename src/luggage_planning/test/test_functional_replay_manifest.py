"""A replay must bind every implicitly loaded boundary, not only CLI paths."""
import hashlib
import importlib.util
import json
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[3] / 'scripts/placement_prior/functional_replay.py'
spec = importlib.util.spec_from_file_location('functional_replay', SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.fixture
def fixture(tmp_path):
    manifest = dict(schema='placement-prior-functional/1', mode='captured_consumer_plan_only',
                    select_index=0, choice_index=0, observe_pose='observe', capture='capture')
    for key in module.FILE_OPTIONS:
        manifest[key] = key + '.data'
    ledger = 'capture/dumps/typed_service_boundaries/boundaries.jsonl'
    contents = {path:b'input' for path in manifest.values() if isinstance(path, str) and path.endswith('.data')}
    contents.update({
        ledger: ('\n'.join(json.dumps(row) for row in [
            {'boundary':'/cargo_map/get_stats:response','file':'stats.cdr'},
            {'boundary':'/trajectory_selector/select_place_path_v2:request','file':'select.cdr'}])).encode(),
        'capture/dumps/typed_service_boundaries/stats.cdr':b'stats',
        'capture/dumps/typed_service_boundaries/select.cdr':b'select',
        'capture/dumps/runtime_identity.json':b'{}',
        'capture/dumps/box_00_paths/candidates.json':b'{}'})
    for name, data in contents.items():
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    manifest['artifacts'] = {name:hashlib.sha256(data).hexdigest() for name,data in contents.items()}
    return manifest, tmp_path


def test_exact_command_uses_validated_paths_without_launch(fixture):
    manifest, root = fixture
    command = module.prepare(manifest, root, root/'output')
    assert command[command.index('--attachment-response')+1] == str(root/'attachment_response.data')
    assert command[command.index('--choice-index')+1] == '0'
    assert '--measure-resources' in command
    assert not (root/'output').exists()


@pytest.mark.parametrize('name', ['scene.data', 'capture/dumps/runtime_identity.json',
                                'capture/dumps/box_00_paths/candidates.json',
                                'capture/dumps/typed_service_boundaries/select.cdr',
                                'capture/dumps/typed_service_boundaries/stats.cdr'])
def test_changed_explicit_or_implicit_input_is_refused(fixture, name):
    manifest, root = fixture
    (root/name).write_bytes(b'changed')
    with pytest.raises(ValueError, match='hash mismatch'):
        module.prepare(manifest, root, root/'output')


def test_missing_hash_and_unsupported_hardware_claim_are_refused(fixture):
    manifest, root = fixture
    del manifest['artifacts']['capture/dumps/runtime_identity.json']
    with pytest.raises(ValueError, match='not frozen'):
        module.prepare(manifest, root, root/'output')
    manifest['mode'] = 'hardware_F2_pass'
    with pytest.raises(ValueError, match='only captured consumer'):
        module.prepare(manifest, root, root/'output')
