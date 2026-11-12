"""Behavioral installation migration checks, using only disposable files."""
import json
from pathlib import Path

import pytest

from module_layout import load_layout, prune_relocated_modules


@pytest.fixture
def layout(tmp_path):
    source = tmp_path/'src/luggage_planning/luggage_planning'
    install = tmp_path/'install/luggage_planning/local/lib/python3.10/dist-packages/luggage_planning'
    config = tmp_path/'src/luggage_planning/config'
    config.mkdir(parents=True)
    record = {'schema_version': 1, 'modules': {
        'luggage_planning.old_route': 'luggage_planning.routing.producer'}}
    (config/'python_module_layout.json').write_text(json.dumps(record))
    for package in (source, install):
        (package/'routing').mkdir(parents=True)
        (package/'routing/producer.py').write_text('value = 2\n')
    (install/'old_route.py').write_text('value = 1\n')
    (install/'shared.py').write_text('value = 3\n')
    (install/'__pycache__').mkdir()
    (install/'__pycache__/old_route.cpython-310.pyc').write_bytes(b'old')
    return tmp_path, source, install, config


def test_cleanup_removes_only_relocated_files_and_is_repeatable(layout):
    root, _, install, _ = layout
    removed = prune_relocated_modules(root, root/'install')
    assert len(removed) == 2
    assert not (install/'old_route.py').exists()
    assert not list((install/'__pycache__').iterdir())
    assert (install/'routing/producer.py').read_text() == 'value = 2\n'
    assert (install/'shared.py').read_text() == 'value = 3\n'
    assert prune_relocated_modules(root, root/'install') == []


@pytest.mark.parametrize('damage', ['source_missing', 'install_missing', 'old_source_restored'])
def test_incomplete_migration_preserves_previous_install(layout, damage):
    root, source, install, _ = layout
    if damage == 'source_missing':
        (source/'routing/producer.py').unlink()
    elif damage == 'install_missing':
        (install/'routing/producer.py').unlink()
    else:
        (source/'old_route.py').write_text('restored = True\n')
    with pytest.raises(ValueError):
        prune_relocated_modules(root, root/'install')
    assert (install/'old_route.py').read_text() == 'value = 1\n'


def test_invalid_module_path_refuses_before_filesystem_mutation(layout):
    root, _, install, config = layout
    (config/'python_module_layout.json').write_text(json.dumps({
        'schema_version': 1, 'modules': {'../../file': 'luggage_planning.routing.producer'}}))
    with pytest.raises(ValueError):
        load_layout(root)
    assert (install/'old_route.py').exists()
