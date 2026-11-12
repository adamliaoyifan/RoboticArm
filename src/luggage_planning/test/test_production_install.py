"""Admission must exercise actual imports/files, including stale mixed prefixes."""
import importlib.util
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('production_install', ROOT/'scripts/production/check_install.py')
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)


@pytest.fixture
def module_at_prefix(tmp_path, monkeypatch):
    prefix = tmp_path/'prefix'
    prefix.mkdir()
    module = prefix/'production_install_fixture.py'
    module.write_text('value = 17\n')
    monkeypatch.syspath_prepend(str(prefix))
    sys.modules.pop('production_install_fixture', None)
    yield prefix, module
    sys.modules.pop('production_install_fixture', None)


def test_import_matching_source(module_at_prefix):
    prefix, module = module_at_prefix
    assert checker.check_module('production_install_fixture', prefix, source=module)['passed']


def test_actual_import_outside_selected_prefix(module_at_prefix, tmp_path):
    _, module = module_at_prefix
    result = checker.check_module('production_install_fixture', tmp_path/'other', source=module)
    assert result['reason'] == 'WRONG_IMPORT_PREFIX'


def test_stale_installed_module(module_at_prefix, tmp_path):
    prefix, _ = module_at_prefix
    source = tmp_path/'current.py'
    source.write_text('value = 18\n')
    assert checker.check_module('production_install_fixture', prefix, source=source)['reason'] == 'SOURCE_INSTALL_MISMATCH'


def test_missing_module(tmp_path):
    assert checker.check_module('production_install_missing_fixture', tmp_path)['reason'] == 'IMPORT_FAILED'


def test_executable_missing_not_executable_stale_and_matching(tmp_path):
    source = tmp_path/'source.py'
    source.write_text('#!/usr/bin/python3\nprint(17)\n')
    prefix = tmp_path/'prefix'
    assert checker.check_executable(prefix, 'pkg', 'entry.py')['reason'] == 'EXECUTABLE_MISSING'
    entry = prefix/'pkg/lib/pkg/entry.py'
    entry.parent.mkdir(parents=True)
    entry.write_bytes(source.read_bytes())
    assert checker.check_executable(prefix, 'pkg', 'entry.py')['reason'] == 'EXECUTABLE_MISSING'
    entry.chmod(0o755)
    assert checker.check_executable(prefix, 'pkg', 'entry.py', source=source)['passed']
    source.write_text('#!/usr/bin/python3\nprint(18)\n')
    assert checker.check_executable(prefix, 'pkg', 'entry.py', source=source)['reason'] == 'SOURCE_INSTALL_MISMATCH'
    entry.unlink()
    entry.symlink_to(source)
    source.chmod(0o755)
    assert checker.check_executable(prefix, 'pkg', 'entry.py')['reason'] == 'EXECUTABLE_OUTSIDE_PREFIX'
