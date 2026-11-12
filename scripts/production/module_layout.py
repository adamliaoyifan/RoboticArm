"""Remove only explicitly relocated Python modules from a production install.

Colcon installs new module paths but leaves obsolete files behind. The checked-in
layout records the retired paths; it is migration metadata, never an import hook.
"""
import json
from pathlib import Path
import re


def load_layout(root):
    path = Path(root)/'src/luggage_planning/config/python_module_layout.json'
    record = json.loads(path.read_text())
    if record.get('schema_version') != 1 or not isinstance(record.get('modules'), dict):
        raise ValueError('invalid planning module layout')
    modules = record['modules']
    for old, new in modules.items():
        if (not re.fullmatch(r'luggage_planning\.[a-z][a-z0-9_]*', old)
                or not re.fullmatch(r'luggage_planning\.[a-z][a-z0-9_]*\.[a-z][a-z0-9_]*', new)):
            raise ValueError('invalid relocated module name')
    if len(set(modules.values())) != len(modules):
        raise ValueError('duplicate module destination')
    return modules


def installed_package_directory(prefix):
    prefix = Path(prefix).resolve()
    candidates = []
    for local in ('', 'local/'):
        for kind in ('dist-packages', 'site-packages'):
            candidates.extend((prefix/'luggage_planning').glob(
                local+'lib/python*/'+kind+'/luggage_planning'))
    candidates = [p for p in candidates if p.is_dir()]
    if len(candidates) != 1 or not candidates[0].resolve().is_relative_to(prefix):
        raise ValueError('one concrete installed planning package required')
    return candidates[0]


def prune_relocated_modules(root, prefix):
    root = Path(root)
    modules = load_layout(root)
    source = root/'src/luggage_planning'
    installed = installed_package_directory(prefix)
    obsolete = []
    # Validate the entire migration before deleting anything. An incomplete new
    # install or a reintroduced source module must leave the old install intact.
    for old, new in modules.items():
        old_relative = Path(*old.split('.')).with_suffix('.py')
        new_relative = Path(*new.split('.')).with_suffix('.py')
        if (source/old_relative).exists():
            raise ValueError('retired module exists in source: '+old)
        if not (source/new_relative).is_file() or not (installed.parent/new_relative).is_file():
            raise ValueError('relocated module missing from source/install: '+new)
        obsolete.append(installed/old_relative.name)
    removed = []
    for path in obsolete:
        if path.is_file() or path.is_symlink():
            path.unlink()
            removed.append(str(path))
        for cached in (installed/'__pycache__').glob(path.stem+'.*.pyc'):
            cached.unlink()
            removed.append(str(cached))
    return removed
