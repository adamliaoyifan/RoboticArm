"""Evaluation children load ROS/install dependencies without parent overlays."""
import importlib.util
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys

import pytest


SCRIPT = Path(__file__).resolve().parents[3]/'scripts/g3/occ_query_runtime_environment.py'
spec = importlib.util.spec_from_file_location('evaluation_environment', SCRIPT)
environment_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(environment_module)


def test_setup_loads_dependencies_and_isolates_two_installs(tmp_path, monkeypatch):
    ros = tmp_path/'ros packages'
    ros.mkdir()
    (ros/'fixture_ros_dependency.py').write_text("marker = 'ros-underlay'\n")
    setup = tmp_path/'ros setup.bash'
    setup.write_text('export PYTHONPATH='+shlex.quote(str(ros))+'\n'
                     'export ROS_DISTRO=fixture_ros\n')
    installs = []
    for name in ('baseline', 'candidate'):
        install = tmp_path/(name+' install')
        install.mkdir()
        (install/'fixture_product.py').write_text('marker = '+repr(name)+'\n')
        (install/'setup.bash').write_text(
            'export PYTHONPATH='+shlex.quote(str(install))+':"$PYTHONPATH"\n'
            'export AMENT_PREFIX_PATH='+shlex.quote(str(install))+'\n')
        installs.append(install)
    monkeypatch.setenv('PYTHONPATH', '/unwanted-candidate-overlay')
    monkeypatch.setenv('AMENT_PREFIX_PATH', '/unwanted-candidate-overlay')
    monkeypatch.setenv('LD_LIBRARY_PATH', '/unwanted-candidate-overlay')
    monkeypatch.setenv('ROS_DOMAIN_ID', '99')
    monkeypatch.setenv('OCC_ENV_KEEP', 'value=with\nnewline')
    before = dict(os.environ)
    for name, install in zip(('baseline', 'candidate'), installs):
        environment = environment_module.installed_environment(install, ros_setup=setup)
        result = subprocess.run([sys.executable, '-c',
            'import json,fixture_product,fixture_ros_dependency; '
            'print(json.dumps([fixture_product.marker,fixture_ros_dependency.marker]))'],
            env=environment, check=True, capture_output=True, text=True)
        assert json.loads(result.stdout) == [name, 'ros-underlay']
        assert environment['AMENT_PREFIX_PATH'] == str(install)
        assert environment['ROS_DISTRO'] == 'fixture_ros'
        assert 'ROS_DOMAIN_ID' not in environment
        assert 'LD_LIBRARY_PATH' not in environment
        assert environment['OCC_ENV_KEEP'] == before['OCC_ENV_KEEP']
    assert dict(os.environ) == before


@pytest.mark.parametrize('missing', ['ros', 'install'])
def test_missing_setup_fails_before_launch(tmp_path, missing):
    setup = tmp_path/'ros.bash'
    install = tmp_path/'install'
    install.mkdir()
    if missing != 'ros':
        setup.write_text('export ROS_DISTRO=fixture_ros\n')
    if missing != 'install':
        (install/'setup.bash').write_text('export AMENT_PREFIX_PATH=fixture_install\n')
    with pytest.raises(subprocess.CalledProcessError):
        environment_module.installed_environment(install, ros_setup=setup)
