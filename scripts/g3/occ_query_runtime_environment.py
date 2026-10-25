"""Construct an isolated ROS environment for no-motion evaluation children."""
import os
from pathlib import Path
import subprocess


def installed_environment(install, *, ros_setup='/opt/ros/humble/setup.bash'):
    """Source one explicit install without inheriting another ROS overlay.

    Return a child environment; never mutate the calling agent's environment.
    The setup override permits behavioral tests using temporary setup scripts.
    """
    excluded = {'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH',
        'PYTHONPATH', 'LD_LIBRARY_PATH', 'ROS_DISTRO', 'ROS_VERSION',
        'ROS_PYTHON_VERSION', 'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY'}
    environment = {k: v for k, v in os.environ.items() if k not in excluded}
    result = subprocess.run(['bash', '--noprofile', '--norc', '-c',
        'source "$1" && source "$2/setup.bash" && env -0', 'occ-g2-env',
        str(Path(ros_setup).resolve()), str(Path(install).resolve())],
        env=environment, check=True, capture_output=True)
    return dict(item.decode().split('=', 1) for item in result.stdout.split(b'\0') if item)
