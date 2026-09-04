"""Locate installed share files without rospkg."""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

ENV_SCENE_TF = "LUGGAGE_SCENE_TF_CONFIG"
ENV_GAZEBO_SHARE = "LUGGAGE_GAZEBO_SHARE"


def package_share(package_name):
    return get_package_share_directory(package_name)


def description_config_path(*parts):
    return os.path.join(package_share("luggage_description"), "config", *parts)


def _gazebo_source_tree():
    """Find src/luggage_gazebo even when this module is a symlink in install/."""
    cur = os.path.dirname(os.path.realpath(__file__))
    for _ in range(8):
        for candidate in (
                os.path.join(cur, "luggage_gazebo"),
                os.path.join(cur, "src", "luggage_gazebo"),
                os.path.normpath(os.path.join(cur, "..", "..", "luggage_gazebo")),
        ):
            if os.path.isdir(os.path.join(candidate, "models")):
                return candidate
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent
    return None


def gazebo_share():
    env = os.environ.get(ENV_GAZEBO_SHARE)
    if env:
        return env
    try:
        return get_package_share_directory("luggage_gazebo")
    except PackageNotFoundError:
        candidate = _gazebo_source_tree()
        if candidate:
            return candidate
        raise
