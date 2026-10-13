"""The rehearsal driver must plan on the scene the stack was launched with.

Rehearsal a (2026-10-07) launched the stack with ``scene_tf_deploy.yaml`` but
started the driver without ``--scene-tf-config``. The driver then resolved the
simulation default ``scene_tf.yaml`` and planned for a container at
(1.5, 0, 0) while the stack ran it at (-1.5, 0.6, 0), yaw pi: every candidate
was ``IK_UNAVAILABLE``.
"""
import importlib.util
from pathlib import Path
import types

import yaml

REPO = Path(__file__).resolve().parents[3]
CONFIG = REPO / "src" / "luggage_description" / "config"


def _load_rehearsal():
    spec = importlib.util.spec_from_file_location(
        "frame_migration_st09_rehearsal",
        REPO / "scripts" / "frame_migration_st09_rehearsal.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _container_translation(path):
    config = yaml.safe_load(Path(path).read_text())
    for entry in config["static_transforms"]:
        if entry["child"] == "container_link":
            return entry["translation"]
    raise AssertionError("no container_link transform in %s" % path)


def test_driver_command_carries_the_launch_scene_file():
    rehearsal = _load_rehearsal()
    deploy = CONFIG / "scene_tf_deploy.yaml"
    args = types.SimpleNamespace(fault="")
    command = rehearsal.driver_command(args, Path("/tmp/out"), deploy)
    assert command[command.index("--scene-tf-config") + 1] == str(deploy)
    fault = rehearsal.driver_command(
        types.SimpleNamespace(fault="pre_place"), Path("/tmp/out"), deploy)
    assert fault[fault.index("--scene-tf-config") + 1] == str(deploy)


def test_deploy_and_simulation_layouts_differ():
    """The two scene files place the container on opposite sides."""
    deploy = _container_translation(CONFIG / "scene_tf_deploy.yaml")
    simulation = _container_translation(CONFIG / "scene_tf.yaml")
    assert deploy == [-1.5, 0.6, 0.0]
    assert simulation == [1.5, 0.0, 0.0]


def _load_layout_check():
    spec = importlib.util.spec_from_file_location(
        "check_live_layout",
        REPO / "scripts" / "st09_i" / "check_live_layout.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_live_layout_check_accepts_deploy_and_rejects_simulation_layout():
    check = _load_layout_check()
    deploy = check.expected_container_pose(CONFIG / "scene_tf_deploy.yaml")
    simulation = check.expected_container_pose(CONFIG / "scene_tf.yaml")
    # What the stack publishes for the deploy file: yaw pi as a quaternion
    # whose w is slightly negative, which reads back as -pi.
    live_deploy = ([-1.5, 0.6, 0.0], check.yaw_from_quaternion(
        -0.0, 0.0, 0.9999999999932537, -3.673205103346574e-06))
    assert check.compare(deploy, live_deploy)[0]
    assert not check.compare(simulation, live_deploy)[0]
    assert not check.compare(deploy, ([1.5, 0.0, 0.0], 0.0))[0]
