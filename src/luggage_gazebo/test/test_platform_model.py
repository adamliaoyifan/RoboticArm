"""Verify spawn geometry agrees with platform-top TF for different scenes."""
from pathlib import Path
import importlib.util
import xml.etree.ElementTree as ET

import pytest

from luggage_description.scene_tf_config_utils import pickup_platform_top_in_world
from luggage_gazebo.platform_model import platform_sdf


TEMPLATE = Path(__file__).parents[1] / "models/pickup_platform/model.sdf"


@pytest.mark.parametrize("size", [(1., 1., .86), (.8, 1.2, .66)])
def test_rendered_and_collision_top_match_scene_tf(size):
    original = TEMPLATE.read_text()
    root = ET.fromstring(platform_sdf(original, size))
    config = {"pickup_platform": {"enabled": True, "size": size,
              "translation": [.559, .6, .1], "rotation_rpy": [0., 0., 0.]}}
    top, _ = pickup_platform_top_in_world(config)
    for kind in ("visual", "collision"):
        part = root.find("model/link/" + kind)
        dimensions = list(map(float, part.findtext("geometry/box/size").split()))
        pose = list(map(float, part.findtext("pose").split()))
        assert dimensions == list(size)
        assert .1 + pose[2] + dimensions[2] / 2 == pytest.approx(top[2])
    assert TEMPLATE.read_text() == original


@pytest.mark.parametrize("size", [(1, 1, -1), (1, 1, float('nan')), (1, 1)])
def test_invalid_dimensions_refused(size):
    with pytest.raises(ValueError):
        platform_sdf(TEMPLATE.read_text(), size)


def test_launch_sends_configured_geometry_to_create(monkeypatch):
    package = Path(__file__).parents[1]
    spec = importlib.util.spec_from_file_location(
        "platform_launch_test", package / "launch/sim_world.launch.py")
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    config = launch.load_scene_tf_config(
        str(package.parent / "luggage_description/config/scene_tf.yaml"))
    config["pickup_platform"]["size"] = [1., 1., .86]
    monkeypatch.setattr(launch, "load_scene_tf_config", lambda _: config)
    monkeypatch.setattr(launch, "get_package_share_directory", lambda _: str(package))
    monkeypatch.setattr(launch, "Node", lambda **kwargs: kwargs)
    actions = launch._scene_model_actions("scene.yaml")
    platform = next(action for action in actions
                    if action['arguments'][action['arguments'].index('-name') + 1]
                    == 'pickup_platform')
    args = platform['arguments']
    root = ET.fromstring(args[args.index('-string') + 1])
    assert float(root.findtext('model/link/visual/geometry/box/size').split()[2]) == .86
    assert float(root.findtext('model/link/collision/geometry/box/size').split()[2]) == .86
    assert '-file' not in args
