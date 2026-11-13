"""Static scene wiring; captured run17 is a regression, not sensor calibration."""
import copy
from pathlib import Path

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.conditions import IfCondition
from launch_ros.utilities import evaluate_parameters
from pickup_continuation_fixtures import case_archive

from luggage_gazebo.simulation_pickup_workspace import (
    pickup_workspace_parameters, simulation_segmenter_node,
)
from luggage_perception.semantic_segmenter import (
    WorkspaceAcceptanceContext, evaluate_detection_acceptance,
)


def deployed_scene():
    path = Path(get_package_share_directory('luggage_description')) / 'config/scene_tf_deploy_update.yaml'
    return yaml.safe_load(path.read_text())


def test_current_scene_workspace(case_archive):
    scene = deployed_scene()
    result = pickup_workspace_parameters(scene)
    case_archive.save('source_scene', scene)
    case_archive.save('resolved_workspace', result)
    assert result == dict(workspace_accept_enabled=True,
                          workspace_center_xy=[.559, .6],
                          workspace_half_extents=[.5, .5],
                          workspace_plane_z=.86, workspace_world_frame='world')
    assert 'workspace_margin_m' not in result


def test_moved_resized_yawed_platform_uses_same_source(case_archive):
    scene = copy.deepcopy(deployed_scene())
    scene['world_frame'] = 'cell_world'
    scene['pickup_platform'].update(translation=[3., -2., .4],
                                   size=[1.2, .8, .7], rotation_rpy=[0., 0., .9])
    result = pickup_workspace_parameters(scene)
    case_archive.save('source_scene', scene)
    case_archive.save('resolved_workspace', result)
    assert result['workspace_center_xy'] == [3., -2.]
    assert result['workspace_plane_z'] == pytest.approx(1.1)
    assert result['workspace_half_extents'] == [.6, .4]
    assert result['workspace_world_frame'] == 'cell_world'


@pytest.mark.parametrize('platform', [None, {'enabled': False}])
def test_no_platform_does_not_retain_stale_square(case_archive, platform):
    scene = {} if platform is None else {'pickup_platform': platform}
    result = pickup_workspace_parameters(scene)
    case_archive.save('source_scene', scene)
    case_archive.save('resolved_workspace', result)
    assert result == {'workspace_accept_enabled': False}


@pytest.mark.parametrize('rotation', [[.1, 0., 0.], [0., .1, .8], [3.141592653589793, 0., 0.]])
def test_unsupported_plane_is_explicit(case_archive, rotation):
    scene = deployed_scene()
    scene['pickup_platform']['rotation_rpy'] = rotation
    case_archive.save('source_scene', scene)
    with pytest.raises(ValueError, match='REQUIRES_HORIZONTAL_PLATFORM'):
        pickup_workspace_parameters(scene)


def test_captured_bbox_and_optical_transform_reproduce_stale_region(case_archive):
    # Run17 acquisition39.864s: exact dynamic TF + frozen static URDF chain.
    optical_to_world = (
        (-.9980968961727382, -.06157381299237229, .003354311244799683, .5992903106897141),
        (-.06148218276419472, .9978494806205339, .022723451055703822, .6015203498603565),
        (-.004746267259308211, .02247397559203034, -.9997361618788191, 1.8517471023155083),
    )
    # Simulator color/depth share the same pinhole sensor. Depth CameraInfo
    # supplies the captured pinhole values; this is not real calibration.
    intrinsics = dict(fx=323.4574352118316, fy=323.4574352118316, cx=319.5, cy=179.5)
    parameters = pickup_workspace_parameters(deployed_scene())
    common = dict(**intrinsics, plane_z=parameters['workspace_plane_z'],
                  half_xy=tuple(parameters['workspace_half_extents']), margin=.15,
                  optical_to_world=optical_to_world)
    old = WorkspaceAcceptanceContext(center_xy=(-1., 0.), **common)
    new = WorkspaceAcceptanceContext(center_xy=tuple(parameters['workspace_center_xy']), **common)
    bbox = [231, 89, 454, 255]
    before = evaluate_detection_acceptance(bbox, old)
    after = evaluate_detection_acceptance(bbox, new)
    case_archive.save('input', dict(bbox=bbox, transform=optical_to_world, **intrinsics))
    case_archive.save('verdicts', dict(before=before, after=after))
    assert before == (False, 'outside_workspace')
    assert after == (True, 'in_workspace')
    assert evaluate_detection_acceptance([-1000, 0, -950, 50], new) == (False, 'outside_workspace')


def test_actual_ros_node_overrides_yaml_region_without_margin_change(case_archive, tmp_path):
    scene = deployed_scene()
    scene['pickup_platform']['translation'] = [3., 2., .2]
    scene_file = tmp_path / 'scene.yaml'
    scene_file.write_text(yaml.safe_dump(scene))
    semantic = Path(get_package_share_directory('luggage_perception')) / 'config/semantic_segmenter.yaml'
    node = simulation_segmenter_node(
        scene_config=str(scene_file), semantic_config=str(semantic),
        model_path='/installed/model.pt', require_backend='bbox_fill',
        environment={'DIAGNOSTIC_MARKER': 'pass_through'}, condition=IfCondition('true'))
    evaluated = evaluate_parameters(LaunchContext(), node._Node__parameters)
    actual = {}
    for block in evaluated:
        if isinstance(block, Path):
            actual.update(yaml.safe_load(block.read_text())['semantic_segmenter']['ros__parameters'])
        else:
            actual.update(block)
    case_archive.save('evaluated_node_parameters', actual)
    assert tuple(actual['workspace_center_xy']) == (3., 2.)
    assert actual['workspace_plane_z'] == pytest.approx(1.06)
    assert tuple(actual['workspace_half_extents']) == (.5, .5)
    assert actual['workspace_margin_m'] == .15
    assert actual['confidence_threshold'] == .2
    assert actual['model_name'] == '/installed/model.pt'
    assert actual['use_sim_time'] is True
