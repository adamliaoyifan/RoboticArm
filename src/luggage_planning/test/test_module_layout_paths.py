"""Relocation must preserve source resource discovery without an install fallback."""
from pathlib import Path
from unittest.mock import patch

from luggage_planning.atlas import runtime


def test_source_atlas_resources_resolve_without_ament_fallback():
    package = Path(__file__).resolve().parents[1]
    with patch('ament_index_python.packages.get_package_share_directory',
               side_effect=LookupError('no installed packages')):
        assert Path(runtime.atlas_runtime_config_path()) == package/'config/atlas_runtime.yaml'
        assert Path(runtime.package_data_dir()) == package/'data'


def test_source_container_mesh_resolves_without_ament_fallback():
    from luggage_description.scene_mesh_utils import container_model_name
    from luggage_description.scene_tf_config_utils import load_scene_tf_config
    source = Path(__file__).resolve().parents[2]
    config = load_scene_tf_config(str(source/'luggage_description/config/scene_tf_deploy_update.yaml'))
    name = container_model_name(config)
    with patch('ament_index_python.packages.get_package_share_directory',
               side_effect=LookupError('no installed packages')):
        result = Path(runtime._collision_mesh_path(name))
        assert result == source/'luggage_gazebo/models'/name/'meshes/container_collision.stl'
        assert result.is_file()


def test_explicit_flat_layout_resource_anchor_is_preserved(tmp_path):
    module = tmp_path/'luggage_planning/atlas_runtime.py'
    (tmp_path/'config').mkdir()
    config = tmp_path/'config/atlas_runtime.yaml'
    config.write_text('enabled: false\n')
    assert Path(runtime.atlas_runtime_config_path(module_file=str(module))) == config
