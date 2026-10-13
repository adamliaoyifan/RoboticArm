"""Source-tree and fresh-install resolution of the deploy profile.

The same four launch-resolution cases must hold from the source tree
and from a fresh install prefix. The source half always runs; the
install half activates when ``ELFIN_PD_INSTALL_ROOT`` names a colcon
install root that carries the deploy assets (the evidence campaign sets
it), comparing every measured identity hash across both roots.
"""

from __future__ import division

import os

import pytest

from luggage_planning import deploy_profile as dp

from conftest import requires_assets


def _install_shares():
    root = os.environ.get("ELFIN_PD_INSTALL_ROOT", "")
    if not root or not os.path.isdir(root):
        return None
    shares = {}
    for package in ("luggage_description", "luggage_planning"):
        share = os.path.join(root, package, "share", package)
        if not os.path.isdir(share):
            return None
        shares[package] = share
    return shares


@requires_assets
def test_source_default_resolution():
    """Default (empty-argument) hardware resolution from the source tree."""
    record = dp.resolve_deploy_profile(check_atlas=True)
    assert record["scene_tf_path"].endswith("scene_tf_deploy.yaml")
    assert record["atlas_runtime_config"].endswith(
        "atlas_runtime_deploy.yaml")


@requires_assets
def test_fresh_install_resolution_matches_source():
    """Default resolution from a fresh install prefix pins the same
    identity bytes as the source tree."""
    shares = _install_shares()
    if shares is None:
        pytest.skip("ELFIN_PD_INSTALL_ROOT not set or incomplete")
    resolver = dp.default_resolver(shares=shares)
    record = dp.resolve_deploy_profile(resolver=resolver)
    source = dp.resolve_deploy_profile()

    assert record["scene_tf_hash"] == source["scene_tf_hash"]
    assert record["urdf_hash"] == source["urdf_hash"]
    for key in ("scene", "urdf", "srdf"):
        installed = record["manifest"][key]
        from_src = source["manifest"][key]
        assert installed["sha256"] == from_src["sha256"], key
    for installed_entry, from_src_entry in zip(record["manifest"]["atlas"],
                                               source["manifest"]["atlas"]):
        assert installed_entry["npz"]["sha256"] == \
            from_src_entry["npz"]["sha256"]
        assert installed_entry["graph"]["sha256"] == \
            from_src_entry["graph"]["sha256"]

    # Explicit selection from the install prefix works too.
    explicit = dp.resolve_deploy_profile(
        scene_arg=os.path.join(
            shares["luggage_description"], "config",
            "scene_tf_deploy.yaml"),
        atlas_config_arg=os.path.join(
            shares["luggage_planning"], "config",
            "atlas_runtime_deploy.yaml"),
        manifest_arg=os.path.join(
            shares["luggage_description"], "config",
            "deploy_asset_manifest.yaml"),
        resolver=resolver)
    assert explicit["scene_tf_hash"] == source["scene_tf_hash"]


@requires_assets
def test_install_sim_and_missing_rejections():
    shares = _install_shares()
    if shares is None:
        pytest.skip("ELFIN_PD_INSTALL_ROOT not set or incomplete")
    from luggage_description.frame_contracts import ContractError

    resolver = dp.default_resolver(shares=shares)
    sim_scene = os.path.join(
        shares["luggage_description"], "config", "scene_tf.yaml.example")
    if os.path.isfile(sim_scene):
        with pytest.raises(ContractError) as caught:
            dp.resolve_deploy_profile(scene_arg=sim_scene, resolver=resolver)
        assert caught.value.code == "SIM_SCENE_ON_HARDWARE"
    else:
        with pytest.raises(ContractError) as caught:
            dp.resolve_deploy_profile(
                scene_arg=os.path.join(
                    shares["luggage_description"], "config",
                    "scene_tf.yaml.example"),
                resolver=resolver)
        assert caught.value.code == "DEPLOY_SCENE_MISSING"


def test_hardware_launch_declares_deploy_defaults():
    """The hardware launch contract: empty arguments name the installed
    deploy assets, and the fail-closed gate runs before any node."""
    import importlib.util

    launch_module_path = os.path.join(
        dp._SRC_ROOTS["luggage_planning"], "launch",
        "hardware_pick.launch.py")
    try:
        spec = importlib.util.spec_from_file_location(
            "hardware_pick_launch_under_test", launch_module_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    except ImportError as error:  # launch / ros packages unavailable
        pytest.skip("launch environment unavailable: %s" % error)

    from launch.actions import DeclareLaunchArgument

    description = module._launch_description()
    defaults = {}
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            defaults[entity.name] = "".join(
                part if isinstance(part, str) else getattr(part, "text", "")
                for part in entity.default_value)
    assert defaults["scene_tf_config"].endswith("scene_tf_deploy.yaml")
    assert defaults["atlas_runtime_config"].endswith(
        "atlas_runtime_deploy.yaml")
    assert defaults["robot_poses_config"].endswith("robot_poses.deploy.yaml")
    assert hasattr(module, "_deploy_profile_gate")
