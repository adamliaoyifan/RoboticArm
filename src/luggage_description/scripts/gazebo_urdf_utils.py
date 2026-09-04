"""URDF helpers for Gazebo spawn vs TF/MoveIt robot_description."""

from __future__ import annotations

import xml.etree.ElementTree as ET

GAZEBO_MODEL_URDF_PARAM = "gazebo_model_urdf"


def _strip_link_collisions(root, link_names):
    link_names = set(link_names)
    stripped = 0
    for link in root.findall("link"):
        if link.get("name") not in link_names:
            continue
        for collision in list(link.findall("collision")):
            link.remove(collision)
            stripped += 1
    return stripped


def gazebo_urdf_from_robot_description(urdf_xml):
    """Return the URDF Gazebo should spawn.

    ``robot_description`` is the source of truth for TF and MoveIt, so it keeps
    detailed end-effector collision geometry. Gazebo, however, uses a
    kinematically position-driven arm in this project. Letting the detailed
    suction-panel CAD meshes participate in ODE contact makes the free suitcase
    or pedestal absorb hard impulses and can explode the simulation. Strip only
    those physical collisions from the Gazebo model while leaving visuals,
    inertials, TF, and MoveIt collision geometry intact.

    The robot's root link is named ``world``. Gazebo anchors any link named
    ``world`` to the global world origin and IGNORES the spawn pose passed to
    spawn_urdf_model, so the base pose must be carried by the patched
    ``world_base`` joint inside the URDF (exactly as robot_state_publisher /
    MoveIt see it). The model is therefore spawned at the origin and this URDF
    keeps the full ``world_base`` transform, so the Gazebo base matches the TF
    base instead of collapsing to z=0 on the floor.
    """
    try:
        root = ET.fromstring(urdf_xml)
    except ET.ParseError:
        return urdf_xml
    stripped = _strip_link_collisions(root, ["suction_panel"])
    if not stripped:
        return urdf_xml
    return ET.tostring(root, encoding="unicode")
