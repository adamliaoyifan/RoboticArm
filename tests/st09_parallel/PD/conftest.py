"""Shared pytest fixtures for the ST09-PD deploy-asset battery.

Puts the repo root (``st09_contracts``), the planning src tree and the
description src tree on ``sys.path`` idempotently, mirroring the ST09-B
conftest. Asset-gated tests skip loudly (with the exact campaign
command) until ``scripts/build_deploy_assets_campaign.sh`` has produced
the deploy artifacts; they never silently pass on missing evidence.
"""

from __future__ import division

import os
import sys
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[3]
for _path in (str(_REPO_ROOT),
              str(_REPO_ROOT / "src" / "luggage_planning"),
              str(_REPO_ROOT / "src" / "luggage_description")):
    if _path not in sys.path:
        sys.path.insert(0, _path)

DATA_DIR = _REPO_ROOT / "src" / "luggage_planning" / "data" / \
    "reachability_atlas_deploy"
MANIFEST_PATH = (_REPO_ROOT / "src" / "luggage_description" / "config" /
                 "deploy_asset_manifest.yaml")

#: The frozen payload envelopes (length, width, height metres).
PAYLOADS = ((0.55, 0.40, 0.25), (0.70, 0.45, 0.28), (0.80, 0.50, 0.32))

PROFILES = (("s20_container_collision_aware", None, "s20_deploy_empty"),) + tuple(
    ("s20_container_collision_aware_payload_%.2fx%.2fx%.2f" % size, size,
     "s20_deploy_payload_%.2fx%.2fx%.2f" % size)
    for size in PAYLOADS)


def assets_present():
    if not MANIFEST_PATH.is_file():
        return False
    if not (DATA_DIR / "robot_description.xml").is_file():
        return False
    return all(
        (DATA_DIR / ("%s.npz" % basename)).is_file()
        and (DATA_DIR / "graphs" / ("%s.json" % graph_id)).is_file()
        for basename, _payload, graph_id in PROFILES)


requires_assets = pytest.mark.skipif(
    not assets_present(),
    reason="deploy assets not generated yet; run "
           "bash scripts/build_deploy_assets_campaign.sh <ws> <evidence-dir>")


def minimal_arm_urdf():
    """Expanded six-joint arm fixture: base->j1..j6->suction frame.

    j1 carries origin (0.1, 0.2, 0.3) rpy (0, pi/2, 0); j2..j6 have zero
    origins; the fixed tool joint carries (0, 0, 0.75) rpy (0, 0, pi/2).
    """
    links = ["elfin_base_link"] + ["elfin_link%d" % i for i in range(1, 7)]
    links.append("suction_contact_frame")
    parts = ['<robot name="S20">']
    parts.extend('<link name="%s"/>' % name for name in links)
    parts.append(
        '<joint name="world_base" type="fixed">'
        '<parent link="world"/><child link="elfin_base_link"/>'
        '<origin xyz="0 0 0.86"/></joint>')
    parts.append(
        '<joint name="elfin_joint1" type="revolute">'
        '<parent link="elfin_base_link"/><child link="elfin_link1"/>'
        '<origin xyz="0.1 0.2 0.3" rpy="0 1.5707963 0"/>'
        '<axis xyz="0 0 1"/><limit lower="-6.28" upper="6.28"/></joint>')
    for index in range(2, 7):
        parts.append(
            '<joint name="elfin_joint%d" type="revolute">'
            '<parent link="elfin_link%d"/><child link="elfin_link%d"/>'
            '<origin xyz="0 0 0"/>'
            '<axis xyz="0 0 1"/><limit lower="-6.28" upper="6.28"/>'
            '</joint>' % (index, index - 1, index))
    parts.append(
        '<joint name="fixed_tool" type="fixed">'
        '<parent link="elfin_link6"/><child link="suction_contact_frame"/>'
        '<origin xyz="0 0 0.75" rpy="0 0 1.5707963"/></joint>')
    parts.append('</robot>\n')
    return "".join(parts)
