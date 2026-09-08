# TCIG-1 Container Geometry Kernel

- role: eng
- agent: codex
- model: gpt-5
- cli: codex
- status: done
- parent: TCIG-20260904
- subtask: TCIG-1
- base_revision: bd942eba3120cf521010c5ba628b2489f5e31546
- started_at: 2026-09-04T17:13:15+08:00
- completed_at: 2026-09-04T17:24:10+08:00

## Summary

Implemented TCIG-1 at revision
`7af40220c9e86feb7f56908d9fcde389aa48dd9d`: a ROS-free canonical
`luggage_description.container_geometry` kernel with normalized descriptor and
hash, exact seven-face volume/floor math, point/oriented-box/swept-box
containment, exact AABB/hull clipping, floor support area, payload eroded
center intervals, and scene_tf compatibility wrappers.

Added the architecture contract and machine-facing rule for container geometry.
`scene_tf_config_utils.py` now delegates inner-hull geometry to the canonical
kernel while preserving legacy helper names.

## Requirement

Implement and test TCIG-1 exactly to Gate G1 at approved plan revision
`bd942eba3120cf521010c5ba628b2489f5e31546`; preserve ROS-free imports, exact
seven-face geometry, normalized identity, compatibility wrappers, and
architecture contract.

## Result

- pass: `PYTHONPATH=src/luggage_description python3 -m pytest src/luggage_description/test/test_container_geometry.py -q`
- pass: `PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:src/luggage_description python3 -m pytest src/luggage_description/test/test_scene_tf_config_utils.py -q`
- pass: `PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:src/luggage_description python3 -m pytest src/luggage_description/test -q`
- pass: `python3 -m py_compile src/luggage_description/luggage_description/container_geometry.py src/luggage_description/luggage_description/scene_tf_config_utils.py scripts/agent_poll_self.py scripts/agent_scheduler.py`
- pass: `scripts/check_agent_contract.sh`
- pass: `git diff --check`

## Pointers

- `src/luggage_description/luggage_description/container_geometry.py`
- `src/luggage_description/luggage_description/scene_tf_config_utils.py`
- `src/luggage_description/test/test_container_geometry.py`
- `docs/architecture/container_geometry.md`
- `.cursor/rules/container-geometry.mdc`
