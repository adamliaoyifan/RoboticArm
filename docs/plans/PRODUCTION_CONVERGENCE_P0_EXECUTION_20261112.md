# 2026-11-12 -- Production convergence P0: build and installed-runtime boundary

- status: approved
- dispatch_ready: yes
- owner: codex/gpt-6
- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- subtask: ST-4-PRODUCTION-P0
- generation: 1
- base_revision: 9652fb5a7a312cec8ae65d97d9244ca04e7daa5d
- authorization: User instructed "按计划执行" after the G3 discussion.

## Scope and dependency boundary

Implement P0 from PRODUCTION_PLACEMENT_CONVERGENCE_G3_20261112.md. Introduce one
named build/install/run boundary (`build-production`, `install-production`),
independent of mixed legacy prefixes and temporary test overlays. Build ROS Humble
workspace closure needed by planning, packing, perception, O and simulation.
Do not alter old prefixes, maps, motion defaults, release distances or models.
This bounded prerequisite does not close the existing whole-chain G2 integration.

| ID | Owner | Dependencies | Base | Scope | Acceptance | Required tests | Commit evidence |
|---|---|---|---|---|---|---|---|
| ST-4-PRODUCTION-P0 | codex/gpt-6 | none | 9652fb5a | Installed build/runtime identity | Matrix below | Exact procedures below | Pending result commit/evidence |

## Acceptance and exact procedures

Environment: /opt/ros/humble under system /usr/bin/python3; no inherited workspace
PYTHONPATH, AMENT/COLCON/CMAKE prefixes or library overlay. Sequential colcon
executor; CMAKE_BUILD_PARALLEL_LEVEL=2; no robot/controller/Gazebo start.

1. `scripts/production/build.sh` reconfigures and installs selected local dependency
   closure from `src` and `st09_contracts`, BUILD_TESTING=OFF and symlink install OFF.
   Zero exit required; log retained. Maximum rehearsal wall time30min. Build failure
   is preserved before adaptive repair, never counted as a scored product cycle.
2. `scripts/production/run.sh python3 scripts/production/check_install.py --output
   docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-PRODUCTION-P0/run_01/install.json`
   must import current prepare/runtime modules, generated operation action/service
   types, description, packing, perception, O interface and st09 contracts solely
   from install-production; package lookups and required executables/config/launch
   files must resolve there. Hash installed Python entry/module files against
   current source counterparts. Every required check must pass.
3. The same checker on an empty prefix and a directory containing a stale copy
   must fail with explicit reasons, not rely only on spelling assertions.
   `python3 -m pytest -q src/luggage_planning/test/test_production_install.py`
   under system Python is the bounded behavioral fixture matrix. Its subprocess
   import tests must use isolated temporary Python packages and exercise origin
   and content mismatch, missing modules/executables and a valid matching prefix.
4. Run installed `hardware_pick.launch.py --show-args` and
   `joint_operation_prepare.launch.py --show-args` through the production wrapper;
   zero exit and parsed launch arguments required. This starts no nodes and does
   not prove model/scene or motion readiness.

Each deterministic positive/negative case runs once; AND over all cases. This is
contract coverage, not a per-operation reliability gate or performance campaign.
Live model, map, whole-chain execution, and robot success are not_evaluated.
Dirty shared workspace is permitted only with saved source/input hashes before
and after build; an unexpected relevant change refuses parity, not a hidden pass.

## Evidence and failure capture

Evidence root: docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-PRODUCTION-P0/.
Retain each attempt separately; do not overwrite a failed build log.

| Boundary | T1 | Failure trigger | T2 payload | Window | Budget | Inspection |
|---|---|---|---|---|---|---|
| Build/install | command,revision,dirty,package statuses,times | nonzero/timeout | full bounded build log,source hashes,CMake flags | whole build | 32MiB/attempt | tail build.log + manifest.json |
| Runtime import | names,origins,hashes,prefix | mismatch/missing/import exception | JSON check result and exception | single call | 2MiB | python3 -m json.tool install.json |
| Launch parse | command,exit,time | nonzero | stdout/stderr | single parse | 2MiB | launch-args logs |

Capture-health gate: manifest, logs and complete check results exist and hashes
are readable before cleanup/result; total<=128MiB; no deletion of failure evidence.
Overhead is hashing/import/parse only; no scored timing claims. No stack started,
so teardown is not_needed. Next P1/P2 executable scopes will be frozen separately.

## Pointers

- docs/plans/PRODUCTION_PLACEMENT_CONVERGENCE_G3_20261112.md
- docs/architecture/cargo_grid.md
- docs/architecture/production_orchestration.md
