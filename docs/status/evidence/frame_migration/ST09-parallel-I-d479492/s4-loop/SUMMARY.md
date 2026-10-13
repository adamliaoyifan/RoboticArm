# S4' closed-loop evidence — candidate revision d479492 (2026-10-03)

Attribution: all fixes behind this evidence (overlay path discipline,
scene-leak reset, cartesian unwrap, bounded sampler rebuild, CAS
authority reads) are in commit d479492. 751129f contains only the
first-loop milestone WITHOUT these; it is not the evidence revision.

Command (identical for every counted run):

    source /opt/ros/humble/setup.bash && source install-st09/setup.bash
    export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=$PWD/src:$PYTHONPATH ROS_LOCALHOST_ONLY=1
    bash scripts/st09_i/kill_stack.sh   # before each pytest session
    python3 -m pytest tests/st09_parallel/I -q

## Runs at the final revision (d479492 code state)

| run    | result                  | wall   |
|--------|-------------------------|--------|
| irun15 | 34 passed / 0 skipped   | 166.8s |
| irun16 | 34 passed / 0 skipped   | 167.6s |
| irun17 | 34 passed / 0 skipped   | 166.6s |

irun15+irun16 were consecutive; irun17 was run after the attribution
review with no code change between. Three green suite runs at the
SAME revision => n=3 repetition evidence. (irun11 and irun13 also
passed 34/0 but at intermediate code states — development history,
not repetition evidence; the interleaved irun12/irun14 failures at
those states are what motivated the final CAS/read fixes.)

## Revision series (from the committed full_loop.json, identical in
irun15/16/17)

    box1 pre-observation bumps to 5 -> select@5 -> land commit@6
    box2 pre-observation bumps to 7 -> select@7 -> land commit@8

Deterministic across all three runs. The map does not move during
planning or route execution (asserted per box against the
authoritative mapper revision).

Per box: 4 pick legs (pre_grasp/approach/attach/pick_retreat) + 7
certified route legs (staging transit, portal traverse, traverse,
insert, descend, retreat, return), typed vacuum attach on the
TF-measured grasp, typed vacuum off with detach confirmed,
post-release observation at floor_z + the box's actual depth.

Regression at this revision: I 34p/0s (x3), A 29, B 140, C 19,
C0 69, D 40, E 11, planning 648+171.

## Scope boundaries (not claimed here)

- The two boxes use DIFFERENT hand-chosen near-slot stimuli
  (y=-0.15 / +0.2): this proves two full route cycles and the map
  revision handoff, NOT that the solver autonomously avoids box 1's
  occupancy (the mock map's own candidates are all PRIOR_UNREACHABLE;
  the perception front end is synthetic injection).
- Physical release, landed pose quality, and real sensor occupancy
  are Gazebo-M3 scope.
- OMPL sampling repeatability and the wrist-rig geometry margins
  (aperture y limits vs the ~0.6 m rig offset, derived from contact
  positions, not verified against the URDF) are recorded P3/M3
  risks, not S4 functionality.
