# P3 atlas wiring evidence — candidate revision d585a63 (2026-10-03)

Command (both counted runs, consecutive):

    source /opt/ros/humble/setup.bash && source install-st09/setup.bash
    export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=$PWD/src:$PYTHONPATH ROS_LOCALHOST_ONLY=1
    bash scripts/st09_i/kill_stack.sh
    python3 -m pytest tests/st09_parallel/I -q

Result: 34 passed / 0 skipped, 171.7 s and 180.0 s — with the slot
stimulus DELETED. The chain is placement -> atlas-informed select ->
certified route -> execution, no hand-injected candidates.

Chosen slots (from the committed full_loop.json, this revision):
box1 (-0.17, -0.26) select@5 -> land@6; box2 (-0.42, +0.24) select@7
-> land@8. Box 2's placement autonomously avoided box 1's occupied
cells through the observed height map.

Atlas resample provenance: four variants rebuilt against the live
planning stack on domain 69 (empty + 0.55x0.40x0.25 +
0.70x0.45x0.28 + 0.80x0.50x0.32), urdf_hash 22ed00b5..., scene hash
unchanged b40c85ae... . Sampling-side payload attach is +h/2 link-z
(envelope semantics, touch_links suction_panel only).

Producer ordering (frozen-budget note): seeded shallow-first; a deep
attempt alone costs ~the whole 10 s route budget, so deep-first
enumeration cannot reach a certifiable slot under the current
contract. Packing's deep-first preference governs the candidate set;
raising the budget restores deep-first ordering.

Regression at this revision: I 34p/0s (x2), PD 42p/2s, B 140, A 29,
C 19, C0 69, D 40, E 11, planning 648+171.
