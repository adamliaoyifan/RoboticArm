# S4' closed-loop evidence (2026-10-03)

Command (both suite passes, irun15 + irun16, back to back):

    source /opt/ros/humble/setup.bash && source install-st09/setup.bash
    export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=$PWD/src:$PYTHONPATH ROS_LOCALHOST_ONLY=1
    bash scripts/st09_i/kill_stack.sh   # before each pytest session
    python3 -m pytest tests/st09_parallel/I -q

Result: 34 passed / 0 skipped / 0 failed, ~166 s per run (three green
suite runs total: irun11, irun13 after the latch-race fixes, then
irun15/irun16 back to back). The two-box full packing loop runs inside
the suite (full_loop.json): per box, 4 pick legs + 7 certified route
legs, revision handoff across boxes, land observation committed after
release; no map movement during planning/execution.

Fixes this round (each with its live symptom, see the commit):
1. WRONG_OVERLAY suite skip: test_route_producer/test_moveit_state_
   validity inserted src/luggage_planning at sys.path[0], flipping the
   in-process luggage_planning resolution off the install overlay; the
   in-process probe's module provenance was then rejected by every
   stack node (all five live tests skipped in directory runs).
2. Scene leaks between selects: a select failing after entry planning
   stranded the frozen payload ATTACHED and the planning-added
   released/placed boxes in the WORLD scene; reset_scene at the service
   boundary (place_service) now removes both on every exit.
3. GetCartesianPath whole-turn flips: re-seeded IK samples can answer
   on a 2*pi-equivalent branch; edges now unwrap the response onto one
   branch and the excursion judge measures angular distance.
4. Sampler-miss rebuild: NO_FEASIBLE after an OMPL staging-transit
   miss is retried (bounded, inside the frozen budget).
5. Latch-race CAS: the latched surface_v2 can lag an integrate's
   republish; the loop test reads the AUTHORITATIVE revision
   (GetCargoMapStats) with catch-up for every CAS consumer.

Regression at this revision: I 34p/0s (x2), A 29, B 140, C 19, C0 69,
D 40, E 11, planning 648+171 — all green.

Known margins recorded in the loop test (not planner knowledge): the
aperture y limits vs the ~0.6 m wrist-rig offset (slot stimulus
y=-0.15/+0.2), the frozen 10 s route budget shared with the bounded
rebuilds.
