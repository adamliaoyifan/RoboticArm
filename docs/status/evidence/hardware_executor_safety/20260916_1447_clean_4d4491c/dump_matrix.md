# HW-1 offline dump matrix

| Boundary | Always-on trace | Failure trigger | Failure payload | Window / budget | Replay |
|---|---|---|---|---|---|
| Goal admission | Goal acceptance/result assertions and owner state | More than one admission, wrong cancellation, missing terminal result | Failing unittest identity, ROS action state, executor event/log context | One test case; under 1 MiB | Run `test_action_single_goal.py` and `test_goal_ownership.py` |
| Command preflight | Controller vectors, fraction, returned velocity/acceleration, waypoint-call count | Missing limits accepted, cap above 0.8, or waypoint sent after rejection | Fake CPS inputs, computed profile, call count, assertion | One parameterized case; under 1 MiB | Run `test_cps_parse.py` and `test_hardware_safety.py` |
| Completion polling | API, return code/payload, stop count, connection state | Nonzero/malformed poll does not stop and enter `ERROR` | Fake CPS response, decoded code, state transitions | One poll call; under 1 MiB | Run `test_hardware_safety.py` |

No failure bundle was triggered. The exact commit and commands reproduce all
three boundaries without hardware. Future real-cell capture requirements stay
in `docs/plans/elfin_real_trajectory_hardware_acceptance.md`.
