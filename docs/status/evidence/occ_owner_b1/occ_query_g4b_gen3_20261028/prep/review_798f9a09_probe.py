"""Read-only Stage 1 judge probes. Pass the B worktree as argv[1]."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(sys.argv[1]) / 'scripts/g3'))
from occ_g4b_capacity_core import (CurrentCheck, bounded_current_plan,
                                 bounded_current_verdict, classify_commits,
                                 C2Admission, c2_closure)

for mode in ('retry', 'reset', 'infinity'):
    checks = []
    for planned in bounded_current_plan():
        start = planned['scheduled_s']
        check = CurrentCheck(planned['check_id'], start)
        check.start(start)
        if mode == 'retry':
            check.attempt(start + .01, 'named_propagation_wait',
                          reason='PERMISSION_DENIED')
            check.attempt(start + .011, 'ok', identity=None)
        elif mode == 'reset':
            check.attempt(start + 1.9, 'named_propagation_wait',
                          reason='REPLICA_LAGGING')
            check.start(start + 2)
            check.attempt(start + 3, 'ok', identity='A')
        else:
            check.start(float('inf'))
            check.attempt(float('inf'), 'ok', identity='A')
        checks.append(check)
    print(mode, bounded_current_verdict(checks))

print('false_membership', classify_commits(
    [dict(identity='A', commit_t_s=1., members={'SHARED'})],
    {'A': dict(status='coalesced', into='B', t_s=1.1)},
    [dict(target_identity='B', t_s=1.5, covered_members={'SHARED'})]))
admission = C2Admission()
admission.on_arrival(0., dict(frame_id='f', t_s=0., input_hash='wrong'))
admission.on_send_result(1., 'f', 'committed')
admission.records['f']['propagation'] = 'exact_visible'
print('frame_payload_mismatch', c2_closure(
    admission, [dict(frame_id='f', t_s=0., input_hash='correct')], True))
