"""Q-20261028-1 handover close-out: the ONE real chain on the FIXED
install (review round 4, 2026-10-29): writer process -> FULL -> DELTA
-> independent reader process -> typed summary, with source/identity/
field verification and the v1-rejection -> forced-FULL recovery in
the SAME run. Raw evidence is kept under the output dir.

Environment: run with the worktree's message library first on
LD_LIBRARY_PATH (the V2 srv typesupport postdates the shared
workspace install):

  LD_LIBRARY_PATH=<repo>/install/luggage_msgs/lib:$LD_LIBRARY_PATH \
  pytest src/luggage_perception/test/test_summary_wire_handover_chain.py
"""

import importlib.util
import json
import os
import random
import sys

_HERE = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
_REPO = os.path.normpath(_HERE)
for _rel in ('install/luggage_msgs/local/lib/python3.10/dist-packages',
             'src/luggage_description',
             'src/luggage_perception'):
    _path = os.path.join(_REPO, _rel)
    if _path not in sys.path:
        sys.path.insert(0, _path)

_SPEC = importlib.util.spec_from_file_location(
    'occ_summary_wire_handover_chain',
    os.path.join(_REPO, 'scripts', 'g3',
                 'occ_summary_wire_handover_chain.py'))
CHAIN = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(CHAIN)


def test_handover_single_real_chain_fixed_install(tmp_path):
    """One run covers the whole handover: version locks for BOTH
    subprocesses (install tree == committed source), FULL and DELTA
    both APPLIED by the independent reader, typed identity + ALL
    business fields equal the writer's own record over the DELTA
    head, the named v1 rejection, the forced-FULL anchored recovery,
    the typed comparison again after recovery, and head identity ==
    writer's latest on-disk manifest."""
    install = os.path.join(_REPO, 'install')
    out = str(tmp_path / 'handover_chain')
    verdict = CHAIN.run_chain(
        install, out, source_repo=_REPO,
        domain_id=random.randint(40, 90))
    assert verdict['ok'], verdict
    assert verdict['anchor_full_seqs'], verdict
    assert verdict['head_seq'] >= max(verdict['anchor_full_seqs'])
    evidence = json.load(open(os.path.join(out, 'chain_evidence.json')))
    assert evidence['version_locks']['writer']['all_locked']
    assert evidence['version_locks']['reader']['all_locked']
    phase1 = evidence['phases']['phase1']['reader']
    assert 1 in phase1['applied_full_seqs']
    assert 2 in phase1['applied_seqs']
    assert 'UNKNOWN_SCHEMA' in evidence['phases']['v1_rejection'][
        'last_error']
    for label in ('delta_head', 'recovered_head'):
        assert evidence['typed_checks'][label]['ok'], label
    head = evidence['phases']['head_manifest']
    assert [str(part) for part in head['target_identity'][:3]] == \
        head['writer_identity'][:3]
