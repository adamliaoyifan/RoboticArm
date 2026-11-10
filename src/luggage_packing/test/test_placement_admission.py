"""Offline checks for the reset-contract admission gate (plan
OCTOMAP_RESET_VALIDITY_NEXT_20261103 §3): recovery gating, double-check
commit semantics, late-result rejection and execution_active reporting.

These run without ROS: the gate is a pure function of observations, which is
exactly why the ROS planner node and the reset campaign can share it.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def load_module():
    # Package import (not bare file loading): the dataclass decorator needs
    # a resolvable module entry in sys.modules.
    from luggage_packing.placement_admission import (  # noqa: F401
        AdmissionGate, MapObservation, RobotStateSample, ValidityObservation)
    import luggage_packing.placement_admission as module
    return module


def test_boot_closed_until_valid_map_and_robot():
    m = load_module()
    gate = m.AdmissionGate()
    # No validity observation: closed (the node treats no-stream as
    # ungated; the gate object itself starts INVALID/0 and stays closed).
    decision = gate.gate_state()
    assert not decision.admitted
    assert decision.reason == m.AdmissionGate.REASON_CLOSED_INVALID
    # VALID alone is not enough: no map.
    gate.observe_validity(m.ValidityObservation("VALID", 1, 2,
                                                transition_mono_ns=1000))
    decision = gate.gate_state()
    assert not decision.admitted
    assert decision.reason == m.AdmissionGate.REASON_NO_MAP
    # Map without robot state: still closed.
    admitted = gate.observe_map(m.MapObservation(epoch=1, generation=2))
    assert admitted.admitted
    decision = gate.gate_state()
    assert not decision.admitted
    assert decision.reason == m.AdmissionGate.REASON_ROBOT_MISSING
    # Pre-transition robot state keeps the gate closed.
    gate.observe_robot_state(m.RobotStateSample(stamp_mono_ns=100))
    decision = gate.gate_state()
    assert not decision.admitted
    assert decision.reason == m.AdmissionGate.REASON_ROBOT_PRE_TRANSITION
    # Stale sample (age bound) keeps it closed.
    gate.observe_robot_state(m.RobotStateSample(stamp_mono_ns=5000))
    decision = gate.gate_state(now_mono_ns=5000 + 10_000_000_000,
                               robot_max_age_ns=1_000_000_000)
    assert not decision.admitted
    assert decision.reason == m.AdmissionGate.REASON_ROBOT_STALE
    # Fresh post-transition sample opens it.
    decision = gate.gate_state(now_mono_ns=5100, robot_max_age_ns=1_000_000)
    assert decision.admitted


def test_invalidation_drops_map_and_rejects_late_results():
    m = load_module()
    gate = m.AdmissionGate()
    gate.observe_validity(m.ValidityObservation("VALID", 5, 10))
    assert gate.observe_map(m.MapObservation(epoch=5, generation=10)).admitted
    gate.observe_robot_state(m.RobotStateSample(stamp_mono_ns=10_000))
    assert gate.gate_state(now_mono_ns=10_100).admitted
    # Reset: map dies, gate closes.
    gate.observe_validity(m.ValidityObservation("INVALID", 6, 11,
                                                transition_mono_ns=10_200))
    assert not gate.gate_state(now_mono_ns=10_300).admitted
    assert gate.admitted_map is None
    # The late old-epoch result is rejected and counted.
    decision = gate.accept_result(5, 10)
    assert not decision.admitted
    assert gate.rejected_late_results == 1
    # Old map arriving for the new epoch: rejected by epoch binding.
    assert not gate.observe_map(m.MapObservation(epoch=5, generation=10)).admitted
    # A delayed older publication cannot move the gate backwards.
    gate.observe_validity(m.ValidityObservation("VALID", 5, 9))
    assert gate.validity.epoch == "6"


def test_commit_double_check_and_execution_active():
    m = load_module()
    gate = m.AdmissionGate()
    gate.observe_validity(m.ValidityObservation("VALID", 2, 4))
    gate.observe_map(m.MapObservation(epoch=2, generation=4))
    gate.observe_robot_state(m.RobotStateSample(stamp_mono_ns=900))
    admitted = gate.admit_planning(now_mono_ns=1000,
                                   robot_max_age_ns=1_000_000)
    assert admitted.admitted
    # The admission token is the immutable capture of the start generation.
    token = admitted.admission_token
    assert token is not None and token.generation == 4 and token.epoch == "2"
    # A reset won while planning: the commit against the stale generation is
    # discarded even though the gate object still holds fresh-enough robot
    # state for the NEW epoch (it will be dropped by observe_validity).
    gate.observe_validity(m.ValidityObservation("INVALID", 3, 5,
                                                transition_mono_ns=1100))
    decision = gate.commit_plan(4, now_mono_ns=1200, robot_max_age_ns=1_000_000)
    assert not decision.admitted
    assert decision.reason == m.AdmissionGate.REASON_CLOSED_INVALID
    # Recovery: new VALID epoch, fresh map, fresh robot. The pre-reset
    # admission is VOID: the old candidate cannot commit with its own
    # generation NOR with the recovered board's current generation (review
    # 2026-11-03 finding 1 — the gen1 wiring re-read the board at commit).
    gate.observe_validity(m.ValidityObservation("VALID", 3, 6,
                                                transition_mono_ns=1250))
    gate.observe_map(m.MapObservation(epoch=3, generation=6))
    gate.observe_robot_state(m.RobotStateSample(stamp_mono_ns=1300))
    decision = gate.commit_plan(gate.validity.generation, now_mono_ns=1400,
                                robot_max_age_ns=1_000_000)
    assert not decision.admitted
    assert decision.reason == m.AdmissionGate.REASON_COMMIT_GENERATION
    decision = gate.commit_plan(4, now_mono_ns=1400,
                                robot_max_age_ns=1_000_000)
    assert not decision.admitted
    assert decision.reason == m.AdmissionGate.REASON_COMMIT_GENERATION
    # A fresh admission in the recovered generation is the only commit path.
    re_admitted = gate.admit_planning(now_mono_ns=1400,
                                      robot_max_age_ns=1_000_000)
    assert re_admitted.admitted
    assert re_admitted.admission_token.generation == 6
    decision = gate.commit_plan(
        re_admitted.admission_token.generation,
        admission_token=re_admitted.admission_token,
        now_mono_ns=1400, robot_max_age_ns=1_000_000)
    assert decision.admitted
    # Production records the committed generation; retiring it clears the
    # flag. Until then an invalidation must report execution_active.
    executing = gate.record_execution()
    # An outstanding old-generation execution is reported, never hidden.
    gate.observe_validity(m.ValidityObservation("INVALID", 4, 7,
                                                transition_mono_ns=1500))
    state = gate.gate_state(now_mono_ns=1600)
    assert not state.admitted
    assert state.execution_active
    gate.retire_execution(executing)
    assert not gate.gate_state(now_mono_ns=1700).execution_active


def test_reset_during_building_and_duplicate_generations():
    m = load_module()
    gate = m.AdmissionGate()
    gate.observe_validity(m.ValidityObservation("BUILDING", 2, 3))
    # A map computed for the building epoch cannot be admitted.
    decision = gate.observe_map(m.MapObservation(epoch=2, generation=3))
    assert not decision.admitted
    assert decision.reason == m.AdmissionGate.REASON_CLOSED_BUILDING
    # Recovery publishes VALID(2) gen 4; a gen-3 map is a generation
    # mismatch and rejected.
    gate.observe_validity(m.ValidityObservation("VALID", 2, 4,
                                                transition_mono_ns=10))
    decision = gate.observe_map(m.MapObservation(epoch=2, generation=3))
    assert not decision.admitted
    assert decision.reason == m.AdmissionGate.REASON_MAP_GENERATION
    # [G1] Generation 0 is NOT an undeclared pass anymore: both
    # production backends publish validity_generation, so a map without
    # one is missing a required identity field and fails closed (the
    # historical "when declared" bypass is the A3 forbidden default).
    decision = gate.observe_map(m.MapObservation(epoch=2, generation=0))
    assert not decision.admitted
    assert decision.reason == \
        m.AdmissionGate.REASON_MAP_GENERATION_MISSING


def test_admission_token_lifecycle():
    m = load_module()
    gate = m.AdmissionGate()
    gate.observe_validity(m.ValidityObservation("VALID", 1, 2))
    gate.observe_map(m.MapObservation(epoch=1, generation=2))
    gate.observe_robot_state(m.RobotStateSample(stamp_mono_ns=100))
    # Tokens are single-use: one admission buys exactly one commit.
    first = gate.admit_planning(now_mono_ns=200, robot_max_age_ns=10**9)
    second = gate.admit_planning(now_mono_ns=200, robot_max_age_ns=10**9)
    assert first.admission_token != second.admission_token
    assert len(gate.outstanding_admissions) == 2
    decision = gate.commit_plan(
        first.admission_token.generation,
        admission_token=first.admission_token,
        now_mono_ns=300, robot_max_age_ns=10**9)
    assert decision.admitted
    replay = gate.commit_plan(
        first.admission_token.generation,
        admission_token=first.admission_token,
        now_mono_ns=300, robot_max_age_ns=10**9)
    assert not replay.admitted
    assert replay.reason == m.AdmissionGate.REASON_COMMIT_GENERATION
    # A forged/unknown token never commits, even naming the right generation.
    forged = m.AdmissionToken(9999, 1, 2)
    decision = gate.commit_plan(2, admission_token=forged,
                                now_mono_ns=300, robot_max_age_ns=10**9)
    assert not decision.admitted
    # A same-generation re-publication (idempotent latch) keeps the
    # surviving admission valid; any generation advance voids it.
    gate.observe_validity(m.ValidityObservation("VALID", 1, 2))
    assert [t.seq for t in gate.outstanding_admissions] == [second.admission_token.seq]
    gate.observe_validity(m.ValidityObservation("VALID", 2, 3,
                                                transition_mono_ns=400))
    assert gate.outstanding_admissions == []
    # And a commit without any admission is refused, open gate or not.
    gate.observe_map(m.MapObservation(epoch=2, generation=3))
    gate.observe_robot_state(m.RobotStateSample(stamp_mono_ns=500))
    decision = gate.commit_plan(3, now_mono_ns=600, robot_max_age_ns=10**9)
    assert not decision.admitted
    assert decision.reason == m.AdmissionGate.REASON_COMMIT_GENERATION


if __name__ == "__main__":
    test_boot_closed_until_valid_map_and_robot()
    test_invalidation_drops_map_and_rejects_late_results()
    test_commit_double_check_and_execution_active()
    test_reset_during_building_and_duplicate_generations()
    test_admission_token_lifecycle()
    print("placement admission checks passed")


# ----------------------------------------------------------------------
# G1 common-consumer extensions (OCC_GRID_CONSUMER_COMPAT_G1_20261105):
# lossless canonical epochs, backend binding, per-key generation
# high-water with controlled producer confirmation.

G_UUID = "0123456789abcdef0123456789abcdef"      # all-digit prefix + hex
G_UUID_2 = "fedcba9876543210fedcba9876543210"


def test_normalize_epoch_lossless_per_backend():
    m = load_module()
    # Observer uint64: canonical decimal, reversible, no leading zeros.
    assert m.normalize_epoch(7, m.BACKEND_OBSERVER_O) == "7"
    assert m.normalize_epoch("007", m.BACKEND_OBSERVER_O) == "7"
    # Grid UUID: verbatim, lowercase, dashed normalized at fixed positions.
    assert m.normalize_epoch(G_UUID.upper(), m.BACKEND_GRID_G) == G_UUID
    dashed = "%s-%s-%s-%s-%s" % (G_UUID[0:8], G_UUID[8:12], G_UUID[12:16],
                                 G_UUID[16:20], G_UUID[20:32])
    assert m.normalize_epoch(dashed, m.BACKEND_GRID_G) == G_UUID
    # The pathological all-digit UUID with a leading zero survives verbatim
    # (numeric reinterpretation would corrupt the identity).
    all_digit = "01234567890123456789012345678901"
    assert m.normalize_epoch(all_digit, m.BACKEND_GRID_G) == all_digit
    # A grid epoch that is not a full UUID is rejected, not defaulted.
    for bad in ("abc", "0123456789abcdef", "0x0123456789abcdef0123456789abcdef"):
        try:
            m.normalize_epoch(bad, m.BACKEND_GRID_G)
            assert False, "expected EpochFormatError for %r" % bad
        except m.EpochFormatError:
            pass
    # An observer epoch that is not a uint64 is rejected.
    try:
        m.normalize_epoch(G_UUID, m.BACKEND_OBSERVER_O)
        assert False, "expected EpochFormatError"
    except m.EpochFormatError:
        pass
    # bools never pass as epochs.
    try:
        m.normalize_epoch(True, m.BACKEND_OBSERVER_O)
        assert False, "expected EpochFormatError"
    except m.EpochFormatError:
        pass


def test_backend_binding_rejects_cross_backend_splice():
    m = load_module()
    gate = m.AdmissionGate()
    gate.observe_validity(m.ValidityObservation(
        "VALID", 1, 3, backend=m.BACKEND_OBSERVER_O))
    # A G UUID spliced onto the O stream: identity collision impossible,
    # and the backend key must reject it even if epochs were made equal.
    decision = gate.observe_map(m.MapObservation(
        epoch=1, generation=3, backend=m.BACKEND_GRID_G))
    assert not decision.admitted
    assert decision.reason == m.AdmissionGate.REASON_MAP_BACKEND
    # Missing epoch fails closed, never defaults to 0.
    decision = gate.observe_map(m.MapObservation(
        epoch="", generation=3, backend=m.BACKEND_OBSERVER_O))
    assert not decision.admitted
    assert decision.reason == m.AdmissionGate.REASON_MAP_EPOCH_MISSING


def test_unseen_epoch_parks_until_confirmed():
    m = load_module()
    gate = m.AdmissionGate()
    gate.observe_validity(m.ValidityObservation(
        "VALID", 1, 5, backend=m.BACKEND_OBSERVER_O))
    gate.observe_map(m.MapObservation(
        epoch=1, generation=5, backend=m.BACKEND_OBSERVER_O))
    gate.observe_robot_state(m.RobotStateSample(100))
    assert gate.admit_planning(now_mono_ns=200).admitted
    # A restarted producer renumbers from low generations: the stream alone
    # cannot distinguish this from a delayed old publication, so it parks
    # (never switches)...
    gate.observe_validity(m.ValidityObservation(
        "VALID", 1, 1, backend=m.BACKEND_OBSERVER_O))
    assert gate.validity.generation == 5
    assert gate.pending_confirmation
    # ...and the gate stays open on the old view until the producer's REAL
    # status service confirms the restart (controlled confirmation).
    decision = gate.admit_planning(now_mono_ns=200)
    assert decision.admitted  # pre-restart view still coherent
    confirm_status = m.ValidityObservation(
        "VALID", 1, 1, backend=m.BACKEND_OBSERVER_O,
        transition_mono_ns=300)
    # Two consecutive agreeing statuses (ghost-endpoint guard).
    gate.confirm_producer(confirm_status)
    gate.confirm_producer(confirm_status)
    assert gate.validity.generation == 1
    # The confirmed restart invalidated the outstanding admission and the
    # map/robot premises: closed until fresh inputs arrive.
    gate.observe_map(m.MapObservation(
        epoch=1, generation=1, backend=m.BACKEND_OBSERVER_O))
    decision = gate.admit_planning(now_mono_ns=200)
    assert not decision.admitted  # robot sample predates the new epoch


def test_grid_producer_restart_recovers_after_confirmation():
    m = load_module()
    gate = m.AdmissionGate()
    gate.observe_validity(m.ValidityObservation(
        "VALID", G_UUID, 4, backend=m.BACKEND_GRID_G,
        transition_mono_ns=10))
    gate.observe_map(m.MapObservation(
        epoch=G_UUID, generation=4, backend=m.BACKEND_GRID_G))
    gate.observe_robot_state(m.RobotStateSample(100))
    # New process, new UUID, generation renumbered from 1: parked.
    gate.observe_validity(m.ValidityObservation(
        "VALID", G_UUID_2, 1, backend=m.BACKEND_GRID_G,
        transition_mono_ns=500))
    assert gate.validity.epoch == G_UUID
    assert (m.BACKEND_GRID_G, G_UUID_2) in gate.pending_confirmation
    confirm_status = m.ValidityObservation(
        "VALID", G_UUID_2, 1, backend=m.BACKEND_GRID_G,
        transition_mono_ns=500)
    gate.confirm_producer(confirm_status)
    gate.confirm_producer(confirm_status)
    assert gate.validity.epoch == G_UUID_2
    gate.observe_map(m.MapObservation(
        epoch=G_UUID_2, generation=1, backend=m.BACKEND_GRID_G))
    gate.observe_robot_state(m.RobotStateSample(600))
    assert gate.admit_planning(now_mono_ns=700).admitted


def test_retired_epoch_never_switches_back():
    m = load_module()
    gate = m.AdmissionGate()
    gate.observe_validity(m.ValidityObservation(
        "VALID", 1, 2, backend=m.BACKEND_OBSERVER_O))
    # Normal same-producer reset advance: epoch +1 AND generation up.
    gate.observe_validity(m.ValidityObservation(
        "BUILDING", 2, 4, backend=m.BACKEND_OBSERVER_O))
    assert gate.validity.epoch == "2"
    # A delayed publication of the retired epoch (any generation) parks.
    gate.observe_validity(m.ValidityObservation(
        "VALID", 1, 9, backend=m.BACKEND_OBSERVER_O))
    assert gate.validity.epoch == "2"
    assert (m.BACKEND_OBSERVER_O, "1") in gate.pending_confirmation
    assert gate.rejected_backward_publications == 1


def test_same_generation_different_content_is_a_conflict():
    m = load_module()
    gate = m.AdmissionGate()
    gate.observe_validity(m.ValidityObservation(
        "VALID", 1, 3, backend=m.BACKEND_OBSERVER_O))
    same_gen_conflict = m.ValidityObservation(
        "INVALID", 1, 3, backend=m.BACKEND_OBSERVER_O)
    gate.observe_validity(same_gen_conflict)
    # The conflicting payload at the same generation never overwrote.
    assert gate.validity.state == "VALID"
    assert gate.rejected_content_conflicts == 1
    # An identical duplicate is idempotent (no conflict count).
    gate.observe_validity(m.ValidityObservation(
        "VALID", 1, 3, backend=m.BACKEND_OBSERVER_O))
    assert gate.rejected_content_conflicts == 1


def test_grid_all_digit_epoch_admission_end_to_end():
    m = load_module()
    gate = m.AdmissionGate()
    epoch = "01234567890123456789012345678901"   # all-digit, leading zero
    gate.observe_validity(m.ValidityObservation(
        "VALID", epoch, 2, backend=m.BACKEND_GRID_G,
        transition_mono_ns=1))
    decision = gate.observe_map(m.MapObservation(
        epoch=epoch, generation=2, backend=m.BACKEND_GRID_G))
    assert decision.admitted
    gate.observe_robot_state(m.RobotStateSample(10))
    # The identity survives verbatim through the token.
    token = gate.admit_planning(
        now_mono_ns=50, robot_max_age_ns=10**9).admission_token
    assert token.epoch == epoch
    assert token.backend == m.BACKEND_GRID_G
