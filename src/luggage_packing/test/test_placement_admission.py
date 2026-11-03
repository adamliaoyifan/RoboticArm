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
    assert gate.validity.epoch == 6


def test_commit_double_check_and_execution_active():
    m = load_module()
    gate = m.AdmissionGate()
    gate.observe_validity(m.ValidityObservation("VALID", 2, 4))
    gate.observe_map(m.MapObservation(epoch=2, generation=4))
    gate.observe_robot_state(m.RobotStateSample(stamp_mono_ns=900))
    assert gate.admit_planning(now_mono_ns=1000,
                               robot_max_age_ns=1_000_000).admitted
    # A reset won while planning: the commit against the stale generation is
    # discarded even though the gate object still holds fresh-enough robot
    # state for the NEW epoch (it will be dropped by observe_validity).
    gate.observe_validity(m.ValidityObservation("INVALID", 3, 5,
                                                transition_mono_ns=1100))
    decision = gate.commit_plan(4, now_mono_ns=1200, robot_max_age_ns=1_000_000)
    assert not decision.admitted
    assert decision.reason == m.AdmissionGate.REASON_CLOSED_INVALID
    # Recovery: new VALID epoch, fresh map, fresh robot -> one admission.
    gate.observe_validity(m.ValidityObservation("VALID", 3, 6,
                                                transition_mono_ns=1250))
    gate.observe_map(m.MapObservation(epoch=3, generation=6))
    gate.observe_robot_state(m.RobotStateSample(stamp_mono_ns=1300))
    decision = gate.commit_plan(6, now_mono_ns=1400, robot_max_age_ns=1_000_000)
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
    # Generation 0 (undeclared, pre-contract publishers) binds by epoch.
    assert gate.observe_map(m.MapObservation(epoch=2, generation=0)).admitted


if __name__ == "__main__":
    test_boot_closed_until_valid_map_and_robot()
    test_invalidation_drops_map_and_rejects_late_results()
    test_commit_double_check_and_execution_active()
    test_reset_during_building_and_duplicate_generations()
    print("placement admission checks passed")
