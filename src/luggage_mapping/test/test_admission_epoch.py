"""Backend identities must survive normalization without changing producer."""
import pytest

from luggage_mapping import admission as m


O = m.BACKEND_OBSERVER_O
G = m.BACKEND_GRID_G
MAX_UINT64 = (1 << 64) - 1
DIGIT_UUID = "00000000000000000000000000000017"
HEX_UUID = "0123456789abcdef0123456789abcdef"


@pytest.mark.parametrize("value,expected", [
    (0, "0"), (MAX_UINT64, str(MAX_UINT64)), ("000", "0"),
    ("007", "7"), (str(MAX_UINT64), str(MAX_UINT64)),
    ("000" + str(MAX_UINT64), str(MAX_UINT64)),
    ("0" * 5000 + "17", "17"), (" 17 ", "17"),
])
def test_observer_uint64_boundary_and_decimal_identity(value, expected):
    assert m.normalize_epoch(value, O) == expected


@pytest.mark.parametrize("value,expected", [
    (DIGIT_UUID, DIGIT_UUID), (HEX_UUID.upper(), HEX_UUID),
    ("01234567-89AB-CDEF-0123-456789ABCDEF", HEX_UUID),
    ("  " + DIGIT_UUID + "  ", DIGIT_UUID),
])
def test_grid_uuid_preserves_full_identity(value, expected):
    assert m.normalize_epoch(value, G) == expected


@pytest.mark.parametrize("backend,bad", [
    (G, 0), (G, 17), (G, MAX_UINT64), (G, 1 << 128),
    (G, "17"), (G, "0" * 31), (G, "0" * 33),
    (G, "0123456789-abcd-ef01-2345-6789abcdef"),
    (O, -1), (O, MAX_UINT64 + 1), (O, 1 << 128),
    (O, "-1"), (O, "+17"), (O, str(MAX_UINT64 + 1)),
    (O, "000" + str(MAX_UINT64 + 1)), (O, "9" * 5000),
    (O, HEX_UUID), (O, "1.0"), (O, "１７"),
] + [(backend, value) for backend in (O, G, "")
     for value in (True, False, 17.0, None, b"17")])
def test_invalid_epoch_never_becomes_another_identity(backend, bad):
    with pytest.raises(m.EpochFormatError):
        m.normalize_epoch(bad, backend)


@pytest.mark.parametrize("backend", [O, G, ""])
@pytest.mark.parametrize("value", ["", " \t\n"])
def test_missing_string_is_not_fabricated_as_zero(backend, value):
    assert m.normalize_epoch(value, backend) == ""
    gate = m.AdmissionGate()
    gate.observe_validity(m.ValidityObservation("VALID", value, 1, backend=backend))
    gate.observe_map(m.MapObservation(value, 1, backend=backend))
    gate.observe_robot_state(m.RobotStateSample(100))
    assert not gate.admit_planning(now_mono_ns=200).admitted


@pytest.mark.parametrize("value,expected", [
    (17, "17"), (MAX_UINT64 + 1, str(MAX_UINT64 + 1)),
    (" 007 ", "007"), ("legacy-identity", "legacy-identity"),
])
def test_undeclared_legacy_backend_keeps_existing_semantics(value, expected):
    assert m.normalize_epoch(value) == expected


@pytest.mark.parametrize("backend,epoch", [(O, MAX_UINT64), (G, DIGIT_UUID)])
def test_legal_boundaries_survive_admission_and_commit(backend, epoch):
    validity = m.ValidityObservation.from_json(
        dict(state="VALID", epoch=epoch, generation=3), backend=backend)
    gate = m.AdmissionGate()
    gate.observe_validity(validity)
    assert gate.observe_map(m.MapObservation(epoch, 3, backend=backend)).admitted
    gate.observe_robot_state(m.RobotStateSample(100))
    decision = gate.admit_planning(now_mono_ns=200)
    assert decision.admitted
    assert gate.commit_plan(admission_token=decision.admission_token).admitted
    assert gate.accept_result(epoch, 3, backend=backend).admitted
    assert gate.validity.identity == (backend, m.normalize_epoch(epoch, backend))


@pytest.mark.parametrize("backend,good,bad", [
    (O, 17, MAX_UINT64 + 1), (O, 17, str(MAX_UINT64 + 1)),
    (G, DIGIT_UUID, 17), (G, DIGIT_UUID, "17"),
])
def test_malformed_replacement_cannot_mutate_existing_valid_gate(backend, good, bad):
    gate = m.AdmissionGate()
    original = m.ValidityObservation("VALID", good, 3, backend=backend)
    gate.observe_validity(original)
    for construct in (
        lambda: m.ValidityObservation("VALID", bad, 4, backend=backend),
        lambda: m.ValidityObservation.from_json(
            dict(state="VALID", epoch=bad, generation=4), backend=backend),
        lambda: m.MapObservation(bad, 4, backend=backend),
    ):
        with pytest.raises(m.EpochFormatError):
            construct()
        assert gate.validity == original
