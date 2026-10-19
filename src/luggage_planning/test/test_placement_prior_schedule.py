import pytest
from luggage_planning.placement_prior_schedule import SnapshotQuietGate


def test_short_revisions_are_coalesced_and_final_snapshot_becomes_ready():
    gate = SnapshotQuietGate(2.)
    for revision in range(1, 6):
        now = revision * .5
        gate.observe(str(revision), now)
        assert not gate.ready(str(revision), now + .49)
    assert not gate.ready('5', 4.49)
    assert gate.ready('5', 4.5)
    # A repeated publication must not postpone useful work indefinitely.
    gate.observe('5', 4.6)
    assert gate.ready('5', 4.6)
    assert not gate.ready('4', 4.6)


def test_confirmation_loss_restarts_quiet_period_even_for_same_identity():
    gate = SnapshotQuietGate(2.)
    gate.observe('a', 10.)
    assert gate.ready('a', 12.)
    gate.observe(None, 12.1)
    assert not gate.ready('a', 100.)
    gate.observe('a', 12.2)
    assert not gate.ready('a', 14.19)
    assert gate.ready('a', 14.2)


def test_zero_delay_preserves_immediate_scheduling_but_requires_confirmation():
    gate = SnapshotQuietGate(0.)
    assert not gate.ready(None, 0.)
    gate.observe('a', 1.)
    assert gate.ready('a', 1.)
    assert not gate.ready('a', .9)


@pytest.mark.parametrize('value', [-1., float('inf'), float('nan'), True])
def test_invalid_interval_is_rejected(value):
    with pytest.raises(ValueError):
        SnapshotQuietGate(value)
