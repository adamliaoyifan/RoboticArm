from luggage_gazebo.motion_replay_boundary import replay_boundary


def test_current_typed_submission_wins_over_legacy_boundary():
    trajectory = dict(n_points=2, truncated=False, points=[dict(q=[.123456789])])
    row = dict(name='return', trajectory=trajectory, submit_wall=1., result_wall=2.)
    result = replay_boundary([row], 'return', dict(name='old', trajectory='stale'))
    assert result['source'] == 'trial_submitted_leg'
    assert result['trajectory'] == trajectory
    assert result['submit_wall'] == 1.
    assert 'source' not in row


def test_previous_stage_cannot_supply_failed_stage_or_fall_back_to_legacy():
    assert replay_boundary([dict(name='transit')], 'return', dict(name='return')) is None


def test_truncated_submission_is_not_disguised_as_complete():
    result = replay_boundary([dict(name='return', trajectory=dict(truncated=True))], 'return')
    assert result['trajectory']['truncated'] is True


def test_legacy_path_remains_available_without_typed_submissions():
    legacy = dict(name='legacy')
    assert replay_boundary([], 'legacy', legacy) == legacy
