"""Choose a failure boundary from this trial, without stale legacy fallback."""


def replay_boundary(submitted_legs, stage, legacy_boundary=None):
    """Typed submissions own their boundary; only the latest stage may match.

    Preserve the full trajectory and its truncation marker. Callers must refuse
    a truncated trajectory as replay evidence. No planning or execution occurs.
    """
    if not submitted_legs:
        return legacy_boundary
    latest = submitted_legs[-1]
    if latest.get('name') != stage:
        return None
    return dict(latest, source='trial_submitted_leg')
