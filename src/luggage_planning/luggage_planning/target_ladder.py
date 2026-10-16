"""Choose a complete entry ladder before spending motion-search budget.

Target synthesis owns allowed box symmetries. This module only reorders those
targets; it never changes poses, adds grasps, or certifies an IK solution.
"""


def prepare_target_ladder(targets, prepare, regions, *, required, expired):
    targets = tuple(targets)
    if not targets:
        raise ValueError('target ladder requires at least one target')
    attempts = []
    first = None
    for index, target in enumerate(targets):
        if index and expired():
            break
        prepared = prepare(target)
        if first is None:
            first = prepared
        missing = sorted(set(required) - set(regions(prepared)))
        attempts.append({'target_index': index, 'missing_regions': missing,
                         'position': list(target.position),
                         'quaternion_xyzw': list(target.quaternion_xyzw)})
        if not missing:
            ordered = (target,) + targets[:index] + targets[index+1:]
            return prepared, ordered, attempts
    # Preserve the original refusal behavior when every ladder is incomplete.
    # Remaining route logic retains the deadline and named refusal authority.
    return first, targets, attempts
