"""Bounded observation IK alternatives, each requiring ordinary route checks."""
import math


def observation_goals(refine, position, quaternion, current, fallback, expired):
    """Try the actual seed, four wrist alternatives, then the legacy fallback.

    Perturbations are solver seeds, not executable goals. The solver retains
    joint-limit and pose checks. Do not wrap a solution or claim a path from
    endpoint distance; the caller builds and certifies each unique solution.
    """
    seeds = [('current', tuple(current))]
    for joint in (5, 3):
        for shift in (-math.pi, math.pi):
            seed = list(current)
            seed[joint] += shift
            seeds.append(('j%d%+g' % (joint + 1, shift), tuple(seed)))
    seeds.append(('fallback', tuple(fallback)))
    seen = []
    for label, seed in seeds:
        if expired():
            yield None, dict(reason='PLANNING_BUDGET_EXHAUSTED', seed=label)
            return
        try:
            goal, info = refine(position, quaternion, list(seed))
        except Exception as error:  # Refusal evidence; never authorize motion.
            goal, info = None, dict(error=repr(error)[:200])
        info = dict(info or {}, seed=label)
        if goal is None:
            yield None, info
            continue
        goal = tuple(float(value) for value in goal)
        if any(max(abs(a-b) for a, b in zip(goal, old)) < 1e-6 for old in seen):
            continue
        seen.append(goal)
        yield goal, info
