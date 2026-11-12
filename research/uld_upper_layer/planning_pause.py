"""Freeze only the owned simulation clock during offline computation.

No pose or controller command is changed; every executed motion uses live
physics. Resume runs before returning a plan and on all exception paths.
"""
from contextlib import contextmanager


@contextmanager
def paused_for_planning(runtime):
    try:
        runtime.set_physics_paused(True)
        yield
    finally:
        runtime.set_physics_paused(False)
