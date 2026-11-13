"""Compatibility adapter for the shared observed placement task selector."""
from luggage_planning.placement_prior.task_slots import observed_placement_slots


def observed_task_slots(captured,dimensions,mode,*,limits,margin,vertical_margin):
    return observed_placement_slots(captured.surface,captured.volume,dimensions,mode,
        limits=limits,margin=margin,vertical_margin=vertical_margin)
