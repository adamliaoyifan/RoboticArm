"""Compatibility re-export: the admission gate moved to luggage_planning.

OCC-GRID-CONSUMER-COMPAT G1: planning-side components
(:mod:`luggage_planning.hardware_place_planning`) need the same gate the
planner node uses, and luggage_packing depends on luggage_planning (not
the other way around). The implementation lives in
``luggage_planning.placement_admission``; this module keeps every
historical import path (the planner node, the observer's reset campaign,
existing tests) working unchanged.
"""
from luggage_planning.placement_admission import (  # noqa: F401
    BACKEND_GRID_G,
    BACKEND_OBSERVER_O,
    AdmissionGate,
    AdmissionToken,
    Decision,
    EpochFormatError,
    MapObservation,
    RobotStateSample,
    ValidityObservation,
    normalize_epoch,
)
