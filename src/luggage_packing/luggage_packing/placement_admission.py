"""Compatibility imports of the shared luggage_mapping admission contract."""
from luggage_mapping.admission import (  # noqa: F401
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
