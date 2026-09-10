"""ROS-free adapter around the accepted production state reducer.

The adapter owns no transition rules. It serializes event delivery, retains
the reducer's immutable state, and forwards every declarative effect to an
injected sink. The sink is the only boundary implemented by the ROS node.
"""

from __future__ import annotations

from threading import RLock
from typing import Callable

from luggage_planning.orchestration_contracts import (
    Effect,
    OperatorEvent,
    OrchestratorContractState,
    Transition,
    initial_state,
    reduce_event,
)


EffectSink = Callable[[Effect], None]


class ProductionAdapter:
    """Deliver events to the reducer and emit its effects in order."""

    def __init__(self, effect_sink: EffectSink) -> None:
        self._effect_sink = effect_sink
        self._state = initial_state()
        self._lock = RLock()

    @property
    def state(self) -> OrchestratorContractState:
        with self._lock:
            return self._state

    def accept(self, event: OperatorEvent) -> Transition:
        with self._lock:
            transition = reduce_event(self._state, event)
            self._state = transition.state
            for effect in transition.effects:
                self._effect_sink(effect)
            return transition
