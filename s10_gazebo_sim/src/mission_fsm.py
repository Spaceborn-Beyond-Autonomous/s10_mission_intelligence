#!/usr/bin/env python3
"""
S10 Mission FSM
===============
Finite State Machine that tracks the overall mission lifecycle:

    IDLE → PLANNING → VALIDATING → ARMED → EXECUTING → REPLANNING → EXECUTING
        ↘ ABORTED                                        ↘ COMPLETED

Published topics
    ~/fsm_state   (std_msgs/String)   — current FSM state name
    ~/fsm_event   (std_msgs/String)   — JSON event log entry
"""
import json
import time
from enum import Enum, auto
from typing import Optional, List, Dict, Callable


class MissionState(Enum):
    IDLE        = auto()
    PLANNING    = auto()
    VALIDATING  = auto()
    ARMED       = auto()
    EXECUTING   = auto()
    REPLANNING  = auto()
    COMPLETED   = auto()
    ABORTED     = auto()


# Valid transitions: current_state → set of allowed next states
_TRANSITIONS = {
    MissionState.IDLE:        {MissionState.PLANNING, MissionState.ABORTED},
    MissionState.PLANNING:    {MissionState.VALIDATING, MissionState.ABORTED},
    MissionState.VALIDATING:  {MissionState.ARMED, MissionState.PLANNING, MissionState.ABORTED},
    MissionState.ARMED:       {MissionState.EXECUTING, MissionState.ABORTED},
    MissionState.EXECUTING:   {MissionState.REPLANNING, MissionState.COMPLETED, MissionState.ABORTED},
    MissionState.REPLANNING:  {MissionState.VALIDATING, MissionState.EXECUTING, MissionState.ABORTED},
    MissionState.COMPLETED:   {MissionState.IDLE},
    MissionState.ABORTED:     {MissionState.IDLE},
}


class MissionFSM:
    """Mission lifecycle state machine with event logging."""

    def __init__(self):
        self.state = MissionState.IDLE
        self.events: List[Dict] = []
        self._callbacks: List[Callable] = []

    def register_callback(self, cb: Callable):
        """Register a callback invoked on every transition: cb(old, new, reason)."""
        self._callbacks.append(cb)

    def transition(self, target: MissionState, reason: str = "") -> bool:
        """
        Attempt a state transition.

        Returns True if the transition was valid, False otherwise.
        """
        if target not in _TRANSITIONS.get(self.state, set()):
            self._log_event("REJECTED", self.state, target, reason)
            return False

        old = self.state
        self.state = target
        self._log_event("TRANSITION", old, target, reason)

        for cb in self._callbacks:
            try:
                cb(old, target, reason)
            except Exception:
                pass
        return True

    def _log_event(self, kind: str, src: MissionState, dst: MissionState, reason: str):
        event = {
            "kind": kind,
            "from": src.name,
            "to": dst.name,
            "reason": reason,
            "timestamp": time.time(),
        }
        self.events.append(event)

    def last_event_json(self) -> str:
        if self.events:
            return json.dumps(self.events[-1])
        return "{}"

    def history_json(self) -> str:
        return json.dumps(self.events, indent=2)
