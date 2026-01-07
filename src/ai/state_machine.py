"""
State machine for HAPS mission management.

Defines mission states and transitions for autonomous operation.
"""

import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Callable, Dict, List, Optional, Set

import structlog

logger = structlog.get_logger()


class MissionState(Enum):
    """High-level mission states."""
    # Initialization
    PREFLIGHT = auto()  # System checks, waiting for launch
    LAUNCHING = auto()  # Launch/takeoff in progress

    # Climb phase
    CLIMBING_TO_ALTITUDE = auto()  # Ascending to operational altitude

    # Operational states
    STATION_KEEPING = auto()  # Primary operational mode - holding position
    TRANSIT = auto()  # Moving to new location
    SURVEY = auto()  # Executing survey pattern
    TRACKING = auto()  # Tracking ground target

    # Energy states
    ENERGY_RECOVERY = auto()  # Low energy - optimizing for charging
    NIGHT_CRUISE = auto()  # Night mode - minimal operations

    # Emergency states
    RTL = auto()  # Return to launch
    EMERGENCY_DESCENT = auto()  # Emergency controlled descent
    LINK_LOST = auto()  # Communication lost - autonomous operation

    # Terminal states
    LANDING = auto()  # Landing in progress
    LANDED = auto()  # Mission complete


class MissionPhase(Enum):
    """Mission phase (day/night cycle)."""
    DAY_OPERATIONS = "day_operations"
    DUSK_TRANSITION = "dusk_transition"
    NIGHT_CRUISE = "night_cruise"
    DAWN_TRANSITION = "dawn_transition"


@dataclass
class StateTransition:
    """Definition of a state transition."""
    from_state: MissionState
    to_state: MissionState
    condition: str  # Description of condition
    priority: int = 0  # Higher = more important


@dataclass
class MissionContext:
    """Context data for state machine decisions."""
    # Position
    latitude: float = 0.0
    longitude: float = 0.0
    altitude_m: float = 0.0

    # Energy
    battery_soc: float = 100.0
    solar_power_w: float = 0.0
    power_balance_w: float = 0.0
    is_daylight: bool = True

    # Wind
    wind_speed_ms: float = 0.0
    wind_direction_deg: float = 0.0

    # System
    is_armed: bool = False
    flight_mode: str = "MANUAL"
    gps_fix: bool = False
    link_quality: float = 1.0

    # Mission
    distance_to_target_m: float = 0.0
    time_on_station_s: float = 0.0


class StateMachine:
    """
    Mission state machine for HAPS operations.

    Manages state transitions based on flight conditions,
    energy state, and mission requirements.
    """

    def __init__(self):
        """Initialize state machine."""
        self._state = MissionState.PREFLIGHT
        self._phase = MissionPhase.DAY_OPERATIONS
        self._state_entry_time = time.time()
        self._state_history: List[tuple] = []

        # Callbacks for state entry/exit
        self._on_enter: Dict[MissionState, List[Callable]] = {}
        self._on_exit: Dict[MissionState, List[Callable]] = {}

        # Define valid transitions
        self._transitions = self._build_transitions()

        # Emergency override flag
        self._emergency_override = False

    @property
    def state(self) -> MissionState:
        """Get current state."""
        return self._state

    @property
    def phase(self) -> MissionPhase:
        """Get current mission phase."""
        return self._phase

    @property
    def time_in_state(self) -> float:
        """Get time in current state (seconds)."""
        return time.time() - self._state_entry_time

    def _build_transitions(self) -> Dict[MissionState, Set[MissionState]]:
        """Build valid state transitions."""
        return {
            MissionState.PREFLIGHT: {
                MissionState.LAUNCHING,
            },
            MissionState.LAUNCHING: {
                MissionState.CLIMBING_TO_ALTITUDE,
                MissionState.EMERGENCY_DESCENT,
            },
            MissionState.CLIMBING_TO_ALTITUDE: {
                MissionState.STATION_KEEPING,
                MissionState.TRANSIT,
                MissionState.RTL,
                MissionState.EMERGENCY_DESCENT,
            },
            MissionState.STATION_KEEPING: {
                MissionState.TRANSIT,
                MissionState.SURVEY,
                MissionState.TRACKING,
                MissionState.ENERGY_RECOVERY,
                MissionState.NIGHT_CRUISE,
                MissionState.RTL,
                MissionState.EMERGENCY_DESCENT,
                MissionState.LINK_LOST,
            },
            MissionState.TRANSIT: {
                MissionState.STATION_KEEPING,
                MissionState.SURVEY,
                MissionState.TRACKING,
                MissionState.ENERGY_RECOVERY,
                MissionState.RTL,
                MissionState.EMERGENCY_DESCENT,
                MissionState.LINK_LOST,
            },
            MissionState.SURVEY: {
                MissionState.STATION_KEEPING,
                MissionState.TRANSIT,
                MissionState.ENERGY_RECOVERY,
                MissionState.RTL,
                MissionState.EMERGENCY_DESCENT,
            },
            MissionState.TRACKING: {
                MissionState.STATION_KEEPING,
                MissionState.TRANSIT,
                MissionState.ENERGY_RECOVERY,
                MissionState.RTL,
                MissionState.EMERGENCY_DESCENT,
            },
            MissionState.ENERGY_RECOVERY: {
                MissionState.STATION_KEEPING,
                MissionState.NIGHT_CRUISE,
                MissionState.RTL,
                MissionState.EMERGENCY_DESCENT,
            },
            MissionState.NIGHT_CRUISE: {
                MissionState.STATION_KEEPING,
                MissionState.ENERGY_RECOVERY,
                MissionState.RTL,
                MissionState.EMERGENCY_DESCENT,
            },
            MissionState.RTL: {
                MissionState.LANDING,
                MissionState.STATION_KEEPING,  # Can abort RTL
                MissionState.EMERGENCY_DESCENT,
            },
            MissionState.EMERGENCY_DESCENT: {
                MissionState.LANDING,
            },
            MissionState.LINK_LOST: {
                MissionState.STATION_KEEPING,  # If link recovers
                MissionState.RTL,  # After timeout
                MissionState.EMERGENCY_DESCENT,
            },
            MissionState.LANDING: {
                MissionState.LANDED,
            },
            MissionState.LANDED: set(),  # Terminal state
        }

    def can_transition(self, to_state: MissionState) -> bool:
        """Check if transition to state is valid."""
        if to_state == self._state:
            return True  # Already in state

        valid_targets = self._transitions.get(self._state, set())
        return to_state in valid_targets

    def transition(self, to_state: MissionState, reason: str = "") -> bool:
        """
        Attempt state transition.

        Args:
            to_state: Target state
            reason: Reason for transition

        Returns:
            True if transition successful
        """
        if not self.can_transition(to_state):
            logger.warning(
                "Invalid state transition",
                from_state=self._state.name,
                to_state=to_state.name,
            )
            return False

        if to_state == self._state:
            return True

        old_state = self._state
        now = time.time()

        # Exit callbacks
        if old_state in self._on_exit:
            for callback in self._on_exit[old_state]:
                try:
                    callback(old_state, to_state)
                except Exception as e:
                    logger.error("Exit callback error", error=str(e))

        # Record history
        self._state_history.append((
            now,
            old_state.name,
            to_state.name,
            reason,
        ))

        # Update state
        self._state = to_state
        self._state_entry_time = now

        logger.info(
            "State transition",
            from_state=old_state.name,
            to_state=to_state.name,
            reason=reason,
        )

        # Entry callbacks
        if to_state in self._on_enter:
            for callback in self._on_enter[to_state]:
                try:
                    callback(old_state, to_state)
                except Exception as e:
                    logger.error("Entry callback error", error=str(e))

        return True

    def evaluate(self, context: MissionContext) -> Optional[MissionState]:
        """
        Evaluate conditions and determine if state change needed.

        Args:
            context: Current mission context

        Returns:
            Recommended next state or None if no change
        """
        # Update phase based on daylight
        self._update_phase(context)

        # Emergency conditions (highest priority)
        emergency_state = self._check_emergency(context)
        if emergency_state:
            return emergency_state

        # State-specific logic
        if self._state == MissionState.PREFLIGHT:
            return self._evaluate_preflight(context)
        elif self._state == MissionState.LAUNCHING:
            return self._evaluate_launching(context)
        elif self._state == MissionState.CLIMBING_TO_ALTITUDE:
            return self._evaluate_climbing(context)
        elif self._state == MissionState.STATION_KEEPING:
            return self._evaluate_station_keeping(context)
        elif self._state == MissionState.TRANSIT:
            return self._evaluate_transit(context)
        elif self._state == MissionState.ENERGY_RECOVERY:
            return self._evaluate_energy_recovery(context)
        elif self._state == MissionState.NIGHT_CRUISE:
            return self._evaluate_night_cruise(context)
        elif self._state == MissionState.LINK_LOST:
            return self._evaluate_link_lost(context)

        return None

    def _update_phase(self, context: MissionContext) -> None:
        """Update mission phase based on conditions."""
        if context.is_daylight:
            if context.solar_power_w < 10 and self._phase != MissionPhase.DUSK_TRANSITION:
                self._phase = MissionPhase.DUSK_TRANSITION
            elif context.solar_power_w > 50:
                self._phase = MissionPhase.DAY_OPERATIONS
        else:
            if context.solar_power_w > 10:
                self._phase = MissionPhase.DAWN_TRANSITION
            else:
                self._phase = MissionPhase.NIGHT_CRUISE

    def _check_emergency(self, context: MissionContext) -> Optional[MissionState]:
        """Check for emergency conditions."""
        # Critical battery
        if context.battery_soc < 10:
            return MissionState.EMERGENCY_DESCENT

        # Lost link timeout
        if context.link_quality < 0.1 and self._state != MissionState.LINK_LOST:
            return MissionState.LINK_LOST

        return None

    def _evaluate_preflight(self, context: MissionContext) -> Optional[MissionState]:
        """Evaluate preflight state."""
        if context.is_armed and context.gps_fix:
            return MissionState.LAUNCHING
        return None

    def _evaluate_launching(self, context: MissionContext) -> Optional[MissionState]:
        """Evaluate launching state."""
        if context.altitude_m > 100:  # Above 100m
            return MissionState.CLIMBING_TO_ALTITUDE
        return None

    def _evaluate_climbing(self, context: MissionContext) -> Optional[MissionState]:
        """Evaluate climbing state."""
        if context.altitude_m >= 18000:  # Reached stratosphere
            return MissionState.STATION_KEEPING
        if context.battery_soc < 30:
            return MissionState.RTL
        return None

    def _evaluate_station_keeping(self, context: MissionContext) -> Optional[MissionState]:
        """Evaluate station keeping state."""
        # Energy recovery needed
        if context.battery_soc < 40 and context.solar_power_w < 20:
            return MissionState.ENERGY_RECOVERY

        # Night mode
        if not context.is_daylight and self._phase == MissionPhase.NIGHT_CRUISE:
            return MissionState.NIGHT_CRUISE

        return None

    def _evaluate_transit(self, context: MissionContext) -> Optional[MissionState]:
        """Evaluate transit state."""
        if context.distance_to_target_m < 1000:  # Arrived
            return MissionState.STATION_KEEPING
        return None

    def _evaluate_energy_recovery(self, context: MissionContext) -> Optional[MissionState]:
        """Evaluate energy recovery state."""
        if context.battery_soc > 70:
            return MissionState.STATION_KEEPING
        if context.battery_soc < 20:
            return MissionState.RTL
        return None

    def _evaluate_night_cruise(self, context: MissionContext) -> Optional[MissionState]:
        """Evaluate night cruise state."""
        if context.is_daylight and context.solar_power_w > 50:
            return MissionState.STATION_KEEPING
        if context.battery_soc < 15:
            return MissionState.RTL
        return None

    def _evaluate_link_lost(self, context: MissionContext) -> Optional[MissionState]:
        """Evaluate link lost state."""
        if context.link_quality > 0.5:
            return MissionState.STATION_KEEPING  # Link recovered

        # 48 hours autonomous, then RTL
        if self.time_in_state > 48 * 3600:
            return MissionState.RTL

        return None

    def register_on_enter(self, state: MissionState, callback: Callable) -> None:
        """Register callback for state entry."""
        if state not in self._on_enter:
            self._on_enter[state] = []
        self._on_enter[state].append(callback)

    def register_on_exit(self, state: MissionState, callback: Callable) -> None:
        """Register callback for state exit."""
        if state not in self._on_exit:
            self._on_exit[state] = []
        self._on_exit[state].append(callback)

    def get_history(self, limit: int = 20) -> List[tuple]:
        """Get state transition history."""
        return self._state_history[-limit:]

    def get_status_dict(self) -> dict:
        """Get status as dictionary."""
        return {
            "state": self._state.name,
            "phase": self._phase.value,
            "time_in_state_s": round(self.time_in_state, 0),
            "history_count": len(self._state_history),
        }
