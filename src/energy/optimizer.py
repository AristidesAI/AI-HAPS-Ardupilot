"""
Energy optimization for HAPS Glider.

Manages power budgets, day/night transitions, and energy-optimal flight planning.
"""

import time
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone
from enum import Enum
from typing import List, Optional, Tuple

import structlog

from .battery import BatteryManager, BatteryState
from .solar import SolarManager, SunPosition

logger = structlog.get_logger()


class EnergyMode(Enum):
    """Energy operational mode."""
    DAY_NORMAL = "day_normal"  # Normal daylight operations
    DAY_CHARGING = "day_charging"  # Aggressive charging
    DUSK_TRANSITION = "dusk_transition"  # Preparing for night
    NIGHT_CRUISE = "night_cruise"  # Night energy-saving mode
    NIGHT_EMERGENCY = "night_emergency"  # Critical night energy
    DAWN_TRANSITION = "dawn_transition"  # Waiting for sunrise
    EMERGENCY = "emergency"  # Critical battery


@dataclass
class PowerBudget:
    """Power budget allocation."""
    total_available_w: float = 0.0  # Total power available
    flight_systems_w: float = 10.0  # Critical flight systems
    navigation_w: float = 5.0  # Navigation and communication
    payload_w: float = 0.0  # Payload allocation
    reserve_w: float = 0.0  # Unallocated reserve
    is_sufficient: bool = True  # Can meet minimum requirements


@dataclass
class EnergyPrediction:
    """Energy prediction for future time."""
    timestamp: float
    hours_ahead: float
    predicted_soc: float
    predicted_solar_w: float
    predicted_net_w: float  # Solar - consumption
    is_daylight: bool


@dataclass
class EnergyConfig:
    """Energy optimization configuration."""
    # Power consumption baselines
    baseline_power_w: float = 10.0  # Minimum flight systems
    nav_power_w: float = 5.0  # Navigation/communication
    payload_power_w: float = 50.0  # Max payload power

    # Transition thresholds
    dusk_sun_elevation: float = 5.0  # Degrees - start dusk transition
    dawn_sun_elevation: float = -6.0  # Degrees - civil twilight
    night_sun_elevation: float = -6.0  # Degrees - full night

    # Battery thresholds
    min_soc_for_payload: float = 60.0  # Minimum SOC to run payload
    min_soc_for_night: float = 80.0  # Target SOC before night
    emergency_soc: float = 20.0  # Emergency threshold

    # Night mode settings
    night_airspeed_reduction: float = 0.8  # Reduce to 80% of cruise


class EnergyOptimizer:
    """
    Optimizes energy usage for multi-day HAPS operations.

    Coordinates between solar generation and battery consumption
    to maximize mission duration and capability.
    """

    def __init__(
        self,
        battery_manager: BatteryManager,
        solar_manager: SolarManager,
        config: Optional[EnergyConfig] = None,
    ):
        """
        Initialize energy optimizer.

        Args:
            battery_manager: Battery management instance
            solar_manager: Solar management instance
            config: Optimization configuration
        """
        self.battery = battery_manager
        self.solar = solar_manager
        self.config = config or EnergyConfig()

        self._mode = EnergyMode.DAY_NORMAL
        self._power_budget = PowerBudget()
        self._last_update = 0.0

        # Predictions cache
        self._predictions: List[EnergyPrediction] = []

    @property
    def mode(self) -> EnergyMode:
        """Get current energy mode."""
        return self._mode

    @property
    def power_budget(self) -> PowerBudget:
        """Get current power budget."""
        return self._power_budget

    @property
    def is_night_mode(self) -> bool:
        """Check if in night mode."""
        return self._mode in (
            EnergyMode.NIGHT_CRUISE,
            EnergyMode.NIGHT_EMERGENCY,
        )

    @property
    def can_run_payload(self) -> bool:
        """Check if payload can be operated."""
        if self._mode in (EnergyMode.NIGHT_CRUISE, EnergyMode.NIGHT_EMERGENCY, EnergyMode.EMERGENCY):
            return False
        return self.battery.soc >= self.config.min_soc_for_payload

    def update(self, latitude: float, longitude: float, altitude_m: float) -> EnergyMode:
        """
        Update energy state and determine optimal mode.

        Args:
            latitude: Current latitude
            longitude: Current longitude
            altitude_m: Current altitude

        Returns:
            Current energy mode
        """
        self._last_update = time.time()

        # Update solar status
        self.solar.set_location(latitude, longitude, altitude_m)
        solar_status = self.solar.update()

        # Determine energy mode
        self._mode = self._determine_mode(solar_status.sun_position)

        # Calculate power budget
        self._power_budget = self._calculate_budget(solar_status.power_available_w)

        # Generate predictions
        self._predictions = self._generate_predictions(
            latitude, longitude, altitude_m, hours_ahead=24
        )

        logger.debug(
            "Energy update",
            mode=self._mode.value,
            soc=round(self.battery.soc, 1),
            solar_w=round(solar_status.power_available_w, 1),
            sun_elevation=round(solar_status.sun_position.elevation, 1),
        )

        return self._mode

    def _determine_mode(self, sun_pos: SunPosition) -> EnergyMode:
        """Determine energy mode based on sun and battery."""
        cfg = self.config
        soc = self.battery.soc
        sun_el = sun_pos.elevation

        # Emergency override
        if soc <= cfg.emergency_soc:
            return EnergyMode.EMERGENCY

        # Night modes
        if sun_el < cfg.night_sun_elevation:
            if soc < 30:
                return EnergyMode.NIGHT_EMERGENCY
            return EnergyMode.NIGHT_CRUISE

        # Dawn transition
        if cfg.night_sun_elevation <= sun_el < cfg.dawn_sun_elevation:
            return EnergyMode.DAWN_TRANSITION

        # Dusk transition
        if cfg.dusk_sun_elevation >= sun_el > cfg.night_sun_elevation:
            return EnergyMode.DUSK_TRANSITION

        # Daytime modes
        if soc < cfg.min_soc_for_night:
            return EnergyMode.DAY_CHARGING
        return EnergyMode.DAY_NORMAL

    def _calculate_budget(self, solar_power_w: float) -> PowerBudget:
        """Calculate power budget allocation."""
        cfg = self.config

        # Determine total available power
        if self.is_night_mode:
            # On battery only
            total = self.battery.get_safe_power_budget(mission_hours=8)
        else:
            # Solar + battery discharge if needed
            total = solar_power_w

        # Allocate power by priority
        remaining = total

        # Priority 1: Critical flight systems (always needed)
        flight = min(cfg.baseline_power_w, remaining)
        remaining -= flight

        # Priority 2: Navigation and communication
        nav = min(cfg.nav_power_w, remaining)
        remaining -= nav

        # Priority 3: Payload (if allowed)
        payload = 0.0
        if self.can_run_payload and remaining > 0:
            payload = min(cfg.payload_power_w, remaining)
            remaining -= payload

        # Check if budget meets minimum requirements
        is_sufficient = flight >= cfg.baseline_power_w

        return PowerBudget(
            total_available_w=total,
            flight_systems_w=flight,
            navigation_w=nav,
            payload_w=payload,
            reserve_w=max(0, remaining),
            is_sufficient=is_sufficient,
        )

    def _generate_predictions(
        self,
        latitude: float,
        longitude: float,
        altitude_m: float,
        hours_ahead: int = 24,
    ) -> List[EnergyPrediction]:
        """Generate energy predictions for upcoming hours."""
        predictions = []
        now = datetime.now(timezone.utc)

        # Estimate average consumption
        consumption_w = (
            self.config.baseline_power_w
            + self.config.nav_power_w
            + (self.config.payload_power_w * 0.5 if self.can_run_payload else 0)
        )

        current_soc = self.battery.soc

        for h in range(hours_ahead):
            future_time = now + timedelta(hours=h)

            # Predict solar power
            solar_w = self.solar.predict_power(
                latitude, longitude, altitude_m, future_time
            )

            # Predict SOC
            net_power = solar_w - consumption_w
            energy_change_wh = net_power  # 1 hour interval
            soc_change = (energy_change_wh / self.battery._capacity_wh) * 100
            predicted_soc = max(0, min(100, current_soc + soc_change))

            # Check daylight
            sun_pos = self.solar.calculate_sun_position(latitude, longitude, future_time)

            predictions.append(EnergyPrediction(
                timestamp=future_time.timestamp(),
                hours_ahead=h,
                predicted_soc=predicted_soc,
                predicted_solar_w=solar_w,
                predicted_net_w=net_power,
                is_daylight=sun_pos.elevation > -6,
            ))

            current_soc = predicted_soc

        return predictions

    def get_optimal_airspeed_factor(self) -> float:
        """
        Get airspeed reduction factor for energy optimization.

        Returns:
            Factor to multiply cruise airspeed by (0.0-1.0)
        """
        if self._mode == EnergyMode.NIGHT_CRUISE:
            return self.config.night_airspeed_reduction
        elif self._mode == EnergyMode.NIGHT_EMERGENCY:
            return 0.7  # Maximum efficiency
        elif self._mode == EnergyMode.EMERGENCY:
            return 0.7
        elif self._mode == EnergyMode.DAY_CHARGING:
            return 0.9  # Slightly slower to charge faster
        return 1.0  # Normal speed

    def get_optimal_heading_adjustment(self) -> float:
        """
        Get heading adjustment for solar optimization.

        Returns:
            Suggested heading change in degrees (-180 to 180)
        """
        if not self.solar.status or not self.solar.is_daylight():
            return 0.0

        if self._mode in (EnergyMode.DAY_CHARGING, EnergyMode.DUSK_TRANSITION):
            # Actively seek optimal heading
            optimal = self.solar.status.heading_optimal_deg
            current = self.solar._heading

            diff = optimal - current
            if diff > 180:
                diff -= 360
            elif diff < -180:
                diff += 360

            # Only suggest significant adjustments
            if abs(diff) > 30:
                return diff
        return 0.0

    def should_reduce_altitude(self) -> Tuple[bool, float]:
        """
        Check if altitude reduction is recommended for energy.

        Returns:
            Tuple of (should_reduce, recommended_altitude_change_m)
        """
        # At night, descending slowly uses less energy than maintaining altitude
        if self._mode == EnergyMode.NIGHT_CRUISE:
            if self.battery.soc < 50:
                return True, -500  # Descend 500m to reduce energy use

        if self._mode == EnergyMode.NIGHT_EMERGENCY:
            return True, -1000  # More aggressive descent

        return False, 0

    def time_to_sunrise(self) -> float:
        """
        Calculate hours until sunrise.

        Returns:
            Hours until sunrise (0 if already daylight)
        """
        if self.solar.is_daylight():
            return 0.0

        for pred in self._predictions:
            if pred.is_daylight:
                return pred.hours_ahead

        return 12.0  # Default if no prediction available

    def time_to_sunset(self) -> float:
        """
        Calculate hours until sunset.

        Returns:
            Hours until sunset (0 if already night)
        """
        if not self.solar.is_daylight():
            return 0.0

        for pred in self._predictions:
            if not pred.is_daylight:
                return pred.hours_ahead

        return 12.0  # Default

    def can_complete_mission(self, mission_hours: float) -> bool:
        """
        Check if mission can be completed with current energy.

        Args:
            mission_hours: Required mission duration

        Returns:
            True if energy is sufficient
        """
        if not self._predictions:
            return False

        # Check minimum SOC throughout mission
        for pred in self._predictions:
            if pred.hours_ahead <= mission_hours:
                if pred.predicted_soc < self.config.emergency_soc:
                    return False

        return True

    def get_recommended_actions(self) -> List[str]:
        """Get list of recommended actions based on energy state."""
        actions = []

        if self._mode == EnergyMode.EMERGENCY:
            actions.append("EMERGENCY: Initiate RTL immediately")
            actions.append("Disable all non-essential systems")

        elif self._mode == EnergyMode.NIGHT_EMERGENCY:
            actions.append("Reduce airspeed to minimum safe")
            actions.append("Disable payload systems")
            actions.append("Consider controlled descent to reduce power")

        elif self._mode == EnergyMode.NIGHT_CRUISE:
            actions.append("Maintain energy-efficient cruise")
            actions.append("Monitor battery SOC closely")

        elif self._mode == EnergyMode.DUSK_TRANSITION:
            if self.battery.soc < self.config.min_soc_for_night:
                actions.append(f"WARNING: SOC ({self.battery.soc:.0f}%) below night threshold")
                actions.append("Maximize charging before sunset")
            actions.append("Prepare for night mode transition")
            actions.append("Reduce payload operations")

        elif self._mode == EnergyMode.DAWN_TRANSITION:
            actions.append("Prepare for day mode")
            actions.append("Systems will activate after sunrise")

        elif self._mode == EnergyMode.DAY_CHARGING:
            actions.append("Prioritize battery charging")
            optimal_heading = self.get_optimal_heading_adjustment()
            if abs(optimal_heading) > 30:
                actions.append(f"Adjust heading by {optimal_heading:.0f}° for better solar")

        else:  # DAY_NORMAL
            if self.solar.status and self.solar.status.heading_gain_percent > 20:
                actions.append("Consider heading adjustment for +20% solar gain")

        return actions

    def get_status_dict(self) -> dict:
        """Get comprehensive status dictionary."""
        return {
            "mode": self._mode.value,
            "battery": self.battery.get_status_dict(),
            "solar": self.solar.get_status_dict() if self.solar.status else {},
            "power_budget": {
                "total_available_w": round(self._power_budget.total_available_w, 1),
                "flight_systems_w": round(self._power_budget.flight_systems_w, 1),
                "navigation_w": round(self._power_budget.navigation_w, 1),
                "payload_w": round(self._power_budget.payload_w, 1),
                "reserve_w": round(self._power_budget.reserve_w, 1),
                "is_sufficient": self._power_budget.is_sufficient,
            },
            "can_run_payload": self.can_run_payload,
            "time_to_sunrise_h": round(self.time_to_sunrise(), 1),
            "time_to_sunset_h": round(self.time_to_sunset(), 1),
            "airspeed_factor": self.get_optimal_airspeed_factor(),
        }
