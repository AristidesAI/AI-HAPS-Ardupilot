"""
Altitude control for HAPS station keeping.

Manages altitude band, energy-optimized climbing/descending, and altitude holds.
"""

import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple

import structlog

logger = structlog.get_logger()


class AltitudeState(Enum):
    """Altitude control state."""
    HOLDING = "holding"  # Maintaining altitude
    CLIMBING = "climbing"  # Gaining altitude
    DESCENDING = "descending"  # Losing altitude
    EMERGENCY_DESCENT = "emergency_descent"  # Emergency descent


@dataclass
class AltitudeBand:
    """Altitude operating band."""
    min_m: float = 18000.0  # ~59,000 ft - minimum stratospheric
    max_m: float = 25000.0  # ~82,000 ft - maximum operational
    target_m: float = 20000.0  # Default target
    tolerance_m: float = 100.0  # Acceptable deviation


@dataclass
class AltitudeConfig:
    """Altitude control configuration."""
    # Operating limits
    band: AltitudeBand = None  # Will be set in __init__
    absolute_min_m: float = 15000.0  # Emergency minimum
    absolute_max_m: float = 26000.0  # Never exceed

    # Performance limits
    max_climb_rate_ms: float = 3.0  # m/s
    max_descent_rate_ms: float = 2.0  # m/s
    efficient_climb_rate_ms: float = 1.5  # Energy-efficient climb
    efficient_descent_rate_ms: float = 0.5  # Glide-efficient descent

    # Energy optimization
    climb_when_excess_power_w: float = 50.0  # Climb if excess solar power
    descend_when_deficit_power_w: float = -30.0  # Allow descent if deficit

    # Air density considerations
    # At 20km: ~0.088 kg/m³ (vs 1.225 at sea level)
    min_air_density_kg_m3: float = 0.05  # Too thin for safe flight

    def __post_init__(self):
        if self.band is None:
            self.band = AltitudeBand()


@dataclass
class AltitudeStatus:
    """Current altitude control status."""
    state: AltitudeState = AltitudeState.HOLDING
    current_m: float = 20000.0
    target_m: float = 20000.0
    error_m: float = 0.0
    climb_rate_ms: float = 0.0
    commanded_rate_ms: float = 0.0
    time_to_target_s: float = 0.0
    air_density_kg_m3: float = 0.1
    is_in_band: bool = True


class AltitudeController:
    """
    Controls altitude for HAPS station keeping.

    Features:
    - Altitude band enforcement
    - Energy-optimized altitude changes
    - Air density monitoring
    - Emergency descent handling
    """

    def __init__(self, config: Optional[AltitudeConfig] = None):
        """
        Initialize altitude controller.

        Args:
            config: Altitude configuration
        """
        self.config = config or AltitudeConfig()
        self.status = AltitudeStatus()

        self._target_altitude = self.config.band.target_m
        self._climb_history: list = []  # For rate averaging

    @property
    def current_altitude(self) -> float:
        """Get current altitude."""
        return self.status.current_m

    @property
    def target_altitude(self) -> float:
        """Get target altitude."""
        return self._target_altitude

    @property
    def is_in_band(self) -> bool:
        """Check if within operating band."""
        return self.status.is_in_band

    @property
    def state(self) -> AltitudeState:
        """Get current state."""
        return self.status.state

    def set_target(self, altitude_m: float) -> bool:
        """
        Set target altitude.

        Args:
            altitude_m: Target altitude (meters MSL)

        Returns:
            True if target is valid and set
        """
        # Enforce limits
        if altitude_m < self.config.absolute_min_m:
            logger.warning(
                "Target below minimum",
                requested=altitude_m,
                minimum=self.config.absolute_min_m,
            )
            return False

        if altitude_m > self.config.absolute_max_m:
            logger.warning(
                "Target above maximum",
                requested=altitude_m,
                maximum=self.config.absolute_max_m,
            )
            return False

        self._target_altitude = altitude_m
        logger.info("Altitude target set", target_m=altitude_m)
        return True

    def update(
        self,
        current_altitude_m: float,
        climb_rate_ms: float,
        power_balance_w: float = 0.0,
    ) -> Tuple[float, AltitudeState]:
        """
        Update altitude control and get commanded climb rate.

        Args:
            current_altitude_m: Current altitude MSL (meters)
            climb_rate_ms: Current climb rate (m/s)
            power_balance_w: Current power balance (positive = excess)

        Returns:
            Tuple of (commanded_climb_rate_ms, state)
        """
        now = time.time()

        # Track climb rate history
        self._climb_history.append((now, climb_rate_ms))
        self._climb_history = [
            (t, r) for t, r in self._climb_history
            if now - t < 60
        ]  # Keep last minute

        # Calculate altitude error
        error = self._target_altitude - current_altitude_m

        # Check if in operating band
        in_band = (
            self.config.band.min_m <= current_altitude_m <= self.config.band.max_m
        )

        # Calculate air density
        air_density = self._calculate_air_density(current_altitude_m)

        # Determine state and commanded rate
        state, commanded_rate = self._calculate_command(
            current_altitude_m, error, power_balance_w, in_band
        )

        # Calculate time to target
        if abs(commanded_rate) > 0.1:
            time_to_target = abs(error) / abs(commanded_rate)
        else:
            time_to_target = float('inf')

        # Update status
        self.status = AltitudeStatus(
            state=state,
            current_m=current_altitude_m,
            target_m=self._target_altitude,
            error_m=error,
            climb_rate_ms=climb_rate_ms,
            commanded_rate_ms=commanded_rate,
            time_to_target_s=time_to_target,
            air_density_kg_m3=air_density,
            is_in_band=in_band,
        )

        return commanded_rate, state

    def _calculate_command(
        self,
        current_alt: float,
        error: float,
        power_balance: float,
        in_band: bool,
    ) -> Tuple[AltitudeState, float]:
        """Calculate altitude command based on state."""
        cfg = self.config
        tolerance = cfg.band.tolerance_m

        # Emergency: below absolute minimum
        if current_alt < cfg.absolute_min_m:
            logger.warning("Below absolute minimum altitude!")
            return AltitudeState.EMERGENCY_DESCENT, 0.0  # Hold, don't descend further

        # Emergency: above absolute maximum
        if current_alt > cfg.absolute_max_m:
            logger.warning("Above absolute maximum altitude!")
            return AltitudeState.DESCENDING, -cfg.max_descent_rate_ms

        # Check if at target (within tolerance)
        if abs(error) < tolerance:
            return AltitudeState.HOLDING, 0.0

        # Need to climb
        if error > tolerance:
            # Can we afford to climb?
            if power_balance > cfg.climb_when_excess_power_w:
                # Full climb rate
                rate = min(cfg.max_climb_rate_ms, error / 60)  # Smooth approach
            elif power_balance > 0:
                # Efficient climb rate
                rate = min(cfg.efficient_climb_rate_ms, error / 120)
            else:
                # No power for climbing, hold
                rate = 0.0
                return AltitudeState.HOLDING, rate

            return AltitudeState.CLIMBING, rate

        # Need to descend
        if error < -tolerance:
            # Check if we're above band maximum
            if current_alt > cfg.band.max_m:
                # Need to get back in band
                rate = -cfg.max_descent_rate_ms
            elif power_balance < cfg.descend_when_deficit_power_w:
                # Energy deficit - descend to save power
                rate = -cfg.efficient_descent_rate_ms
            else:
                # Controlled descent
                rate = max(-cfg.efficient_descent_rate_ms, error / 60)

            return AltitudeState.DESCENDING, rate

        return AltitudeState.HOLDING, 0.0

    def _calculate_air_density(self, altitude_m: float) -> float:
        """
        Calculate air density at altitude using standard atmosphere.

        Args:
            altitude_m: Altitude in meters

        Returns:
            Air density in kg/m³
        """
        # International Standard Atmosphere model
        # Reference: https://en.wikipedia.org/wiki/International_Standard_Atmosphere

        T0 = 288.15  # Sea level temperature (K)
        P0 = 101325  # Sea level pressure (Pa)
        L = 0.0065  # Temperature lapse rate (K/m) in troposphere
        R = 287.05  # Specific gas constant for air (J/(kg·K))
        g = 9.80665  # Gravity (m/s²)

        # Barometric formula exponent
        exponent = g / (R * L)  # ≈ 5.255

        if altitude_m < 11000:
            # Troposphere (0-11 km)
            T = T0 - L * altitude_m
            P = P0 * (T / T0) ** exponent
        elif altitude_m < 20000:
            # Lower stratosphere (11-20 km) - isothermal
            T = 216.65  # K (constant)
            T11 = T0 - L * 11000  # Temperature at 11km
            P11 = P0 * (T11 / T0) ** exponent
            P = P11 * math.exp(-g * (altitude_m - 11000) / (R * T))
        else:
            # Upper stratosphere (20-32 km) - temperature increases
            T11 = T0 - L * 11000
            P11 = P0 * (T11 / T0) ** exponent
            P20 = P11 * math.exp(-g * 9000 / (R * 216.65))

            L2 = 0.001  # Lapse rate in upper stratosphere (K/m)
            T = 216.65 + L2 * (altitude_m - 20000)
            P = P20 * (T / 216.65) ** (-g / (R * L2))

        # Density from ideal gas law: rho = P / (R * T)
        density = P / (R * T)

        return density

    def optimize_for_energy(
        self,
        power_balance_w: float,
        battery_soc: float,
        is_daylight: bool,
    ) -> Optional[float]:
        """
        Get energy-optimized altitude recommendation.

        Args:
            power_balance_w: Current power balance
            battery_soc: Battery state of charge (%)
            is_daylight: Whether currently daylight

        Returns:
            Recommended altitude change (meters) or None
        """
        current = self.status.current_m
        band = self.config.band

        if is_daylight:
            # During day: climb if excess power to store energy as altitude
            if power_balance_w > 100 and battery_soc > 80:
                if current < band.max_m - 500:
                    return 500  # Recommend climbing 500m
        else:
            # At night: descend slowly to conserve energy
            if battery_soc < 50:
                if current > band.min_m + 1000:
                    return -500  # Recommend descending 500m

        return None

    def get_safe_descent_altitude(self) -> float:
        """
        Get minimum safe altitude for emergency descent.

        Returns:
            Minimum safe altitude (meters)
        """
        return self.config.absolute_min_m

    def get_status_dict(self) -> dict:
        """Get status as dictionary."""
        return {
            "state": self.status.state.value,
            "current_m": round(self.status.current_m, 0),
            "target_m": round(self.status.target_m, 0),
            "error_m": round(self.status.error_m, 1),
            "climb_rate_ms": round(self.status.climb_rate_ms, 2),
            "commanded_rate_ms": round(self.status.commanded_rate_ms, 2),
            "air_density_kg_m3": round(self.status.air_density_kg_m3, 4),
            "is_in_band": self.status.is_in_band,
        }
