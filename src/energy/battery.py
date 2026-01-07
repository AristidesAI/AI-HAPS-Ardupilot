"""
Battery management for HAPS Glider.

Monitors battery state, predicts endurance, and manages charging/discharging.
"""

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional

import structlog

logger = structlog.get_logger()


class BatteryState(Enum):
    """Battery operational state."""
    UNKNOWN = "unknown"
    CHARGING = "charging"
    DISCHARGING = "discharging"
    FULL = "full"
    CRITICAL = "critical"
    EMERGENCY = "emergency"


@dataclass
class BatteryReading:
    """Single battery telemetry reading."""
    timestamp: float
    voltage: float
    current: float
    soc_percent: float
    temperature: float
    power_watts: float


@dataclass
class BatteryConfig:
    """Battery configuration parameters."""
    capacity_ah: float = 50.0  # Amp-hours
    nominal_voltage: float = 44.4  # 12S Li-ion nominal
    cells: int = 12
    cell_voltage_min: float = 3.0  # Minimum safe cell voltage
    cell_voltage_max: float = 4.2  # Maximum cell voltage
    cell_voltage_nominal: float = 3.7
    critical_soc: float = 20.0  # Critical SOC percentage
    low_soc: float = 40.0  # Low warning SOC percentage
    emergency_soc: float = 10.0  # Emergency SOC percentage
    reserve_soc: float = 20.0  # Reserve for RTL


@dataclass
class BatteryStatus:
    """Current battery status."""
    state: BatteryState = BatteryState.UNKNOWN
    voltage: float = 0.0
    current: float = 0.0
    soc_percent: float = 0.0
    temperature: float = 25.0
    power_watts: float = 0.0
    energy_remaining_wh: float = 0.0
    time_remaining_hours: float = 0.0
    cells_balanced: bool = True
    timestamp: float = 0.0


class BatteryManager:
    """
    Manages battery state and predictions for HAPS operations.

    Features:
    - State-of-charge tracking
    - Endurance prediction
    - Temperature compensation
    - Cell balancing monitoring
    - Power budget enforcement
    """

    def __init__(self, config: Optional[BatteryConfig] = None):
        """
        Initialize battery manager.

        Args:
            config: Battery configuration
        """
        self.config = config or BatteryConfig()
        self.status = BatteryStatus()

        # History for averaging and prediction
        self._history: List[BatteryReading] = []
        self._history_max_size = 3600  # 1 hour at 1Hz

        # Capacity in Wh
        self._capacity_wh = self.config.capacity_ah * self.config.nominal_voltage

        # Energy consumed tracking
        self._energy_consumed_wh = 0.0
        self._last_update_time = 0.0

    @property
    def state(self) -> BatteryState:
        """Get current battery state."""
        return self.status.state

    @property
    def soc(self) -> float:
        """Get state of charge (0-100%)."""
        return self.status.soc_percent

    @property
    def voltage(self) -> float:
        """Get battery voltage."""
        return self.status.voltage

    @property
    def power(self) -> float:
        """Get current power (positive = charging)."""
        return self.status.power_watts

    @property
    def is_critical(self) -> bool:
        """Check if battery is in critical state."""
        return self.status.soc_percent <= self.config.critical_soc

    @property
    def is_emergency(self) -> bool:
        """Check if battery is in emergency state."""
        return self.status.soc_percent <= self.config.emergency_soc

    @property
    def is_charging(self) -> bool:
        """Check if battery is charging."""
        return self.status.current > 0

    def update(
        self,
        voltage: float,
        current: float,
        soc_percent: Optional[float] = None,
        temperature: float = 25.0,
    ) -> None:
        """
        Update battery state from telemetry.

        Args:
            voltage: Battery voltage (V)
            current: Battery current (A, positive = charging)
            soc_percent: State of charge from BMS (optional)
            temperature: Battery temperature (C)
        """
        now = time.time()
        power_watts = voltage * current

        # Calculate SOC if not provided
        if soc_percent is None:
            soc_percent = self._estimate_soc_from_voltage(voltage, temperature)

        # Track energy consumed
        if self._last_update_time > 0:
            dt_hours = (now - self._last_update_time) / 3600
            energy_delta = power_watts * dt_hours
            self._energy_consumed_wh -= energy_delta  # Negative current = consumption

        self._last_update_time = now

        # Determine state
        state = self._determine_state(soc_percent, current)

        # Calculate remaining energy and time
        energy_remaining_wh = (soc_percent / 100) * self._capacity_wh
        avg_consumption = self._get_average_consumption()
        time_remaining_hours = (
            energy_remaining_wh / avg_consumption if avg_consumption > 0 else float('inf')
        )

        # Update status
        self.status = BatteryStatus(
            state=state,
            voltage=voltage,
            current=current,
            soc_percent=soc_percent,
            temperature=temperature,
            power_watts=power_watts,
            energy_remaining_wh=energy_remaining_wh,
            time_remaining_hours=time_remaining_hours,
            timestamp=now,
        )

        # Add to history
        reading = BatteryReading(
            timestamp=now,
            voltage=voltage,
            current=current,
            soc_percent=soc_percent,
            temperature=temperature,
            power_watts=power_watts,
        )
        self._add_to_history(reading)

        # Log state changes
        if state in (BatteryState.CRITICAL, BatteryState.EMERGENCY):
            logger.warning(
                "Battery state alert",
                state=state.value,
                soc=soc_percent,
                voltage=voltage,
            )

    def _determine_state(self, soc: float, current: float) -> BatteryState:
        """Determine battery state from SOC and current."""
        if soc <= self.config.emergency_soc:
            return BatteryState.EMERGENCY
        elif soc <= self.config.critical_soc:
            return BatteryState.CRITICAL
        elif soc >= 99:
            return BatteryState.FULL
        elif current > 0.5:  # Charging threshold
            return BatteryState.CHARGING
        elif current < -0.5:  # Discharging threshold
            return BatteryState.DISCHARGING
        else:
            return BatteryState.UNKNOWN

    def _estimate_soc_from_voltage(self, voltage: float, temperature: float) -> float:
        """
        Estimate SOC from voltage using Li-ion discharge curve.

        This is a simplified model - real BMS data should be used when available.
        """
        cell_voltage = voltage / self.config.cells

        # Temperature compensation (Li-ion loses ~0.3% capacity per degree below 25C)
        temp_factor = 1.0 - max(0, (25 - temperature)) * 0.003

        # Simplified Li-ion discharge curve
        v_min = self.config.cell_voltage_min
        v_max = self.config.cell_voltage_max

        if cell_voltage >= v_max:
            soc = 100.0
        elif cell_voltage <= v_min:
            soc = 0.0
        else:
            # Non-linear mapping approximating Li-ion curve
            normalized = (cell_voltage - v_min) / (v_max - v_min)
            # Apply curve shaping (more linear in middle, steep at ends)
            soc = 100 * (0.8 * normalized + 0.2 * normalized ** 2)

        return soc * temp_factor

    def _get_average_consumption(self, window_seconds: float = 300) -> float:
        """Get average power consumption over time window."""
        if not self._history:
            return 10.0  # Default baseline consumption

        cutoff = time.time() - window_seconds
        recent = [r for r in self._history if r.timestamp > cutoff and r.power_watts < 0]

        if not recent:
            return 10.0

        return abs(sum(r.power_watts for r in recent) / len(recent))

    def _add_to_history(self, reading: BatteryReading) -> None:
        """Add reading to history, maintaining max size."""
        self._history.append(reading)
        if len(self._history) > self._history_max_size:
            self._history = self._history[-self._history_max_size:]

    def predict_endurance(self, power_consumption_w: float) -> float:
        """
        Predict remaining flight time at given power consumption.

        Args:
            power_consumption_w: Expected power consumption (watts)

        Returns:
            Predicted endurance in hours
        """
        if power_consumption_w <= 0:
            return float('inf')

        # Available energy (accounting for reserve)
        available_soc = max(0, self.status.soc_percent - self.config.reserve_soc)
        available_wh = (available_soc / 100) * self._capacity_wh

        return available_wh / power_consumption_w

    def predict_soc_at_time(self, hours_ahead: float, power_consumption_w: float) -> float:
        """
        Predict SOC at future time.

        Args:
            hours_ahead: Hours in the future
            power_consumption_w: Expected power consumption (watts)

        Returns:
            Predicted SOC percentage
        """
        energy_consumed = power_consumption_w * hours_ahead
        remaining_wh = self.status.energy_remaining_wh - energy_consumed
        return max(0, (remaining_wh / self._capacity_wh) * 100)

    def get_safe_power_budget(self, mission_hours: float) -> float:
        """
        Calculate safe power budget for mission duration.

        Args:
            mission_hours: Planned mission duration

        Returns:
            Maximum safe power consumption (watts)
        """
        # Available energy (with reserve)
        available_soc = max(0, self.status.soc_percent - self.config.reserve_soc)
        available_wh = (available_soc / 100) * self._capacity_wh

        if mission_hours <= 0:
            return available_wh  # Immediate available

        return available_wh / mission_hours

    def can_complete_rtl(self, rtl_energy_wh: float) -> bool:
        """
        Check if there's enough energy for RTL.

        Args:
            rtl_energy_wh: Estimated RTL energy requirement

        Returns:
            True if RTL is feasible
        """
        return self.status.energy_remaining_wh > rtl_energy_wh

    def get_status_dict(self) -> dict:
        """Get status as dictionary for logging/telemetry."""
        return {
            "state": self.status.state.value,
            "voltage": round(self.status.voltage, 2),
            "current": round(self.status.current, 2),
            "soc_percent": round(self.status.soc_percent, 1),
            "temperature": round(self.status.temperature, 1),
            "power_watts": round(self.status.power_watts, 1),
            "energy_remaining_wh": round(self.status.energy_remaining_wh, 1),
            "time_remaining_hours": round(self.status.time_remaining_hours, 2),
        }
