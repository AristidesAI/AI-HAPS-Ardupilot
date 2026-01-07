"""
Main AI Agent for HAPS Glider autonomous operations.

Coordinates all subsystems for multi-day stratospheric flight.
"""

import asyncio
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional

import structlog

from ..config import HAPSConfig
from ..energy import EnergyOptimizer, BatteryManager, SolarManager
from ..flight import FlightController, FlightMode
from ..station_keeping import LoiterController, WindEstimator, AltitudeController
from .state_machine import StateMachine, MissionState, MissionContext

logger = structlog.get_logger()


class AgentState(Enum):
    """AI Agent operational state."""
    INITIALIZING = "initializing"
    RUNNING = "running"
    PAUSED = "paused"
    STOPPING = "stopping"
    STOPPED = "stopped"
    ERROR = "error"


@dataclass
class AgentConfig:
    """AI Agent configuration."""
    update_rate_hz: float = 10.0  # Main loop rate
    decision_rate_hz: float = 1.0  # Decision-making rate
    telemetry_log_rate_hz: float = 0.1  # Status logging rate

    # Autonomous behavior
    autonomous_enabled: bool = True
    auto_rtl_on_critical: bool = True
    auto_mode_switching: bool = True

    # Safety limits
    max_altitude_m: float = 25000.0
    min_altitude_m: float = 18000.0
    max_distance_from_home_m: float = 50000.0


class HAPSAgent:
    """
    Main AI Agent for autonomous HAPS operations.

    Coordinates:
    - Flight control
    - Energy management
    - Station keeping
    - Mission state machine
    - Safety monitoring
    """

    def __init__(
        self,
        flight_controller: FlightController,
        config: Optional[AgentConfig] = None,
    ):
        """
        Initialize HAPS Agent.

        Args:
            flight_controller: Flight controller instance
            config: Agent configuration
        """
        self.flight = flight_controller
        self.config = config or AgentConfig()

        # Initialize subsystems
        self.battery = BatteryManager()
        self.solar = SolarManager()
        self.energy = EnergyOptimizer(self.battery, self.solar)
        self.loiter = LoiterController()
        self.wind = WindEstimator()
        self.altitude = AltitudeController()
        self.state_machine = StateMachine()

        # Agent state
        self._state = AgentState.INITIALIZING
        self._running = False
        self._last_decision_time = 0.0
        self._last_log_time = 0.0

        # Mission target
        self._target_lat: Optional[float] = None
        self._target_lon: Optional[float] = None
        self._target_alt: Optional[float] = None

        # Task handles
        self._main_task: Optional[asyncio.Task] = None

        logger.info("HAPS Agent initialized")

    @property
    def state(self) -> AgentState:
        """Get agent state."""
        return self._state

    @property
    def mission_state(self) -> MissionState:
        """Get current mission state."""
        return self.state_machine.state

    async def start(self) -> bool:
        """
        Start the AI agent.

        Returns:
            True if started successfully
        """
        if self._state == AgentState.RUNNING:
            logger.warning("Agent already running")
            return False

        logger.info("Starting HAPS Agent")
        self._state = AgentState.RUNNING
        self._running = True

        # Request telemetry streams
        self.flight.request_data_streams(rate_hz=4)

        # Start main loop
        self._main_task = asyncio.create_task(self._main_loop())

        return True

    async def stop(self) -> None:
        """Stop the AI agent."""
        logger.info("Stopping HAPS Agent")
        self._state = AgentState.STOPPING
        self._running = False

        if self._main_task:
            self._main_task.cancel()
            try:
                await self._main_task
            except asyncio.CancelledError:
                pass

        self._state = AgentState.STOPPED

    async def pause(self) -> None:
        """Pause autonomous operations."""
        if self._state == AgentState.RUNNING:
            self._state = AgentState.PAUSED
            logger.info("Agent paused")

    async def resume(self) -> None:
        """Resume autonomous operations."""
        if self._state == AgentState.PAUSED:
            self._state = AgentState.RUNNING
            logger.info("Agent resumed")

    def set_target(self, lat: float, lon: float, alt: float) -> None:
        """
        Set mission target location.

        Args:
            lat: Target latitude
            lon: Target longitude
            alt: Target altitude (MSL meters)
        """
        self._target_lat = lat
        self._target_lon = lon
        self._target_alt = alt

        logger.info(
            "Target set",
            lat=lat,
            lon=lon,
            alt=alt,
        )

    async def _main_loop(self) -> None:
        """Main agent control loop."""
        update_interval = 1.0 / self.config.update_rate_hz
        decision_interval = 1.0 / self.config.decision_rate_hz
        log_interval = 1.0 / self.config.telemetry_log_rate_hz

        logger.info("Main loop started")

        while self._running:
            try:
                now = time.time()

                # Update subsystems
                await self._update_subsystems()

                # Make decisions at lower rate
                if now - self._last_decision_time >= decision_interval:
                    if self._state == AgentState.RUNNING:
                        await self._make_decisions()
                    self._last_decision_time = now

                # Log status at low rate
                if now - self._last_log_time >= log_interval:
                    self._log_status()
                    self._last_log_time = now

                # Sleep until next update
                await asyncio.sleep(update_interval)

            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error("Main loop error", error=str(e))
                self._state = AgentState.ERROR
                await asyncio.sleep(1)

        logger.info("Main loop stopped")

    async def _update_subsystems(self) -> None:
        """Update all subsystems with current telemetry."""
        telem = self.flight.telemetry

        # Update battery manager
        self.battery.update(
            voltage=telem.battery.voltage,
            current=telem.battery.current,
            soc_percent=telem.battery.remaining,
            temperature=telem.battery.temperature,
        )

        # Update solar manager
        self.solar.set_location(
            telem.position.lat,
            telem.position.lon,
            telem.position.alt_msl,
        )
        self.solar.set_heading(telem.velocity.heading)
        self.solar.update()

        # Update energy optimizer
        self.energy.update(
            telem.position.lat,
            telem.position.lon,
            telem.position.alt_msl,
        )

        # Update wind estimator
        self.wind.update_from_telemetry(
            telem.wind.speed,
            telem.wind.direction,
            altitude=telem.position.alt_msl,
        )

        # Update altitude controller
        power_balance = (
            self.solar.status.power_available_w if self.solar.status else 0
        ) - self.config.update_rate_hz * 15  # Approximate consumption
        self.altitude.update(
            telem.position.alt_msl,
            telem.velocity.climb_rate,
            power_balance,
        )

        # Update loiter controller if active
        if self.loiter.is_active:
            correction = self.loiter.update(
                telem.position.lat,
                telem.position.lon,
                telem.velocity.heading,
            )
            if correction[0] is not None:
                # Apply drift correction
                await self.flight.goto(
                    correction[0],
                    correction[1],
                    self.loiter.state.target_alt_m,
                )

    async def _make_decisions(self) -> None:
        """Make autonomous decisions based on current state."""
        if not self.config.autonomous_enabled:
            return

        telem = self.flight.telemetry

        # Build mission context
        context = MissionContext(
            latitude=telem.position.lat,
            longitude=telem.position.lon,
            altitude_m=telem.position.alt_msl,
            battery_soc=self.battery.soc,
            solar_power_w=self.solar.status.power_available_w if self.solar.status else 0,
            power_balance_w=self.energy.power_budget.reserve_w,
            is_daylight=self.energy.mode.value.startswith("day"),
            wind_speed_ms=self.wind.wind_speed,
            wind_direction_deg=self.wind.wind_direction,
            is_armed=telem.system.armed,
            flight_mode=telem.system.mode,
            gps_fix=True,  # Assume valid if we have position
            link_quality=1.0,  # Would come from link status
            distance_to_target_m=self._calculate_distance_to_target(),
        )

        # Evaluate state machine
        recommended_state = self.state_machine.evaluate(context)

        if recommended_state:
            # Transition to new state
            self.state_machine.transition(
                recommended_state,
                reason="Automatic evaluation",
            )
            # Execute state entry actions
            await self._execute_state_actions(recommended_state, context)

        # Execute ongoing state behaviors
        await self._execute_state_behavior(context)

    async def _execute_state_actions(
        self,
        new_state: MissionState,
        context: MissionContext,
    ) -> None:
        """Execute actions when entering a new state."""
        if new_state == MissionState.STATION_KEEPING:
            # Start loiter at target or current position
            if self._target_lat and self._target_lon:
                self.loiter.start_loiter(
                    self._target_lat,
                    self._target_lon,
                    self._target_alt or 20000,
                )
            else:
                self.loiter.start_loiter(
                    context.latitude,
                    context.longitude,
                    context.altitude_m,
                )

            if self.config.auto_mode_switching:
                await self.flight.set_mode(FlightMode.GUIDED)

        elif new_state == MissionState.RTL:
            self.loiter.stop_loiter()
            await self.flight.return_to_launch()

        elif new_state == MissionState.NIGHT_CRUISE:
            # Reduce operations
            logger.info("Entering night cruise mode")
            # Adjust airspeed for efficiency
            factor = self.energy.get_optimal_airspeed_factor()
            cruise_speed = self.flight.limits.airspeed_min_ms / factor
            await self.flight.set_airspeed(cruise_speed)

        elif new_state == MissionState.ENERGY_RECOVERY:
            logger.info("Entering energy recovery mode")
            # Optimize heading for solar
            heading_adj = self.energy.get_optimal_heading_adjustment()
            if abs(heading_adj) > 30:
                logger.info(f"Adjusting heading by {heading_adj:.0f}° for solar")

        elif new_state == MissionState.EMERGENCY_DESCENT:
            logger.warning("Emergency descent initiated!")
            self.loiter.stop_loiter()
            # Set minimum safe altitude
            min_alt = self.altitude.get_safe_descent_altitude()
            await self.flight.set_altitude(min_alt)

    async def _execute_state_behavior(self, context: MissionContext) -> None:
        """Execute ongoing behavior for current state."""
        state = self.state_machine.state

        if state == MissionState.STATION_KEEPING:
            # Adjust for wind
            wind_speed, wind_dir = self.loiter.get_wind_estimate_from_drift()
            if wind_speed > 5:
                self.loiter.adjust_pattern_for_wind(wind_speed, wind_dir)

            # Energy-optimized altitude
            alt_recommendation = self.altitude.optimize_for_energy(
                self.energy.power_budget.reserve_w,
                self.battery.soc,
                context.is_daylight,
            )
            if alt_recommendation:
                new_alt = context.altitude_m + alt_recommendation
                if self.altitude.set_target(new_alt):
                    self.loiter.state.target_alt_m = new_alt

        elif state == MissionState.NIGHT_CRUISE:
            # Monitor battery closely
            if self.battery.is_critical:
                if self.config.auto_rtl_on_critical:
                    self.state_machine.transition(
                        MissionState.RTL,
                        reason="Critical battery during night",
                    )

            # Consider altitude descent to save energy
            should_descend, descent_m = self.energy.should_reduce_altitude()
            if should_descend:
                current_target = self.altitude.target_altitude
                new_target = current_target + descent_m
                self.altitude.set_target(new_target)

        elif state == MissionState.TRANSIT:
            # Check if arrived at target
            if context.distance_to_target_m < 500:
                self.state_machine.transition(
                    MissionState.STATION_KEEPING,
                    reason="Arrived at target",
                )

    def _calculate_distance_to_target(self) -> float:
        """Calculate distance to target in meters."""
        if not self._target_lat or not self._target_lon:
            return 0.0

        telem = self.flight.telemetry
        import math

        lat1 = math.radians(telem.position.lat)
        lat2 = math.radians(self._target_lat)
        dlat = lat2 - lat1
        dlon = math.radians(self._target_lon - telem.position.lon)

        a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return 6371000 * c

    def _log_status(self) -> None:
        """Log current status."""
        logger.info(
            "Agent status",
            agent_state=self._state.value,
            mission_state=self.state_machine.state.name,
            soc=round(self.battery.soc, 1),
            solar_w=round(self.solar.status.power_available_w, 1) if self.solar.status else 0,
            altitude_m=round(self.flight.telemetry.position.alt_msl, 0),
            wind_ms=round(self.wind.wind_speed, 1),
        )

    def get_status(self) -> dict:
        """Get comprehensive agent status."""
        return {
            "agent_state": self._state.value,
            "mission": self.state_machine.get_status_dict(),
            "energy": self.energy.get_status_dict(),
            "altitude": self.altitude.get_status_dict(),
            "loiter": self.loiter.get_status_dict(),
            "flight": self.flight.get_status_summary(),
            "recommendations": self.energy.get_recommended_actions(),
        }

    async def command_goto(self, lat: float, lon: float, alt: float) -> bool:
        """
        Command agent to go to location.

        Args:
            lat: Target latitude
            lon: Target longitude
            alt: Target altitude

        Returns:
            True if command accepted
        """
        self.set_target(lat, lon, alt)

        # Stop current loiter
        self.loiter.stop_loiter()

        # Transition to transit state
        self.state_machine.transition(
            MissionState.TRANSIT,
            reason="Goto command",
        )

        # Send goto command
        return await self.flight.goto(lat, lon, alt)

    async def command_loiter_here(self) -> bool:
        """Command agent to loiter at current position."""
        telem = self.flight.telemetry

        self.set_target(
            telem.position.lat,
            telem.position.lon,
            telem.position.alt_msl,
        )

        self.state_machine.transition(
            MissionState.STATION_KEEPING,
            reason="Loiter here command",
        )

        return True

    async def command_rtl(self) -> bool:
        """Command return to launch."""
        self.state_machine.transition(
            MissionState.RTL,
            reason="RTL command",
        )

        return await self.flight.return_to_launch()
