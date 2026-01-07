"""
Flight controller interface for ArduPilot.

Provides high-level control commands and monitoring.
"""

import asyncio
from dataclasses import dataclass
from typing import Optional, Tuple

import structlog
from pymavlink import mavutil

from ..mavlink import MAVLinkConnection, MessageHandler, TelemetryData
from ..mavlink.parameters import ParameterManager
from .modes import FlightMode, ModeManager
from .navigation import NavigationController, Waypoint

logger = structlog.get_logger()


@dataclass
class FlightLimits:
    """Flight envelope limits."""
    altitude_min_m: float = 18000.0
    altitude_max_m: float = 25000.0
    airspeed_min_ms: float = 15.0
    airspeed_max_ms: float = 30.0
    bank_angle_max_deg: float = 45.0
    pitch_max_deg: float = 20.0
    pitch_min_deg: float = -15.0


class FlightController:
    """
    High-level flight controller for HAPS glider.

    Provides:
    - Arming/disarming
    - Flight mode management
    - Position and altitude control
    - Navigation commands
    - Telemetry monitoring
    """

    def __init__(self, connection: MAVLinkConnection):
        """
        Initialize flight controller.

        Args:
            connection: MAVLinkConnection instance
        """
        self.connection = connection
        self.message_handler = MessageHandler()
        self.params = ParameterManager(connection)
        self.modes = ModeManager(connection)
        self.nav = NavigationController(connection)

        self.limits = FlightLimits()
        self._armed = False

        # Register message handler
        connection.register_all_messages_callback(self.message_handler.handle_message)

    @property
    def telemetry(self) -> TelemetryData:
        """Get current telemetry data."""
        return self.message_handler.telemetry

    @property
    def is_armed(self) -> bool:
        """Check if vehicle is armed."""
        return self.telemetry.system.armed

    @property
    def current_mode(self) -> str:
        """Get current flight mode name."""
        return self.telemetry.system.mode

    @property
    def position(self) -> Tuple[float, float, float]:
        """Get current position (lat, lon, alt_msl)."""
        pos = self.telemetry.position
        return (pos.lat, pos.lon, pos.alt_msl)

    @property
    def altitude(self) -> float:
        """Get current altitude MSL in meters."""
        return self.telemetry.position.alt_msl

    @property
    def airspeed(self) -> float:
        """Get current airspeed in m/s."""
        return self.telemetry.velocity.airspeed

    @property
    def groundspeed(self) -> float:
        """Get current groundspeed in m/s."""
        return self.telemetry.velocity.groundspeed

    @property
    def heading(self) -> float:
        """Get current heading in degrees."""
        return self.telemetry.velocity.heading

    async def arm(self, force: bool = False) -> bool:
        """
        Arm the vehicle.

        Args:
            force: Force arming even with pre-arm check failures

        Returns:
            True if arm command accepted
        """
        if self.is_armed:
            logger.info("Already armed")
            return True

        logger.info("Arming vehicle", force=force)

        param = 21196 if force else 0

        success = self.connection.send_command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            param1=1,  # Arm
            param2=param,  # Force flag
        )

        if success:
            # Wait for arm confirmation
            for _ in range(50):  # 5 second timeout
                await asyncio.sleep(0.1)
                if self.is_armed:
                    logger.info("Vehicle armed")
                    return True

        logger.error("Arm failed")
        return False

    async def disarm(self, force: bool = False) -> bool:
        """
        Disarm the vehicle.

        Args:
            force: Force disarm even while flying

        Returns:
            True if disarm command accepted
        """
        if not self.is_armed:
            logger.info("Already disarmed")
            return True

        logger.info("Disarming vehicle", force=force)

        param = 21196 if force else 0

        success = self.connection.send_command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            param1=0,  # Disarm
            param2=param,
        )

        if success:
            for _ in range(50):
                await asyncio.sleep(0.1)
                if not self.is_armed:
                    logger.info("Vehicle disarmed")
                    return True

        logger.error("Disarm failed")
        return False

    async def set_mode(self, mode: FlightMode) -> bool:
        """
        Set flight mode.

        Args:
            mode: Target flight mode

        Returns:
            True if mode change successful
        """
        return await self.modes.set_mode(mode)

    async def goto(
        self,
        lat: float,
        lon: float,
        alt: float,
        groundspeed: Optional[float] = None,
    ) -> bool:
        """
        Navigate to position (requires GUIDED mode).

        Args:
            lat: Target latitude (degrees)
            lon: Target longitude (degrees)
            alt: Target altitude MSL (meters)
            groundspeed: Optional groundspeed (m/s)

        Returns:
            True if command accepted
        """
        # Enforce altitude limits
        alt = max(self.limits.altitude_min_m, min(alt, self.limits.altitude_max_m))

        if self.current_mode != "GUIDED":
            logger.warning("Not in GUIDED mode, switching...")
            if not await self.set_mode(FlightMode.GUIDED):
                return False

        return await self.nav.goto(lat, lon, alt, groundspeed)

    async def loiter_at(
        self,
        lat: float,
        lon: float,
        alt: float,
        radius: float = 1000.0,
    ) -> bool:
        """
        Loiter at position.

        Args:
            lat: Center latitude
            lon: Center longitude
            alt: Loiter altitude MSL
            radius: Loiter radius in meters

        Returns:
            True if command accepted
        """
        alt = max(self.limits.altitude_min_m, min(alt, self.limits.altitude_max_m))

        if self.current_mode != "GUIDED":
            if not await self.set_mode(FlightMode.GUIDED):
                return False

        return await self.nav.loiter_at(lat, lon, alt, radius)

    async def set_altitude(self, alt: float) -> bool:
        """
        Change altitude while maintaining current position.

        Args:
            alt: Target altitude MSL (meters)

        Returns:
            True if command accepted
        """
        alt = max(self.limits.altitude_min_m, min(alt, self.limits.altitude_max_m))

        pos = self.telemetry.position
        return await self.goto(pos.lat, pos.lon, alt)

    async def set_airspeed(self, airspeed: float) -> bool:
        """
        Set target airspeed.

        Args:
            airspeed: Target airspeed (m/s)

        Returns:
            True if command accepted
        """
        airspeed = max(
            self.limits.airspeed_min_ms,
            min(airspeed, self.limits.airspeed_max_ms)
        )

        success = self.connection.send_command_long(
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            param1=0,  # Airspeed
            param2=airspeed,
            param3=-1,  # No throttle change
        )

        if success:
            logger.info("Set airspeed", airspeed=airspeed)

        return success

    async def return_to_launch(self) -> bool:
        """
        Initiate return to launch.

        Returns:
            True if RTL initiated
        """
        logger.info("Initiating RTL")
        return await self.set_mode(FlightMode.RTL)

    async def set_home(
        self,
        lat: Optional[float] = None,
        lon: Optional[float] = None,
        alt: Optional[float] = None,
    ) -> bool:
        """
        Set home position.

        Args:
            lat: Home latitude (None = current)
            lon: Home longitude (None = current)
            alt: Home altitude (None = current)

        Returns:
            True if home set successfully
        """
        if lat is None:
            # Use current position
            return self.connection.send_command_long(
                mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                param1=1,  # Use current location
            )
        else:
            return self.connection.send_command_long(
                mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                param1=0,  # Use specified location
                param5=lat,
                param6=lon,
                param7=alt or 0,
            )

    def request_data_streams(self, rate_hz: int = 4) -> None:
        """
        Request telemetry data streams at specified rate.

        Args:
            rate_hz: Stream rate in Hz
        """
        streams = [
            mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,
        ]

        for stream in streams:
            self.connection._connection.mav.request_data_stream_send(
                self.connection.target_system,
                self.connection.target_component,
                stream,
                rate_hz,
                1,  # Start
            )

        logger.info("Requested data streams", rate_hz=rate_hz)

    async def wait_for_position(
        self,
        lat: float,
        lon: float,
        tolerance_m: float = 100.0,
        timeout: float = 300.0,
    ) -> bool:
        """
        Wait until vehicle reaches target position.

        Args:
            lat: Target latitude
            lon: Target longitude
            tolerance_m: Position tolerance in meters
            timeout: Maximum wait time in seconds

        Returns:
            True if position reached within timeout
        """
        import math

        start_time = asyncio.get_event_loop().time()

        while (asyncio.get_event_loop().time() - start_time) < timeout:
            current_lat = self.telemetry.position.lat
            current_lon = self.telemetry.position.lon

            # Calculate distance (simple approximation)
            dlat = (lat - current_lat) * 111320
            dlon = (lon - current_lon) * 111320 * math.cos(math.radians(lat))
            distance = math.sqrt(dlat**2 + dlon**2)

            if distance < tolerance_m:
                return True

            await asyncio.sleep(1.0)

        return False

    async def wait_for_altitude(
        self,
        alt: float,
        tolerance_m: float = 50.0,
        timeout: float = 300.0,
    ) -> bool:
        """
        Wait until vehicle reaches target altitude.

        Args:
            alt: Target altitude MSL (meters)
            tolerance_m: Altitude tolerance in meters
            timeout: Maximum wait time in seconds

        Returns:
            True if altitude reached within timeout
        """
        start_time = asyncio.get_event_loop().time()

        while (asyncio.get_event_loop().time() - start_time) < timeout:
            current_alt = self.telemetry.position.alt_msl
            if abs(current_alt - alt) < tolerance_m:
                return True
            await asyncio.sleep(1.0)

        return False

    def get_status_summary(self) -> dict:
        """Get summary of current flight status."""
        t = self.telemetry
        return {
            "armed": t.system.armed,
            "mode": t.system.mode,
            "position": {
                "lat": t.position.lat,
                "lon": t.position.lon,
                "alt_msl": t.position.alt_msl,
            },
            "velocity": {
                "airspeed": t.velocity.airspeed,
                "groundspeed": t.velocity.groundspeed,
                "climb_rate": t.velocity.climb_rate,
                "heading": t.velocity.heading,
            },
            "battery": {
                "voltage": t.battery.voltage,
                "current": t.battery.current,
                "remaining": t.battery.remaining,
            },
            "wind": {
                "speed": t.wind.speed,
                "direction": t.wind.direction,
            },
        }
