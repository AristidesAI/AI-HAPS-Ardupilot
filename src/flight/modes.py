"""
Flight mode management for ArduPlane.
"""

import asyncio
from enum import IntEnum

import structlog
from pymavlink import mavutil

logger = structlog.get_logger()


class FlightMode(IntEnum):
    """ArduPlane flight modes."""
    MANUAL = 0
    CIRCLE = 1
    STABILIZE = 2
    TRAINING = 3
    ACRO = 4
    FBWA = 5
    FBWB = 6
    CRUISE = 7
    AUTOTUNE = 8
    AUTO = 10
    RTL = 11
    LOITER = 12
    TAKEOFF = 13
    AVOID_ADSB = 14
    GUIDED = 15
    QSTABILIZE = 17
    QHOVER = 18
    QLOITER = 19
    QLAND = 20
    QRTL = 21
    QAUTOTUNE = 22
    QACRO = 23
    THERMAL = 24
    LOITER_ALT_QLAND = 25


# Mode names for display
MODE_NAMES = {
    FlightMode.MANUAL: "MANUAL",
    FlightMode.CIRCLE: "CIRCLE",
    FlightMode.STABILIZE: "STABILIZE",
    FlightMode.TRAINING: "TRAINING",
    FlightMode.ACRO: "ACRO",
    FlightMode.FBWA: "FBWA",
    FlightMode.FBWB: "FBWB",
    FlightMode.CRUISE: "CRUISE",
    FlightMode.AUTOTUNE: "AUTOTUNE",
    FlightMode.AUTO: "AUTO",
    FlightMode.RTL: "RTL",
    FlightMode.LOITER: "LOITER",
    FlightMode.TAKEOFF: "TAKEOFF",
    FlightMode.AVOID_ADSB: "AVOID_ADSB",
    FlightMode.GUIDED: "GUIDED",
    FlightMode.QSTABILIZE: "QSTABILIZE",
    FlightMode.QHOVER: "QHOVER",
    FlightMode.QLOITER: "QLOITER",
    FlightMode.QLAND: "QLAND",
    FlightMode.QRTL: "QRTL",
    FlightMode.QAUTOTUNE: "QAUTOTUNE",
    FlightMode.QACRO: "QACRO",
    FlightMode.THERMAL: "THERMAL",
    FlightMode.LOITER_ALT_QLAND: "LOITER_ALT_QLAND",
}


# HAPS-relevant modes
HAPS_MODES = {
    FlightMode.AUTO,      # Autonomous waypoint following
    FlightMode.GUIDED,    # AI-controlled navigation
    FlightMode.LOITER,    # Station keeping
    FlightMode.RTL,       # Return to launch
    FlightMode.CRUISE,    # Cruise at altitude
    FlightMode.THERMAL,   # Thermal soaring
    FlightMode.FBWB,      # Manual backup
}


class ModeManager:
    """
    Manages flight mode transitions for ArduPlane.
    """

    def __init__(self, connection):
        """
        Initialize mode manager.

        Args:
            connection: MAVLinkConnection instance
        """
        self.connection = connection
        self._current_mode: FlightMode = FlightMode.MANUAL
        self._mode_change_event = asyncio.Event()

        # Register for heartbeat to track mode
        connection.register_callback("HEARTBEAT", self._handle_heartbeat)

    @property
    def current_mode(self) -> FlightMode:
        """Get current flight mode."""
        return self._current_mode

    @property
    def mode_name(self) -> str:
        """Get current mode name."""
        return MODE_NAMES.get(self._current_mode, f"MODE_{self._current_mode}")

    def _handle_heartbeat(self, msg) -> None:
        """Handle heartbeat to track mode changes."""
        if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            return

        new_mode = FlightMode(msg.custom_mode)
        if new_mode != self._current_mode:
            old_mode = self._current_mode
            self._current_mode = new_mode
            self._mode_change_event.set()
            logger.info(
                "Mode changed",
                old=MODE_NAMES.get(old_mode, str(old_mode)),
                new=MODE_NAMES.get(new_mode, str(new_mode)),
            )

    async def set_mode(self, mode: FlightMode, timeout: float = 10.0) -> bool:
        """
        Set flight mode.

        Args:
            mode: Target flight mode
            timeout: Maximum time to wait for mode change

        Returns:
            True if mode change confirmed
        """
        if self._current_mode == mode:
            logger.debug("Already in requested mode", mode=MODE_NAMES.get(mode))
            return True

        logger.info("Setting mode", target=MODE_NAMES.get(mode, str(mode)))

        self._mode_change_event.clear()

        # Send mode change command
        self.connection._connection.mav.set_mode_send(
            self.connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode.value,
        )

        # Wait for confirmation
        try:
            await asyncio.wait_for(
                self._wait_for_mode(mode),
                timeout=timeout,
            )
            return True
        except asyncio.TimeoutError:
            logger.error("Mode change timeout", target=MODE_NAMES.get(mode))
            return False

    async def _wait_for_mode(self, target_mode: FlightMode) -> None:
        """Wait for mode to change to target."""
        while self._current_mode != target_mode:
            self._mode_change_event.clear()
            await self._mode_change_event.wait()

    async def set_guided(self) -> bool:
        """Switch to GUIDED mode for AI control."""
        return await self.set_mode(FlightMode.GUIDED)

    async def set_auto(self) -> bool:
        """Switch to AUTO mode for mission execution."""
        return await self.set_mode(FlightMode.AUTO)

    async def set_loiter(self) -> bool:
        """Switch to LOITER mode for station keeping."""
        return await self.set_mode(FlightMode.LOITER)

    async def set_rtl(self) -> bool:
        """Switch to RTL mode for return to launch."""
        return await self.set_mode(FlightMode.RTL)

    async def set_thermal(self) -> bool:
        """Switch to THERMAL mode for soaring."""
        return await self.set_mode(FlightMode.THERMAL)

    async def set_cruise(self) -> bool:
        """Switch to CRUISE mode."""
        return await self.set_mode(FlightMode.CRUISE)

    def is_autonomous_mode(self) -> bool:
        """Check if current mode is autonomous (AI can control)."""
        return self._current_mode in {
            FlightMode.AUTO,
            FlightMode.GUIDED,
            FlightMode.LOITER,
            FlightMode.CRUISE,
            FlightMode.THERMAL,
        }

    def is_manual_mode(self) -> bool:
        """Check if current mode requires pilot input."""
        return self._current_mode in {
            FlightMode.MANUAL,
            FlightMode.STABILIZE,
            FlightMode.FBWA,
            FlightMode.FBWB,
            FlightMode.ACRO,
            FlightMode.TRAINING,
        }

    def is_emergency_mode(self) -> bool:
        """Check if in emergency/recovery mode."""
        return self._current_mode in {
            FlightMode.RTL,
            FlightMode.QLAND,
            FlightMode.QRTL,
        }

    @staticmethod
    def mode_from_name(name: str) -> FlightMode:
        """Get mode enum from name string."""
        name_upper = name.upper()
        for mode, mode_name in MODE_NAMES.items():
            if mode_name == name_upper:
                return mode
        raise ValueError(f"Unknown mode: {name}")

    @staticmethod
    def mode_to_name(mode: FlightMode) -> str:
        """Get name string from mode enum."""
        return MODE_NAMES.get(mode, f"MODE_{mode.value}")
