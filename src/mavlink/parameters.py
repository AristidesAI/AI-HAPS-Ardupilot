"""
ArduPilot parameter management.
"""

import asyncio
import time
from dataclasses import dataclass
from typing import Dict, Optional

import structlog
from pymavlink import mavutil

logger = structlog.get_logger()


@dataclass
class Parameter:
    """ArduPilot parameter."""
    name: str
    value: float
    param_type: int
    index: int
    timestamp: float = 0.0


# HAPS-specific ArduPilot parameters
HAPS_DEFAULT_PARAMS = {
    # Flight Mode Configuration
    "FLTMODE1": 10,  # AUTO
    "FLTMODE2": 11,  # RTL
    "FLTMODE3": 5,   # FBWB
    "FLTMODE6": 15,  # GUIDED

    # Glider-Specific Parameters
    "SOARING_ENABLE": 1,
    "SOAR_ALT_MAX": 25000,  # meters
    "SOAR_ALT_MIN": 18000,  # meters
    "SOAR_ALT_CUTOFF": 15000,

    # Airspeed Configuration
    "ARSPD_FBW_MIN": 15,  # m/s
    "ARSPD_FBW_MAX": 30,  # m/s
    "TRIM_ARSPD_CM": 2000,  # 20 m/s cruise

    # Throttle Limits
    "THR_MAX": 80,
    "THR_MIN": 0,

    # TECS Parameters for Glider
    "TECS_SINK_MAX": 0.5,
    "TECS_CLMB_MAX": 3.0,
    "TECS_TIME_CONST": 8.0,
    "TECS_SPDWEIGHT": 1.0,

    # Navigation
    "NAVL1_PERIOD": 25,
    "WP_RADIUS": 200,
    "WP_LOITER_RAD": 1000,

    # Pitch Tuning
    "PTCH2SRV_TCONST": 0.8,
    "PTCH_LIM_MAX_DEG": 20,
    "PTCH_LIM_MIN_DEG": -15,

    # Roll Limits
    "LIM_ROLL_CD": 4500,  # 45 degrees max bank

    # Geofencing
    "FENCE_ENABLE": 1,
    "FENCE_TYPE": 7,  # altitude + circle
    "FENCE_ALT_MAX": 26000,
    "FENCE_ALT_MIN": 17000,
    "FENCE_RADIUS": 50000,  # 50km
    "FENCE_ACTION": 1,  # RTL on breach

    # Battery Configuration
    "BATT_MONITOR": 4,
    "BATT_CAPACITY": 50000,  # 50Ah
    "BATT_CRT_VOLT": 42.0,
    "BATT_LOW_VOLT": 46.8,
    "BATT_FS_LOW_ACT": 2,  # RTL
    "BATT_FS_CRT_ACT": 1,  # LAND

    # Telemetry Rates
    "SR1_EXT_STAT": 5,
    "SR1_POSITION": 5,
    "SR1_RAW_SENS": 2,
}


class ParameterManager:
    """
    Manages ArduPilot parameter read/write operations.
    """

    def __init__(self, connection):
        """
        Initialize parameter manager.

        Args:
            connection: MAVLinkConnection instance
        """
        self.connection = connection
        self._params: Dict[str, Parameter] = {}
        self._fetch_complete = asyncio.Event()
        self._param_received = asyncio.Event()
        self._total_params = 0
        self._received_params = 0

    @property
    def params(self) -> Dict[str, Parameter]:
        """Get all cached parameters."""
        return self._params.copy()

    def get(self, name: str) -> Optional[float]:
        """
        Get cached parameter value.

        Args:
            name: Parameter name

        Returns:
            Parameter value or None if not cached
        """
        param = self._params.get(name)
        return param.value if param else None

    async def fetch_all(self, timeout: float = 60.0) -> bool:
        """
        Fetch all parameters from vehicle.

        Args:
            timeout: Maximum time to wait for all parameters

        Returns:
            True if all parameters received
        """
        if not self.connection.is_connected:
            logger.error("Cannot fetch parameters - not connected")
            return False

        self._params.clear()
        self._fetch_complete.clear()
        self._received_params = 0
        self._total_params = 0

        # Register callback for parameter messages
        self.connection.register_callback("PARAM_VALUE", self._handle_param_value)

        # Request parameter list
        self.connection._connection.mav.param_request_list_send(
            self.connection.target_system,
            self.connection.target_component,
        )

        logger.info("Requesting parameter list...")

        try:
            await asyncio.wait_for(self._fetch_complete.wait(), timeout)
            logger.info(
                "Parameter fetch complete",
                count=len(self._params),
            )
            return True
        except asyncio.TimeoutError:
            logger.warning(
                "Parameter fetch timeout",
                received=self._received_params,
                total=self._total_params,
            )
            return False

    async def read(self, name: str, timeout: float = 5.0) -> Optional[float]:
        """
        Read a single parameter from vehicle.

        Args:
            name: Parameter name
            timeout: Maximum time to wait

        Returns:
            Parameter value or None on failure
        """
        if not self.connection.is_connected:
            return None

        self._param_received.clear()

        # Register temporary callback
        received_value = None

        def callback(msg):
            nonlocal received_value
            if msg.param_id.rstrip('\x00') == name:
                received_value = msg.param_value
                self._param_received.set()

        self.connection.register_callback("PARAM_VALUE", callback)

        # Request parameter
        self.connection._connection.mav.param_request_read_send(
            self.connection.target_system,
            self.connection.target_component,
            name.encode('utf-8'),
            -1,  # Use name, not index
        )

        try:
            await asyncio.wait_for(self._param_received.wait(), timeout)
            return received_value
        except asyncio.TimeoutError:
            logger.warning("Parameter read timeout", name=name)
            return None

    async def write(self, name: str, value: float, timeout: float = 5.0) -> bool:
        """
        Write a parameter to vehicle.

        Args:
            name: Parameter name
            value: Parameter value
            timeout: Maximum time to wait for confirmation

        Returns:
            True if write confirmed
        """
        if not self.connection.is_connected:
            return False

        self._param_received.clear()
        confirmed = False

        def callback(msg):
            nonlocal confirmed
            if msg.param_id.rstrip('\x00') == name:
                if abs(msg.param_value - value) < 0.0001:
                    confirmed = True
                    self._param_received.set()

        self.connection.register_callback("PARAM_VALUE", callback)

        # Determine parameter type (default to float)
        param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32

        self.connection._connection.mav.param_set_send(
            self.connection.target_system,
            self.connection.target_component,
            name.encode('utf-8'),
            value,
            param_type,
        )

        try:
            await asyncio.wait_for(self._param_received.wait(), timeout)
            if confirmed:
                logger.info("Parameter set", name=name, value=value)
                # Update cache
                self._params[name] = Parameter(
                    name=name,
                    value=value,
                    param_type=param_type,
                    index=-1,
                    timestamp=time.time(),
                )
            return confirmed
        except asyncio.TimeoutError:
            logger.warning("Parameter write timeout", name=name)
            return False

    async def apply_haps_defaults(self) -> dict[str, bool]:
        """
        Apply HAPS-specific default parameters.

        Returns:
            Dict of parameter name -> success status
        """
        results = {}

        for name, value in HAPS_DEFAULT_PARAMS.items():
            success = await self.write(name, value)
            results[name] = success

            if not success:
                logger.warning("Failed to set parameter", name=name, value=value)

            # Small delay between writes
            await asyncio.sleep(0.1)

        success_count = sum(1 for v in results.values() if v)
        logger.info(
            "Applied HAPS parameters",
            success=success_count,
            total=len(results),
        )

        return results

    def _handle_param_value(self, msg) -> None:
        """Handle PARAM_VALUE message."""
        name = msg.param_id.rstrip('\x00')

        self._params[name] = Parameter(
            name=name,
            value=msg.param_value,
            param_type=msg.param_type,
            index=msg.param_index,
            timestamp=time.time(),
        )

        self._total_params = msg.param_count
        self._received_params = len(self._params)

        if self._received_params >= self._total_params and self._total_params > 0:
            self._fetch_complete.set()

    def export_to_file(self, filepath: str) -> None:
        """Export parameters to file."""
        with open(filepath, 'w') as f:
            for name, param in sorted(self._params.items()):
                f.write(f"{name},{param.value}\n")

    def import_from_file(self, filepath: str) -> dict[str, float]:
        """
        Import parameters from file.

        Returns:
            Dict of parameter name -> value
        """
        params = {}
        with open(filepath, 'r') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#'):
                    parts = line.split(',')
                    if len(parts) >= 2:
                        name = parts[0].strip()
                        value = float(parts[1].strip())
                        params[name] = value
        return params
