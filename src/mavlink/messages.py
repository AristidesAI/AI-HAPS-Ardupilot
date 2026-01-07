"""
MAVLink message handling and telemetry data structures.
"""

import time
from dataclasses import dataclass, field
from typing import Optional

import structlog
from pymavlink import mavutil

logger = structlog.get_logger()


@dataclass
class Position:
    """GPS position data."""
    lat: float = 0.0  # degrees
    lon: float = 0.0  # degrees
    alt_msl: float = 0.0  # meters above mean sea level
    alt_rel: float = 0.0  # meters above home
    timestamp: float = 0.0


@dataclass
class Attitude:
    """Vehicle attitude data."""
    roll: float = 0.0  # radians
    pitch: float = 0.0  # radians
    yaw: float = 0.0  # radians
    rollspeed: float = 0.0  # rad/s
    pitchspeed: float = 0.0  # rad/s
    yawspeed: float = 0.0  # rad/s
    timestamp: float = 0.0


@dataclass
class Velocity:
    """Velocity data."""
    vx: float = 0.0  # m/s north
    vy: float = 0.0  # m/s east
    vz: float = 0.0  # m/s down
    groundspeed: float = 0.0  # m/s
    airspeed: float = 0.0  # m/s
    climb_rate: float = 0.0  # m/s
    heading: float = 0.0  # degrees
    timestamp: float = 0.0


@dataclass
class BatteryStatus:
    """Battery state data."""
    voltage: float = 0.0  # volts
    current: float = 0.0  # amps
    remaining: int = 0  # percent
    consumed: float = 0.0  # mAh
    temperature: float = 0.0  # celsius
    timestamp: float = 0.0


@dataclass
class SystemStatus:
    """Overall system status."""
    armed: bool = False
    mode: str = "UNKNOWN"
    mode_num: int = 0
    system_status: int = 0
    cpu_load: float = 0.0
    errors_count: int = 0
    timestamp: float = 0.0


@dataclass
class WindEstimate:
    """Wind estimation data."""
    direction: float = 0.0  # degrees (direction wind is coming FROM)
    speed: float = 0.0  # m/s
    speed_z: float = 0.0  # m/s vertical
    timestamp: float = 0.0


@dataclass
class SolarStatus:
    """Solar panel status (custom telemetry)."""
    power_watts: float = 0.0
    voltage: float = 0.0
    current: float = 0.0
    efficiency: float = 0.0
    sun_elevation: float = 0.0
    sun_azimuth: float = 0.0
    timestamp: float = 0.0


@dataclass
class TelemetryData:
    """Complete telemetry state."""
    position: Position = field(default_factory=Position)
    attitude: Attitude = field(default_factory=Attitude)
    velocity: Velocity = field(default_factory=Velocity)
    battery: BatteryStatus = field(default_factory=BatteryStatus)
    system: SystemStatus = field(default_factory=SystemStatus)
    wind: WindEstimate = field(default_factory=WindEstimate)
    solar: SolarStatus = field(default_factory=SolarStatus)

    # Home position
    home_lat: float = 0.0
    home_lon: float = 0.0
    home_alt: float = 0.0

    def distance_to_home(self) -> float:
        """Calculate distance to home in meters."""
        if self.home_lat == 0 and self.home_lon == 0:
            return 0.0

        from math import radians, sin, cos, sqrt, atan2

        R = 6371000  # Earth radius in meters

        lat1 = radians(self.home_lat)
        lat2 = radians(self.position.lat)
        dlat = lat2 - lat1
        dlon = radians(self.position.lon - self.home_lon)

        a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))

        return R * c


# ArduPlane flight modes
PLANE_MODES = {
    0: "MANUAL",
    1: "CIRCLE",
    2: "STABILIZE",
    3: "TRAINING",
    4: "ACRO",
    5: "FBWA",
    6: "FBWB",
    7: "CRUISE",
    8: "AUTOTUNE",
    10: "AUTO",
    11: "RTL",
    12: "LOITER",
    13: "TAKEOFF",
    14: "AVOID_ADSB",
    15: "GUIDED",
    17: "QSTABILIZE",
    18: "QHOVER",
    19: "QLOITER",
    20: "QLAND",
    21: "QRTL",
    22: "QAUTOTUNE",
    23: "QACRO",
    24: "THERMAL",
    25: "LOITER_ALT_QLAND",
}


class MessageHandler:
    """
    Handles incoming MAVLink messages and updates telemetry state.
    """

    def __init__(self):
        self.telemetry = TelemetryData()
        self._message_counts: dict[str, int] = {}

    def handle_message(self, msg) -> None:
        """
        Process incoming MAVLink message and update telemetry.

        Args:
            msg: MAVLink message
        """
        msg_type = msg.get_type()
        self._message_counts[msg_type] = self._message_counts.get(msg_type, 0) + 1

        handler_name = f"_handle_{msg_type.lower()}"
        handler = getattr(self, handler_name, None)

        if handler:
            try:
                handler(msg)
            except Exception as e:
                logger.error(f"Error handling {msg_type}", error=str(e))

    def _handle_global_position_int(self, msg) -> None:
        """Handle GLOBAL_POSITION_INT message."""
        self.telemetry.position = Position(
            lat=msg.lat / 1e7,
            lon=msg.lon / 1e7,
            alt_msl=msg.alt / 1000.0,
            alt_rel=msg.relative_alt / 1000.0,
            timestamp=time.time(),
        )
        self.telemetry.velocity.vx = msg.vx / 100.0
        self.telemetry.velocity.vy = msg.vy / 100.0
        self.telemetry.velocity.vz = msg.vz / 100.0
        self.telemetry.velocity.heading = msg.hdg / 100.0

    def _handle_attitude(self, msg) -> None:
        """Handle ATTITUDE message."""
        self.telemetry.attitude = Attitude(
            roll=msg.roll,
            pitch=msg.pitch,
            yaw=msg.yaw,
            rollspeed=msg.rollspeed,
            pitchspeed=msg.pitchspeed,
            yawspeed=msg.yawspeed,
            timestamp=time.time(),
        )

    def _handle_vfr_hud(self, msg) -> None:
        """Handle VFR_HUD message."""
        self.telemetry.velocity.airspeed = msg.airspeed
        self.telemetry.velocity.groundspeed = msg.groundspeed
        self.telemetry.velocity.climb_rate = msg.climb
        self.telemetry.velocity.heading = msg.heading
        self.telemetry.velocity.timestamp = time.time()

    def _handle_sys_status(self, msg) -> None:
        """Handle SYS_STATUS message."""
        self.telemetry.battery.voltage = msg.voltage_battery / 1000.0
        self.telemetry.battery.current = msg.current_battery / 100.0
        self.telemetry.battery.remaining = msg.battery_remaining
        self.telemetry.battery.timestamp = time.time()
        self.telemetry.system.cpu_load = msg.load / 10.0
        self.telemetry.system.errors_count = msg.errors_count1

    def _handle_battery_status(self, msg) -> None:
        """Handle BATTERY_STATUS message."""
        if msg.voltages[0] != 65535:
            # Calculate total voltage from cell voltages
            total_voltage = sum(v for v in msg.voltages if v != 65535) / 1000.0
            self.telemetry.battery.voltage = total_voltage
        self.telemetry.battery.current = msg.current_battery / 100.0
        self.telemetry.battery.remaining = msg.battery_remaining
        self.telemetry.battery.consumed = msg.current_consumed
        self.telemetry.battery.temperature = msg.temperature / 100.0 if msg.temperature != 32767 else 0.0
        self.telemetry.battery.timestamp = time.time()

    def _handle_heartbeat(self, msg) -> None:
        """Handle HEARTBEAT message."""
        # Only process from autopilot, not GCS
        if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            return

        armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
        mode_name = PLANE_MODES.get(msg.custom_mode, f"MODE_{msg.custom_mode}")

        self.telemetry.system = SystemStatus(
            armed=armed,
            mode=mode_name,
            mode_num=msg.custom_mode,
            system_status=msg.system_status,
            cpu_load=self.telemetry.system.cpu_load,
            errors_count=self.telemetry.system.errors_count,
            timestamp=time.time(),
        )

    def _handle_wind(self, msg) -> None:
        """Handle WIND message."""
        self.telemetry.wind = WindEstimate(
            direction=msg.direction,
            speed=msg.speed,
            speed_z=msg.speed_z,
            timestamp=time.time(),
        )

    def _handle_home_position(self, msg) -> None:
        """Handle HOME_POSITION message."""
        self.telemetry.home_lat = msg.latitude / 1e7
        self.telemetry.home_lon = msg.longitude / 1e7
        self.telemetry.home_alt = msg.altitude / 1000.0

    def _handle_nav_controller_output(self, msg) -> None:
        """Handle NAV_CONTROLLER_OUTPUT for navigation info."""
        # Contains nav_roll, nav_pitch, nav_bearing, target_bearing, etc.
        pass

    def _handle_mission_current(self, msg) -> None:
        """Handle MISSION_CURRENT message."""
        # Track current mission waypoint
        pass

    def get_message_stats(self) -> dict[str, int]:
        """Get message reception statistics."""
        return self._message_counts.copy()


def mode_number_to_name(mode_num: int) -> str:
    """Convert mode number to name."""
    return PLANE_MODES.get(mode_num, f"MODE_{mode_num}")


def mode_name_to_number(mode_name: str) -> Optional[int]:
    """Convert mode name to number."""
    for num, name in PLANE_MODES.items():
        if name == mode_name:
            return num
    return None
