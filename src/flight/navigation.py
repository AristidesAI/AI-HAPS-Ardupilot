"""
Navigation controller for ArduPilot.

Handles waypoint management, guided navigation, and mission control.
"""

import asyncio
from dataclasses import dataclass, field
from enum import IntEnum
from typing import List, Optional

import structlog
from pymavlink import mavutil

logger = structlog.get_logger()


class WaypointType(IntEnum):
    """Mission waypoint types."""
    WAYPOINT = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
    LOITER_UNLIM = mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM
    LOITER_TURNS = mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS
    LOITER_TIME = mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME
    LOITER_TO_ALT = mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT
    RETURN_TO_LAUNCH = mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
    LAND = mavutil.mavlink.MAV_CMD_NAV_LAND
    TAKEOFF = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF


@dataclass
class Waypoint:
    """Mission waypoint definition."""
    lat: float  # degrees
    lon: float  # degrees
    alt: float  # meters MSL
    wp_type: WaypointType = WaypointType.WAYPOINT
    param1: float = 0  # Hold time (seconds) or turns
    param2: float = 0  # Acceptance radius (meters)
    param3: float = 0  # Pass through (0) or orbit (1)
    param4: float = 0  # Yaw angle
    seq: int = 0  # Sequence number


@dataclass
class Mission:
    """Complete mission definition."""
    waypoints: List[Waypoint] = field(default_factory=list)
    current_wp: int = 0
    name: str = ""

    def add_waypoint(
        self,
        lat: float,
        lon: float,
        alt: float,
        wp_type: WaypointType = WaypointType.WAYPOINT,
        hold_time: float = 0,
        acceptance_radius: float = 50,
    ) -> None:
        """Add waypoint to mission."""
        wp = Waypoint(
            lat=lat,
            lon=lon,
            alt=alt,
            wp_type=wp_type,
            param1=hold_time,
            param2=acceptance_radius,
            seq=len(self.waypoints),
        )
        self.waypoints.append(wp)

    def add_loiter(
        self,
        lat: float,
        lon: float,
        alt: float,
        radius: float = 1000,
        duration: float = 0,
    ) -> None:
        """Add loiter waypoint."""
        if duration > 0:
            wp_type = WaypointType.LOITER_TIME
            param1 = duration
        else:
            wp_type = WaypointType.LOITER_UNLIM
            param1 = 0

        wp = Waypoint(
            lat=lat,
            lon=lon,
            alt=alt,
            wp_type=wp_type,
            param1=param1,
            param3=radius,  # Loiter radius
            seq=len(self.waypoints),
        )
        self.waypoints.append(wp)

    def clear(self) -> None:
        """Clear all waypoints."""
        self.waypoints.clear()
        self.current_wp = 0


class NavigationController:
    """
    Handles navigation commands and mission management.
    """

    def __init__(self, connection):
        """
        Initialize navigation controller.

        Args:
            connection: MAVLinkConnection instance
        """
        self.connection = connection
        self.current_mission = Mission()
        self._mission_ack_event = asyncio.Event()
        self._mission_ack_result = 0

        # Register for mission messages
        connection.register_callback("MISSION_ACK", self._handle_mission_ack)
        connection.register_callback("MISSION_CURRENT", self._handle_mission_current)

    def _handle_mission_ack(self, msg) -> None:
        """Handle mission acknowledgement."""
        self._mission_ack_result = msg.type
        self._mission_ack_event.set()

    def _handle_mission_current(self, msg) -> None:
        """Handle current mission item update."""
        self.current_mission.current_wp = msg.seq

    async def goto(
        self,
        lat: float,
        lon: float,
        alt: float,
        groundspeed: Optional[float] = None,
    ) -> bool:
        """
        Navigate to position (GUIDED mode).

        Args:
            lat: Target latitude (degrees)
            lon: Target longitude (degrees)
            alt: Target altitude MSL (meters)
            groundspeed: Optional groundspeed (m/s)

        Returns:
            True if command sent successfully
        """
        logger.info("Goto position", lat=lat, lon=lon, alt=alt)

        # Use MISSION_ITEM_INT for better precision
        self.connection._connection.mav.mission_item_int_send(
            self.connection.target_system,
            self.connection.target_component,
            0,  # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            2,  # current = 2 means guided mode goto
            1,  # autocontinue
            0,  # param1 (hold time)
            50,  # param2 (acceptance radius)
            0,  # param3 (pass through)
            0,  # param4 (yaw)
            int(lat * 1e7),  # x (lat)
            int(lon * 1e7),  # y (lon)
            alt,  # z (alt)
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )

        # Set speed if specified
        if groundspeed:
            self.connection.send_command_long(
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                param1=1,  # Groundspeed
                param2=groundspeed,
                param3=-1,  # No throttle change
            )

        return True

    async def loiter_at(
        self,
        lat: float,
        lon: float,
        alt: float,
        radius: float = 1000.0,
        direction: int = 1,  # 1 = clockwise, -1 = counter-clockwise
    ) -> bool:
        """
        Loiter at position.

        Args:
            lat: Center latitude
            lon: Center longitude
            alt: Loiter altitude MSL
            radius: Loiter radius in meters
            direction: Turn direction (1=CW, -1=CCW)

        Returns:
            True if command sent
        """
        logger.info("Loiter at", lat=lat, lon=lon, alt=alt, radius=radius)

        # Set loiter radius parameter
        self.connection._connection.mav.param_set_send(
            self.connection.target_system,
            self.connection.target_component,
            b'WP_LOITER_RAD',
            radius * direction,  # Negative = CCW
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )

        # Navigate to loiter center
        self.connection._connection.mav.mission_item_int_send(
            self.connection.target_system,
            self.connection.target_component,
            0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
            2,  # current = 2 for guided
            1,
            0,  # param1
            0,  # param2
            radius * direction,  # param3 = radius
            0,  # param4
            int(lat * 1e7),
            int(lon * 1e7),
            alt,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )

        return True

    async def set_position_target(
        self,
        lat: float,
        lon: float,
        alt: float,
        vx: float = 0,
        vy: float = 0,
        vz: float = 0,
    ) -> bool:
        """
        Set position/velocity target for GUIDED mode.

        Args:
            lat: Target latitude
            lon: Target longitude
            alt: Target altitude MSL
            vx: Velocity X (north) m/s
            vy: Velocity Y (east) m/s
            vz: Velocity Z (down) m/s

        Returns:
            True if sent
        """
        type_mask = (
            0b0000_0000_0111_1000  # Ignore acceleration, yaw, yaw_rate
        )

        self.connection._connection.mav.set_position_target_global_int_send(
            0,  # time_boot_ms
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            type_mask,
            int(lat * 1e7),
            int(lon * 1e7),
            alt,
            vx, vy, vz,  # velocity
            0, 0, 0,  # acceleration (ignored)
            0,  # yaw
            0,  # yaw_rate
        )

        return True

    async def upload_mission(self, mission: Mission, timeout: float = 30.0) -> bool:
        """
        Upload mission to vehicle.

        Args:
            mission: Mission to upload
            timeout: Upload timeout in seconds

        Returns:
            True if upload successful
        """
        if not mission.waypoints:
            logger.warning("No waypoints in mission")
            return False

        count = len(mission.waypoints)
        logger.info("Uploading mission", waypoint_count=count)

        # Send mission count
        self._mission_ack_event.clear()
        self.connection._connection.mav.mission_count_send(
            self.connection.target_system,
            self.connection.target_component,
            count,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )

        # Wait for mission requests and send items
        items_sent = 0
        start_time = asyncio.get_event_loop().time()

        def handle_request(msg):
            nonlocal items_sent
            if msg.seq < len(mission.waypoints):
                wp = mission.waypoints[msg.seq]
                self._send_mission_item(wp, msg.seq)
                items_sent += 1

        self.connection.register_callback("MISSION_REQUEST_INT", handle_request)
        self.connection.register_callback("MISSION_REQUEST", handle_request)

        try:
            await asyncio.wait_for(self._mission_ack_event.wait(), timeout)

            if self._mission_ack_result == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                logger.info("Mission uploaded successfully")
                self.current_mission = mission
                return True
            else:
                logger.error("Mission upload rejected", result=self._mission_ack_result)
                return False

        except asyncio.TimeoutError:
            logger.error("Mission upload timeout")
            return False

    def _send_mission_item(self, wp: Waypoint, seq: int) -> None:
        """Send single mission item."""
        self.connection._connection.mav.mission_item_int_send(
            self.connection.target_system,
            self.connection.target_component,
            seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            wp.wp_type.value,
            0,  # current
            1,  # autocontinue
            wp.param1,
            wp.param2,
            wp.param3,
            wp.param4,
            int(wp.lat * 1e7),
            int(wp.lon * 1e7),
            wp.alt,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )

    async def download_mission(self, timeout: float = 30.0) -> Optional[Mission]:
        """
        Download mission from vehicle.

        Args:
            timeout: Download timeout

        Returns:
            Downloaded mission or None on failure
        """
        logger.info("Downloading mission")

        mission = Mission()
        mission_count = 0
        items_received = 0

        count_event = asyncio.Event()
        complete_event = asyncio.Event()

        def handle_count(msg):
            nonlocal mission_count
            mission_count = msg.count
            count_event.set()

        def handle_item(msg):
            nonlocal items_received
            wp = Waypoint(
                lat=msg.x / 1e7,
                lon=msg.y / 1e7,
                alt=msg.z,
                wp_type=WaypointType(msg.command),
                param1=msg.param1,
                param2=msg.param2,
                param3=msg.param3,
                param4=msg.param4,
                seq=msg.seq,
            )
            mission.waypoints.append(wp)
            items_received += 1

            if items_received >= mission_count:
                complete_event.set()
            else:
                # Request next item
                self.connection._connection.mav.mission_request_int_send(
                    self.connection.target_system,
                    self.connection.target_component,
                    items_received,
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                )

        self.connection.register_callback("MISSION_COUNT", handle_count)
        self.connection.register_callback("MISSION_ITEM_INT", handle_item)

        # Request mission count
        self.connection._connection.mav.mission_request_list_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )

        try:
            await asyncio.wait_for(count_event.wait(), timeout=10)

            if mission_count == 0:
                logger.info("No mission on vehicle")
                return mission

            # Request first item
            self.connection._connection.mav.mission_request_int_send(
                self.connection.target_system,
                self.connection.target_component,
                0,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )

            await asyncio.wait_for(complete_event.wait(), timeout)

            # Send ack
            self.connection._connection.mav.mission_ack_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_MISSION_ACCEPTED,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )

            logger.info("Mission downloaded", waypoint_count=len(mission.waypoints))
            self.current_mission = mission
            return mission

        except asyncio.TimeoutError:
            logger.error("Mission download timeout")
            return None

    async def clear_mission(self) -> bool:
        """Clear mission on vehicle."""
        self._mission_ack_event.clear()

        self.connection._connection.mav.mission_clear_all_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )

        try:
            await asyncio.wait_for(self._mission_ack_event.wait(), timeout=10)
            if self._mission_ack_result == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                self.current_mission.clear()
                logger.info("Mission cleared")
                return True
            return False
        except asyncio.TimeoutError:
            return False

    async def set_current_waypoint(self, seq: int) -> bool:
        """Set current mission waypoint."""
        self.connection._connection.mav.mission_set_current_send(
            self.connection.target_system,
            self.connection.target_component,
            seq,
        )
        return True

    def create_survey_pattern(
        self,
        center_lat: float,
        center_lon: float,
        alt: float,
        width: float,
        height: float,
        spacing: float,
    ) -> Mission:
        """
        Create a survey/lawnmower pattern mission.

        Args:
            center_lat: Survey center latitude
            center_lon: Survey center longitude
            alt: Survey altitude MSL
            width: Survey width in meters (east-west)
            height: Survey height in meters (north-south)
            spacing: Line spacing in meters

        Returns:
            Mission with survey waypoints
        """
        import math

        mission = Mission(name="Survey Pattern")

        # Convert meters to degrees (approximate)
        lat_per_m = 1 / 111320
        lon_per_m = 1 / (111320 * math.cos(math.radians(center_lat)))

        half_width = width / 2
        half_height = height / 2
        num_lines = int(width / spacing) + 1

        for i in range(num_lines):
            x_offset = -half_width + (i * spacing)
            lon = center_lon + (x_offset * lon_per_m)

            if i % 2 == 0:
                # North to south
                mission.add_waypoint(
                    center_lat + (half_height * lat_per_m),
                    lon,
                    alt,
                )
                mission.add_waypoint(
                    center_lat - (half_height * lat_per_m),
                    lon,
                    alt,
                )
            else:
                # South to north
                mission.add_waypoint(
                    center_lat - (half_height * lat_per_m),
                    lon,
                    alt,
                )
                mission.add_waypoint(
                    center_lat + (half_height * lat_per_m),
                    lon,
                    alt,
                )

        return mission
