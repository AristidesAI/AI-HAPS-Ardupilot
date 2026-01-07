"""
MAVLink connection handler for ArduPilot communication.
"""

import asyncio
import time
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Optional

import structlog
from pymavlink import mavutil

logger = structlog.get_logger()


class ConnectionState(Enum):
    """MAVLink connection state."""
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    HEARTBEAT_LOST = "heartbeat_lost"


@dataclass
class HeartbeatInfo:
    """Information from last heartbeat message."""
    system_id: int = 0
    component_id: int = 0
    autopilot: int = 0
    vehicle_type: int = 0
    system_status: int = 0
    base_mode: int = 0
    custom_mode: int = 0
    mavlink_version: int = 0
    timestamp: float = 0.0


class MAVLinkConnection:
    """
    Manages MAVLink connection to ArduPilot flight controller.

    Supports UDP, TCP, and serial connections. Handles heartbeat
    monitoring, connection state management, and message routing.
    """

    def __init__(
        self,
        connection_string: str = "udp:127.0.0.1:14550",
        source_system: int = 255,
        source_component: int = 0,
        heartbeat_interval: float = 1.0,
        heartbeat_timeout: float = 5.0,
    ):
        """
        Initialize MAVLink connection.

        Args:
            connection_string: MAVLink connection string (udp:, tcp:, /dev/tty*, etc.)
            source_system: Source system ID for outgoing messages
            source_component: Source component ID for outgoing messages
            heartbeat_interval: Interval between heartbeat messages (seconds)
            heartbeat_timeout: Time without heartbeat before connection is lost (seconds)
        """
        self.connection_string = connection_string
        self.source_system = source_system
        self.source_component = source_component
        self.heartbeat_interval = heartbeat_interval
        self.heartbeat_timeout = heartbeat_timeout

        self._connection: Optional[mavutil.mavlink_connection] = None
        self._state = ConnectionState.DISCONNECTED
        self._heartbeat_info = HeartbeatInfo()
        self._last_heartbeat_time = 0.0
        self._running = False

        # Message callbacks
        self._message_callbacks: dict[str, list[Callable]] = {}
        self._all_message_callback: Optional[Callable] = None

        # Background tasks
        self._heartbeat_task: Optional[asyncio.Task] = None
        self._receive_task: Optional[asyncio.Task] = None

    @property
    def state(self) -> ConnectionState:
        """Get current connection state."""
        return self._state

    @property
    def is_connected(self) -> bool:
        """Check if connection is active."""
        return self._state == ConnectionState.CONNECTED

    @property
    def heartbeat(self) -> HeartbeatInfo:
        """Get last heartbeat information."""
        return self._heartbeat_info

    @property
    def target_system(self) -> int:
        """Get target system ID (from heartbeat)."""
        return self._heartbeat_info.system_id

    @property
    def target_component(self) -> int:
        """Get target component ID (from heartbeat)."""
        return self._heartbeat_info.component_id

    async def connect(self) -> bool:
        """
        Establish MAVLink connection.

        Returns:
            True if connection established, False otherwise.
        """
        if self._state != ConnectionState.DISCONNECTED:
            logger.warning("Already connected or connecting")
            return False

        self._state = ConnectionState.CONNECTING
        logger.info("Connecting to MAVLink", connection=self.connection_string)

        try:
            self._connection = mavutil.mavlink_connection(
                self.connection_string,
                source_system=self.source_system,
                source_component=self.source_component,
            )

            # Wait for heartbeat to confirm connection
            logger.info("Waiting for heartbeat...")
            msg = self._connection.wait_heartbeat(timeout=30)

            if msg is None:
                logger.error("No heartbeat received")
                self._state = ConnectionState.DISCONNECTED
                return False

            self._update_heartbeat(msg)
            self._state = ConnectionState.CONNECTED
            self._running = True

            # Start background tasks
            self._heartbeat_task = asyncio.create_task(self._heartbeat_loop())
            self._receive_task = asyncio.create_task(self._receive_loop())

            logger.info(
                "Connected to vehicle",
                system_id=self._heartbeat_info.system_id,
                autopilot=self._heartbeat_info.autopilot,
            )
            return True

        except Exception as e:
            logger.error("Connection failed", error=str(e))
            self._state = ConnectionState.DISCONNECTED
            return False

    async def disconnect(self) -> None:
        """Disconnect from MAVLink."""
        self._running = False

        if self._heartbeat_task:
            self._heartbeat_task.cancel()
            try:
                await self._heartbeat_task
            except asyncio.CancelledError:
                pass

        if self._receive_task:
            self._receive_task.cancel()
            try:
                await self._receive_task
            except asyncio.CancelledError:
                pass

        if self._connection:
            self._connection.close()
            self._connection = None

        self._state = ConnectionState.DISCONNECTED
        logger.info("Disconnected from MAVLink")

    def register_callback(self, message_type: str, callback: Callable) -> None:
        """
        Register callback for specific message type.

        Args:
            message_type: MAVLink message type name (e.g., "HEARTBEAT", "GLOBAL_POSITION_INT")
            callback: Function to call when message received
        """
        if message_type not in self._message_callbacks:
            self._message_callbacks[message_type] = []
        self._message_callbacks[message_type].append(callback)

    def register_all_messages_callback(self, callback: Callable) -> None:
        """Register callback for all messages."""
        self._all_message_callback = callback

    def send_message(self, msg) -> bool:
        """
        Send a MAVLink message.

        Args:
            msg: MAVLink message to send

        Returns:
            True if sent successfully
        """
        if not self._connection or not self.is_connected:
            logger.warning("Cannot send message - not connected")
            return False

        try:
            self._connection.mav.send(msg)
            return True
        except Exception as e:
            logger.error("Failed to send message", error=str(e))
            return False

    def send_command_long(
        self,
        command: int,
        param1: float = 0,
        param2: float = 0,
        param3: float = 0,
        param4: float = 0,
        param5: float = 0,
        param6: float = 0,
        param7: float = 0,
        confirmation: int = 0,
    ) -> bool:
        """
        Send a MAV_CMD command via COMMAND_LONG message.

        Args:
            command: MAV_CMD_* command ID
            param1-param7: Command parameters
            confirmation: Confirmation count

        Returns:
            True if sent successfully
        """
        if not self._connection:
            return False

        try:
            self._connection.mav.command_long_send(
                self.target_system,
                self.target_component,
                command,
                confirmation,
                param1, param2, param3, param4, param5, param6, param7,
            )
            return True
        except Exception as e:
            logger.error("Failed to send command", command=command, error=str(e))
            return False

    def request_message(self, message_id: int, interval_us: int = 0) -> bool:
        """
        Request a specific message at given interval.

        Args:
            message_id: MAVLink message ID
            interval_us: Interval in microseconds (0 = one-time request)

        Returns:
            True if request sent successfully
        """
        if not self._connection:
            return False

        try:
            self._connection.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,  # confirmation
                message_id,
                interval_us,
                0, 0, 0, 0, 0,
            )
            return True
        except Exception as e:
            logger.error("Failed to request message", message_id=message_id, error=str(e))
            return False

    def _update_heartbeat(self, msg) -> None:
        """Update heartbeat info from message."""
        self._heartbeat_info = HeartbeatInfo(
            system_id=msg.get_srcSystem(),
            component_id=msg.get_srcComponent(),
            autopilot=msg.autopilot,
            vehicle_type=msg.type,
            system_status=msg.system_status,
            base_mode=msg.base_mode,
            custom_mode=msg.custom_mode,
            mavlink_version=msg.mavlink_version,
            timestamp=time.time(),
        )
        self._last_heartbeat_time = time.time()

    async def _heartbeat_loop(self) -> None:
        """Send periodic heartbeats and check connection health."""
        while self._running:
            try:
                # Send our heartbeat
                if self._connection:
                    self._connection.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0, 0, 0,
                    )

                # Check for heartbeat timeout
                elapsed = time.time() - self._last_heartbeat_time
                if elapsed > self.heartbeat_timeout:
                    if self._state == ConnectionState.CONNECTED:
                        logger.warning("Heartbeat lost", elapsed=elapsed)
                        self._state = ConnectionState.HEARTBEAT_LOST
                elif self._state == ConnectionState.HEARTBEAT_LOST:
                    logger.info("Heartbeat recovered")
                    self._state = ConnectionState.CONNECTED

                await asyncio.sleep(self.heartbeat_interval)

            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error("Heartbeat loop error", error=str(e))
                await asyncio.sleep(1)

    async def _receive_loop(self) -> None:
        """Receive and dispatch incoming messages."""
        while self._running:
            try:
                if not self._connection:
                    await asyncio.sleep(0.1)
                    continue

                # Non-blocking receive
                msg = self._connection.recv_match(blocking=False)

                if msg is None:
                    await asyncio.sleep(0.001)  # Small delay to prevent busy loop
                    continue

                msg_type = msg.get_type()

                # Handle heartbeat specially
                if msg_type == "HEARTBEAT":
                    self._update_heartbeat(msg)

                # Dispatch to callbacks
                if self._all_message_callback:
                    try:
                        self._all_message_callback(msg)
                    except Exception as e:
                        logger.error("All-message callback error", error=str(e))

                if msg_type in self._message_callbacks:
                    for callback in self._message_callbacks[msg_type]:
                        try:
                            callback(msg)
                        except Exception as e:
                            logger.error(
                                "Message callback error",
                                msg_type=msg_type,
                                error=str(e),
                            )

            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error("Receive loop error", error=str(e))
                await asyncio.sleep(0.1)

    @property
    def mav(self):
        """Get raw pymavlink MAVLink object for advanced operations."""
        if self._connection:
            return self._connection.mav
        return None
