#!/usr/bin/env python3
"""
HAPS Glider - Main Entry Point

High Altitude Pseudo-Satellite Autonomous Flight System
ArduPilot AI Agent Integration
"""

import argparse
import asyncio
import signal
import sys
from pathlib import Path

import structlog

from config import HAPSConfig, get_config
from mavlink import MAVLinkConnection
from flight import FlightController
from ai import HAPSAgent

# Configure structured logging
structlog.configure(
    processors=[
        structlog.stdlib.filter_by_level,
        structlog.stdlib.add_logger_name,
        structlog.stdlib.add_log_level,
        structlog.stdlib.PositionalArgumentsFormatter(),
        structlog.processors.TimeStamper(fmt="iso"),
        structlog.processors.StackInfoRenderer(),
        structlog.processors.format_exc_info,
        structlog.processors.UnicodeDecoder(),
        structlog.dev.ConsoleRenderer()
    ],
    wrapper_class=structlog.stdlib.BoundLogger,
    context_class=dict,
    logger_factory=structlog.stdlib.LoggerFactory(),
    cache_logger_on_first_use=True,
)

logger = structlog.get_logger()


class HAPSGlider:
    """
    Main HAPS Glider application.

    Coordinates MAVLink connection, flight control, and AI agent.
    """

    def __init__(self, config: HAPSConfig):
        """
        Initialize HAPS Glider.

        Args:
            config: System configuration
        """
        self.config = config
        self.connection: MAVLinkConnection = None
        self.flight: FlightController = None
        self.agent: HAPSAgent = None
        self._running = False

    async def start(self) -> bool:
        """
        Start the HAPS Glider system.

        Returns:
            True if started successfully
        """
        logger.info("Starting HAPS Glider system")

        # Create MAVLink connection
        self.connection = MAVLinkConnection(
            connection_string=self.config.mavlink.connection_string,
            source_system=self.config.mavlink.source_system,
            source_component=self.config.mavlink.source_component,
            heartbeat_interval=self.config.mavlink.heartbeat_interval,
            heartbeat_timeout=self.config.mavlink.timeout,
        )

        # Connect to vehicle
        logger.info("Connecting to vehicle...")
        if not await self.connection.connect():
            logger.error("Failed to connect to vehicle")
            return False

        logger.info("Connected to vehicle")

        # Create flight controller
        self.flight = FlightController(self.connection)

        # Set flight limits from config
        self.flight.limits.altitude_min_m = self.config.flight.altitude_min_m
        self.flight.limits.altitude_max_m = self.config.flight.altitude_max_m
        self.flight.limits.airspeed_min_ms = self.config.flight.airspeed_min_ms
        self.flight.limits.airspeed_max_ms = self.config.flight.airspeed_max_ms

        # Create AI agent
        self.agent = HAPSAgent(self.flight)

        # Set home position
        self.agent.set_target(
            self.config.home_lat,
            self.config.home_lon,
            self.config.flight.altitude_target_m,
        )

        # Start agent
        await self.agent.start()

        self._running = True
        logger.info("HAPS Glider system started")

        return True

    async def stop(self) -> None:
        """Stop the HAPS Glider system."""
        logger.info("Stopping HAPS Glider system")

        self._running = False

        if self.agent:
            await self.agent.stop()

        if self.connection:
            await self.connection.disconnect()

        logger.info("HAPS Glider system stopped")

    async def run(self) -> None:
        """Run the main application loop."""
        try:
            while self._running:
                await asyncio.sleep(1)

                # Print periodic status
                if self.agent:
                    status = self.agent.get_status()
                    logger.info(
                        "Status",
                        mission=status["mission"]["state"],
                        soc=status["energy"]["battery"]["soc_percent"],
                        alt=status["flight"]["position"]["alt_msl"],
                    )

        except asyncio.CancelledError:
            pass


async def main_async(config: HAPSConfig) -> int:
    """
    Async main entry point.

    Args:
        config: System configuration

    Returns:
        Exit code
    """
    glider = HAPSGlider(config)

    # Setup signal handlers
    loop = asyncio.get_event_loop()

    def signal_handler():
        logger.info("Received shutdown signal")
        asyncio.create_task(glider.stop())

    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, signal_handler)

    # Start system
    if not await glider.start():
        return 1

    # Run main loop
    await glider.run()

    return 0


def main() -> int:
    """
    Main entry point.

    Returns:
        Exit code
    """
    parser = argparse.ArgumentParser(
        description="HAPS Glider - Autonomous Stratospheric Flight System"
    )

    parser.add_argument(
        "-c", "--config",
        type=Path,
        default=None,
        help="Configuration file path (YAML)",
    )

    parser.add_argument(
        "--connection",
        type=str,
        default=None,
        help="MAVLink connection string (overrides config)",
    )

    parser.add_argument(
        "--sitl",
        action="store_true",
        help="Use SITL default connection (udp:127.0.0.1:14550)",
    )

    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Enable verbose logging",
    )

    args = parser.parse_args()

    # Load configuration
    if args.config:
        config = get_config(args.config)
    else:
        # Try default config location
        default_config = Path(__file__).parent.parent / "config" / "default.yaml"
        if default_config.exists():
            config = get_config(default_config)
        else:
            config = HAPSConfig()

    # Override connection if specified
    if args.connection:
        config.mavlink.connection_string = args.connection
    elif args.sitl:
        config.mavlink.connection_string = "udp:127.0.0.1:14550"

    # Print startup banner
    print("""
    ╔═══════════════════════════════════════════════════════════════╗
    ║                      HAPS GLIDER                               ║
    ║         High Altitude Pseudo-Satellite Flight System           ║
    ║                                                                ║
    ║              ArduPilot AI Agent Integration                    ║
    ╚═══════════════════════════════════════════════════════════════╝
    """)

    logger.info(
        "Configuration loaded",
        connection=config.mavlink.connection_string,
        altitude_range=f"{config.flight.altitude_min_m}-{config.flight.altitude_max_m}m",
    )

    # Run async main
    try:
        return asyncio.run(main_async(config))
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
        return 0


if __name__ == "__main__":
    sys.exit(main())
