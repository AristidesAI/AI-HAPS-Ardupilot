"""
MAVLink communication module for HAPS Glider.

Handles all communication with ArduPilot flight controller via MAVLink protocol.
"""

from .connection import MAVLinkConnection
from .messages import MessageHandler, TelemetryData
from .parameters import ParameterManager

__all__ = [
    "MAVLinkConnection",
    "MessageHandler",
    "TelemetryData",
    "ParameterManager",
]
