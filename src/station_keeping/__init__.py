"""
Station keeping module for HAPS Glider.

Maintains position over target area accounting for stratospheric winds.
"""

from .loiter import LoiterController, LoiterPattern
from .wind import WindEstimator, WindCompensator
from .altitude import AltitudeController, AltitudeBand

__all__ = [
    "LoiterController",
    "LoiterPattern",
    "WindEstimator",
    "WindCompensator",
    "AltitudeController",
    "AltitudeBand",
]
