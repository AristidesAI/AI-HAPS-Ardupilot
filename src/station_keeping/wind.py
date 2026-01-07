"""
Wind estimation and compensation for station keeping.

Provides wind modeling and flight path compensation for stratospheric operations.
"""

import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple

import structlog

logger = structlog.get_logger()


@dataclass
class WindReading:
    """Single wind measurement."""
    timestamp: float
    speed_ms: float
    direction_deg: float  # Direction wind is coming FROM
    altitude_m: float
    vertical_ms: float  # Updraft (+) / downdraft (-)


@dataclass
class WindEstimate:
    """Estimated wind conditions."""
    speed_ms: float = 0.0
    direction_deg: float = 0.0  # Direction wind is coming FROM
    speed_north_ms: float = 0.0  # North component
    speed_east_ms: float = 0.0  # East component
    vertical_ms: float = 0.0  # Vertical component
    confidence: float = 0.0  # 0-1 confidence level
    timestamp: float = 0.0


class WindEstimator:
    """
    Estimates wind conditions from flight data.

    Uses multiple methods:
    - Direct telemetry from ArduPilot WIND message
    - Ground speed vs airspeed comparison
    - Position drift analysis
    """

    def __init__(self, history_size: int = 300):
        """
        Initialize wind estimator.

        Args:
            history_size: Number of readings to keep
        """
        self._history: Deque[WindReading] = deque(maxlen=history_size)
        self._current_estimate = WindEstimate()

        # For ground/air speed comparison
        self._last_position: Optional[Tuple[float, float]] = None
        self._last_position_time = 0.0

    @property
    def estimate(self) -> WindEstimate:
        """Get current wind estimate."""
        return self._current_estimate

    @property
    def wind_speed(self) -> float:
        """Get estimated wind speed (m/s)."""
        return self._current_estimate.speed_ms

    @property
    def wind_direction(self) -> float:
        """Get wind direction (degrees, from)."""
        return self._current_estimate.direction_deg

    def update_from_telemetry(
        self,
        wind_speed: float,
        wind_direction: float,
        vertical_speed: float = 0.0,
        altitude: float = 20000.0,
    ) -> WindEstimate:
        """
        Update wind estimate from ArduPilot WIND message.

        Args:
            wind_speed: Wind speed (m/s)
            wind_direction: Wind direction (degrees, from)
            vertical_speed: Vertical wind component (m/s)
            altitude: Current altitude (m)

        Returns:
            Updated wind estimate
        """
        now = time.time()

        reading = WindReading(
            timestamp=now,
            speed_ms=wind_speed,
            direction_deg=wind_direction,
            altitude_m=altitude,
            vertical_ms=vertical_speed,
        )
        self._history.append(reading)

        self._update_estimate()
        return self._current_estimate

    def update_from_velocity(
        self,
        groundspeed_ms: float,
        ground_track_deg: float,
        airspeed_ms: float,
        heading_deg: float,
        altitude: float = 20000.0,
    ) -> WindEstimate:
        """
        Estimate wind from ground speed vs airspeed.

        Args:
            groundspeed_ms: Ground speed (m/s)
            ground_track_deg: Ground track (degrees)
            airspeed_ms: True airspeed (m/s)
            heading_deg: Aircraft heading (degrees)
            altitude: Current altitude (m)

        Returns:
            Updated wind estimate
        """
        now = time.time()

        # Convert to components
        # Ground velocity
        gnd_north = groundspeed_ms * math.cos(math.radians(ground_track_deg))
        gnd_east = groundspeed_ms * math.sin(math.radians(ground_track_deg))

        # Air velocity (what we'd have with no wind)
        air_north = airspeed_ms * math.cos(math.radians(heading_deg))
        air_east = airspeed_ms * math.sin(math.radians(heading_deg))

        # Wind is the difference
        wind_north = gnd_north - air_north
        wind_east = gnd_east - air_east

        # Convert to speed and direction
        wind_speed = math.sqrt(wind_north**2 + wind_east**2)
        wind_direction = math.degrees(math.atan2(wind_east, wind_north))
        wind_direction = (wind_direction + 180) % 360  # Convert to "from"

        reading = WindReading(
            timestamp=now,
            speed_ms=wind_speed,
            direction_deg=wind_direction,
            altitude_m=altitude,
            vertical_ms=0.0,
        )
        self._history.append(reading)

        self._update_estimate()
        return self._current_estimate

    def update_from_drift(
        self,
        current_lat: float,
        current_lon: float,
        altitude: float = 20000.0,
    ) -> Optional[WindEstimate]:
        """
        Estimate wind from position drift (for loitering aircraft).

        Args:
            current_lat: Current latitude
            current_lon: Current longitude
            altitude: Current altitude

        Returns:
            Wind estimate or None if insufficient data
        """
        now = time.time()

        if self._last_position is None:
            self._last_position = (current_lat, current_lon)
            self._last_position_time = now
            return None

        # Calculate drift
        dt = now - self._last_position_time
        if dt < 10:  # Need at least 10 seconds
            return None

        # Distance and direction
        dlat = current_lat - self._last_position[0]
        dlon = current_lon - self._last_position[1]

        # Convert to meters
        drift_north = dlat * 111320
        drift_east = dlon * 111320 * math.cos(math.radians(current_lat))

        drift_speed = math.sqrt(drift_north**2 + drift_east**2) / dt
        drift_direction = math.degrees(math.atan2(drift_east, drift_north))

        # Wind direction is opposite of drift
        wind_direction = (drift_direction + 180) % 360

        reading = WindReading(
            timestamp=now,
            speed_ms=drift_speed,
            direction_deg=wind_direction,
            altitude_m=altitude,
            vertical_ms=0.0,
        )
        self._history.append(reading)

        self._last_position = (current_lat, current_lon)
        self._last_position_time = now

        self._update_estimate()
        return self._current_estimate

    def _update_estimate(self) -> None:
        """Update wind estimate from history."""
        if not self._history:
            return

        # Use exponentially weighted moving average
        # More recent readings have more weight
        now = time.time()
        total_weight = 0.0
        weighted_speed = 0.0
        weighted_north = 0.0
        weighted_east = 0.0
        weighted_vertical = 0.0

        for reading in self._history:
            age = now - reading.timestamp
            weight = math.exp(-age / 60)  # 1-minute time constant

            weighted_speed += reading.speed_ms * weight
            dir_rad = math.radians(reading.direction_deg)
            weighted_north += reading.speed_ms * math.cos(dir_rad) * weight
            weighted_east += reading.speed_ms * math.sin(dir_rad) * weight
            weighted_vertical += reading.vertical_ms * weight
            total_weight += weight

        if total_weight > 0:
            avg_speed = weighted_speed / total_weight
            avg_north = weighted_north / total_weight
            avg_east = weighted_east / total_weight
            avg_vertical = weighted_vertical / total_weight

            direction = math.degrees(math.atan2(avg_east, avg_north))
            direction = (direction + 360) % 360

            # Confidence based on data recency and consistency
            recent_readings = [r for r in self._history if now - r.timestamp < 60]
            confidence = min(1.0, len(recent_readings) / 30)

            self._current_estimate = WindEstimate(
                speed_ms=avg_speed,
                direction_deg=direction,
                speed_north_ms=avg_north,
                speed_east_ms=avg_east,
                vertical_ms=avg_vertical,
                confidence=confidence,
                timestamp=now,
            )

    def get_wind_at_altitude(self, target_altitude: float) -> WindEstimate:
        """
        Get wind estimate for specific altitude (if altitude data available).

        Args:
            target_altitude: Target altitude in meters

        Returns:
            Wind estimate for that altitude
        """
        # Filter readings near target altitude
        nearby = [
            r for r in self._history
            if abs(r.altitude_m - target_altitude) < 500
        ]

        if not nearby:
            return self._current_estimate

        # Average nearby readings
        avg_speed = sum(r.speed_ms for r in nearby) / len(nearby)
        avg_dir = sum(r.direction_deg for r in nearby) / len(nearby)
        avg_vert = sum(r.vertical_ms for r in nearby) / len(nearby)

        return WindEstimate(
            speed_ms=avg_speed,
            direction_deg=avg_dir,
            vertical_ms=avg_vert,
            confidence=min(1.0, len(nearby) / 10),
            timestamp=time.time(),
        )


class WindCompensator:
    """
    Compensates flight path for wind conditions.

    Calculates heading adjustments to maintain desired track.
    """

    def __init__(self, wind_estimator: WindEstimator):
        """
        Initialize wind compensator.

        Args:
            wind_estimator: Wind estimator instance
        """
        self.wind = wind_estimator

    def calculate_heading_for_track(
        self,
        desired_track_deg: float,
        airspeed_ms: float,
    ) -> Tuple[float, float]:
        """
        Calculate heading to maintain desired ground track.

        Args:
            desired_track_deg: Desired ground track (degrees)
            airspeed_ms: True airspeed (m/s)

        Returns:
            Tuple of (required_heading_deg, expected_groundspeed_ms)
        """
        wind = self.wind.estimate

        if wind.speed_ms < 0.5 or airspeed_ms < 1:
            return desired_track_deg, airspeed_ms

        # Wind triangle calculation
        # Convert to radians
        track_rad = math.radians(desired_track_deg)
        wind_from_rad = math.radians(wind.direction_deg)

        # Wind components relative to track
        wind_to_rad = wind_from_rad + math.pi  # Wind direction we're going
        crosswind = wind.speed_ms * math.sin(wind_to_rad - track_rad)
        headwind = wind.speed_ms * math.cos(wind_to_rad - track_rad)

        # Calculate wind correction angle
        if abs(crosswind) > airspeed_ms:
            # Wind too strong to compensate
            logger.warning(
                "Crosswind exceeds airspeed",
                crosswind=crosswind,
                airspeed=airspeed_ms,
            )
            wca = math.copysign(90, crosswind)
        else:
            wca = math.degrees(math.asin(crosswind / airspeed_ms))

        # Required heading
        heading = (desired_track_deg + wca + 360) % 360

        # Expected groundspeed
        groundspeed = math.sqrt(
            airspeed_ms**2 - crosswind**2
        ) - headwind

        return heading, max(0, groundspeed)

    def calculate_drift_vector(
        self,
        heading_deg: float,
        airspeed_ms: float,
    ) -> Tuple[float, float]:
        """
        Calculate expected drift from heading.

        Args:
            heading_deg: Aircraft heading (degrees)
            airspeed_ms: True airspeed (m/s)

        Returns:
            Tuple of (expected_track_deg, expected_groundspeed_ms)
        """
        wind = self.wind.estimate

        # Air velocity
        air_north = airspeed_ms * math.cos(math.radians(heading_deg))
        air_east = airspeed_ms * math.sin(math.radians(heading_deg))

        # Add wind
        wind_to_rad = math.radians((wind.direction_deg + 180) % 360)
        wind_north = wind.speed_ms * math.cos(wind_to_rad)
        wind_east = wind.speed_ms * math.sin(wind_to_rad)

        # Ground velocity
        gnd_north = air_north + wind_north
        gnd_east = air_east + wind_east

        groundspeed = math.sqrt(gnd_north**2 + gnd_east**2)
        track = math.degrees(math.atan2(gnd_east, gnd_north))
        track = (track + 360) % 360

        return track, groundspeed

    def get_position_after_time(
        self,
        start_lat: float,
        start_lon: float,
        heading_deg: float,
        airspeed_ms: float,
        time_seconds: float,
    ) -> Tuple[float, float]:
        """
        Predict position after flying for given time.

        Args:
            start_lat: Starting latitude
            start_lon: Starting longitude
            heading_deg: Aircraft heading
            airspeed_ms: True airspeed
            time_seconds: Time to fly

        Returns:
            Tuple of (predicted_lat, predicted_lon)
        """
        track, groundspeed = self.calculate_drift_vector(heading_deg, airspeed_ms)
        distance = groundspeed * time_seconds

        # Calculate new position
        lat_rad = math.radians(start_lat)
        track_rad = math.radians(track)
        angular_dist = distance / 6371000  # Earth radius

        new_lat = math.asin(
            math.sin(lat_rad) * math.cos(angular_dist)
            + math.cos(lat_rad) * math.sin(angular_dist) * math.cos(track_rad)
        )

        new_lon_rad = math.radians(start_lon) + math.atan2(
            math.sin(track_rad) * math.sin(angular_dist) * math.cos(lat_rad),
            math.cos(angular_dist) - math.sin(lat_rad) * math.sin(new_lat)
        )

        return math.degrees(new_lat), math.degrees(new_lon_rad)

    def find_altitude_for_wind(
        self,
        target_direction: float,
        altitude_range: Tuple[float, float] = (18000, 25000),
    ) -> Optional[float]:
        """
        Find altitude with favorable wind direction.

        This is a placeholder - real implementation would use wind profiles.

        Args:
            target_direction: Desired wind direction
            altitude_range: Min/max altitude to search

        Returns:
            Recommended altitude or None if no data
        """
        # Would need historical wind data at different altitudes
        # For now, return None to indicate no recommendation
        return None
