"""
Loiter pattern management for station keeping.

Provides algorithms for maintaining position in stratospheric winds.
"""

import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Tuple

import structlog

logger = structlog.get_logger()


class LoiterPattern(Enum):
    """Loiter pattern types."""
    CIRCLE = "circle"  # Standard circular loiter
    RACETRACK = "racetrack"  # Elongated pattern for wind
    FIGURE_EIGHT = "figure_eight"  # Wind-compensating pattern
    ORBIT = "orbit"  # Wide orbit with drift correction


@dataclass
class LoiterConfig:
    """Loiter configuration parameters."""
    default_radius_m: float = 1000.0  # Default loiter radius
    min_radius_m: float = 500.0  # Minimum safe radius
    max_radius_m: float = 3000.0  # Maximum radius
    position_tolerance_m: float = 200.0  # Acceptable drift from center
    drift_correction_interval_s: float = 60.0  # How often to check drift
    max_correction_angle_deg: float = 45.0  # Max heading adjustment


@dataclass
class LoiterState:
    """Current loiter state."""
    pattern: LoiterPattern = LoiterPattern.CIRCLE
    center_lat: float = 0.0
    center_lon: float = 0.0
    target_alt_m: float = 20000.0
    radius_m: float = 1000.0
    direction: int = 1  # 1 = clockwise, -1 = counter-clockwise
    laps_completed: int = 0
    time_in_pattern_s: float = 0.0
    drift_distance_m: float = 0.0
    drift_heading_deg: float = 0.0
    is_active: bool = False


class LoiterController:
    """
    Controls loiter patterns for station keeping.

    Features:
    - Multiple pattern types for different wind conditions
    - Drift detection and correction
    - Dynamic radius adjustment
    - Wind-compensating patterns
    """

    # Earth radius for distance calculations
    EARTH_RADIUS_M = 6371000

    def __init__(self, config: Optional[LoiterConfig] = None):
        """
        Initialize loiter controller.

        Args:
            config: Loiter configuration
        """
        self.config = config or LoiterConfig()
        self.state = LoiterState()

        # Position tracking
        self._center_set_time = 0.0
        self._last_position = (0.0, 0.0)
        self._last_position_time = 0.0
        self._lap_start_heading = 0.0
        self._cumulative_heading_change = 0.0

        # Drift history for analysis
        self._drift_history: List[Tuple[float, float, float]] = []  # (time, distance, heading)

    @property
    def is_active(self) -> bool:
        """Check if loiter is active."""
        return self.state.is_active

    @property
    def drift_distance(self) -> float:
        """Get current drift distance from center."""
        return self.state.drift_distance_m

    def start_loiter(
        self,
        center_lat: float,
        center_lon: float,
        altitude_m: float,
        radius_m: Optional[float] = None,
        pattern: LoiterPattern = LoiterPattern.CIRCLE,
        direction: int = 1,
    ) -> None:
        """
        Start loiter at specified location.

        Args:
            center_lat: Center latitude
            center_lon: Center longitude
            altitude_m: Target altitude MSL
            radius_m: Loiter radius (None = default)
            pattern: Loiter pattern type
            direction: Turn direction (1 = CW, -1 = CCW)
        """
        if radius_m is None:
            radius_m = self.config.default_radius_m

        radius_m = max(self.config.min_radius_m, min(radius_m, self.config.max_radius_m))

        self.state = LoiterState(
            pattern=pattern,
            center_lat=center_lat,
            center_lon=center_lon,
            target_alt_m=altitude_m,
            radius_m=radius_m,
            direction=direction,
            is_active=True,
        )

        self._center_set_time = time.time()
        self._drift_history.clear()
        self._cumulative_heading_change = 0.0

        logger.info(
            "Loiter started",
            lat=center_lat,
            lon=center_lon,
            alt=altitude_m,
            radius=radius_m,
            pattern=pattern.value,
        )

    def stop_loiter(self) -> None:
        """Stop loiter pattern."""
        self.state.is_active = False
        logger.info("Loiter stopped")

    def update(
        self,
        current_lat: float,
        current_lon: float,
        current_heading: float,
    ) -> Tuple[Optional[float], Optional[float]]:
        """
        Update loiter state and get correction if needed.

        Args:
            current_lat: Current latitude
            current_lon: Current longitude
            current_heading: Current heading (degrees)

        Returns:
            Tuple of (target_lat, target_lon) for correction, or (None, None) if no correction needed
        """
        if not self.state.is_active:
            return None, None

        now = time.time()

        # Calculate distance from center
        drift_distance, drift_heading = self._calculate_drift(
            current_lat, current_lon
        )

        self.state.drift_distance_m = drift_distance
        self.state.drift_heading_deg = drift_heading
        self.state.time_in_pattern_s = now - self._center_set_time

        # Track laps
        self._track_lap_completion(current_heading)

        # Record drift history
        self._drift_history.append((now, drift_distance, drift_heading))
        if len(self._drift_history) > 3600:  # Keep 1 hour max
            self._drift_history = self._drift_history[-3600:]

        # Check if correction is needed
        if drift_distance > self.config.position_tolerance_m:
            # Calculate correction target
            correction = self._calculate_correction(
                current_lat, current_lon, drift_heading, drift_distance
            )
            return correction

        # Update position tracking
        self._last_position = (current_lat, current_lon)
        self._last_position_time = now

        return None, None

    def _calculate_drift(
        self,
        current_lat: float,
        current_lon: float,
    ) -> Tuple[float, float]:
        """
        Calculate drift from loiter center.

        Returns:
            Tuple of (distance_m, heading_deg)
        """
        # Distance calculation using haversine
        lat1 = math.radians(self.state.center_lat)
        lat2 = math.radians(current_lat)
        dlat = lat2 - lat1
        dlon = math.radians(current_lon - self.state.center_lon)

        a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = self.EARTH_RADIUS_M * c

        # Heading from center to current position
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        heading = math.degrees(math.atan2(y, x))
        heading = (heading + 360) % 360

        return distance, heading

    def _calculate_correction(
        self,
        current_lat: float,
        current_lon: float,
        drift_heading: float,
        drift_distance: float,
    ) -> Tuple[float, float]:
        """
        Calculate target position for drift correction.

        Returns:
            Tuple of (target_lat, target_lon)
        """
        # Calculate target point towards center
        # Move halfway back to center
        correction_distance = drift_distance / 2

        # Heading towards center is opposite of drift heading
        towards_center = (drift_heading + 180) % 360

        # Calculate target position
        target_lat, target_lon = self._calculate_point_at_distance(
            current_lat, current_lon,
            towards_center, correction_distance
        )

        return target_lat, target_lon

    def _calculate_point_at_distance(
        self,
        lat: float,
        lon: float,
        heading: float,
        distance: float,
    ) -> Tuple[float, float]:
        """Calculate point at given distance and heading from start."""
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        heading_rad = math.radians(heading)
        angular_distance = distance / self.EARTH_RADIUS_M

        new_lat = math.asin(
            math.sin(lat_rad) * math.cos(angular_distance)
            + math.cos(lat_rad) * math.sin(angular_distance) * math.cos(heading_rad)
        )

        new_lon = lon_rad + math.atan2(
            math.sin(heading_rad) * math.sin(angular_distance) * math.cos(lat_rad),
            math.cos(angular_distance) - math.sin(lat_rad) * math.sin(new_lat)
        )

        return math.degrees(new_lat), math.degrees(new_lon)

    def _track_lap_completion(self, current_heading: float) -> None:
        """Track lap completion based on heading changes."""
        if self._lap_start_heading == 0:
            self._lap_start_heading = current_heading
            return

        # Calculate heading change
        delta = current_heading - self._lap_start_heading
        if delta > 180:
            delta -= 360
        elif delta < -180:
            delta += 360

        self._cumulative_heading_change += abs(delta)
        self._lap_start_heading = current_heading

        # Check for lap completion (360 degrees of turning)
        while self._cumulative_heading_change >= 360:
            self.state.laps_completed += 1
            self._cumulative_heading_change -= 360
            logger.debug("Loiter lap completed", total_laps=self.state.laps_completed)

    def get_wind_estimate_from_drift(self) -> Tuple[float, float]:
        """
        Estimate wind from drift history.

        Returns:
            Tuple of (wind_speed_ms, wind_direction_deg)
        """
        if len(self._drift_history) < 10:
            return 0.0, 0.0

        # Analyze drift rate and direction
        recent = self._drift_history[-60:]  # Last minute

        if len(recent) < 2:
            return 0.0, 0.0

        # Calculate average drift rate
        total_time = recent[-1][0] - recent[0][0]
        if total_time <= 0:
            return 0.0, 0.0

        # Average drift direction
        avg_heading = sum(d[2] for d in recent) / len(recent)

        # Drift rate
        drift_rate = (recent[-1][1] - recent[0][1]) / total_time  # m/s

        # Wind direction is where it's coming FROM (opposite of drift)
        wind_direction = (avg_heading + 180) % 360

        return abs(drift_rate), wind_direction

    def adjust_pattern_for_wind(
        self,
        wind_speed_ms: float,
        wind_direction_deg: float,
    ) -> None:
        """
        Adjust loiter pattern for wind conditions.

        Args:
            wind_speed_ms: Wind speed in m/s
            wind_direction_deg: Wind direction (from)
        """
        if wind_speed_ms < 5:
            # Light winds - use standard circle
            if self.state.pattern != LoiterPattern.CIRCLE:
                self.state.pattern = LoiterPattern.CIRCLE
                logger.info("Switched to CIRCLE pattern for light winds")

        elif wind_speed_ms < 15:
            # Moderate winds - use racetrack aligned with wind
            if self.state.pattern != LoiterPattern.RACETRACK:
                self.state.pattern = LoiterPattern.RACETRACK
                logger.info("Switched to RACETRACK pattern for moderate winds")

        else:
            # Strong winds - use figure-eight for better wind compensation
            if self.state.pattern != LoiterPattern.FIGURE_EIGHT:
                self.state.pattern = LoiterPattern.FIGURE_EIGHT
                logger.info("Switched to FIGURE_EIGHT pattern for strong winds")

    def get_pattern_waypoints(self) -> List[Tuple[float, float, float]]:
        """
        Get waypoints for current loiter pattern.

        Returns:
            List of (lat, lon, alt) tuples
        """
        if self.state.pattern == LoiterPattern.CIRCLE:
            return self._get_circle_waypoints()
        elif self.state.pattern == LoiterPattern.RACETRACK:
            return self._get_racetrack_waypoints()
        elif self.state.pattern == LoiterPattern.FIGURE_EIGHT:
            return self._get_figure_eight_waypoints()
        else:
            return self._get_circle_waypoints()

    def _get_circle_waypoints(self, num_points: int = 8) -> List[Tuple[float, float, float]]:
        """Generate circular loiter waypoints."""
        waypoints = []

        for i in range(num_points):
            angle = (360 / num_points) * i * self.state.direction
            lat, lon = self._calculate_point_at_distance(
                self.state.center_lat,
                self.state.center_lon,
                angle,
                self.state.radius_m,
            )
            waypoints.append((lat, lon, self.state.target_alt_m))

        return waypoints

    def _get_racetrack_waypoints(self) -> List[Tuple[float, float, float]]:
        """Generate racetrack pattern waypoints."""
        # Elongated pattern - 2:1 aspect ratio
        length = self.state.radius_m * 2
        width = self.state.radius_m

        waypoints = []
        # Simplified racetrack with 4 main points
        angles = [0, 90, 180, 270]

        for angle in angles:
            if angle in (0, 180):
                dist = length
            else:
                dist = width

            lat, lon = self._calculate_point_at_distance(
                self.state.center_lat,
                self.state.center_lon,
                angle,
                dist / 2,
            )
            waypoints.append((lat, lon, self.state.target_alt_m))

        return waypoints

    def _get_figure_eight_waypoints(self) -> List[Tuple[float, float, float]]:
        """Generate figure-eight pattern waypoints."""
        waypoints = []
        radius = self.state.radius_m / 2

        # Two circles offset from center
        offsets = [(-radius, 0), (radius, 0)]

        for offset_lat, offset_lon in offsets:
            center_lat = self.state.center_lat + (offset_lat / 111320)
            center_lon = self.state.center_lon + (offset_lon / (111320 * math.cos(math.radians(self.state.center_lat))))

            for angle in [0, 90, 180, 270]:
                lat, lon = self._calculate_point_at_distance(
                    center_lat, center_lon, angle, radius
                )
                waypoints.append((lat, lon, self.state.target_alt_m))

        return waypoints

    def get_status_dict(self) -> dict:
        """Get status as dictionary."""
        return {
            "is_active": self.state.is_active,
            "pattern": self.state.pattern.value,
            "center_lat": self.state.center_lat,
            "center_lon": self.state.center_lon,
            "target_alt_m": self.state.target_alt_m,
            "radius_m": self.state.radius_m,
            "drift_distance_m": round(self.state.drift_distance_m, 1),
            "drift_heading_deg": round(self.state.drift_heading_deg, 1),
            "laps_completed": self.state.laps_completed,
            "time_in_pattern_s": round(self.state.time_in_pattern_s, 0),
        }
