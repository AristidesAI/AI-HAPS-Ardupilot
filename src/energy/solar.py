"""
Solar power management for HAPS Glider.

Calculates sun position, predicts solar generation, and optimizes flight heading.
"""

import math
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Optional, Tuple

import structlog

logger = structlog.get_logger()


@dataclass
class SunPosition:
    """Sun position in sky."""
    elevation: float  # degrees above horizon
    azimuth: float  # degrees from north (clockwise)
    timestamp: float


@dataclass
class SolarConfig:
    """Solar panel configuration."""
    panel_area_m2: float = 4.0  # Total panel area
    panel_efficiency: float = 0.22  # Cell efficiency (22%)
    panel_tilt_deg: float = 0.0  # Panel tilt from horizontal
    temperature_coefficient: float = -0.004  # Power loss per degree above 25C
    soiling_factor: float = 0.98  # Dust/dirt reduction
    inverter_efficiency: float = 0.95  # MPPT efficiency


@dataclass
class SolarStatus:
    """Current solar power status."""
    sun_position: SunPosition
    irradiance_w_m2: float  # Solar irradiance at altitude
    power_available_w: float  # Theoretical max power
    power_actual_w: float  # Actual generated power
    heading_optimal_deg: float  # Optimal heading for max power
    heading_gain_percent: float  # Power gain from optimal heading
    is_daylight: bool
    timestamp: float


class SolarManager:
    """
    Manages solar power prediction and optimization.

    Features:
    - Sun position calculation
    - Solar irradiance modeling
    - Power generation prediction
    - Optimal heading calculation
    - Day/night detection
    """

    # Solar constant (W/m^2 at top of atmosphere)
    SOLAR_CONSTANT = 1361.0

    def __init__(self, config: Optional[SolarConfig] = None):
        """
        Initialize solar manager.

        Args:
            config: Solar panel configuration
        """
        self.config = config or SolarConfig()
        self.status: Optional[SolarStatus] = None

        # Location for sun calculations
        self._latitude = 0.0
        self._longitude = 0.0
        self._altitude_m = 20000.0

        # Current heading
        self._heading = 0.0

    def set_location(self, lat: float, lon: float, alt_m: float) -> None:
        """Update current location for sun calculations."""
        self._latitude = lat
        self._longitude = lon
        self._altitude_m = alt_m

    def set_heading(self, heading: float) -> None:
        """Update current aircraft heading."""
        self._heading = heading

    def update(self, actual_power_w: float = 0.0) -> SolarStatus:
        """
        Update solar status with current conditions.

        Args:
            actual_power_w: Actual measured solar power (if available)

        Returns:
            Updated solar status
        """
        now = time.time()
        dt = datetime.fromtimestamp(now, tz=timezone.utc)

        # Calculate sun position
        sun_pos = self.calculate_sun_position(
            self._latitude,
            self._longitude,
            dt,
        )

        # Calculate irradiance at altitude
        irradiance = self.calculate_irradiance(sun_pos.elevation, self._altitude_m)

        # Calculate available power
        power_available = self.calculate_power(irradiance, sun_pos.elevation)

        # Calculate optimal heading
        heading_optimal = self.calculate_optimal_heading(sun_pos)
        heading_gain = self.calculate_heading_gain(self._heading, sun_pos)

        self.status = SolarStatus(
            sun_position=sun_pos,
            irradiance_w_m2=irradiance,
            power_available_w=power_available,
            power_actual_w=actual_power_w if actual_power_w > 0 else power_available,
            heading_optimal_deg=heading_optimal,
            heading_gain_percent=heading_gain,
            is_daylight=sun_pos.elevation > -6,  # Civil twilight
            timestamp=now,
        )

        return self.status

    def calculate_sun_position(
        self,
        latitude: float,
        longitude: float,
        dt: datetime,
    ) -> SunPosition:
        """
        Calculate sun position using simplified algorithm.

        Based on NOAA Solar Calculator equations.
        """
        # Julian day
        jd = self._julian_day(dt)
        jc = (jd - 2451545) / 36525  # Julian century

        # Sun's geometric mean longitude (degrees)
        L0 = (280.46646 + jc * (36000.76983 + 0.0003032 * jc)) % 360

        # Sun's mean anomaly (degrees)
        M = (357.52911 + jc * (35999.05029 - 0.0001537 * jc)) % 360

        # Eccentricity of Earth's orbit
        e = 0.016708634 - jc * (0.000042037 + 0.0000001267 * jc)

        # Sun's equation of center
        C = (
            (1.914602 - jc * (0.004817 + 0.000014 * jc)) * math.sin(math.radians(M))
            + (0.019993 - 0.000101 * jc) * math.sin(math.radians(2 * M))
            + 0.000289 * math.sin(math.radians(3 * M))
        )

        # Sun's true longitude
        sun_lon = L0 + C

        # Sun's apparent longitude
        omega = 125.04 - 1934.136 * jc
        sun_apparent_lon = sun_lon - 0.00569 - 0.00478 * math.sin(math.radians(omega))

        # Mean obliquity of ecliptic
        obliquity = (
            23.439291
            - 0.013004167 * jc
            - 0.000000164 * jc**2
            + 0.000000504 * jc**3
        )

        # Corrected obliquity
        obliquity_corr = obliquity + 0.00256 * math.cos(math.radians(omega))

        # Sun's declination
        declination = math.degrees(
            math.asin(
                math.sin(math.radians(obliquity_corr))
                * math.sin(math.radians(sun_apparent_lon))
            )
        )

        # Equation of time (minutes)
        y = math.tan(math.radians(obliquity_corr / 2)) ** 2
        eot = 4 * math.degrees(
            y * math.sin(2 * math.radians(L0))
            - 2 * e * math.sin(math.radians(M))
            + 4 * e * y * math.sin(math.radians(M)) * math.cos(2 * math.radians(L0))
            - 0.5 * y**2 * math.sin(4 * math.radians(L0))
            - 1.25 * e**2 * math.sin(2 * math.radians(M))
        )

        # True solar time
        time_offset = eot + 4 * longitude
        hour = dt.hour + dt.minute / 60 + dt.second / 3600
        true_solar_time = (hour * 60 + time_offset) % 1440

        # Hour angle
        if true_solar_time / 4 < 0:
            hour_angle = true_solar_time / 4 + 180
        else:
            hour_angle = true_solar_time / 4 - 180

        # Solar zenith and elevation
        lat_rad = math.radians(latitude)
        dec_rad = math.radians(declination)
        ha_rad = math.radians(hour_angle)

        cos_zenith = (
            math.sin(lat_rad) * math.sin(dec_rad)
            + math.cos(lat_rad) * math.cos(dec_rad) * math.cos(ha_rad)
        )
        zenith = math.degrees(math.acos(max(-1, min(1, cos_zenith))))
        elevation = 90 - zenith

        # Solar azimuth
        if hour_angle > 0:
            azimuth = (
                math.degrees(
                    math.acos(
                        (math.sin(lat_rad) * cos_zenith - math.sin(dec_rad))
                        / (math.cos(lat_rad) * math.sin(math.radians(zenith)))
                    )
                )
                + 180
            ) % 360
        else:
            azimuth = (
                540
                - math.degrees(
                    math.acos(
                        (math.sin(lat_rad) * cos_zenith - math.sin(dec_rad))
                        / (math.cos(lat_rad) * math.sin(math.radians(zenith)))
                    )
                )
            ) % 360

        return SunPosition(
            elevation=elevation,
            azimuth=azimuth,
            timestamp=time.time(),
        )

    def _julian_day(self, dt: datetime) -> float:
        """Calculate Julian Day from datetime."""
        a = (14 - dt.month) // 12
        y = dt.year + 4800 - a
        m = dt.month + 12 * a - 3

        jdn = (
            dt.day
            + (153 * m + 2) // 5
            + 365 * y
            + y // 4
            - y // 100
            + y // 400
            - 32045
        )

        jd = (
            jdn
            + (dt.hour - 12) / 24
            + dt.minute / 1440
            + dt.second / 86400
        )

        return jd

    def calculate_irradiance(self, sun_elevation: float, altitude_m: float) -> float:
        """
        Calculate solar irradiance at altitude.

        Args:
            sun_elevation: Sun elevation angle (degrees)
            altitude_m: Aircraft altitude (meters)

        Returns:
            Solar irradiance (W/m^2)
        """
        if sun_elevation <= 0:
            return 0.0

        # Air mass calculation (Kasten-Young formula)
        zenith = 90 - sun_elevation
        zenith_rad = math.radians(zenith)

        if zenith < 90:
            air_mass = 1 / (
                math.cos(zenith_rad)
                + 0.50572 * (96.07995 - zenith) ** -1.6364
            )
        else:
            air_mass = 40  # Near horizon limit

        # Atmospheric pressure ratio at altitude
        # Standard atmosphere: P = P0 * exp(-altitude / 8500)
        pressure_ratio = math.exp(-altitude_m / 8500)

        # Effective air mass at altitude
        effective_air_mass = air_mass * pressure_ratio

        # Direct normal irradiance (simplified Bird model)
        # At stratospheric altitudes, very little atmospheric absorption
        tau_r = 0.97  # Rayleigh scattering
        tau_a = 0.99  # Aerosol (minimal at altitude)
        tau_o = 0.99  # Ozone
        tau_w = 1.0  # Water vapor (none at altitude)

        transmittance = (
            tau_r ** effective_air_mass
            * tau_a ** effective_air_mass
            * tau_o ** effective_air_mass
            * tau_w ** effective_air_mass
        )

        # Irradiance on surface perpendicular to sun
        dni = self.SOLAR_CONSTANT * transmittance

        # Irradiance on horizontal surface
        ghi = dni * math.sin(math.radians(sun_elevation))

        return max(0, ghi)

    def calculate_power(self, irradiance: float, sun_elevation: float) -> float:
        """
        Calculate available solar power.

        Args:
            irradiance: Solar irradiance (W/m^2)
            sun_elevation: Sun elevation (degrees)

        Returns:
            Available power (watts)
        """
        if irradiance <= 0 or sun_elevation <= 0:
            return 0.0

        cfg = self.config

        # Panel receives portion of irradiance based on tilt
        # For wing-mounted panels, assume horizontal
        effective_irradiance = irradiance * math.sin(
            math.radians(sun_elevation + cfg.panel_tilt_deg)
        )

        # Calculate power
        power = (
            effective_irradiance
            * cfg.panel_area_m2
            * cfg.panel_efficiency
            * cfg.soiling_factor
            * cfg.inverter_efficiency
        )

        return max(0, power)

    def calculate_optimal_heading(self, sun_pos: SunPosition) -> float:
        """
        Calculate optimal heading for maximum solar power.

        For horizontal panels, perpendicular to sun is optimal.
        """
        if sun_pos.elevation <= 0:
            return self._heading  # Keep current heading at night

        # Optimal heading is perpendicular to sun (wing faces sun)
        # Add 90 degrees to sun azimuth
        optimal = (sun_pos.azimuth + 90) % 360

        return optimal

    def calculate_heading_gain(
        self,
        current_heading: float,
        sun_pos: SunPosition,
    ) -> float:
        """
        Calculate power gain from flying optimal heading vs current.

        Returns percentage gain (0-100).
        """
        if sun_pos.elevation <= 0:
            return 0.0

        optimal_heading = self.calculate_optimal_heading(sun_pos)

        # Calculate heading difference
        diff = abs(current_heading - optimal_heading)
        if diff > 180:
            diff = 360 - diff

        # Power varies with cosine of heading difference
        # At 0 degrees difference: 100% power
        # At 90 degrees difference: ~70% power (wing still gets light)
        current_factor = 0.7 + 0.3 * math.cos(math.radians(diff))
        optimal_factor = 1.0

        gain = ((optimal_factor - current_factor) / current_factor) * 100

        return max(0, gain)

    def predict_power(
        self,
        latitude: float,
        longitude: float,
        altitude_m: float,
        dt: datetime,
    ) -> float:
        """
        Predict solar power at given time and location.

        Args:
            latitude: Latitude (degrees)
            longitude: Longitude (degrees)
            altitude_m: Altitude (meters)
            dt: Datetime for prediction

        Returns:
            Predicted power (watts)
        """
        sun_pos = self.calculate_sun_position(latitude, longitude, dt)
        irradiance = self.calculate_irradiance(sun_pos.elevation, altitude_m)
        return self.calculate_power(irradiance, sun_pos.elevation)

    def get_daylight_hours(
        self,
        latitude: float,
        longitude: float,
        date: datetime,
    ) -> Tuple[datetime, datetime]:
        """
        Get sunrise and sunset times for location.

        Returns:
            Tuple of (sunrise, sunset) datetime objects
        """
        # Simplified calculation - for production use, consider using astral library
        from datetime import timedelta

        # Calculate at noon
        noon = date.replace(hour=12, minute=0, second=0, microsecond=0)
        sun_noon = self.calculate_sun_position(latitude, longitude, noon)

        # Approximate day length based on declination
        # This is simplified - use astral library for accurate times
        lat_rad = math.radians(latitude)

        # Hour angle at sunrise/sunset (when elevation = 0)
        # cos(hour_angle) = -tan(lat) * tan(declination)
        dec_rad = math.radians(sun_noon.elevation + 23.5 * math.sin(math.radians(360 * (date.timetuple().tm_yday - 81) / 365)))

        cos_ha = -math.tan(lat_rad) * math.tan(dec_rad)
        cos_ha = max(-1, min(1, cos_ha))  # Clamp for polar regions

        if cos_ha <= -1:
            # Polar day
            sunrise = noon.replace(hour=0)
            sunset = noon.replace(hour=23, minute=59)
        elif cos_ha >= 1:
            # Polar night
            sunrise = sunset = noon
        else:
            hour_angle = math.degrees(math.acos(cos_ha))
            day_length_hours = 2 * hour_angle / 15

            half_day = timedelta(hours=day_length_hours / 2)
            sunrise = noon - half_day
            sunset = noon + half_day

        return sunrise, sunset

    def is_daylight(self) -> bool:
        """Check if currently daylight."""
        if self.status:
            return self.status.is_daylight
        return True  # Default to day

    def get_status_dict(self) -> dict:
        """Get status as dictionary."""
        if not self.status:
            return {}

        return {
            "sun_elevation": round(self.status.sun_position.elevation, 1),
            "sun_azimuth": round(self.status.sun_position.azimuth, 1),
            "irradiance_w_m2": round(self.status.irradiance_w_m2, 1),
            "power_available_w": round(self.status.power_available_w, 1),
            "power_actual_w": round(self.status.power_actual_w, 1),
            "heading_optimal_deg": round(self.status.heading_optimal_deg, 1),
            "heading_gain_percent": round(self.status.heading_gain_percent, 1),
            "is_daylight": self.status.is_daylight,
        }
