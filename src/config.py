"""
Configuration management for HAPS Glider system.
"""

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import yaml


@dataclass
class MAVLinkConfig:
    """MAVLink connection configuration."""
    connection_string: str = "udp:127.0.0.1:14550"  # SITL default
    baud_rate: int = 57600
    source_system: int = 255
    source_component: int = 0
    heartbeat_interval: float = 1.0  # seconds
    timeout: float = 30.0  # seconds


@dataclass
class EnergyConfig:
    """Energy management configuration."""
    battery_capacity_wh: float = 600.0  # 50Ah * 12V nominal
    battery_cells: int = 12
    cell_voltage_min: float = 3.5
    cell_voltage_max: float = 4.2
    cell_voltage_nominal: float = 3.7
    critical_soc_percent: float = 20.0
    low_soc_percent: float = 40.0
    solar_panel_area_m2: float = 4.0
    solar_efficiency: float = 0.22  # 22% efficiency cells
    power_baseline_w: float = 10.0  # Minimum flight systems


@dataclass
class FlightConfig:
    """Flight control configuration."""
    altitude_min_m: float = 18000.0  # ~59,000 ft
    altitude_max_m: float = 25000.0  # ~82,000 ft
    altitude_target_m: float = 20000.0
    airspeed_min_ms: float = 15.0
    airspeed_max_ms: float = 30.0
    airspeed_cruise_ms: float = 20.0
    loiter_radius_m: float = 1000.0
    waypoint_radius_m: float = 200.0
    max_bank_angle_deg: float = 30.0
    max_climb_rate_ms: float = 3.0
    max_sink_rate_ms: float = 0.5


@dataclass
class StationKeepingConfig:
    """Station keeping configuration."""
    geofence_radius_m: float = 50000.0  # 50km operational radius
    position_tolerance_m: float = 500.0
    altitude_tolerance_m: float = 100.0
    wind_compensation_enabled: bool = True
    drift_correction_interval_s: float = 60.0


@dataclass
class PayloadConfig:
    """Payload system configuration."""
    camera_enabled: bool = True
    comms_relay_enabled: bool = True
    max_payload_power_w: float = 100.0
    camera_power_w: float = 15.0
    comms_power_w: float = 50.0


@dataclass
class SafetyConfig:
    """Safety system configuration."""
    geofence_enabled: bool = True
    link_loss_timeout_s: float = 300.0  # 5 minutes
    link_loss_action: str = "continue"  # continue, rtl, loiter
    low_battery_action: str = "rtl"
    critical_battery_action: str = "land"
    emergency_descent_rate_ms: float = 5.0


@dataclass
class HAPSConfig:
    """Main configuration container."""
    mavlink: MAVLinkConfig = field(default_factory=MAVLinkConfig)
    energy: EnergyConfig = field(default_factory=EnergyConfig)
    flight: FlightConfig = field(default_factory=FlightConfig)
    station_keeping: StationKeepingConfig = field(default_factory=StationKeepingConfig)
    payload: PayloadConfig = field(default_factory=PayloadConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)

    # Operating location
    home_lat: float = -33.8688  # Sydney, Australia
    home_lon: float = 151.2093
    home_alt_m: float = 0.0

    @classmethod
    def from_yaml(cls, path: Path) -> "HAPSConfig":
        """Load configuration from YAML file."""
        with open(path, "r") as f:
            data = yaml.safe_load(f)

        config = cls()

        if "mavlink" in data:
            config.mavlink = MAVLinkConfig(**data["mavlink"])
        if "energy" in data:
            config.energy = EnergyConfig(**data["energy"])
        if "flight" in data:
            config.flight = FlightConfig(**data["flight"])
        if "station_keeping" in data:
            config.station_keeping = StationKeepingConfig(**data["station_keeping"])
        if "payload" in data:
            config.payload = PayloadConfig(**data["payload"])
        if "safety" in data:
            config.safety = SafetyConfig(**data["safety"])

        if "home_lat" in data:
            config.home_lat = data["home_lat"]
        if "home_lon" in data:
            config.home_lon = data["home_lon"]
        if "home_alt_m" in data:
            config.home_alt_m = data["home_alt_m"]

        return config

    def to_yaml(self, path: Path) -> None:
        """Save configuration to YAML file."""
        data = {
            "mavlink": self.mavlink.__dict__,
            "energy": self.energy.__dict__,
            "flight": self.flight.__dict__,
            "station_keeping": self.station_keeping.__dict__,
            "payload": self.payload.__dict__,
            "safety": self.safety.__dict__,
            "home_lat": self.home_lat,
            "home_lon": self.home_lon,
            "home_alt_m": self.home_alt_m,
        }

        with open(path, "w") as f:
            yaml.dump(data, f, default_flow_style=False)


def get_config(config_path: Optional[Path] = None) -> HAPSConfig:
    """Get configuration, loading from file if provided."""
    if config_path and config_path.exists():
        return HAPSConfig.from_yaml(config_path)
    return HAPSConfig()
