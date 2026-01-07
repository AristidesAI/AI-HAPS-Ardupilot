"""
Tests for energy management module.
"""

import pytest
from datetime import datetime, timezone

import sys
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from pathlib import Path
from energy.battery import BatteryManager, BatteryConfig, BatteryState
from energy.solar import SolarManager, SolarConfig


class TestBatteryManager:
    """Tests for BatteryManager."""

    def test_init_default_config(self):
        """Test initialization with default config."""
        manager = BatteryManager()
        assert manager.config.capacity_ah == 50.0
        assert manager.config.cells == 12

    def test_update_charging(self):
        """Test battery update when charging."""
        manager = BatteryManager()
        manager.update(
            voltage=50.4,  # 4.2V per cell - full
            current=5.0,   # Positive = charging
            soc_percent=95.0,
            temperature=25.0,
        )
        assert manager.state == BatteryState.CHARGING
        assert manager.soc == 95.0

    def test_update_discharging(self):
        """Test battery update when discharging."""
        manager = BatteryManager()
        manager.update(
            voltage=44.4,  # 3.7V per cell - nominal
            current=-2.0,  # Negative = discharging
            soc_percent=60.0,
            temperature=25.0,
        )
        assert manager.state == BatteryState.DISCHARGING
        assert manager.soc == 60.0

    def test_critical_detection(self):
        """Test critical battery detection."""
        manager = BatteryManager()
        manager.update(
            voltage=36.0,  # 3.0V per cell - low
            current=-1.0,
            soc_percent=15.0,  # Below critical threshold
            temperature=25.0,
        )
        assert manager.is_critical

    def test_endurance_prediction(self):
        """Test endurance prediction."""
        manager = BatteryManager()
        manager.update(voltage=44.4, current=-2.0, soc_percent=80.0)

        # At 20W consumption with 80% SOC of 600Wh battery
        # (80% - 20% reserve) = 60% * 600Wh = 360Wh available
        # 360Wh / 20W = 18 hours
        endurance = manager.predict_endurance(20.0)
        assert endurance > 15  # Should be around 18 hours


class TestSolarManager:
    """Tests for SolarManager."""

    def test_init_default_config(self):
        """Test initialization with default config."""
        manager = SolarManager()
        assert manager.config.panel_area_m2 == 4.0
        assert manager.config.panel_efficiency == 0.22

    def test_sun_position_calculation(self):
        """Test sun position calculation."""
        manager = SolarManager()

        # Test at solar noon in Sydney summer
        dt = datetime(2024, 1, 15, 12, 0, 0, tzinfo=timezone.utc)
        pos = manager.calculate_sun_position(-33.87, 151.21, dt)

        # Sun should be high in the sky at noon in summer
        assert pos.elevation > 0  # Sun is up
        assert 0 <= pos.azimuth < 360

    def test_irradiance_at_altitude(self):
        """Test irradiance calculation at stratospheric altitude."""
        manager = SolarManager()

        # High sun angle, high altitude
        irradiance = manager.calculate_irradiance(
            sun_elevation=60.0,
            altitude_m=20000.0,
        )

        # Should be high at stratospheric altitude
        assert irradiance > 800  # Should be close to solar constant

    def test_no_power_at_night(self):
        """Test no power generation at night."""
        manager = SolarManager()

        irradiance = manager.calculate_irradiance(
            sun_elevation=-10.0,  # Below horizon
            altitude_m=20000.0,
        )

        assert irradiance == 0.0

    def test_power_calculation(self):
        """Test power calculation."""
        manager = SolarManager()

        # High irradiance, good sun angle
        power = manager.calculate_power(
            irradiance=1000.0,
            sun_elevation=60.0,
        )

        # Expected: 1000 * 4m² * 0.22 * 0.98 * 0.95 * sin(60+0)
        # Should be several hundred watts
        assert power > 500


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
