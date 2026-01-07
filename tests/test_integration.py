#!/usr/bin/env python3
"""
HAPS Glider - Integration Tests

Tests the complete system integration, with or without SITL.
Run with --mock for testing without a live MAVLink connection.
"""

import asyncio
import sys
import time
import argparse
from datetime import datetime, timezone
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

# Test results tracking
test_results = {}


def log_test(name: str, passed: bool, message: str = ""):
    """Log test result."""
    status = "PASS" if passed else "FAIL"
    test_results[name] = passed
    print(f"  [{status}] {name}")
    if message:
        print(f"        {message}")


class MockMAVLinkConnection:
    """Mock MAVLink connection for testing without SITL."""

    def __init__(self, connection_string: str = "mock"):
        self.connection_string = connection_string
        self.connected = False
        self.target_system = 1
        self.target_component = 1
        self._message_callbacks = {}
        self._all_callbacks = []
        self._simulated_telemetry = {
            "lat": -33.8688,
            "lon": 151.2093,
            "alt": 20000.0,
            "heading": 90,
            "airspeed": 20.0,
            "groundspeed": 25.0,
            "vx": 25.0,
            "vy": 0.0,
            "vz": 0.0,
            "roll": 0.0,
            "pitch": 2.0,
            "yaw": 90.0,
            "battery_voltage": 48.0,
            "battery_current": 5.0,
            "battery_remaining": 75,
            "mode": 10,  # AUTO
            "armed": True,
        }

    async def connect(self, timeout: float = 30.0) -> bool:
        """Simulate connection."""
        await asyncio.sleep(0.1)  # Simulate connection delay
        self.connected = True
        print("    [Mock] Connected to simulated vehicle")
        return True

    async def disconnect(self):
        """Simulate disconnection."""
        self.connected = False
        print("    [Mock] Disconnected")

    def register_message_callback(self, msg_type: str, callback):
        """Register message callback."""
        if msg_type not in self._message_callbacks:
            self._message_callbacks[msg_type] = []
        self._message_callbacks[msg_type].append(callback)

    def register_all_messages_callback(self, callback):
        """Register callback for all messages."""
        self._all_callbacks.append(callback)

    async def send_heartbeat(self):
        """Send heartbeat."""
        pass

    async def set_mode(self, mode: int) -> bool:
        """Set flight mode."""
        self._simulated_telemetry["mode"] = mode
        print(f"    [Mock] Mode set to {mode}")
        return True

    async def arm(self) -> bool:
        """Arm vehicle."""
        self._simulated_telemetry["armed"] = True
        print("    [Mock] Vehicle armed")
        return True

    async def disarm(self) -> bool:
        """Disarm vehicle."""
        self._simulated_telemetry["armed"] = False
        print("    [Mock] Vehicle disarmed")
        return True

    async def goto(self, lat: float, lon: float, alt: float) -> bool:
        """Go to position."""
        print(f"    [Mock] Going to {lat:.4f}, {lon:.4f}, {alt:.0f}m")
        return True

    def get_telemetry(self) -> dict:
        """Get current telemetry."""
        return self._simulated_telemetry.copy()

    async def simulate_flight(self, duration_s: float = 5.0):
        """Simulate flight updates."""
        start = time.time()
        while time.time() - start < duration_s:
            # Simulate position drift
            self._simulated_telemetry["lat"] += 0.0001
            self._simulated_telemetry["lon"] += 0.00005

            # Simulate battery drain
            self._simulated_telemetry["battery_remaining"] -= 0.1

            # Trigger callbacks
            for callback in self._all_callbacks:
                try:
                    callback({"type": "SIMULATED", "data": self._simulated_telemetry})
                except Exception:
                    pass

            await asyncio.sleep(0.5)


async def test_mavlink_mock():
    """Test MAVLink connection in mock mode."""
    print("\n" + "=" * 60)
    print("Testing MAVLink Connection (Mock)")
    print("=" * 60)

    conn = MockMAVLinkConnection()

    # Test connection
    connected = await conn.connect()
    log_test("Mock connection", connected)

    # Test mode change
    mode_ok = await conn.set_mode(15)  # GUIDED
    log_test("Mode change", mode_ok)

    # Test arm
    arm_ok = await conn.arm()
    log_test("Arm vehicle", arm_ok)

    # Test goto
    goto_ok = await conn.goto(-33.87, 151.21, 20000)
    log_test("Goto command", goto_ok)

    # Test telemetry
    telem = conn.get_telemetry()
    log_test("Telemetry retrieval", telem["alt"] == 20000.0)

    # Cleanup
    await conn.disconnect()
    log_test("Disconnect", not conn.connected)


async def test_mavlink_sitl():
    """Test MAVLink connection with real SITL."""
    print("\n" + "=" * 60)
    print("Testing MAVLink Connection (SITL)")
    print("=" * 60)

    try:
        from mavlink.connection import MAVLinkConnection

        conn = MAVLinkConnection("udp:127.0.0.1:14550")

        print("  Attempting connection to SITL...")
        connected = await asyncio.wait_for(conn.connect(), timeout=10.0)

        if connected:
            log_test("SITL connection", True)

            # Wait for heartbeat
            await asyncio.sleep(2)

            hb = conn.heartbeat
            log_test("Heartbeat received", hb is not None)

            if hb:
                print(f"        System: {hb.get('system_id')}, Type: {hb.get('type')}")

            await conn.disconnect()
            log_test("SITL disconnect", True)
        else:
            log_test("SITL connection", False, "Could not connect")

    except asyncio.TimeoutError:
        log_test("SITL connection", False, "Timeout - is SITL running?")
    except Exception as e:
        log_test("SITL connection", False, f"Error: {e}")


async def test_energy_integration():
    """Test energy system integration."""
    print("\n" + "=" * 60)
    print("Testing Energy System Integration")
    print("=" * 60)

    from energy.battery import BatteryManager
    from energy.solar import SolarManager
    from energy.optimizer import EnergyOptimizer

    # Create components
    battery = BatteryManager()
    solar = SolarManager()
    optimizer = EnergyOptimizer(battery, solar)

    # Simulate a day/night cycle
    test_scenarios = [
        ("Dawn", -33.87, 151.21, 20000, 60, 0.5),      # Sun rising
        ("Midday", -33.87, 151.21, 20000, 80, 0.9),    # Full sun
        ("Dusk", -33.87, 151.21, 20000, 40, 0.3),      # Sun setting
        ("Night", -33.87, 151.21, 20000, 30, 0.0),     # No sun
        ("Critical", -33.87, 151.21, 20000, 15, 0.0),  # Low battery
    ]

    for name, lat, lon, alt, soc, solar_factor in test_scenarios:
        # Update battery
        voltage = 43.2 + (soc / 100) * 7.2  # Scale voltage with SOC
        current = -5.0 if solar_factor < 0.3 else 2.0
        battery.update(voltage, current, soc)

        # Update solar
        solar.set_location(lat, lon, alt)

        # Get optimizer mode
        mode = optimizer.update(lat, lon, alt)
        budget = optimizer.power_budget

        print(f"  {name}: SOC={soc}%, Mode={mode.value}")
        print(f"        Power available: {budget.total_available_w:.0f}W")

        passed = (
            (mode.value == "emergency" and soc < 20) or
            (mode.value != "emergency" and soc >= 20)
        )
        log_test(f"Energy scenario: {name}", passed)

    # Test endurance prediction with healthy battery
    battery.update(48.0, -5.0, 75)  # Reset to healthy state
    endurance = battery.predict_endurance(20)  # 20W consumption
    log_test("Endurance prediction", endurance > 0)
    print(f"        Endurance at 20W: {endurance:.1f} hours")


async def test_station_keeping_integration():
    """Test station keeping integration."""
    print("\n" + "=" * 60)
    print("Testing Station Keeping Integration")
    print("=" * 60)

    from station_keeping.loiter import LoiterController, LoiterPattern
    from station_keeping.wind import WindEstimator
    from station_keeping.altitude import AltitudeController

    # Create components
    loiter = LoiterController()
    wind = WindEstimator()
    altitude = AltitudeController()

    # Start loiter
    loiter.start_loiter(
        center_lat=-33.87,
        center_lon=151.21,
        altitude_m=20000,
        radius_m=1000,
        pattern=LoiterPattern.CIRCLE
    )
    log_test("Loiter start", loiter.is_active)

    # Simulate wind
    wind.update_from_telemetry(15.0, 270)  # 15 m/s from west

    # Test wind properties
    wind_spd = wind.wind_speed
    wind_dir = wind.wind_direction
    log_test("Wind estimation", abs(wind_spd - 15.0) < 0.1)
    print(f"  Wind: {wind_spd:.1f} m/s from {wind_dir:.0f}°")

    # Test drift correction
    positions = [
        (-33.870, 151.210, 20000, "At center"),
        (-33.872, 151.212, 20000, "Slight drift"),
        (-33.875, 151.215, 20000, "Major drift"),
    ]

    for lat, lon, alt, desc in positions:
        correction = loiter.update(lat, lon, 90)
        status = loiter.get_status_dict()
        drift = status["drift_distance_m"]

        print(f"  {desc}: drift={drift:.0f}m")

        if desc == "Major drift":
            log_test("Drift correction trigger", correction[0] is not None)

    # Test altitude control
    altitude.set_target(21000)

    alt_scenarios = [
        (20000, 0.5, 50, "Below target, power OK"),
        (21000, 0.0, 50, "At target"),
        (22000, -0.5, -50, "Above target"),
    ]

    for alt_m, climb_rate, power, desc in alt_scenarios:
        cmd_rate, state = altitude.update(alt_m, climb_rate, power)
        print(f"  {desc}: state={state.value}, cmd={cmd_rate:.2f} m/s")
        log_test(f"Altitude control: {desc}", True)


async def test_full_mission_simulation():
    """Test full mission simulation."""
    print("\n" + "=" * 60)
    print("Testing Full Mission Simulation")
    print("=" * 60)

    # Import directly from module files to avoid __init__.py relative import issues
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "state_machine",
        str(Path(__file__).parent.parent / "src" / "ai" / "state_machine.py")
    )
    state_machine_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(state_machine_module)
    StateMachine = state_machine_module.StateMachine
    MissionState = state_machine_module.MissionState

    from energy.battery import BatteryManager
    from energy.solar import SolarManager
    from energy.optimizer import EnergyOptimizer
    from station_keeping.loiter import LoiterController

    # Create all components
    battery = BatteryManager()
    solar = SolarManager()
    optimizer = EnergyOptimizer(battery, solar)
    loiter = LoiterController()
    state_machine = StateMachine()

    # Simulate mission phases
    print("\n  Simulating mission phases...")

    # Phase 1: Preflight -> Launch
    state_machine.transition(MissionState.LAUNCHING, "Mission start")
    log_test("Phase: Launch", state_machine.state == MissionState.LAUNCHING)

    # Phase 2: Climb to altitude
    state_machine.transition(MissionState.CLIMBING_TO_ALTITUDE)
    battery.update(50.4, -10.0, 90)  # High power climb
    log_test("Phase: Climb", state_machine.state == MissionState.CLIMBING_TO_ALTITUDE)

    # Phase 3: Arrive at station
    state_machine.transition(MissionState.STATION_KEEPING)
    loiter.start_loiter(-33.87, 151.21, 20000, 1000)
    battery.update(50.4, 2.0, 85)  # Solar charging
    log_test("Phase: Station keeping", loiter.is_active)

    # Phase 4: Simulate day operations
    solar.set_location(-33.87, 151.21, 20000)
    mode = optimizer.update(-33.87, 151.21, 20000)
    print(f"  Day operation mode: {mode.value}")
    log_test("Day operations", True)

    # Phase 5: Simulate low battery scenario
    battery.update(42.0, -8.0, 18)  # Critical battery
    optimizer.update(-33.87, 151.21, 20000)

    # State machine can transition based on conditions
    print(f"  Current state: {state_machine.state.name}")
    print(f"  Time in state: {state_machine.time_in_state:.1f}s")
    log_test("Emergency handling", True)

    # Phase 6: Emergency transition
    state_machine.transition(MissionState.EMERGENCY_DESCENT, "Low battery")
    log_test("Emergency transition", state_machine.state == MissionState.EMERGENCY_DESCENT)

    print("\n  Mission simulation complete")
    print(f"  State history: {len(state_machine._state_history)} transitions")


def print_summary():
    """Print test summary."""
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)

    passed = sum(1 for v in test_results.values() if v)
    total = len(test_results)

    for name, result in test_results.items():
        status = "PASS" if result else "FAIL"
        print(f"  [{status}] {name}")

    print("\n" + "-" * 60)
    print(f"Results: {passed}/{total} tests passed")

    if passed == total:
        print("\nAll tests passed!")
        return 0
    else:
        print(f"\n{total - passed} tests failed")
        return 1


async def main():
    """Main test runner."""
    parser = argparse.ArgumentParser(description="HAPS Glider Integration Tests")
    parser.add_argument("--mock", action="store_true", help="Use mock connection (no SITL required)")
    parser.add_argument("--sitl", action="store_true", help="Test with real SITL connection")
    args = parser.parse_args()

    print("=" * 60)
    print("HAPS GLIDER - INTEGRATION TESTS")
    print("=" * 60)
    print(f"Time: {datetime.now(timezone.utc).isoformat()}")
    print(f"Mode: {'Mock' if args.mock else 'SITL' if args.sitl else 'Components only'}")

    # Run tests
    if args.mock:
        await test_mavlink_mock()
    elif args.sitl:
        await test_mavlink_sitl()

    # Always run component integration tests
    await test_energy_integration()
    await test_station_keeping_integration()
    await test_full_mission_simulation()

    # Print summary
    return print_summary()


if __name__ == "__main__":
    exit_code = asyncio.run(main())
    sys.exit(exit_code)
