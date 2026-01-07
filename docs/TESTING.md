# HAPS Glider - Testing Guide

## Overview

This guide covers testing the HAPS Glider system using ArduPilot's Software-In-The-Loop (SITL) simulator.

## Prerequisites

### ArduPilot SITL Installation

#### Ubuntu/Debian

```bash
# Clone ArduPilot
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive

# Install prerequisites
Tools/environment_install/install-prereqs-ubuntu.sh -y

# Reload profile
. ~/.profile

# Build ArduPlane
./waf configure --board sitl
./waf plane
```

#### macOS

```bash
# Install dependencies
brew install python3 ccache gawk

# Clone and setup ArduPilot
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive

# Install Python dependencies
pip3 install -r Tools/environment_install/install-prereqs-mac.txt

# Build
./waf configure --board sitl
./waf plane
```

### HAPS Glider Installation

```bash
cd /path/to/HAPSglider

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Install in development mode
pip install -e .
```

## Running SITL

### Basic SITL Launch

```bash
# From ArduPilot directory
cd ardupilot

# Launch ArduPlane SITL
sim_vehicle.py -v ArduPlane --console --map
```

This opens:
- MAVProxy console (command line)
- Map window showing vehicle position

### SITL with Custom Location

```bash
# Sydney, Australia at 100m altitude
sim_vehicle.py -v ArduPlane \
    --console --map \
    --custom-location=-33.8688,151.2093,100,0
```

### SITL for HAPS Testing

```bash
# Launch with HAPS-friendly parameters
sim_vehicle.py -v ArduPlane \
    --console --map \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14551 \
    -w  # Wipe EEPROM for clean start
```

## Connecting HAPS Glider

### Terminal 1: SITL

```bash
cd ardupilot
sim_vehicle.py -v ArduPlane --console --map
```

### Terminal 2: HAPS Glider

```bash
cd HAPSglider
source venv/bin/activate

# Connect to SITL
python src/main.py --sitl

# Or with custom connection
python src/main.py --connection "udp:127.0.0.1:14550"
```

## SITL Commands (MAVProxy)

### Basic Commands

```bash
# In MAVProxy console:

# Arm the vehicle
arm throttle

# Disarm
disarm

# Set mode
mode GUIDED
mode AUTO
mode LOITER
mode RTL

# Takeoff (when armed)
takeoff 100

# Check status
status
```

### Simulating High Altitude

SITL doesn't simulate stratospheric conditions, but we can test at lower altitudes:

```bash
# Set altitude to 1000m for testing
mode GUIDED
arm throttle
takeoff 100

# In MAVProxy, fly to altitude
guided 1000  # meters
```

### Loading HAPS Parameters

```bash
# In MAVProxy console:
param load /path/to/HAPSglider/config/haps_sitl.param
```

### Simulating Wind

```bash
# Set wind speed and direction
param set SIM_WIND_SPD 10    # 10 m/s wind
param set SIM_WIND_DIR 270   # From west
```

## Test Scenarios

### Scenario 1: Basic Connection Test

**Objective**: Verify MAVLink connection and telemetry reception.

```python
# test_connection.py
import asyncio
from mavlink import MAVLinkConnection

async def test_connection():
    conn = MAVLinkConnection("udp:127.0.0.1:14550")

    if await conn.connect():
        print(f"Connected to system {conn.target_system}")
        print(f"Heartbeat: {conn.heartbeat}")
        await asyncio.sleep(5)
        await conn.disconnect()
        print("Test PASSED")
    else:
        print("Test FAILED: Could not connect")

asyncio.run(test_connection())
```

### Scenario 2: Telemetry Test

**Objective**: Verify all telemetry streams are received.

```python
# test_telemetry.py
import asyncio
from mavlink import MAVLinkConnection, MessageHandler

async def test_telemetry():
    conn = MAVLinkConnection("udp:127.0.0.1:14550")
    handler = MessageHandler()

    conn.register_all_messages_callback(handler.handle_message)

    if await conn.connect():
        # Wait for telemetry
        await asyncio.sleep(5)

        t = handler.telemetry
        print(f"Position: {t.position.lat}, {t.position.lon}")
        print(f"Altitude: {t.position.alt_msl}m")
        print(f"Battery: {t.battery.voltage}V, {t.battery.remaining}%")
        print(f"Mode: {t.system.mode}")

        await conn.disconnect()

asyncio.run(test_telemetry())
```

### Scenario 3: Flight Control Test

**Objective**: Test arming, mode changes, and navigation.

```python
# test_flight_control.py
import asyncio
from mavlink import MAVLinkConnection
from flight import FlightController, FlightMode

async def test_flight_control():
    conn = MAVLinkConnection("udp:127.0.0.1:14550")

    if not await conn.connect():
        print("Connection failed")
        return

    flight = FlightController(conn)
    flight.request_data_streams()
    await asyncio.sleep(2)

    # Test mode change
    print("Changing to GUIDED mode...")
    if await flight.set_mode(FlightMode.GUIDED):
        print("Mode change: PASSED")
    else:
        print("Mode change: FAILED")

    # Test arming (may fail in SITL without proper setup)
    print("Attempting to arm...")
    if await flight.arm():
        print("Arming: PASSED")

        # Test goto
        lat, lon = -33.87, 151.21
        print(f"Going to {lat}, {lon}...")
        await flight.goto(lat, lon, 200)

        await asyncio.sleep(10)
        await flight.disarm()
    else:
        print("Arming: FAILED (expected in SITL)")

    await conn.disconnect()

asyncio.run(test_flight_control())
```

### Scenario 4: Energy Management Test

**Objective**: Test battery and solar calculations.

```python
# test_energy.py
from energy import BatteryManager, SolarManager, EnergyOptimizer
from datetime import datetime, timezone

def test_energy():
    battery = BatteryManager()
    solar = SolarManager()
    optimizer = EnergyOptimizer(battery, solar)

    # Simulate battery state
    battery.update(voltage=44.4, current=-2.0, soc_percent=75.0)
    print(f"Battery state: {battery.state.value}")
    print(f"SOC: {battery.soc}%")
    print(f"Endurance at 20W: {battery.predict_endurance(20):.1f} hours")

    # Test solar calculations
    solar.set_location(-33.87, 151.21, 20000)
    solar.set_heading(90)
    status = solar.update()

    print(f"\nSolar status:")
    print(f"  Sun elevation: {status.sun_position.elevation:.1f}°")
    print(f"  Available power: {status.power_available_w:.0f}W")
    print(f"  Is daylight: {status.is_daylight}")

    # Test optimizer
    mode = optimizer.update(-33.87, 151.21, 20000)
    print(f"\nEnergy mode: {mode.value}")
    print(f"Power budget: {optimizer.power_budget.total_available_w:.0f}W")

test_energy()
```

### Scenario 5: Station Keeping Test

**Objective**: Test loiter pattern and drift correction.

```python
# test_station_keeping.py
from station_keeping import LoiterController, LoiterPattern

def test_station_keeping():
    loiter = LoiterController()

    # Start loiter
    loiter.start_loiter(
        center_lat=-33.87,
        center_lon=151.21,
        altitude_m=20000,
        radius_m=1000,
        pattern=LoiterPattern.CIRCLE
    )

    # Simulate position updates with drift
    positions = [
        (-33.870, 151.210),  # At center
        (-33.871, 151.211),  # Slight drift
        (-33.873, 151.213),  # More drift
        (-33.875, 151.215),  # Significant drift
    ]

    for lat, lon in positions:
        correction = loiter.update(lat, lon, 90)
        status = loiter.get_status_dict()

        print(f"Position: {lat}, {lon}")
        print(f"  Drift: {status['drift_distance_m']:.0f}m")

        if correction[0] is not None:
            print(f"  Correction: {correction[0]:.4f}, {correction[1]:.4f}")
        print()

test_station_keeping()
```

### Scenario 6: Full Integration Test

**Objective**: Test complete system with AI agent.

```python
# test_integration.py
import asyncio
from mavlink import MAVLinkConnection
from flight import FlightController
from ai import HAPSAgent

async def test_integration():
    # Connect
    conn = MAVLinkConnection("udp:127.0.0.1:14550")
    if not await conn.connect():
        print("Connection failed")
        return

    # Create components
    flight = FlightController(conn)
    agent = HAPSAgent(flight)

    # Set target
    agent.set_target(-33.87, 151.21, 20000)

    # Start agent
    await agent.start()

    # Let it run
    print("Agent running for 60 seconds...")
    for i in range(60):
        await asyncio.sleep(1)
        status = agent.get_status()
        print(f"[{i:02d}] State: {status['mission']['state']}, "
              f"SOC: {status['energy']['battery']['soc_percent']:.0f}%")

    # Stop
    await agent.stop()
    await conn.disconnect()
    print("Test complete")

asyncio.run(test_integration())
```

## Running Unit Tests

```bash
cd HAPSglider

# Run all tests
pytest tests/ -v

# Run specific test file
pytest tests/test_energy.py -v

# Run with coverage
pytest tests/ --cov=src --cov-report=html
```

## Debugging

### Enable Debug Logging

```bash
# Set log level
export HAPS_LOG_LEVEL=DEBUG

# Run with verbose output
python src/main.py --sitl -v
```

### MAVLink Message Debugging

```python
# Log all messages
def log_message(msg):
    print(f"[{msg.get_type()}] {msg}")

connection.register_all_messages_callback(log_message)
```

### Common SITL Issues

#### "Pre-arm: Throttle not zero"

In MAVProxy:
```bash
rc 3 1000  # Set throttle to minimum
```

#### "Pre-arm: GPS not healthy"

Wait for GPS lock (simulated, takes ~10 seconds).

#### "Pre-arm: Compass not calibrated"

```bash
param set COMPASS_LEARN 3  # Auto-learn compass
```

## Performance Testing

### Message Rate Test

```python
import time

messages_received = 0
start_time = time.time()

def count_messages(msg):
    global messages_received
    messages_received += 1

connection.register_all_messages_callback(count_messages)

# Run for 10 seconds
await asyncio.sleep(10)

elapsed = time.time() - start_time
rate = messages_received / elapsed
print(f"Message rate: {rate:.1f} msg/sec")
```

### CPU Usage Monitoring

```bash
# While running HAPS Glider
top -p $(pgrep -f "python.*main.py")
```

## Test Checklist

### Pre-Flight Checklist

- [ ] SITL running and connected
- [ ] GPS lock acquired
- [ ] Battery simulated > 50%
- [ ] No pre-arm errors
- [ ] Telemetry streaming

### Connection Tests

- [ ] UDP connection works
- [ ] TCP connection works
- [ ] Heartbeat received
- [ ] All telemetry types received

### Flight Control Tests

- [ ] Mode changes work
- [ ] Arming/disarming works
- [ ] Goto commands work
- [ ] Loiter commands work
- [ ] RTL works

### Energy Tests

- [ ] Battery state tracking
- [ ] Solar calculations (daylight)
- [ ] Solar calculations (night)
- [ ] Energy mode transitions

### Station Keeping Tests

- [ ] Loiter pattern generation
- [ ] Drift detection
- [ ] Drift correction
- [ ] Wind compensation

### AI Agent Tests

- [ ] Agent starts/stops
- [ ] State machine transitions
- [ ] Autonomous decisions
- [ ] Emergency handling
