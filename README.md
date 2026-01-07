# HAPS Glider - Autonomous Stratospheric Flight System

An AI-powered autonomous flight control system for High Altitude Pseudo-Satellite (HAPS) solar gliders, integrating with ArduPilot for multi-day stratospheric missions.

## Overview

HAPS Glider is a companion computer software system that runs alongside ArduPilot to enable fully autonomous stratospheric operations. It provides:

- **Energy Management**: Solar power optimization and battery management for day/night cycles
- **Station Keeping**: Wind-compensated position maintenance in the stratosphere
- **Autonomous Decision Making**: AI agent with mission state machine for multi-day operations
- **ArduPilot Integration**: Full MAVLink communication with ArduPlane

## Target Platform

- **Altitude**: 18,000m - 25,000m (stratosphere)
- **Endurance**: Multi-day missions (48+ hours)
- **Power**: Solar panels + liquid lithium batteries
- **Airframe**: High aspect ratio glider (L/D > 30)

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    HAPS GLIDER SYSTEM                           │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐    MAVLink     ┌─────────────────────────┐ │
│  │   ArduPilot     │◄──────────────►│   Companion Computer    │ │
│  │   (Plane)       │                │   (HAPS Glider Agent)   │ │
│  │                 │                │                         │ │
│  │  - Stabilization│                │  - Energy Optimization  │ │
│  │  - Basic Nav    │                │  - Station Keeping      │ │
│  │  - Failsafes    │                │  - Mission Planning     │ │
│  │  - Sensors      │                │  - AI Decision Engine   │ │
│  └─────────────────┘                └─────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

## Quick Start

### Prerequisites

- Python 3.10+
- ArduPilot SITL (for testing)
- NVIDIA Jetson (for deployment) or any Linux system

### Installation

```bash
# Clone the repository
cd /path/to/HAPSglider

# Install dependencies
pip install -r requirements.txt

# Install package in development mode
pip install -e .
```

### Running with SITL

1. Start ArduPilot SITL in one terminal:
```bash
# If you have ArduPilot installed
sim_vehicle.py -v ArduPlane --console --map
```

2. Run the HAPS Glider agent in another terminal:
```bash
python src/main.py --sitl
```

### Configuration

Edit `config/default.yaml` for your specific setup:

```yaml
mavlink:
  connection_string: "udp:127.0.0.1:14550"

flight:
  altitude_min_m: 18000.0
  altitude_max_m: 25000.0

energy:
  battery_capacity_wh: 600.0
  solar_panel_area_m2: 4.0
```

## Features

### Energy Management

The system continuously optimizes energy usage based on:

- **Solar Position**: Calculates sun elevation and azimuth for power prediction
- **Battery State**: Monitors SOC, voltage, current, and temperature
- **Power Budget**: Allocates power to flight systems, payload, and reserves
- **Day/Night Modes**: Automatic transition between operational modes

```python
# Energy modes
DAY_NORMAL      # Full operations, charging
DAY_CHARGING    # Priority charging, reduced payload
DUSK_TRANSITION # Preparing for night
NIGHT_CRUISE    # Minimum power, maintaining altitude
DAWN_TRANSITION # Waiting for sunrise
EMERGENCY       # Critical battery, RTL
```

### Station Keeping

Maintains position over target area despite stratospheric winds:

- **Loiter Patterns**: Circle, racetrack, figure-eight
- **Wind Compensation**: Estimates wind from drift and adjusts heading
- **Drift Correction**: Automatic corrections when position deviates
- **Altitude Management**: Energy-optimized altitude band control

### AI Decision Engine

State machine with autonomous decision making:

```
PREFLIGHT → LAUNCHING → CLIMBING_TO_ALTITUDE → STATION_KEEPING
                                                      ↓
                         ┌──────────────────────────────┐
                         ↓                              ↓
                   TRANSIT ←→ SURVEY ←→ TRACKING
                         ↓                              ↓
                   ENERGY_RECOVERY ←→ NIGHT_CRUISE
                         ↓
                    RTL → LANDING → LANDED
```

## Module Reference

| Module | Description |
|--------|-------------|
| `mavlink/` | MAVLink communication with ArduPilot |
| `flight/` | Flight control commands and navigation |
| `energy/` | Solar and battery management |
| `station_keeping/` | Position maintenance algorithms |
| `ai/` | Autonomous decision engine |

## Documentation

- [Architecture Guide](docs/ARCHITECTURE.md)
- [API Reference](docs/API.md)
- [Configuration Guide](docs/CONFIGURATION.md)
- [Testing Guide](docs/TESTING.md)

## Hardware Requirements

### Flight Controller
- Pixhawk 6X or Cube Orange+
- ArduPlane 4.4+

### Companion Computer
- NVIDIA Jetson Orin NX (recommended)
- Raspberry Pi 4/5 (basic operations)

### Sensors
- Dual GPS with RTK
- Airspeed sensor (MS4525DO)
- High-altitude barometer
- Solar irradiance sensor (optional)

## Safety Features

1. **Geofencing**: Altitude and radius limits enforced
2. **Battery Failsafes**: Automatic RTL on low/critical battery
3. **Link Loss Handling**: 48-hour autonomous operation, then RTL
4. **Emergency Descent**: Controlled descent if critical failure

## License

Proprietary - All Rights Reserved

## Contributing

Contact the HAPS Development Team for contribution guidelines.
