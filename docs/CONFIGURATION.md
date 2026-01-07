# HAPS Glider - Configuration Guide

## Overview

HAPS Glider uses a hierarchical YAML configuration system. Configuration can be loaded from files or set programmatically.

## Configuration Files

### File Locations

```
config/
├── default.yaml           # Default configuration
├── ardupilot_params.yaml  # ArduPilot parameter presets
└── mission_profiles/      # Pre-defined mission configurations
    ├── surveillance.yaml
    └── relay.yaml
```

### Loading Configuration

```python
from config import HAPSConfig, get_config
from pathlib import Path

# Load from file
config = get_config(Path("config/default.yaml"))

# Or use defaults
config = HAPSConfig()

# Override specific values
config.mavlink.connection_string = "tcp:192.168.1.100:5760"
```

## Configuration Sections

### MAVLink Configuration

Connection settings for ArduPilot communication.

```yaml
mavlink:
  # Connection string (UDP, TCP, or serial)
  connection_string: "udp:127.0.0.1:14550"

  # Serial baud rate (if using serial)
  baud_rate: 57600

  # Source system/component ID for outgoing messages
  source_system: 255
  source_component: 0

  # Heartbeat interval in seconds
  heartbeat_interval: 1.0

  # Connection timeout in seconds
  timeout: 30.0
```

#### Connection String Formats

| Format | Example | Use Case |
|--------|---------|----------|
| UDP | `udp:127.0.0.1:14550` | SITL, local GCS |
| TCP Client | `tcp:192.168.1.100:5760` | Remote FC |
| TCP Server | `tcpin:0.0.0.0:5760` | FC connects to us |
| Serial | `/dev/ttyUSB0` | Direct serial |
| Serial (Jetson) | `/dev/ttyTHS1` | Jetson UART |

### Energy Configuration

Battery and solar panel settings.

```yaml
energy:
  # Battery capacity in watt-hours
  battery_capacity_wh: 600.0  # 50Ah @ 12V

  # Number of series cells
  battery_cells: 12

  # Cell voltage limits
  cell_voltage_min: 3.0   # Empty
  cell_voltage_max: 4.2   # Full
  cell_voltage_nominal: 3.7

  # State of charge thresholds (%)
  critical_soc_percent: 20.0  # Trigger emergency
  low_soc_percent: 40.0       # Trigger warning

  # Solar panel specifications
  solar_panel_area_m2: 4.0    # Total panel area
  solar_efficiency: 0.22       # 22% efficiency

  # Base power consumption
  power_baseline_w: 10.0      # Minimum flight systems
```

#### Battery Capacity Calculation

```
Capacity (Wh) = Capacity (Ah) × Nominal Voltage (V)
Capacity (Wh) = 50 Ah × 12 cells × 3.7 V/cell = 2220 Wh

For 12V nominal system:
Capacity (Wh) = 50 Ah × 12 V = 600 Wh
```

### Flight Configuration

Flight envelope and performance limits.

```yaml
flight:
  # Altitude limits (meters MSL)
  altitude_min_m: 18000.0    # ~59,000 ft - stratosphere floor
  altitude_max_m: 25000.0    # ~82,000 ft - operational ceiling
  altitude_target_m: 20000.0 # Default target

  # Airspeed limits (m/s)
  airspeed_min_ms: 15.0      # Stall margin
  airspeed_max_ms: 30.0      # Never exceed
  airspeed_cruise_ms: 20.0   # Optimal cruise

  # Navigation parameters
  loiter_radius_m: 1000.0    # Default loiter radius
  waypoint_radius_m: 200.0   # Waypoint acceptance

  # Attitude limits
  max_bank_angle_deg: 30.0
  max_climb_rate_ms: 3.0
  max_sink_rate_ms: 0.5
```

### Station Keeping Configuration

Position maintenance settings.

```yaml
station_keeping:
  # Geofence radius (meters)
  geofence_radius_m: 50000.0  # 50km operational area

  # Position tolerance before correction
  position_tolerance_m: 500.0

  # Altitude tolerance
  altitude_tolerance_m: 100.0

  # Wind compensation
  wind_compensation_enabled: true

  # How often to check drift (seconds)
  drift_correction_interval_s: 60.0
```

### Payload Configuration

Camera and communication relay settings.

```yaml
payload:
  # Enable/disable payloads
  camera_enabled: true
  comms_relay_enabled: true

  # Power limits
  max_payload_power_w: 100.0  # Total payload budget
  camera_power_w: 15.0        # Camera consumption
  comms_power_w: 50.0         # Relay consumption
```

### Safety Configuration

Failsafe and emergency settings.

```yaml
safety:
  # Geofencing
  geofence_enabled: true

  # Link loss handling
  link_loss_timeout_s: 300.0  # 5 minutes
  link_loss_action: "continue"  # continue, rtl, loiter

  # Battery failsafes
  low_battery_action: "rtl"
  critical_battery_action: "land"

  # Emergency descent rate
  emergency_descent_rate_ms: 5.0
```

#### Link Loss Actions

| Action | Description |
|--------|-------------|
| `continue` | Continue mission autonomously |
| `rtl` | Return to launch immediately |
| `loiter` | Hold position until link recovery |

### Home Location

Default home/launch position.

```yaml
# Sydney, Australia
home_lat: -33.8688
home_lon: 151.2093
home_alt_m: 0.0
```

## ArduPilot Parameters

### Parameter File Format

```yaml
# config/ardupilot_params.yaml

flight_modes:
  FLTMODE1: 10   # AUTO
  FLTMODE2: 11   # RTL
  FLTMODE6: 15   # GUIDED

soaring:
  SOARING_ENABLE: 1
  SOAR_ALT_MAX: 25000
  SOAR_ALT_MIN: 18000

airspeed:
  ARSPD_FBW_MIN: 15
  ARSPD_FBW_MAX: 30
  TRIM_ARSPD_CM: 2000
```

### Critical Parameters for HAPS

#### Soaring Parameters

```yaml
SOARING_ENABLE: 1        # Enable soaring
SOAR_ALT_MAX: 25000      # Max soaring altitude (m)
SOAR_ALT_MIN: 18000      # Min soaring altitude (m)
SOAR_VSPEED: 0.5         # Min climb to detect thermal
```

#### TECS (Energy Control)

```yaml
TECS_SINK_MAX: 0.5       # Glider-optimized sink rate
TECS_CLMB_MAX: 3.0       # Max climb rate
TECS_TIME_CONST: 8.0     # Longer time constant for stability
```

#### Navigation

```yaml
NAVL1_PERIOD: 25         # Larger for stratospheric turns
WP_RADIUS: 200           # Waypoint acceptance (m)
WP_LOITER_RAD: 1000      # Default loiter radius (m)
```

#### Geofencing

```yaml
FENCE_ENABLE: 1          # Enable geofence
FENCE_TYPE: 7            # Altitude + circle
FENCE_ALT_MAX: 26000     # Max altitude (m)
FENCE_ALT_MIN: 17000     # Min altitude (m)
FENCE_RADIUS: 50000      # 50km radius
FENCE_ACTION: 1          # RTL on breach
```

#### Battery Failsafes

```yaml
BATT_MONITOR: 4          # Analog voltage + current
BATT_CAPACITY: 50000     # 50Ah capacity
BATT_CRT_VOLT: 42.0      # Critical voltage (V)
BATT_LOW_VOLT: 46.8      # Low voltage (V)
BATT_FS_LOW_ACT: 2       # RTL on low
BATT_FS_CRT_ACT: 1       # Land on critical
```

## Environment Variables

Override configuration via environment:

```bash
# Connection string
export HAPS_MAVLINK_CONNECTION="tcp:192.168.1.100:5760"

# Log level
export HAPS_LOG_LEVEL="DEBUG"

# Config file
export HAPS_CONFIG_PATH="/etc/haps/production.yaml"
```

## Mission Profiles

Pre-defined mission configurations:

### Surveillance Profile

```yaml
# config/mission_profiles/surveillance.yaml

# Inherit from default
extends: default

flight:
  altitude_target_m: 20000
  loiter_radius_m: 500  # Tighter loiter for coverage

payload:
  camera_enabled: true
  comms_relay_enabled: false  # Save power

station_keeping:
  position_tolerance_m: 200  # Stricter position hold
```

### Communication Relay Profile

```yaml
# config/mission_profiles/relay.yaml

extends: default

flight:
  altitude_target_m: 22000  # Higher for coverage
  loiter_radius_m: 2000     # Wider pattern

payload:
  camera_enabled: false
  comms_relay_enabled: true
  comms_power_w: 80  # Higher power for relay
```

## Platform-Specific Configuration

### NVIDIA Jetson

```yaml
mavlink:
  connection_string: "/dev/ttyTHS1"  # Jetson UART
  baud_rate: 921600

# Enable GPU acceleration
compute:
  use_gpu: true
  cuda_device: 0
```

### Raspberry Pi

```yaml
mavlink:
  connection_string: "/dev/ttyAMA0"  # RPi UART
  baud_rate: 57600

# Disable GPU (not available)
compute:
  use_gpu: false
```

## Configuration Validation

The system validates configuration on load:

```python
from config import HAPSConfig

config = HAPSConfig()

# Validation happens automatically
# Invalid values raise ConfigurationError

# Manual validation
config.validate()  # Raises if invalid
```

### Validation Rules

- `altitude_min_m` < `altitude_max_m`
- `airspeed_min_ms` < `airspeed_max_ms`
- `critical_soc_percent` < `low_soc_percent`
- `cell_voltage_min` < `cell_voltage_max`
- Connection string must be valid format
