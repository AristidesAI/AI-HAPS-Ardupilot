# HAPS Glider - Architecture Guide

## System Overview

The HAPS Glider system is designed as a layered architecture where each layer has specific responsibilities and communicates with adjacent layers through well-defined interfaces.

## Layer Architecture

```
┌───────────────────────────────────────────────────────────────────────┐
│                         APPLICATION LAYER                              │
│  ┌─────────────────────────────────────────────────────────────────┐  │
│  │                      AI Decision Engine                          │  │
│  │  - Mission State Machine    - Autonomous Decisions               │  │
│  │  - Task Scheduling          - Anomaly Detection                  │  │
│  └─────────────────────────────────────────────────────────────────┘  │
├───────────────────────────────────────────────────────────────────────┤
│                          DOMAIN LAYER                                  │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────────┐    │
│  │   Energy     │  │   Station    │  │      Flight              │    │
│  │   Manager    │  │   Keeping    │  │      Control             │    │
│  │              │  │              │  │                          │    │
│  │ - Solar      │  │ - Loiter     │  │ - Mode Management        │    │
│  │ - Battery    │  │ - Wind       │  │ - Navigation             │    │
│  │ - Optimizer  │  │ - Altitude   │  │ - Waypoints              │    │
│  └──────────────┘  └──────────────┘  └──────────────────────────┘    │
├───────────────────────────────────────────────────────────────────────┤
│                        COMMUNICATION LAYER                             │
│  ┌─────────────────────────────────────────────────────────────────┐  │
│  │                     MAVLink Interface                            │  │
│  │  - Connection Management    - Message Routing                    │  │
│  │  - Telemetry Reception      - Command Transmission               │  │
│  │  - Parameter Management     - Heartbeat Monitoring               │  │
│  └─────────────────────────────────────────────────────────────────┘  │
├───────────────────────────────────────────────────────────────────────┤
│                         HARDWARE LAYER                                 │
│  ┌──────────────────────┐  ┌────────────────────────────────────┐    │
│  │   ArduPilot FC       │  │     Companion Computer             │    │
│  │   (Pixhawk)          │  │     (Jetson/RPi)                   │    │
│  └──────────────────────┘  └────────────────────────────────────┘    │
└───────────────────────────────────────────────────────────────────────┘
```

## Module Details

### 1. MAVLink Module (`src/mavlink/`)

Handles all communication with the ArduPilot flight controller.

#### Components

**connection.py** - Connection Management
```python
class MAVLinkConnection:
    """
    Manages MAVLink connection lifecycle.

    Responsibilities:
    - Establish UDP/TCP/Serial connections
    - Send/receive heartbeats
    - Route messages to handlers
    - Monitor connection health
    """
```

**messages.py** - Telemetry Processing
```python
class MessageHandler:
    """
    Processes incoming MAVLink messages.

    Supported Messages:
    - HEARTBEAT: System status, flight mode
    - GLOBAL_POSITION_INT: GPS position
    - ATTITUDE: Roll, pitch, yaw
    - VFR_HUD: Airspeed, groundspeed, heading
    - SYS_STATUS: Battery, CPU load
    - BATTERY_STATUS: Detailed battery info
    - WIND: Wind estimation
    """

@dataclass
class TelemetryData:
    """Aggregated telemetry state."""
    position: Position
    attitude: Attitude
    velocity: Velocity
    battery: BatteryStatus
    system: SystemStatus
    wind: WindEstimate
```

**parameters.py** - Parameter Management
```python
class ParameterManager:
    """
    Manages ArduPilot parameters.

    Features:
    - Read/write individual parameters
    - Fetch all parameters
    - Apply HAPS-specific defaults
    - Export/import parameter files
    """
```

### 2. Flight Module (`src/flight/`)

High-level flight control interface.

#### Components

**controller.py** - Flight Controller Interface
```python
class FlightController:
    """
    High-level flight control.

    Commands:
    - arm() / disarm(): Vehicle arming
    - set_mode(): Change flight mode
    - goto(): Navigate to position
    - loiter_at(): Loiter at location
    - set_altitude(): Change altitude
    - set_airspeed(): Change airspeed
    - return_to_launch(): Initiate RTL
    """
```

**modes.py** - Flight Mode Management
```python
class FlightMode(IntEnum):
    """ArduPlane flight modes."""
    MANUAL = 0
    STABILIZE = 2
    FBWB = 6
    CRUISE = 7
    AUTO = 10
    RTL = 11
    LOITER = 12
    GUIDED = 15
    THERMAL = 24

class ModeManager:
    """
    Manages flight mode transitions.

    Features:
    - Track current mode from heartbeat
    - Request mode changes
    - Validate mode transitions
    """
```

**navigation.py** - Navigation Control
```python
class NavigationController:
    """
    Waypoint and guided navigation.

    Features:
    - GUIDED mode goto commands
    - Mission upload/download
    - Survey pattern generation
    - Loiter position commands
    """

@dataclass
class Waypoint:
    lat: float      # degrees
    lon: float      # degrees
    alt: float      # meters MSL
    wp_type: WaypointType
```

### 3. Energy Module (`src/energy/`)

Solar and battery management for multi-day operations.

#### Components

**battery.py** - Battery Management
```python
class BatteryManager:
    """
    Battery state tracking and prediction.

    Features:
    - SOC tracking from telemetry or voltage
    - Endurance prediction
    - Temperature compensation
    - State detection (charging/discharging/critical)
    """

class BatteryState(Enum):
    CHARGING = "charging"
    DISCHARGING = "discharging"
    FULL = "full"
    CRITICAL = "critical"
    EMERGENCY = "emergency"
```

**solar.py** - Solar Power Management
```python
class SolarManager:
    """
    Solar power prediction and optimization.

    Features:
    - Sun position calculation (elevation, azimuth)
    - Irradiance at altitude calculation
    - Power generation prediction
    - Optimal heading calculation
    - Day/night detection
    """

@dataclass
class SunPosition:
    elevation: float  # degrees above horizon
    azimuth: float    # degrees from north
```

**optimizer.py** - Energy Optimization
```python
class EnergyOptimizer:
    """
    Coordinates energy management.

    Energy Modes:
    - DAY_NORMAL: Full operations
    - DAY_CHARGING: Priority charging
    - DUSK_TRANSITION: Preparing for night
    - NIGHT_CRUISE: Minimum power
    - DAWN_TRANSITION: Waiting for sunrise
    - EMERGENCY: Critical battery

    Features:
    - Power budget allocation
    - Mode transitions based on sun/battery
    - Airspeed optimization for efficiency
    - Heading adjustment for solar gain
    """

@dataclass
class PowerBudget:
    total_available_w: float
    flight_systems_w: float  # Priority 1
    navigation_w: float      # Priority 2
    payload_w: float         # Priority 3
    reserve_w: float
```

### 4. Station Keeping Module (`src/station_keeping/`)

Position maintenance in stratospheric winds.

#### Components

**loiter.py** - Loiter Pattern Management
```python
class LoiterController:
    """
    Manages loiter patterns for station keeping.

    Pattern Types:
    - CIRCLE: Standard circular loiter
    - RACETRACK: Elongated for wind
    - FIGURE_EIGHT: Wind compensation

    Features:
    - Drift detection from center
    - Automatic pattern adjustment
    - Lap counting
    - Wind estimation from drift
    """
```

**wind.py** - Wind Estimation
```python
class WindEstimator:
    """
    Estimates wind from multiple sources.

    Methods:
    - Direct from ArduPilot WIND message
    - Ground speed vs airspeed comparison
    - Position drift analysis

    Output:
    - Wind speed and direction
    - Confidence level
    - Vertical component
    """

class WindCompensator:
    """
    Calculates heading corrections for wind.

    Features:
    - Heading for desired track
    - Drift prediction
    - Position prediction
    """
```

**altitude.py** - Altitude Control
```python
class AltitudeController:
    """
    Manages altitude within operating band.

    Altitude Band: 18,000m - 25,000m

    Features:
    - Target altitude management
    - Energy-optimized altitude changes
    - Air density calculation
    - Climb/descent rate limiting
    """
```

### 5. AI Module (`src/ai/`)

Autonomous decision-making engine.

#### Components

**state_machine.py** - Mission State Machine
```python
class MissionState(Enum):
    """Mission states."""
    PREFLIGHT = auto()
    LAUNCHING = auto()
    CLIMBING_TO_ALTITUDE = auto()
    STATION_KEEPING = auto()
    TRANSIT = auto()
    SURVEY = auto()
    TRACKING = auto()
    ENERGY_RECOVERY = auto()
    NIGHT_CRUISE = auto()
    RTL = auto()
    EMERGENCY_DESCENT = auto()
    LINK_LOST = auto()
    LANDING = auto()
    LANDED = auto()

class StateMachine:
    """
    Manages mission state transitions.

    Features:
    - Valid transition enforcement
    - Condition evaluation
    - Entry/exit callbacks
    - State history tracking
    """
```

**agent.py** - Main AI Agent
```python
class HAPSAgent:
    """
    Main autonomous agent.

    Responsibilities:
    - Coordinate all subsystems
    - Execute state machine decisions
    - Monitor safety conditions
    - Log status and events

    Control Loop:
    1. Update telemetry (10 Hz)
    2. Update subsystems
    3. Evaluate state machine (1 Hz)
    4. Execute state behaviors
    5. Log status (0.1 Hz)
    """
```

## Data Flow

### Telemetry Flow
```
ArduPilot → MAVLink → MessageHandler → TelemetryData
                                            ↓
                    ┌───────────────────────┼───────────────────────┐
                    ↓                       ↓                       ↓
              BatteryManager         FlightController         WindEstimator
                    ↓                       ↓                       ↓
              EnergyOptimizer        ModeManager            LoiterController
                    ↓                       ↓                       ↓
                    └───────────────────────┼───────────────────────┘
                                            ↓
                                       HAPSAgent
                                            ↓
                                      StateMachine
```

### Command Flow
```
StateMachine → HAPSAgent → FlightController → NavigationController
                                                      ↓
                                              MAVLinkConnection
                                                      ↓
                                                  ArduPilot
```

## Threading Model

The system uses Python asyncio for concurrent operations:

```python
# Main event loop
async def main_loop():
    while running:
        await update_subsystems()      # 10 Hz
        await make_decisions()         # 1 Hz
        await log_status()             # 0.1 Hz
        await asyncio.sleep(0.1)

# Background tasks
- Heartbeat sender (1 Hz)
- Message receiver (continuous)
- Telemetry logger (configurable)
```

## Configuration System

```yaml
# Hierarchical configuration
HAPSConfig:
  ├── MAVLinkConfig
  │   ├── connection_string
  │   ├── baud_rate
  │   └── heartbeat_interval
  ├── EnergyConfig
  │   ├── battery_capacity_wh
  │   ├── solar_panel_area_m2
  │   └── critical_soc_percent
  ├── FlightConfig
  │   ├── altitude_min_m
  │   ├── altitude_max_m
  │   └── airspeed_cruise_ms
  └── SafetyConfig
      ├── geofence_enabled
      └── link_loss_action
```

## Error Handling

Each module implements its own error handling:

1. **Connection Errors**: Automatic reconnection attempts
2. **Command Failures**: Logged and reported to state machine
3. **Telemetry Gaps**: Use last known values with timestamp
4. **Critical Failures**: Trigger emergency state transition
