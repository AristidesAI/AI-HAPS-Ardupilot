# HAPS Glider - API Reference

## Table of Contents

1. [MAVLink Module](#mavlink-module)
2. [Flight Module](#flight-module)
3. [Energy Module](#energy-module)
4. [Station Keeping Module](#station-keeping-module)
5. [AI Module](#ai-module)

---

## MAVLink Module

### MAVLinkConnection

```python
from mavlink import MAVLinkConnection

connection = MAVLinkConnection(
    connection_string="udp:127.0.0.1:14550",
    source_system=255,
    source_component=0,
    heartbeat_interval=1.0,
    heartbeat_timeout=5.0,
)
```

#### Properties

| Property | Type | Description |
|----------|------|-------------|
| `state` | `ConnectionState` | Current connection state |
| `is_connected` | `bool` | Whether connection is active |
| `heartbeat` | `HeartbeatInfo` | Last heartbeat information |
| `target_system` | `int` | Target system ID from heartbeat |
| `target_component` | `int` | Target component ID |

#### Methods

##### `async connect() -> bool`
Establish MAVLink connection and wait for heartbeat.

```python
success = await connection.connect()
if success:
    print(f"Connected to system {connection.target_system}")
```

##### `async disconnect() -> None`
Close the MAVLink connection.

```python
await connection.disconnect()
```

##### `register_callback(message_type: str, callback: Callable) -> None`
Register a callback for specific message type.

```python
def on_position(msg):
    print(f"Position: {msg.lat/1e7}, {msg.lon/1e7}")

connection.register_callback("GLOBAL_POSITION_INT", on_position)
```

##### `send_command_long(...) -> bool`
Send a MAV_CMD command.

```python
# Arm the vehicle
connection.send_command_long(
    command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    param1=1,  # 1 = arm
)
```

##### `request_message(message_id: int, interval_us: int) -> bool`
Request a message at specified interval.

```python
# Request ATTITUDE at 10 Hz
connection.request_message(
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
    100000  # 100ms = 10 Hz
)
```

---

### MessageHandler

```python
from mavlink import MessageHandler, TelemetryData

handler = MessageHandler()
connection.register_all_messages_callback(handler.handle_message)
```

#### Properties

| Property | Type | Description |
|----------|------|-------------|
| `telemetry` | `TelemetryData` | Current telemetry state |

#### TelemetryData Structure

```python
@dataclass
class TelemetryData:
    position: Position      # lat, lon, alt_msl, alt_rel
    attitude: Attitude      # roll, pitch, yaw (radians)
    velocity: Velocity      # vx, vy, vz, groundspeed, airspeed
    battery: BatteryStatus  # voltage, current, remaining
    system: SystemStatus    # armed, mode, cpu_load
    wind: WindEstimate      # speed, direction
    home_lat: float
    home_lon: float
    home_alt: float
```

---

### ParameterManager

```python
from mavlink import ParameterManager

params = ParameterManager(connection)
```

#### Methods

##### `async fetch_all(timeout: float = 60.0) -> bool`
Fetch all parameters from vehicle.

```python
if await params.fetch_all():
    print(f"Fetched {len(params.params)} parameters")
```

##### `async read(name: str, timeout: float = 5.0) -> Optional[float]`
Read a single parameter.

```python
value = await params.read("ARSPD_FBW_MIN")
print(f"Minimum airspeed: {value} m/s")
```

##### `async write(name: str, value: float, timeout: float = 5.0) -> bool`
Write a parameter.

```python
if await params.write("WP_LOITER_RAD", 1500):
    print("Loiter radius updated")
```

##### `async apply_haps_defaults() -> dict[str, bool]`
Apply HAPS-specific default parameters.

```python
results = await params.apply_haps_defaults()
success_count = sum(1 for v in results.values() if v)
print(f"Applied {success_count}/{len(results)} parameters")
```

---

## Flight Module

### FlightController

```python
from flight import FlightController

flight = FlightController(connection)
```

#### Properties

| Property | Type | Description |
|----------|------|-------------|
| `telemetry` | `TelemetryData` | Current telemetry |
| `is_armed` | `bool` | Vehicle armed state |
| `current_mode` | `str` | Current flight mode name |
| `position` | `tuple` | (lat, lon, alt_msl) |
| `altitude` | `float` | Altitude MSL in meters |
| `airspeed` | `float` | Airspeed in m/s |
| `groundspeed` | `float` | Ground speed in m/s |
| `heading` | `float` | Heading in degrees |

#### Methods

##### `async arm(force: bool = False) -> bool`
Arm the vehicle.

```python
if await flight.arm():
    print("Vehicle armed")
```

##### `async disarm(force: bool = False) -> bool`
Disarm the vehicle.

```python
await flight.disarm()
```

##### `async set_mode(mode: FlightMode) -> bool`
Change flight mode.

```python
from flight import FlightMode

await flight.set_mode(FlightMode.GUIDED)
await flight.set_mode(FlightMode.LOITER)
await flight.set_mode(FlightMode.AUTO)
```

##### `async goto(lat, lon, alt, groundspeed=None) -> bool`
Navigate to position in GUIDED mode.

```python
await flight.goto(
    lat=-33.8688,
    lon=151.2093,
    alt=20000,
    groundspeed=20  # optional
)
```

##### `async loiter_at(lat, lon, alt, radius=1000) -> bool`
Loiter at position.

```python
await flight.loiter_at(
    lat=-33.8688,
    lon=151.2093,
    alt=20000,
    radius=1000
)
```

##### `async set_altitude(alt: float) -> bool`
Change altitude while maintaining position.

```python
await flight.set_altitude(21000)  # Climb to 21km
```

##### `async set_airspeed(airspeed: float) -> bool`
Set target airspeed.

```python
await flight.set_airspeed(18)  # 18 m/s
```

##### `async return_to_launch() -> bool`
Initiate return to launch.

```python
await flight.return_to_launch()
```

##### `request_data_streams(rate_hz: int = 4) -> None`
Request telemetry streams at specified rate.

```python
flight.request_data_streams(rate_hz=10)
```

##### `get_status_summary() -> dict`
Get summary of flight status.

```python
status = flight.get_status_summary()
print(f"Mode: {status['mode']}, Alt: {status['position']['alt_msl']}m")
```

---

### ModeManager

```python
from flight import FlightMode, ModeManager

modes = ModeManager(connection)
```

#### Flight Modes

| Mode | Value | Description |
|------|-------|-------------|
| `MANUAL` | 0 | Full manual control |
| `STABILIZE` | 2 | Attitude stabilization |
| `FBWB` | 6 | Fly-by-wire B |
| `CRUISE` | 7 | Cruise mode |
| `AUTO` | 10 | Autonomous waypoint |
| `RTL` | 11 | Return to launch |
| `LOITER` | 12 | Loiter at position |
| `GUIDED` | 15 | External control |
| `THERMAL` | 24 | Thermal soaring |

#### Methods

##### `async set_mode(mode: FlightMode, timeout: float = 10.0) -> bool`
```python
await modes.set_mode(FlightMode.GUIDED)
```

##### `is_autonomous_mode() -> bool`
```python
if modes.is_autonomous_mode():
    print("AI can send commands")
```

---

### NavigationController

```python
from flight import NavigationController, Waypoint, Mission

nav = NavigationController(connection)
```

#### Methods

##### `async goto(lat, lon, alt, groundspeed=None) -> bool`
Navigate to position in GUIDED mode.

##### `async loiter_at(lat, lon, alt, radius, direction=1) -> bool`
Loiter at position.

##### `async upload_mission(mission: Mission, timeout=30.0) -> bool`
Upload mission to vehicle.

```python
mission = Mission(name="Survey")
mission.add_waypoint(-33.87, 151.21, 20000)
mission.add_waypoint(-33.88, 151.22, 20000)
mission.add_loiter(-33.88, 151.22, 20000, radius=1000, duration=600)

await nav.upload_mission(mission)
```

##### `async download_mission(timeout=30.0) -> Optional[Mission]`
Download mission from vehicle.

##### `create_survey_pattern(...) -> Mission`
Create lawnmower survey pattern.

```python
mission = nav.create_survey_pattern(
    center_lat=-33.87,
    center_lon=151.21,
    alt=20000,
    width=5000,    # 5km
    height=3000,   # 3km
    spacing=500    # 500m between lines
)
```

---

## Energy Module

### BatteryManager

```python
from energy import BatteryManager, BatteryConfig

battery = BatteryManager(BatteryConfig(
    capacity_ah=50.0,
    cells=12,
    critical_soc=20.0,
))
```

#### Properties

| Property | Type | Description |
|----------|------|-------------|
| `state` | `BatteryState` | Current state |
| `soc` | `float` | State of charge (0-100%) |
| `voltage` | `float` | Battery voltage |
| `power` | `float` | Current power (W) |
| `is_critical` | `bool` | Below critical threshold |
| `is_emergency` | `bool` | Below emergency threshold |
| `is_charging` | `bool` | Currently charging |

#### Methods

##### `update(voltage, current, soc_percent=None, temperature=25.0) -> None`
Update battery state from telemetry.

```python
battery.update(
    voltage=44.4,
    current=-2.0,  # Discharging
    soc_percent=75.0,
    temperature=25.0
)
```

##### `predict_endurance(power_consumption_w: float) -> float`
Predict remaining flight time.

```python
hours = battery.predict_endurance(power_consumption_w=20)
print(f"Endurance: {hours:.1f} hours")
```

##### `get_safe_power_budget(mission_hours: float) -> float`
Calculate safe power budget for mission duration.

```python
max_power = battery.get_safe_power_budget(mission_hours=8)
print(f"Safe power budget: {max_power:.0f}W")
```

---

### SolarManager

```python
from energy import SolarManager, SolarConfig

solar = SolarManager(SolarConfig(
    panel_area_m2=4.0,
    panel_efficiency=0.22,
))
```

#### Methods

##### `set_location(lat, lon, alt_m) -> None`
Update current location.

##### `set_heading(heading: float) -> None`
Update current heading.

##### `update(actual_power_w: float = 0.0) -> SolarStatus`
Update solar status.

```python
solar.set_location(-33.87, 151.21, 20000)
solar.set_heading(90)
status = solar.update()

print(f"Sun elevation: {status.sun_position.elevation}°")
print(f"Available power: {status.power_available_w}W")
print(f"Optimal heading: {status.heading_optimal_deg}°")
```

##### `calculate_sun_position(lat, lon, dt) -> SunPosition`
Calculate sun position for any time/location.

```python
from datetime import datetime, timezone

pos = solar.calculate_sun_position(
    latitude=-33.87,
    longitude=151.21,
    dt=datetime.now(timezone.utc)
)
```

##### `predict_power(lat, lon, alt_m, dt) -> float`
Predict power at future time.

```python
from datetime import datetime, timedelta

future = datetime.now(timezone.utc) + timedelta(hours=6)
power = solar.predict_power(-33.87, 151.21, 20000, future)
```

##### `is_daylight() -> bool`
Check if currently daylight.

---

### EnergyOptimizer

```python
from energy import EnergyOptimizer, EnergyMode

optimizer = EnergyOptimizer(battery, solar)
```

#### Energy Modes

| Mode | Description |
|------|-------------|
| `DAY_NORMAL` | Normal daylight operations |
| `DAY_CHARGING` | Priority battery charging |
| `DUSK_TRANSITION` | Preparing for night |
| `NIGHT_CRUISE` | Minimum power mode |
| `NIGHT_EMERGENCY` | Critical night energy |
| `DAWN_TRANSITION` | Waiting for sunrise |
| `EMERGENCY` | Critical battery |

#### Properties

| Property | Type | Description |
|----------|------|-------------|
| `mode` | `EnergyMode` | Current energy mode |
| `power_budget` | `PowerBudget` | Current allocation |
| `is_night_mode` | `bool` | In night operation |
| `can_run_payload` | `bool` | Payload allowed |

#### Methods

##### `update(latitude, longitude, altitude_m) -> EnergyMode`
Update energy state and get mode.

```python
mode = optimizer.update(-33.87, 151.21, 20000)
print(f"Energy mode: {mode.value}")
```

##### `get_optimal_airspeed_factor() -> float`
Get airspeed multiplier for energy optimization.

```python
factor = optimizer.get_optimal_airspeed_factor()
optimal_speed = cruise_speed * factor
```

##### `get_optimal_heading_adjustment() -> float`
Get heading adjustment for better solar.

```python
adjustment = optimizer.get_optimal_heading_adjustment()
if abs(adjustment) > 30:
    print(f"Adjust heading by {adjustment}° for +20% solar")
```

##### `get_recommended_actions() -> List[str]`
Get list of recommended actions.

```python
for action in optimizer.get_recommended_actions():
    print(f"- {action}")
```

##### `time_to_sunrise() -> float`
Hours until sunrise.

##### `time_to_sunset() -> float`
Hours until sunset.

---

## Station Keeping Module

### LoiterController

```python
from station_keeping import LoiterController, LoiterPattern

loiter = LoiterController()
```

#### Loiter Patterns

| Pattern | Description |
|---------|-------------|
| `CIRCLE` | Standard circular |
| `RACETRACK` | Elongated for wind |
| `FIGURE_EIGHT` | Wind compensation |

#### Methods

##### `start_loiter(center_lat, center_lon, altitude_m, radius_m=None, pattern=CIRCLE, direction=1) -> None`
Start loiter pattern.

```python
loiter.start_loiter(
    center_lat=-33.87,
    center_lon=151.21,
    altitude_m=20000,
    radius_m=1000,
    pattern=LoiterPattern.CIRCLE,
    direction=1  # clockwise
)
```

##### `stop_loiter() -> None`
Stop loiter pattern.

##### `update(current_lat, current_lon, current_heading) -> Tuple[Optional[float], Optional[float]]`
Update loiter and get correction if needed.

```python
correction_lat, correction_lon = loiter.update(
    current_lat=-33.871,
    current_lon=151.211,
    current_heading=90
)

if correction_lat is not None:
    # Apply drift correction
    await flight.goto(correction_lat, correction_lon, altitude)
```

##### `adjust_pattern_for_wind(wind_speed_ms, wind_direction_deg) -> None`
Adjust pattern based on wind.

```python
loiter.adjust_pattern_for_wind(wind_speed_ms=15, wind_direction_deg=270)
```

---

### WindEstimator

```python
from station_keeping import WindEstimator

wind = WindEstimator()
```

#### Methods

##### `update_from_telemetry(wind_speed, wind_direction, vertical_speed=0, altitude=20000) -> WindEstimate`
Update from ArduPilot WIND message.

##### `update_from_velocity(groundspeed_ms, ground_track_deg, airspeed_ms, heading_deg, altitude=20000) -> WindEstimate`
Estimate wind from velocity comparison.

##### `update_from_drift(current_lat, current_lon, altitude=20000) -> Optional[WindEstimate]`
Estimate wind from position drift.

---

### AltitudeController

```python
from station_keeping import AltitudeController, AltitudeBand

altitude_ctrl = AltitudeController()
```

#### Methods

##### `set_target(altitude_m: float) -> bool`
Set target altitude.

```python
altitude_ctrl.set_target(21000)
```

##### `update(current_altitude_m, climb_rate_ms, power_balance_w=0) -> Tuple[float, AltitudeState]`
Update and get commanded climb rate.

```python
commanded_rate, state = altitude_ctrl.update(
    current_altitude_m=20500,
    climb_rate_ms=0.5,
    power_balance_w=50
)
```

##### `optimize_for_energy(power_balance_w, battery_soc, is_daylight) -> Optional[float]`
Get energy-optimized altitude recommendation.

---

## AI Module

### HAPSAgent

```python
from ai import HAPSAgent, AgentConfig

agent = HAPSAgent(flight_controller, AgentConfig(
    update_rate_hz=10.0,
    decision_rate_hz=1.0,
    autonomous_enabled=True,
))
```

#### Methods

##### `async start() -> bool`
Start the AI agent.

```python
await agent.start()
```

##### `async stop() -> None`
Stop the AI agent.

##### `async pause() -> None`
Pause autonomous operations.

##### `async resume() -> None`
Resume autonomous operations.

##### `set_target(lat, lon, alt) -> None`
Set mission target location.

```python
agent.set_target(
    lat=-33.87,
    lon=151.21,
    alt=20000
)
```

##### `async command_goto(lat, lon, alt) -> bool`
Command agent to go to location.

##### `async command_loiter_here() -> bool`
Command agent to loiter at current position.

##### `async command_rtl() -> bool`
Command return to launch.

##### `get_status() -> dict`
Get comprehensive agent status.

```python
status = agent.get_status()
print(f"Mission state: {status['mission']['state']}")
print(f"Battery SOC: {status['energy']['battery']['soc_percent']}%")
```

---

### StateMachine

```python
from ai import StateMachine, MissionState, MissionContext

sm = StateMachine()
```

#### Mission States

| State | Description |
|-------|-------------|
| `PREFLIGHT` | System checks |
| `LAUNCHING` | Launch in progress |
| `CLIMBING_TO_ALTITUDE` | Ascending |
| `STATION_KEEPING` | Holding position |
| `TRANSIT` | Moving to location |
| `SURVEY` | Survey pattern |
| `TRACKING` | Target tracking |
| `ENERGY_RECOVERY` | Low energy mode |
| `NIGHT_CRUISE` | Night operations |
| `RTL` | Return to launch |
| `EMERGENCY_DESCENT` | Emergency |
| `LINK_LOST` | Comms lost |
| `LANDING` | Landing |
| `LANDED` | Complete |

#### Methods

##### `transition(to_state: MissionState, reason: str = "") -> bool`
Attempt state transition.

```python
sm.transition(MissionState.STATION_KEEPING, reason="Reached altitude")
```

##### `evaluate(context: MissionContext) -> Optional[MissionState]`
Evaluate conditions for state change.

```python
context = MissionContext(
    altitude_m=20000,
    battery_soc=75,
    is_daylight=True,
    # ...
)

next_state = sm.evaluate(context)
if next_state:
    sm.transition(next_state)
```

##### `register_on_enter(state, callback) -> None`
Register callback for state entry.

```python
def on_night_cruise(old_state, new_state):
    print("Entering night mode")

sm.register_on_enter(MissionState.NIGHT_CRUISE, on_night_cruise)
```
