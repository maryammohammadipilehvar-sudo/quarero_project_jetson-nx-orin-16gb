# `interfaces` (ROS 2 interface package)

This package contains the **custom ROS 2 message (`.msg`) and service (`.srv`) definitions** used across the tactical mower system. It contains **no nodes**—it exists so other packages can share strongly-typed data structures.

## Overview

- **Package role**: Central contract for cross-package communication (control, drive, web UI, security/events, video recording).
- **Build type**: `ament_cmake` + `rosidl_default_generators`
- **Generated artifacts**: Language bindings for C/C++/Python (depending on workspace/tooling).

## Dependencies

- **ROS interface generators/runtime**: `rosidl_default_generators`, `rosidl_default_runtime`
- **Message dependencies**:
  - `geometry_msgs` (used in `GeoPath.msg`, `WaypointService.srv`, `GetScheduleAction.srv`)
  - `builtin_interfaces` (used in `SecurityAlert.msg`, `CaptureEventClips.srv`)
  - `std_msgs` (used in `ObstacleSectors.msg`)

## Build & install

From the ROS 2 workspace root (`ros2_ws/`):

```bash
colcon build --packages-select interfaces
source install/setup.bash
```

## Introspection (very useful)

```bash
# List all interfaces from this package
ros2 interface list | grep '^interfaces/'

# Show the full definition of a type
ros2 interface show interfaces/msg/GeoPath
ros2 interface show interfaces/srv/ScheduleService
```

## Messages (`msg/`)

### `interfaces/msg/CommandDrive`

**Differential drive wheel velocity command** (rad/s per wheel).

- **Typical topic**: `/cmd_drive`
- **Typical publisher**: `control` (autonomy), `drive` (manual controller)
- **Typical subscriber**: `drive` (RoboClaw wrapper)

Fields:
- `float64 left_vel`: left wheel velocity (rad/s)
- `float64 right_vel`: right wheel velocity (rad/s)

Example:

```bash
ros2 topic pub -1 /cmd_drive interfaces/msg/CommandDrive "{left_vel: 1.0, right_vel: 1.0}"
```

### `interfaces/msg/Joy`

**PS5 joystick/controller state** (transported from ESP32 and/or web UI).

- **Typical topics**:
  - `/joy_drive_raw` (hardware controller input)
  - `/joy_web` (web UI joystick)
- **Typical subscribers**: `drive` (robot controller/orchestrator)

Notes:
- Stick fields are commonly treated as **percent-like integers** (often \(-100..100\)), but that range is not enforced by the message.
- Triggers `l2`/`r2` are analog (commonly `0..255`).

Example (simple “drive forward + turn right” web joystick message):

```bash
ros2 topic pub -1 /joy_web interfaces/msg/Joy "{
  left_stick_forward: 50,
  left_stick_right: 0,
  right_stick_forward: 0,
  right_stick_right: 20,
  select: false, start: false,
  x: false, square: false, triangle: false, circle: false,
  l1: false, l2: 0, r1: false, r2: 0,
  up: false, down: false, left: false, right: false
}"
```

### `interfaces/msg/GeoPath`

**A GPS waypoint path** plus an execution mode.

- **Typical topic**: `/geopath`
- **Typical publishers**: `robot_web_interface` (user selects route), `control` (scheduler/route manager)
- **Typical subscribers**: `drive` (may use for route tracking), `control` (autonomy controller)

Coordinate convention used in this repo:
- `geometry_msgs/Point.x = latitude`
- `geometry_msgs/Point.y = longitude`
- `geometry_msgs/Point.z = altitude`

Modes:
- `ONCE = 0`: execute path once
- `LOOP = 1`: loop from start after end
- `PING_PONG = 2`: go forward, then backward, repeat
- `STOP = 4`: stop execution

Example:

```bash
ros2 topic pub -1 /geopath interfaces/msg/GeoPath "{
  waypoints: [{x: 52.5200, y: 13.4050, z: 0.0}, {x: 52.5201, y: 13.4052, z: 0.0}],
  mode: 0
}"
```

### `interfaces/msg/Schedule`

**Schedule configuration** used by the scheduler/control loop.

- **Typical service payload**: `ScheduleService` request (`add` / `update`)
- **Typical producers/consumers**: `robot_web_interface` ↔ `control` (scheduler)

Key fields (see `ros2 interface show interfaces/msg/Schedule` for full details):
- **Identity**: `schedule_id`
- **Time window**: `weekdays` (0=Mon..6=Sun), `start_time`/`end_time` (`"HH:MM"`)
- **Routes**: `route_names[]`, `route_repetitions[]`, `route_mode` (`SEQUENTIAL=0`, `RANDOM=1`), `loop_mode`
- **Flags**: `active`, `require_home_return`
- **Battery/home settings**: `battery_threshold`, `home_tolerance`, `auto_charge_return`

### `interfaces/msg/SecurityAlert`

**Security/event notification** (camera/sensor → rest of system).

- **Typical topic**: `/security_alert`
- **Typical publisher**: `eneo_event_publisher`
- **Typical subscribers**: `robot_web_interface` (event UI), `video_ringbuffer` (indirectly, via clip capture requests)

Fields:
- `string event_type`: e.g. `"person"`, `"fire"`, `"motion"`
- `builtin_interfaces/Time event_time`: timestamp when event occurred
- `string device_name`: camera/device identifier
- `string description`: human-readable detail
- `string[] camera_ids`: camera IDs to use for video capture (if applicable)

Example:

```bash
ros2 topic pub -1 /security_alert interfaces/msg/SecurityAlert "{
  event_type: 'person',
  event_time: {sec: 0, nanosec: 0},
  device_name: 'eneo_rgb',
  description: 'Manual test alert',
  camera_ids: ['eneo_rgb', 'eneo_thermal']
}"
```

### `interfaces/msg/SectorInfo` and `interfaces/msg/ObstacleSectors`

**Obstacle-sector summary** used by obstacle avoidance.

- **Typical topic**: `/obstacles/sectors`
- **Typical publisher**: `control` (`sector_publisher` node; derived from `/obstacles/lidar` OccupancyGrid)
- **Typical subscriber**: `control` (waypoint follower / avoidance)

`SectorInfo` fields:
- `int32 sector_id`: 0..N-1
- `bool blocked`
- `string sector_type`: semantic label (commonly `"STOP"` or `"SLOW"`)

`ObstacleSectors` fields:
- `std_msgs/Header header`
- `SectorInfo[] sectors`

Example:

```bash
ros2 topic echo /obstacles/sectors
```

### `interfaces/msg/Status`

**Low-level motor controller status snapshot** (battery/temps/currents/errors).

Fields:
- `float32 battery`: battery voltage (V)
- `string[3] error_status`: per-controller error status, hex string
- `float32[3] temp`: controller temperatures (°C)
- `float32[6] current`: currents (A)

> Note: much of the runtime “robot state” sent to the web UI is currently a JSON `std_msgs/String` on `/robot/state`. This `Status` message is a typed alternative for low-level telemetry.

## Services (`srv/`)

### `interfaces/srv/CommandControl`

**Generic on/off command** service.

Used for toggles like lights, siren, emergency stop, autonomous mode, charging actions, etc.

- **Typical services**:
  - `/control/light`
  - `/control/alarm`
  - `/control/siren`
  - `/control/emergency_stop`
  - `/control/autonomous_operation`
  - `/control/go_to_charge_pos_and_charge`
  - `/control/charge_manual`

Request:
- `string command_id`: correlation ID for logging/ack
- `bool state`: desired state

Response:
- `bool success`
- `string message`

Example:

```bash
ros2 service call /control/light interfaces/srv/CommandControl "{command_id: 'cli-test', state: true}"
```

### `interfaces/srv/WaypointService`

**Send/replace current waypoint list** for navigation.

- **Typical service**: `/control/waypoints`

Request:
- `string command_id`
- `string route_name` (optional)
- `geometry_msgs/Point[] waypoints` (x=lat, y=lon, z=alt)
- `uint8 mode` (0=STOP, 1=LOOP, 2=PING_PONG)

Response:
- `bool success`
- `string message`

Example:

```bash
ros2 service call /control/waypoints interfaces/srv/WaypointService "{
  command_id: 'cli-test',
  route_name: 'test_route',
  waypoints: [{x: 52.5200, y: 13.4050, z: 0.0}, {x: 52.5201, y: 13.4052, z: 0.0}],
  mode: 1
}"
```

### `interfaces/srv/ScheduleService`

**Create/update/remove schedules**.

- **Typical service**: `/tactical/control/schedule`

Request:
- `string command_id`
- `string action`: `"add"`, `"remove"`, `"update"`
- `interfaces/Schedule schedule`: used for `"add"` and `"update"`
- `string schedule_id`: used for `"remove"` and `"update"` (the existing ID)

Response:
- `bool success`
- `string message`

Example (add):

```bash
ros2 service call /tactical/control/schedule interfaces/srv/ScheduleService "{
  command_id: 'cli-test',
  action: 'add',
  schedule: {
    schedule_id: 'weekday-morning',
    weekdays: [0,1,2,3,4],
    start_time: '08:00',
    end_time: '12:00',
    route_names: ['route_a'],
    route_repetitions: [1],
    route_mode: 0,
    loop_mode: false,
    active: true,
    require_home_return: true,
    battery_threshold: 20,
    home_tolerance: 0.5,
    auto_charge_return: true
  },
  schedule_id: ''
}"
```

### `interfaces/srv/GetScheduleAction`

**Query the scheduler for the current “what should I do next?” route data**.

Design note (from the `.srv` file): *the scheduler provides data only; the control loop makes decisions*.

- **Typical service**: `/control/get_schedule_action`
- **Typical caller**: `control` waypoint follower/control loop

Request:
- `string command_id`

Response (high level):
- `bool success`
- `bool has_active_schedule`
- `string schedule_id`, `schedule_display_name`, `route_name`
- `geometry_msgs/Point[] waypoints` + `uint8 geopath_mode` (GeoPath-like)
- progress integers (`current_route_index`, `total_routes`, `current_repetition`, `total_repetitions`)
- `string message`

Example:

```bash
ros2 service call /control/get_schedule_action interfaces/srv/GetScheduleAction "{command_id: 'cli-test'}"
```

### `interfaces/srv/CaptureEventClips`

**Ask the ringbuffer to write video clips around an event timestamp**.

- **Typical service**: `capture_event_clips` (no leading `/` in current nodes)
- **Typical server**: `video_ringbuffer`
- **Typical client**: `robot_web_interface`

Request:
- `string event_id`
- `builtin_interfaces/Time event_time`
- `string[] camera_ids`
- `float32 pre_event_seconds`
- `float32 post_event_seconds`

Response:
- `bool success`
- `string message`
- `string[] file_paths`
- `string[] camera_ids_out` (same order as `file_paths`)

Example:

```bash
ros2 service call /capture_event_clips interfaces/srv/CaptureEventClips "{
  event_id: 'cli-test',
  event_time: {sec: 0, nanosec: 0},
  camera_ids: ['eneo_rgb', 'eneo_thermal'],
  pre_event_seconds: 10.0,
  post_event_seconds: 5.0
}"
```

## Development notes

### Adding/changing an interface

1. Add or edit files in `msg/` or `srv/`.
2. Update `CMakeLists.txt` `rosidl_generate_interfaces(...)` to include the new file.
3. If you introduce new external message dependencies, also update `package.xml` and the `DEPENDENCIES ...` list.
4. Rebuild: `colcon build --packages-select interfaces`.

### Compatibility

- Treat changes to existing fields as **breaking** (renames/removals/type changes).
- Prefer additive changes (new fields at the end, new messages/services).

