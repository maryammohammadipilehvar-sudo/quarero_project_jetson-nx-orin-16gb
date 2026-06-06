# CODEBASE_MENTAL_MODEL.md — read this first

This file is the durable mental model of the `~/gits/` codebase. A SessionStart hook tells every new Claude session to read it before doing anything else. It is meant to be the single doc you can hand to a new operator (or new Claude) and have them be productive in one read.

**Read order recommendation:** this file → `CLAUDE.md` (operating rules) → relevant RCA(s) → the specific file you need.

Companion docs (don't duplicate them here):
- `CLAUDE.md` — operating rules; what you may/may not do without operator approval
- `AUDIT.md` — issue inventory (HIGH/MED/LOW) as of 2026-04-24
- `PRODUCT.md` — product framing (what the robot is, who uses it)
- `BRINGUP.md` + `bringup-new-jetson.sh` — bring a fresh Orin Nano up at a new site
- `HANDOFF.md`, `JETSON_TRANSFER.md` — Nano→Orin NX swap notes (mostly historical now)
- `RCA_2026-05-15_robot_not_driving.md` — UART/deadman/baud diagnosis after Nano→Orin swap
- `RCA_2026-05-16_joystick_stop_delay.md` — drive cascade + watchdog + brake ramp + DDS domain fix
- `RCA_2026-05-17_autonomy_session_learning.md` — 10-issue walkthrough that took autonomy from "doesn't move" to "11 cm waypoint accuracy"
- `DESIGN_ARRIVAL_PIPELINE.md` — full design for the (now-shipped) arrival → ringbuffer → ntfy → 72h-retention pipeline. Phases 1–6 complete as of 2026-05-23.
- **`PLAN_UI_CUSTOMER_HANDOFF.md`** — IN-PROGRESS sprint plan (2 days) to make the web UI usable by a non-technical 60+ customer. **If you are starting a new session and §6 of that file shows TODOs, that's your work list** — read it before doing anything UI-related.

---

## 1. What this robot is

Two-wheeled diff-drive surveillance robot. **Not a mower** despite the directory name. Single Jetson Orin NX 16 GB on an NVIDIA P3768 carrier (`192.168.10.226`) runs five Docker containers; an ESP32 bridges a PS5 controller over Bluetooth; a RoboClaw 2x15 drives the motors. Localization is pure Fixposition Vision-RTK fusion (no SLAM). Obstacle avoidance fuses a Livox MID-360 LiDAR with an Intel RealSense D455. A separate Eneo IP camera (RGB + thermal) generates security events. Operator interface is a FastAPI web UI on port 8020 (no auth, German UI). Customer is a multi-site facility operator; the robot is currently a single-robot deployment.

Non-goals today: no fleet manager, no cloud, no SLAM, no auth, no OTA, no person re-id (the YOLOv8 stream is display-only). Deploy = SSH + `git pull` + rebuild.

---

## 2. Hardware inventory

| Component | Interface | Where used |
|---|---|---|
| Jetson Orin NX 16 GB | P3768 carrier, JetPack 6.2 / L4T R36.5.0 | runs the whole stack |
| RoboClaw 2x15 v4.2.8 | USB `/dev/ttyACM0` @ 115200, address 128, **duty mode, encoderless** | motors |
| ESP32 (custom) | USB CP2102 → `/dev/esp_joy` @ 115200 (text debug stream is the live path) | PS5 BT + relays |
| ESP32 UART1 | `/dev/ttyTHS1` @ 19200 (legacy, *dead on Orin* — serial-tegra @ 19200 garbles frames) | not used in practice |
| Livox MID-360 LiDAR | Ethernet (host IP `192.168.10.226` hardcoded in MID360_config.json) | grid-occupancy → `/obstacles/lidar` |
| Intel RealSense D455 | USB 3 | 5×2 sector obstacle detector → `/obstacle_sectors` |
| OAK-D Lite | USB 3 (named "realsense" in repo — be careful, see HANDOFF.md) | depth, topic `/oak/stereo/image_raw` (`16UC1`, mm) |
| Fixposition Vision-RTK | TCP `192.168.10.109:21000` | GNSS + IMU fusion, dual-antenna heading (was `.107` before the 2026-06-03 router swap) |
| Eneo IP camera | RTSP `192.168.10.193` + UDP events on `:5002` | RGB + thermal + security events |
| AXIS F41 | RTSP `192.168.10.174` (creds `root:axis` — AXIS factory default) | optional person-detection on `quarero-00` |
| Battery | 7S12P 18650, 24 V / 30 Ah / 720 Wh nominal | params in `drive/config/params.yaml` |
| Network | 4G/5G router; camera subnet `192.168.10.x` | reach via NetBird VPN |

Pinmux on the carrier (verified): header pins 8/10 = `uarta` → `/dev/ttyTHS1`. `nvgetty.service` is masked so `/dev/ttyTHS*` is free for the ESP link.

---

## 3. The five containers

| Container | Compose | Notes |
|---|---|---|
| `ros2_jetson` | `tactical_mower/docker-compose.yaml` (service `ros2`) | builds + launches `system_bringup controller.launch.py`. Mounts `../routen → /routen`. |
| `web_app` | `tactical_mower/docker-compose.yaml` (service `web_app`) | FastAPI on `:8020`, `network_mode: host`. Mounts `../routen → /data`. |
| `livox_ros2_jazzy` | `livox_mid_360_obstacle_detection/docker-compose.yaml` | MID-360 driver + obstacle node. |
| `intel_realsense_ros2` | `RealsenseD_camera_Obstacle_avoidance/docker-compose.yml` | depth + sector detector. |
| `theramal_camera_jazzy` (sic) | `Tactical-Thermal-Stream/docker-compose.yml` | Eneo dual RTSP bridge. |

All five forced to `ROS_DOMAIN_ID=99` via untracked `docker-compose.override.{yml,yaml}` files (the override files were the fix to the 2026-05-16 DDS-domain-mismatch incident — see RCA_2026-05-16 §10b). The `ros2_jetson` override also bind-mounts `/dev/esp_joy` (the udev rule `/etc/udev/rules.d/99-esp-joy.rules` pins the CP2102 by VID/PID/serial so the symlink survives USB re-enumeration).

**Service ↔ container name table** (overrides target service names, not container names):

| Compose file | Service | Container |
|---|---|---|
| `livox_mid_360_obstacle_detection/docker-compose.yaml` | `ros2_livox` | `livox_ros2_jazzy` |
| `RealsenseD_camera_Obstacle_avoidance/docker-compose.yml` | `realsense` | `intel_realsense_ros2` |
| `Tactical-Thermal-Stream/docker-compose.yml` | `eneo_INT-8SF0003M0A` | `theramal_camera_jazzy` |
| `tactical_mower/docker-compose.yaml` | `ros2`, `web_app` | `ros2_jetson`, `web_app` |

**Duplicate-tree trap** (AUDIT MED-1): the outer `tactical_mower/ros2_ws/` is the only canonical ROS workspace; `tactical_mower/tactical_mower/` is a partially diverged older copy. `tactical_mower/{livox,realsense,thermal}_obstacle/` and `tactical_mower/routen/` mirror the top-level sibling projects and have diverged in places. **Always edit the outer ones.**

---

## 4. ROS 2 workspace layout (`tactical_mower/ros2_ws/src/`)

| Package | Key responsibility |
|---|---|
| `control` | autonomy: state machine, wp_follower, scheduler, navigation, obstacle avoidance, GNSS monitor, joy_controller |
| `drive` | motor cascade: `robot_controller_node`, `roboclaw_wrapper_node`, kinematics, controllers, hardware |
| `interfaces` | custom msgs/srvs (`CommandDrive`, `Joy`, `GeoPath`, `ObstacleSectors`, `CommandControl`, `WaypointService`, `GetScheduleAction`, etc.) |
| `system_bringup` | top-level `controller.launch.py` |
| `eneo_event_publisher` | UDP `:5002` → `/security_alert` |
| `video_ringbuffer` | pre/post-event clip capture — **NOT launched today** (commented out in `controller.launch.py:64-78`, AUDIT HIGH-2) |
| `robot_web_interface` | FastAPI app (port 8020) |
| `fixposition` | submodule (third-party SDK) |
| `ros2_navigation` | Nav2 wiring used by `WaypointFollowerNav2` |

`system_bringup/launch/controller.launch.py` chains: `drive.launch.py` + `control.launch.py` + `fixposition_driver_ros2 node.launch` + `eneo_event_publisher` (and *should* include `video_ringbuffer` — see HIGH-2).

---

## 5. Manual drive cascade (the L1-held path)

```
PS5  ──BT──►  ESP32 (Bluepad32)  ──UART0 / CP2102 @115200──►  /dev/esp_joy
                                                                        │
  joy_controller_node (control)  ◄────────────────────────────────────┘
   • daemon thread, bounded-drain (32 lines/iter) over text "idx=…" stream
   • parses: idx, dpad, buttons (L1=bit 4 = 0x0010), axis L/R, brake, throttle
   • L1 deadman: emit /joy_drive_raw only while L1 held
   • On L1 held→released edge: publish zero Joy on every loop for 0.5 s
     (wall-clock window; was 3-frame burst before RCA 2026-05-16)
   • Silence watchdog: 0.15 s without an idx line + L1 was held → fire 1 zero
   • Light/charging back-channel: subscribes /control/light & /control/enable_charging,
     CRC-frames `payload,CRC\n` and writes to ESP UART
                                                                        │
                          /joy_drive_raw  (interfaces/msg/Joy)  ──────┘
                                                                        │
  robot_controller_node (drive)  ◄─────────────────────────────────────┘
   • gamepad priority: 500 ms timeout, then /joy_web fallback
   • lockouts: charging_state, tactical_mode_active, e-stop
   • DifferentialDriveKinematics: 12 % deadband (PS5 +8 left-stick rest bias),
     wheel_diameter 0.535 m, wheel_base 0.637 m → wheel rad/s
   • publishes /robot/state JSON @ 1 Hz (single source of truth for charging_state,
     light_state, alarm_state, siren_state, emergency_stop_active, active_route,
     battery_percentage, remaining_runtime_minutes, autonomous_enabled, etc.)
                                                                        │
                          /cmd_drive  (interfaces/msg/CommandDrive)  ──┘
                                                                        │
  roboclaw_wrapper_node (drive)  ◄──────────────────────────────────────┘
   • MultiThreadedExecutor(num_threads=3) + ReentrantCallbackGroup + threading.Lock
     (slow battery USB poll must not starve drive_cmd_cb or watchdog —
      this was RCA 2026-05-17 Issue 8)
   • drive_acceleration_factor 0.8, brake_acceleration_factor 0.8 (both ramp ≈1.25 s)
   • velocity_qpps_to_duty_factor = 3000
   • At qpps=0 use brake_accel (Roboclaw H-bridge configured COAST at duty=0 —
     verified 2026-05-22 — so the DutyAccel ramp itself provides regen braking)
   • velocity_timeout 0.3 s → _slam_brake (20 Hz watchdog timer)
                                                                        │
                          Roboclaw 2x15 over USB /dev/ttyACM0 @115200, addr 128
```

**Key files:**
- `control/control/joy_controller_node.py` (301 LoC)
- `drive/src/drive/nodes/robot_controller_node.py` (633 LoC)
- `drive/src/drive/nodes/roboclaw_wrapper_node.py` (454 LoC)
- `drive/src/drive/kinematics/differential_drive.py`
- `drive/src/drive/hardware/roboclaw_3.py` (1080 LoC, vendor code — don't review unless necessary)

**ESP32 frame (TX, ESP → Jetson)** — `arduino/PS5_ESP32/PS5_ESP32.ino`:
```
mapRX,mapY,right,left,SELECT,Licht,GPS,CRC_HEX\n
```
CRC16-CCITT (poly 0x1021, init 0xFFFF). Stick axes mapped to ±100 by firmware (PS5 native ±511; magic number ±508 is a stale comment — firmware clamps to ±100). The ESP only transmits the binary frame on UART1 — that path is **dead on Orin** (RCA 2026-05-15). The live path is the human-readable debug stream on USB-CP2102 @ 115200 which joy_controller parses.

**ESP32 frame (RX, Jetson → ESP)**: same `payload,CRC\n` shape; `values[5]=1` → Light GPIO 25 ON, `values[7]=1` → Charging GPIO 33 ON. Siren GPIO 26, GPS-status GPIO 5, GPS-save GPIO 2, Lidar GPIO 32, Forward GPIO 27.

---

## 6. Autonomous stack — `tactical_wp_follower_node` (2930 LoC)

The brain. Lives in `control/control/nodes/tactical_wp_follower_node.py`. Runs a 10 Hz `_control_loop` that orchestrates everything; actual behavior lives in state classes (Moore-style: state-entry callbacks do the work, transitions are pure condition checks).

### 6.1 State machine

States (`control/control/robot_state_machine/states/*.py`):
```
UNINITIALIZED ──charge_pos set──► MANUAL ◄── autonomous_disabled ── (any state)
                                     │
                                     │ autonomous_enabled
                                     ▼
                                   IDLE
       ┌──────────────────┬─────────┴──────────┬─────────────────┐
       ▼                  ▼                    ▼                 ▼
    DOCKED            DOCKING             NAVIGATING       RETURNING_TO_HOME
       │                  │                    │                 │
       ▼                  ▼                    └──route done──►IDLE
    CHARGING        on_docked → DOCKED
       │                                       └──need_charge──►RTH
       │ schedule_active + !need_charge
       ▼
    UNDOCKING ──on_undocked──► UNDOCKED ──_pending_geopath──► NAVIGATING
                                              │
                                              └──no pending──► query scheduler
ERROR ← set_error() bypass (only exits to IDLE/MANUAL)
```

**IDLE auto-transition priorities** (`states/idle_state.py:determine_next_state`):
1. → `DOCKED` if at/near charge_pos + RTK fix (charge tolerance is 5 cm normally, 40 cm "near", 50 cm in MANUAL/IDLE state)
2. → `DOCKING` (if at home) or `RETURNING_TO_HOME` (otherwise) when `need_charge`
3. → `DOCKING` auto if at home + no route + no schedule (skipped during 5 s grace period after coming from NAVIGATING/RTH so the scheduler has time to load the next route in LOOP/PING_PONG)
4. → `NAVIGATING` if `route_active` (resume after MANUAL/pause)

**NAVIGATING grace period 6 s** prevents a race where `route_active` flickers False during transitions and would otherwise auto-flip to RETURNING_TO_HOME.

**MANUAL override always wins**: `RobotStateMachine.transition_to` checks `autonomous_operation_enabled` first; if False, only MANUAL is reachable. MANUAL can re-enter IDLE either when autonomous is re-enabled or when at/near charge with RTK (for startup recovery after MANUAL).

### 6.2 Two waypoint followers

`tactical_wp_follower_node.__init__` instantiates **both** and swaps live via `/control/obstacle_avoidance_enabled`:
- **Linear** (`navigation/waypoint_follower.py`, 424 LoC) — pure Python, used when avoidance is off. Hysteresis rotate-gate **25 °/15 °** (replaces a single 22.5 ° threshold that was bang-banging — RCA 2026-05-18). PD rotation with deadlock boost (yaw-rate < 0.3 ° over 0.5 s → +3 °/check, cap 35 °). Rotation timeout 15 s → skip waypoint. Min steering 12 when rotation > 10 °. Modes ONCE / LOOP / PING_PONG with `round_started` latch.
- **Nav2-based** (`navigation/waypoint_follower_nav2.py`, 964 LoC) — used when avoidance is on (path planning around obstacles).

State transfer on swap preserves mode + waypoints + current WP index.

### 6.3 Heading source override

`_get_current_pose_map()` overrides `pose.orientation` with a quaternion built from `/fixposition/ypr.vector.x` (cached yaw in rad). The odom/TF quaternion is unreliable (observed flipping ±180 ° randomly — RCA 2026-05-17 Issue 9). **Critical trap:** Fixposition packs `Vector3` as `(yaw, pitch, roll)`, not the conventional `(roll, pitch, yaw)` — see `fixposition_driver/data_to_ros2.cpp:221`. Reading `.z` gives roll (RCA 2026-05-17 Issue 10).

### 6.4 Two route entry points — both converge

Both ultimately call `_start_navigation_directly(geopath, route_name)`:

1. **Scheduler path**: `tactical_scheduler_node` (every 2 s) → if a schedule is active, serve `/control/get_schedule_action` (async service). wp_follower's `_handle_schedule_action_response` decides:
   - DOCKED/CHARGING → `_trigger_undocking_for_schedule` (queues `_pending_geopath`, transitions to UNDOCKING; on `on_undocked` the pending path is consumed and `_start_navigation_directly` runs)
   - UNDOCKED → `_start_route_from_schedule_response`
   - IDLE → behaves per position (at charge → undock; at home → start directly; elsewhere → start as recovery)

2. **Run-Now path**: web `POST /api/routes/start_now/{name}` → calls `/control/waypoints` service → `_waypoint_service` (in wp_follower). If DOCKED/CHARGING it synthesizes a scheduler-style response and calls the same `_trigger_undocking_for_schedule`. Otherwise direct.

**Both paths share the same docked-safety + map-origin + GPS→map transform + `set_mode`/`set_waypoints` + `is_active()` sanity check.** This convergence was the RCA 2026-05-17 Issues 6+7 fix; previously `_waypoint_service` crashed on a nonexistent `GeoPath.route_name` attribute and called `set_waypoints` with the wrong signature.

### 6.5 Cold Start vs Warm Resume

When leaving NAVIGATING for any reason that's not "completed" or "abort via RTH" (MANUAL override, autonomous off, watchdog, pose unavailable, etc.), wp_follower snapshots `_paused_route_name` + `_paused_wp_idx`. The next Run-Now of the same route resumes from that WP. Previously `find_nearest_waypoint` was used here, which silently skipped early WPs when the robot was physically displaced — RCA 2026-05-18.

### 6.6 Safety gates enforced by `_control_loop` (in order)

1. **MANUAL override** — highest priority; `transition_to` is the choke point.
2. **ERROR state** — can only exit to IDLE / MANUAL.
3. **RTK gate to enable autonomy** — `_autonomous_operation_service` requires both `is_rtk_ready_for_docking()` (both GNSS at status 8 = RTK_FIXED, stable 3-of-5) and `is_fusion_initialized()` (`init_status == 2`).
4. **RTK gate during navigation** — allow RTK_FLOAT (status ≥ 5) but emergency-stop if completely lost.
5. **Fusion init_status == 2 always required** during autonomy (`init_status` is *not* the same as `fusion_status`; see §12 trap).
6. **GPS jump > 2 m** → stop motors + stop follower until RTK re-fixed.
7. **LiDAR timeout 2 s** (when avoidance on, in NAVIGATING/RTH) → halt drive but **keep follower active** (resumes when LiDAR returns).
8. **Roboclaw watchdog 0.3 s** in `roboclaw_wrapper_node` → `_slam_brake` (independent of wp_follower).
9. **Pose unavailable** → halt drive + warn web; resumes on pose recovery.
10. **Charging-state lockout** in `robot_controller_node._process_joy_command` (manual stick silently ignored, German warning published).
11. **ESP L1 deadman** at firmware level (no Joy upstream without L1).

### 6.7 Topics published by wp_follower
- `/cmd_drive` (CommandDrive) — wheel velocities in rad/s
- `/tactical/robot/state` (String, JSON) — current state name + RobotStateContext snapshot + `active_route` (name, waypoints, mode, **`current_waypoint_index` live from follower** — previously was None, recently patched)
- `/control/autonomous_operation` (Bool) — mirror of internal flag (consumed by robot_controller)
- `/tactical/robot/route_completed` (Bool) — pulse on completion (consumed by scheduler)
- `/tactical/robot/charging_status` (String) — docking phase strings (positioning, aligning, following, docked, undocking, completed, error)
- `/tactical/robot/speed_kmh` (Float32)
- `/tactical/logging/{info,warn,error}` (String) — surfaced in web UI (German content where operator-facing)
- `/tactical/control/charging/requested` (Bool)

### 6.8 Topics subscribed by wp_follower
- `/robot/state` (from robot_controller; reads unified `charging_state`)
- `/fixposition/odometry_llh` (NavSatFix; triggers first-fix map-origin auto-set)
- `/fixposition/odometry_enu` (Odometry; orientation, speed, GPS-jump watchdog)
- `/fixposition/ypr` (Vector3Stamped; **`.x` is yaw**, cached in `_last_yaw_rad`)
- `/fixposition/fusion` (FusionEpoch; feeds RTKStatusMonitor)
- `/tactical/control/charging/dock` (Point: x=lat, y=lon, z=yaw_deg)
- `/tactical/control/charging/undock` (Bool)
- `/control/set_speed` (Float32)
- `/control/obstacle_avoidance_enabled` (Bool)
- `/control/enable_charging` (Bool; tracked as `_charging_relay_enabled` for context)
- `/obstacle_detected` (Bool)
- `/obstacles/lidar` (OccupancyGrid; LiDAR-timeout watchdog stamp)
- `/obstacles/sectors` (ObstacleSectors; fed to obstacle avoidance)

---

## 7. Obstacle stack

```
Livox MID-360 ──► /obstacles/lidar  (OccupancyGrid, grid cell 0.1 m, min 2 pts → obstacle)
RealSense D455 ──► /obstacle_sectors  (UInt8MultiArray, 5 columns × 2 rows: near 0–0.4 m, far 0.4–1.0 m)
                            │
        ┌───────────────────┴───────────────────┐
        ▼                                       ▼
  control/control/obstacle_avoidance/fusion_node.py
   • LiDAR within camera FOV (±43°, ≤ 2 m range) is suppressed unless camera confirms
   • 2 s stale timeout on camera → pure LiDAR passthrough
                            │
                  /obstacles/fused (OccupancyGrid)
                            │
  sector_publisher_node + sector_analyzer (config: control/config/sector_config.yaml)
   • near sector: 0–1.2 m forward, ±0.4 m lateral, type STOP, priority 1
   • far sector: -0.3–3.0 m forward, ±1.0 m lateral, type SLOW, priority 0
                            │
                  /obstacles/sectors (ObstacleSectors)
                            │
  navigation_helper.execute_waypoint_following → ObstacleAvoidanceController.apply()
   ┌──────────────────────────────────────────────────────────┐
   │ FREE_DRIVE      (no blocked sectors)  → passthrough      │
   │ SLOW_APPROACH   (SLOW blocked)        → speed × 0.6       │
   │ EMERGENCY_STOP  (STOP blocked)        → (0,0) for 3 s,    │
   │                                          then (steer,0)   │
   │                                          to allow rotate-out │
   └──────────────────────────────────────────────────────────┘
   Deadlock detector: 30 s in EMERGENCY with < 0.5 m movement → warning to web app
```

**Sensor watchdogs:** fusion timeout 2 s (camera), LiDAR timeout 2 s in wp_follower (halts drive), deadlock 30 s (warn only). The thermal stream does **not** feed avoidance — RGB + thermal are display-only.

---

## 8. Docking + charging

`control/control/charging/docking_controller.py` (1235 LoC) — three-phase line-based approach:
1. **APPROACH_LINE** — perpendicular to the home→charge line, until ±3 cm lateral
2. **ALIGN_TO_LINE** — pure rotation to face the charge point (±2 °)
3. **FOLLOW_LINE** — 30 % max speed, PD on lateral + heading, line discretized at 10 cm. Critical zones: 50 cm→3 cm tolerance, 30 cm→8 ° tolerance.

GPS-to-rotation-center offset hardcoded **0.18 m** forward (sensor in front of vehicle center, see `transforms.vrtk_to_base.x` in `control/config/params.yaml`).

**"Docked" = within 5 cm of charge GPS.** There is **no contact sensor** — a misaligned dock falsely succeeds and silently shows charged. There is also no charging-relay watchdog — relay is set once on CHARGING entry, no heartbeat.

Undocking uses more aggressive backward gains; force-stops 5 cm past home distance.

`states/charging_state.py`: enters → publishes relay enable + halts; exits when relay disabled AND (schedule_active OR route_active) AND NOT need_charge. Auto-transition CHARGING→UNDOCKING when `schedule_active && !need_charge`.

Home-return (`control/control/home_return/home_return_controller.py`): `create_reverse_route` reverses (or finds shortest direction in LOOP) the closest route within `max_route_distance` (default 3 m, `settings.yaml`). `RETURNING_TO_HOME` reuses the linear/Nav2 follower with the reverse path.

---

## 9. Scheduling — `tactical_scheduler_node` (1535 LoC)

Loads `/routen/settings/schedules.yaml` via `ScheduleManager`. 2 s tick. For each schedule: `TimeWindowChecker.is_schedule_active` checks `active=True` + today in `weekdays` (0=Mon) + `start_time ≤ now < end_time` (HH:MM; `00:00` end = no end). On match, serves `/control/get_schedule_action` (called by wp_follower) — returns the next route in the schedule's `routes[]` per `route_mode` (`sequential`/`random`) and `repeat_count`. `loop=true` loops the whole route list; `require_home_return=true` forces RTH at completion.

**Known scar:** per-schedule `battery_threshold` is loaded but the scheduler uses `_default_battery_threshold` (28 %) at runtime — overrides are silently ignored.

### Schedule YAML schema (`/routen/settings/schedules.yaml`)
```yaml
schedules:
  - schedule_id: schedule_0_1779363712       # timestamp-based UUID
    active: true
    start_time: "08:00"
    end_time: "00:00"                         # 00:00 = no end
    weekdays: [0, 1, 2, 3, 4, 5, 6]           # 0 = Monday
    routes:
      - route_name: test
        repeat_count: 1
    route_mode: sequential                    # or "random"
    loop: true                                # loop the routes[] indefinitely
    battery_threshold: 28                     # currently ignored (uses default)
    home_tolerance: 0.5
    require_home_return: true
    auto_charge_return: true
    done: false                               # completion flag
```

### Route YAML (`/routen/routes/*.yaml`)
```yaml
loop_mode: false
name: test
waypoints:
  - latitude: 48.98055
    longitude: 8.37646
    altitude: 0
  - latitude: 48.98052
    longitude: 8.37648
    altitude: 0
```

### Settings YAML (`/routen/settings/settings.yaml`)
- `home_point.{latitude, longitude}` (+ optional `yaw`)
- `charge_point.{latitude, longitude, yaw}`
- `speed_factor` (max linear m/s; e.g. 0.874)
- `battery_threshold` (% — default 28)
- `waypoint_tolerance` (m — default 0.5)
- `home_tolerance` (m — default 0.5)
- `max_route_distance` (m — recovery cutoff, default 3)
- `enable_obstacle_avoidance` (bool)
- `auto_charge_return` (bool)
- `charge_detection_radius` (m — default 0.8)
- `security_email.{enabled, sender, recipients, smtp_host, smtp_port, username, password, use_tls}` (read from env vars `SECURITY_EMAIL_USER` / `SECURITY_EMAIL_PASS` — never put creds in code)
- `security_event_defaults.{pre_event_seconds, post_event_seconds}`
- `security_notification_windows: []`
- `security_video_capture_timeout_seconds`
- `security_always_enable_event_types: []`

The web app and the ROS scheduler both touch this file — the ROS scheduler is the single writer at runtime (no file lock; AUDIT-style risk noted).

---

## 10. Web UI — `robot_web_interface`

FastAPI on `:8020`, single worker, `network_mode: host`, **no auth** (AUDIT HIGH-3). Lifespan boots one `RobotNode` on a `MultiThreadedExecutor` in a background thread + a `ConnectionManager` for WebSocket broadcast. Static-file cache busting via MD5 of `/app/static`. Frontend is German.

### REST endpoints (selection)
| Method/Path | Triggers |
|---|---|
| `GET /api/routes` | List routes |
| `GET /api/routes/{name}` | Load route YAML |
| `POST /api/routes/save` | Save new route (calls `publish_waypoints` service) |
| `POST /api/routes/start_now/{name}` | **Run-Now** — load YAML, call `/control/waypoints` (RCA 2026-05-17 Issue 5) |
| `DELETE /api/routes/delete/{name}` | Cascade-delete: remove from all schedules, deactivate empty schedules, then unlink YAML (RCA 2026-05-17 Issue 4) |
| `POST /api/control/stop` | Toggle autonomous operation |
| `POST /api/control/clear_route` | Publish empty waypoints (STOP) |
| `POST /api/control/light` | Light service |
| `POST /api/control/alarm` | Alarm service (**HIGH-5: fake success**) |
| `POST /api/control/siren` | Siren service (**HIGH-5: fake success**) |
| `POST /api/control/emergency_stop` | E-stop service |
| `POST /api/control/charge/manual`, `.../go_to_charge` | Manual charge ops |
| `GET/POST /api/schedule/schedules`, `/save`, `/edit/{i}`, `/delete/{i}` | Schedule CRUD |
| `GET/POST /api/settings/home-position`, `/battery` | Settings |
| `GET /api/events`, `/api/events/{id}`, `/api/events/{id}/video/{cam}`, `DELETE /api/events/{id}` | Security clip viewer (Range support for video) |
| `GET/DELETE /api/logs` | Persistent logs |
| `GET /api/version`, `/api/debug/enabled` | meta |

### WebSockets
`/ws/position`, `/ws/robot_state`, `/ws/camera/{main,thermal1,thermal2,rgb2,lidar_debug,person_detection,depth_debug}`.

### Eneo events
UDP `:5002` → `eneo_event_publisher` parses JSON (filters PersonDetect + FireDetect, 200 ms rate-limit, `eneo_parser.py` hardcodes camera IP `192.168.10.203` — site-specific) → `/security_alert` → web service handler tries to call `/capture_event_clips`. **`video_ringbuffer` is currently disabled** (AUDIT HIGH-2: `controller.launch.py:64-78` commented out); the `/capture_event_clips` service has no provider, so security events fire but no clips are recorded. The package's `EVENT_RECORDING_CAMERA_IDS` falls back to hardcoded `['eneo_rgb', 'eneo_thermal']` (AUDIT MED-6).

### Security events on disk
`/routen/security_events/{event_id}/`:
- `metadata.json`
- `{camera_id}.mp4` (when ringbuffer is enabled — PII per CLAUDE.md §2; never sync off-robot)
- top-level `events_index.json`

---

## 11. RTK / Fixposition — the trap

`control/control/gnss/rtk_status_monitor.py` (`RTKStatusMonitor`):
- GNSS status codes (Fixposition): `0=NO_FIX, 1=SPP, 2=DGPS, 3=PPS, 5=RTK_FLOAT, 6=ESTIMATED, 8=RTK_FIXED`.
- `is_rtk_fixed()` requires both GNSS at `RTK_FIXED` (status 8). Data freshness window: 2 s.
- `is_stable()` requires 3 consecutive good readings in a window of 5.
- `is_rtk_ready_for_docking()` = fixed + stable.
- `is_fusion_initialized()` checks `init_status == 2` (Globally initialised).

**The field-name trap (RCA 2026-05-17 Issue 2):** Fixposition's `/fixposition/fusion` message contains BOTH `fpa_odomstatus.init_status` (0/1/2) and `fpa_odomenu.fusion_status` (0–4, fusion mode). Our code reads `init_status` but stores it under a dict key literally named `'fusion_status'` (see `wp_follower:1785`). So `RTKStatusMonitor.fusion_status` actually means `init_status`. When debugging with `ros2 topic echo`, the on-the-wire field with that name is a *different* number. Don't get fooled by the name.

The RTK + dual-antenna heading needs open sky on both antennas. The web UI's GNSS2 badge (`static/js/index.js:530-557`) is tied directly to `gnss2_status` — a brief red flicker is usually transient.

---

## 12. Useful scripts / artefacts in the repo

- `bringup-new-jetson.sh` — full host setup for a fresh Orin Nano (groups, docker, nvidia runtime, NetBird, masks `nvgetty`, clones repo, brings all 4 compose projects up). See `BRINGUP.md`.
- `swap_ip.sh` / `restart_containers_after_ip_swap.sh` — IP-swap dance (used during the Nano→Orin migration).
- `diag_wp_record.sh` — record a diagnostic bag of the drive cascade + autonomy topics + a YAML snapshot of params. Bag goes to `/tmp/` inside `ros2_jetson` (tmpfs — pull with `docker cp` before reboot).
- `analyze_wp_bag.sh` — companion analyzer.
- `tactical_mower/ros2_ws/{analyze_heading_test.py, analyze_wp_bag.py, inspect_wp_bag.py, diag_traj.py, live_heading_check.py, live_heading_check_patched.py}` — offline analysis tools written during the autonomy RCAs.
- `auto_heading_test/` — recent heading-test artefacts.
- `routen/routes/test.yaml`, `test2.yaml` — current operator routes (recently snapshotted in commit `7a27b50`).

---

## 13. The scars — read these before you change anything load-bearing

### Safety / autonomy
- **AUDIT HIGH-1 — Roboclaw `/cmd_drive` watchdog**: **fixed** in RCA 2026-05-16. 0.3 s timeout, 20 Hz check, `_slam_brake` via `DutyAccel(brake_accel, 0)`. Don't undercut it.
- **AUDIT HIGH-2 — `video_ringbuffer` disabled**: security alerts fire but no clips. `controller.launch.py:64-78`. Stated purpose of the product is broken.
- **AUDIT HIGH-3 — no API auth**: LAN-only assumption. Anyone reachable on `:8020` can drive the robot. Don't widen the network surface without fixing this.
- **AUDIT HIGH-4 — hardcoded RTSP creds**: `Tactical-Thermal-Stream/.../dual_rtsp.launch.py:20`, `live_detect.py:47` (literal `root:axis` is AXIS factory default), plus duplicates. Move to env vars.
- **AUDIT HIGH-5 — alarm/siren fake-success**: `robot_controller_node.py:296-324` set `response.success = True` with no hardware effect. For a security product, worst-case silent fail.
- **AUDIT HIGH-6 — manual-steering LPF rolled back**: history-dependent filters are incompatible with the event-driven Joy stream. Use stateless input shaping (expo/cubic + deadband) if you try again.
- **ESP `lx/ly` saturate at ±508** in the raw text dump; kinematics clamps at ±100 — only ~20 % of stick travel reaches full speed. Cosmetic but felt.
- **22.5 ° rotate-gate bang-bang**: replaced with 25/15 ° hysteresis (RCA 2026-05-18). Don't go back.
- **`wp_idx` previously None** in `/robot/state.active_route`: now patched to publish live from the follower. Tests that assumed None will fail.
- **DDS `ROS_DOMAIN_ID` mismatch**: fixed via overrides in 2026-05-16. If a sensor topic shows in `ros2 topic list` but `info` says `Publisher count: 0`, suspect domain drift first.
- **Roboclaw H-bridge configured COAST at duty=0** (verified 2026-05-22). DutyAccel ramp itself provides regen braking — don't use plain `Duty(0)` (RCA 2026-05-16 Iter 1 chassis-jump regression).

### Architectural
- **AUDIT MED-1 — duplicate trees**: outer `tactical_mower/ros2_ws/` is canonical. The nested `tactical_mower/tactical_mower/` and `{livox,realsense,thermal}_obstacle/` mirrors have diverged. Edit outer only.
- **AUDIT MED-2 — no try/except around Roboclaw serial calls** in `send_velocity`. A CRC mismatch kills the subscription callback. Mirror the `_read_battery_data` try/except pattern.
- **AUDIT MED-3 — `/dev/ttyACM0` hand-edited override** in docker-compose (TODO about renumbering). Confirm the device path is stable.
- **AUDIT MED-4 — settings file path hardcoded** to `/routen/...`. Works in Docker, silent default on native.
- **AUDIT MED-5 — mixed German/English logs**. Operator-facing surface (web `/tactical/logging/*`) is German; dev logs are English. Don't "fix" opportunistically — match the surrounding file.
- **AUDIT MED-7 — `import json` inside `_tactical_robot_state_callback`** (`robot_controller_node.py:432`) signals nobody is reviewing. No try/except on the outer parse.
- **AUDIT MED-8 — GitHub App install failed** for `maryammohammadipilehvar-sudo/quarero-projects`. PR-based review needs the app granted access or the remote moved.
- **AUDIT MED-9 — RealSense `depth_callback` no try/except** on `cv_bridge.imgmsg_to_cv2`.
- **AUDIT MED-10 — `live_detect.py` MJPEG on `0.0.0.0:8080`** unauthenticated.
- **AUDIT MED-11/12 — no CI, no pinned requirements**.
- **No contact sensor for docking**: 5 cm GPS = "docked". A misaligned dock silently fails to charge.
- **Per-schedule `battery_threshold` ignored**: scheduler uses `_default_battery_threshold` (28 %).
- **`fusion_status` field-name trap**: see §11 above.
- **`/dev/ttyTHS1 @19200` is dead on Orin** — live joy path is the CP2102 text stream. Stale param values still reference the dead path.

### Operator workflow rules (from CLAUDE.md)
- **Never commit to `main` directly.** No commits without explicit per-session approval. Never `git add -A` in `~/gits/` (security_events PII risk).
- **Never edit `/etc/systemd/...` or run `sudo`** without explicit approval.
- **Never kill/restart containers while the robot is moving** or on a scheduled patrol.
- **Never `rm` under `~/gits/routen/`** — live operator data.
- **Never test autonomous driving changes without the operator physically present and able to hit e-stop.**

### Memory pointers (durable across sessions)
Lives in `~/.claude/projects/-home-quarero02-gits/memory/`. Indexed in `MEMORY.md`. Key entries (load only when relevant):
- `project_platform_migration.md` — Nano→Orin NX context
- `reference_uart_chain.md` — PS5 → ESP → CP2102 → joy_controller → /cmd_drive → Roboclaw
- `feedback_esp_deadman.md` — L1-held required; silent UART is normal
- `feedback_ps5_fastblink.md` — fast-blink = BT wedge, power-cycle ESP
- `project_demo_2026_05_16.md` — customer demo prep
- `project_joystick_stop_delay_rca.md` — pointer to the RCA + key params
- `reference_diagnostic_bag_recipe.md` — bag record + offline analysis
- `feedback_brake_tuning.md` — `drive_acceleration_factor` / `brake_acceleration_factor` trade-offs
- `project_dds_domain_id_fix.md` — DDS override files
- `reference_dds_domain_recipe.md` — inventory across containers
- `reference_fusion_status_naming.md` — the `init_status` vs `fusion_status` trap
- `reference_person_detection_arch.md` — person detection runs on Jetson 2, not a drive-safety gate
- `project_autonomy_failure_2026_05_18.md` — race on retry skips WP0/WP1

---

## 14. Triage — "the robot is misbehaving"

(From RCA_2026-05-17 §"How to debug")

1. **What does `/robot/state` say?**
   ```bash
   docker exec ros2_jetson bash -lc \
     "source /opt/ros/jazzy/setup.bash && source /app/ros2_ws/install_ros2/setup.bash && \
      ros2 topic echo --field data /robot/state --once"
   ```
   Check `autonomous_enabled`, `autonomous_mode`, `active_route`, `charging_state`. If `active_route` is empty, the robot has nothing to do — load a route via Run-Now or schedule.

2. **What state-machine state is it in?**
   ```bash
   ros2 topic echo /tactical/robot/state --once
   ```
   One of MANUAL / IDLE / DOCKED / CHARGING / UNDOCKING / NAVIGATING / RETURNING_TO_HOME / ERROR / UNINITIALIZED / UNDOCKED.

3. **Is `/cmd_drive` flowing and what values?**
   ```bash
   ros2 topic echo /cmd_drive --once
   ```
   Zero or absent → wp_follower chose not to drive (state machine in IDLE/MANUAL, or a safety gate fired).

4. **Recent warnings?**
   ```bash
   docker logs ros2_jetson --since=2m 2>&1 | grep -iE "warn|error|watchdog|stopping"
   ```

5. **If not obvious, record a bag** with `~/gits/diag_wp_record.sh` and analyze with `analyze_wp_bag.py`.

---

## 15. The first 60 seconds of any new session

1. Read this file (you're doing it).
2. `git status` — note staged vs unstaged vs untracked. Recently the staged work was a snapshot of operator routes + schedule + settings (commit `7a27b50`).
3. Check what was last RCA'd — look at the latest `RCA_*.md` and the latest memory entries in `MEMORY.md` for unfinished work.
4. If the operator's task touches motion / state machine / obstacle handling / RTK / docking — design first, code second (CLAUDE.md §1). Confirm with operator if it isn't strictly a bug fix.
5. If the task touches `~/gits/routen/`, `/etc/systemd/`, anything with `sudo`, or anything that restarts containers while the robot is operating — **ask first**.

---

*Last updated 2026-05-23 during the "understand everything" session that read the full autonomy critical path (`tactical_wp_follower_node.py`, all state classes, drive cascade, configs, launch files) and surveyed the wider subsystems (web/routen, obstacle avoidance, docking/scheduling, ESP32 firmware) via parallel Explore agents. If you change architecture, update this file.*
