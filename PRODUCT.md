# Product: Tactical Mower (Robot)

## What this is
A **two-wheeled mobile robot** for **remote optical surveillance and perimeter inspection**. Runs on a **Jetson Orin Nano** (`192.168.10.226`) mounted on the robot itself. Manual driving from a **PS5 controller over Bluetooth → ESP32 → UART → Jetson**. Autonomous driving via a **GPS/RTK waypoint follower** (Fixposition fusion) with **LiDAR + depth-camera obstacle avoidance**.

This repo (`~/gits/`) is the full on-robot software bundle: ROS 2 Jazzy packages, a FastAPI web UI served on port 8020, and ESP32 firmware.

Not in scope: the customer's separate fixed-camera security-camera project (different codebase).

## Who uses it
- **Customer**: warehouse / multi-site facility operator.
  [OPERATOR INPUT NEEDED: is this a single-site pilot or multi-site rollout today?]
- **Primary users**: on-site guards / shift operators who drive the robot manually from the web UI or PS5 pad, trigger lights/siren, start a scheduled patrol route.
- **Secondary users**: facility manager (reviews captured security event clips, tunes routes/schedules).

## How it's deployed
- One Jetson per robot, docker-compose on boot: `ros2_jetson` container (ROS stack) + `web_app` container (FastAPI UI on :8020).
- Routes and settings live in `~/gits/routen/` on the host and are bind-mounted into both containers.
- Currently pinned to **one robot / one Jetson** at `192.168.10.226`.
  [OPERATOR INPUT NEEDED: fleet size today and planned; any OTA / remote-management expectation?]

## What the robot does
1. **Manual drive** — PS5 controller (L1 = deadman) → ESP32 UART → `control/joy_controller` → `drive/robot_controller` → `drive/roboclaw_wrapper` → RoboClaw → motors.
2. **Web UI drive** — browser joystick → `/joy_web` → same drive path (gamepad wins on contention, 500 ms timeout).
3. **Autonomous patrol** — named routes (YAML under `routen/routes/`) of lat/lon waypoints, followed by `control/tactical_wp_follower` using Fixposition RTK + optional Nav2. Supports `once` / `loop` / `ping_pong` modes, obstacle avoidance on/off per route, low-battery auto-return-home, auto-docking at a charge point.
4. **Scheduled patrols** — `tactical_scheduler` runs routes on a time-of-day / weekday schedule.
5. **Security events** — Eneo IP camera fires UDP events on port 5002 → `eneo_event_publisher` → `/security_alert` → (intended) `video_ringbuffer` captures pre/post clips → files under `routen/security_events/` → surfaced in web UI → optional SMTP email.
6. **Peripherals** — light / alarm / siren / charging-relay commands are routed via ROS topics to the ESP32.

## Hardware (on the robot)
- **Compute**: Jetson Orin Nano (JetPack 6+)
- **Motors**: RoboClaw controller over USB `/dev/ttyACM0` @ 115200, address 128, encoderless duty-cycle mode
- **MCU**: ESP32 on UART `/dev/ttyTHS1` @ 19200 (PS5 bridge + relays)
- **Sensors**:
  - Livox **MID-360** LiDAR (Ethernet, obstacle detection via grid occupancy)
  - Intel **RealSense D455** depth camera (front-facing, 5×2 sector obstacle detector)
  - **Fixposition Vision-RTK** (GPS + fusion)
  - **Eneo** IP camera (RGB + thermal RTSP, UDP event notifications)
  - [OPERATOR INPUT NEEDED: confirm AXIS F41 camera at `.174` — referenced in `Tactical-Thermal-Stream/live_detect.py`, unclear if still in use]
- **Battery**: 7S12P 18650, 24 V / 30 Ah / 720 Wh nominal
- **Power rails**: 24–30 V (motors/RoboClaw/modem), 12 V (Jetson, thermal cam), 5 V (ESP32/relays)
- **Network**: 4G/5G router for remote access; camera subnet `192.168.10.x`

## Non-goals (today)
- Not a lawn-mower. Despite the name `tactical_mower`, there is no cutting hardware in this codebase.
- Not a fleet manager. Single-robot stack; no multi-robot coordination, no cloud backend.
- No mapping / SLAM. Localization is pure GPS/RTK fusion via Fixposition.
- No person-re-id / tracking intelligence. Person detection is a single-class YOLOv8 MJPEG stream (`Tactical-Thermal-Stream/live_detect.py`) — used only for live display, not fed back into autonomy.
- No auth on the web UI. Port 8020 is open on the LAN by design (current assumption: trusted VPN/LAN).
  [OPERATOR INPUT NEEDED: is LAN-only acceptable long-term, or do we need login + HTTPS?]
- No OTA updates. Deploy = SSH to Jetson, `git pull`, rebuild.

## The component layout (today, `~/gits/`)
- `tactical_mower/` — the main on-robot stack. ROS 2 workspace, web UI, Docker compose, ESP32 firmware.
- `livox_mid_360_obstacle_detection/` — standalone Livox driver + grid-based obstacle node (Docker).
- `RealsenseD_camera_Obstacle_avoidance/` — standalone RealSense depth obstacle node (Docker).
- `Tactical-Thermal-Stream/` — standalone RTSP→ROS bridge for the Eneo RGB/thermal camera, plus a YOLOv8 live-detection helper.
- `routen/` — runtime data: routes, schedules, settings, captured event clips. Bind-mounted into containers.
- `AI_camera` — 1-byte placeholder file, not a project.

The three "standalone" component repos are each a sensor pipeline intentionally split out so they can be developed/tested in isolation. At runtime they publish ROS topics that `tactical_mower` subscribes to.
  [OPERATOR INPUT NEEDED: confirm that on the real robot today, each of those three containers is running alongside `tactical_mower`, OR that their code has been inlined somehow — see AUDIT.md, this is currently ambiguous.]
