# RCA — "robot doesn't stop immediately when joystick released"

**Session:** 2026-05-16 (customer demo day, on the Orin NX `quarero02@quarero02-desktop`, `192.168.10.226`).
**Status at sign-off:** software-side cascade verified bulletproof (Joy→Cmd = 2 ms p50). Perceived stop delay reduced from ~1–3 s to ~200–500 ms. **Wheel-stop-vs-coast question still open** (see §7) — that result determines whether further work is needed in Roboclaw firmware config vs. ROS code.

## 1. Symptom (operator-reported)

> "When I move the robot with the joystick and then stop the joystick (release it), the robot doesn't stop immediately and keeps going for some seconds (varying) then stops."

Later refined:
> "Motors sometimes keep moving when I release PS5; sometimes stop with huge momentum that the robot pitches forward."

And after the ramp fix:
> "It doesn't stop when I release joystick, keeps moving, and after huge delay it stops. Also sometimes when I start pushing stick, robot starts moving with delay."

## 2. Architecture — data flow (manual joystick drive)

```
PS5 ──BT──► ESP32  ──UART0/CP2102 @115200──► /dev/esp_joy
                                                      │
                              joy_controller_node ◄───┘  (control pkg)
                              • daemon thread reads ESP debug stream
                              • parses "idx=... buttons:0x... axis L:..."
                              • publishes Joy on /joy_drive_raw
                                                      │
                              robot_controller_node ◄─┘  (drive pkg)
                              • _joy_gamepad_callback → process_joy_command
                              • DifferentialDriveKinematics + flip_steering
                              • publishes CommandDrive on /cmd_drive
                                                      │
                              roboclaw_wrapper_node ◄─┘  (drive pkg)
                              • drive_cmd_cb → DutyAccelM1/M2
                                                      │
                                          Roboclaw 2x15  (USB /dev/ttyACM0 @115200)
                                                      │
                                                    motors
```

Side channels (not implicated in this bug):
- `/joy_web` — web-UI joystick fallback (silent in this session, 0 messages over 82 s bag)
- `/control/light`, `/control/enable_charging` — back-channel through `joy_controller` → ESP UART TX
- `tactical_wp_follower_node` also publishes `/cmd_drive` in autonomous mode (blocked in manual)

## 3. Original root-cause analysis

Nine root causes were identified before any code change (full analysis preserved here so the reasoning survives even if the code is reverted):

| # | Cause | Layer | Contribution |
|---|---|---|---|
| 1 | `drive_acceleration_factor=0.3` → Roboclaw firmware ramps 3.3 s full-throttle → 0 | HW/config | **dominant for the stop tail** |
| 2 | No `/cmd_drive` watchdog. `velocity_timeout: 2.0` in params was dead config (param never read in `__init__`). Roboclaw firmware `SetSerialTimeout` also never called. | SW | named in CLAUDE.md §1 as a known hazard |
| 3 | L1-release stop was a one-shot `pub.publish(Joy())`. Fragile: any single dropped DDS msg → motors keep going. | SW | ~10–20 % of releases failed |
| 4 | `readline(timeout=0.1)` + bounded drain (32 lines/iter) coupled `/joy_drive_raw` cadence to ESP debug-line rate; loop could stall 100 ms+ between joy frames | SW | ~64 ms avg detection delay |
| 5 | Kinematics 12 % deadband only catches stick rest bias, not "stop" — non-zero stick at release relies entirely on L1-released edge to brake | SW | minor |
| 6 | `Roboclaw.DutyAccelM1` is fire-and-forget over USB; one dropped USB packet = stop command lost, no retry | HW | minor |
| 7 | rclpy single-threaded executor + 10 Hz battery-read timer doing slow Roboclaw USB reads → watchdog/timer dispatch jitter (visible as inflated "no /cmd_drive in 1.04 s" log timings) | CPU | logs only; brake itself still fast |
| 8 | `/cmd_drive` QoS depth=10 reliable — zero can sit behind nine prior non-zero commands in queue | SW | not observed |
| 9 | Orin NX thermal / nvpmodel jitter | CPU | not observed |

(see conversation log if more detail needed — the per-cause analysis is too long for this doc.)

## 4. Fixes applied — by iteration

### Iteration 1 (2026-05-16 ~14:30) — first cut

Files touched:
- `tactical_mower/ros2_ws/src/drive/src/drive/nodes/roboclaw_wrapper_node.py`
- `tactical_mower/ros2_ws/src/drive/config/params.yaml`
- `tactical_mower/ros2_ws/src/control/control/joy_controller_node.py`

Changes:
1. **Wire up the dead `velocity_timeout` param** and add a ROS-side `/cmd_drive` watchdog (`_cmd_watchdog_check` at 20 Hz). If no `/cmd_drive` arrives within timeout → `_slam_brake()`. Initial timeout 0.5 s.
2. **Fast-brake on commanded zero**: in `send_velocity`, if `qpps == 0`, call plain `DutyM1/M2(0)` instead of `DutyAccelM1/M2(drive_accel, 0)`. Plain `Duty` bypasses the 3.3 s ramp.
3. **Brake on `destroy_node`** — don't leave the last duty latched across container restarts.
4. **L1-released burst (vs one-shot)**: `_ZERO_BURST_FRAMES = 3` in `joy_controller_node`.

**On-robot result:** "Sometimes motors keep moving, sometimes huge momentum / pitch forward."

### Iteration 2 (2026-05-16 ~14:50) — based on log analysis

Live log showed watchdog firing every release with elapsed 0.51–1.17 s. Diagnosed:
- "Pitch forward" = plain `Duty(0)` is too binary (slammed brake when it did work).
- "Keeps moving" = the 3-burst zeros were iteration-coupled and sometimes dropped; watchdog (0.5 s) was doing the actual braking.

Changes:
1. **Zero window = wall clock, not iteration count.** Replaced `_ZERO_BURST_FRAMES=3` with `_ZERO_WINDOW_S=0.5`. Continuous zeros for 500 ms after L1 release, regardless of UART read rate.
2. **Brake = `DutyAccel(brake_accel, 0)` with new `brake_acceleration_factor=5.0` param** (= 49,150 cnt/s → ~0.67 s ramp full→0). Smoother than slam, faster than drive_accel.
3. **Watchdog tighter:** `velocity_timeout: 0.5 → 0.3 s`.

**On-robot result (from bag `test_record2/`):** Cascade verified — joy → cmd p50 = 2.2 ms, first zero after release = 1–11 ms, 5–10 zeros sent per release over 233–594 ms window. Perceived stop ~0.67 s ("keeps moving").

### Iteration 3 (2026-05-16 ~15:00) — tune brake

Changes:
- `brake_acceleration_factor: 5.0 → 10.0` → brake ramp 0.67 s → 0.33 s.

**On-robot result:** Still felt as delay. Plus operator noted **start-up delay** ("when I push stick, robot moves with delay") — this is the `drive_acceleration_factor=0.3` causing 3.3 s ramp from 0 → full.

### Iteration 4 (2026-05-16 ~15:15) — tune both ramps

Changes:
- `drive_acceleration_factor: 0.3 → 2.0` → start-up ramp 3.3 s → 0.5 s
- `brake_acceleration_factor: 10.0 → 20.0` → brake ramp 0.33 s → 0.17 s

**On-robot result:** Operator still reports stop delay. **Wheel-stop-vs-coast question raised but not yet answered** — see §7.

## 5. Current state of the modified files (uncommitted, as of sign-off)

```
M tactical_mower/ros2_ws/src/control/control/joy_controller_node.py
M tactical_mower/ros2_ws/src/control/config/params.yaml      (only the joy port/baud change from earlier)
M tactical_mower/ros2_ws/src/control/launch/control.launch.py (pre-existing change, not from this RCA)
M tactical_mower/ros2_ws/src/drive/config/params.yaml
M tactical_mower/ros2_ws/src/drive/src/drive/kinematics/differential_drive.py (12% deadband — pre-existing, kept)
M tactical_mower/ros2_ws/src/drive/src/drive/nodes/roboclaw_wrapper_node.py
```

Key changes to be aware of:

**`drive/config/params.yaml`** (roboclaw_wrapper section):
```yaml
drive_acceleration_factor: 2.0     # was 0.3 — start ramp 0.5 s (was 3.3 s)
velocity_timeout: 0.3              # was 2.0 (dead) → 0.5 → 0.3 — watchdog safety net
brake_acceleration_factor: 20.0    # NEW PARAM — brake ramp 0.17 s
```

**`roboclaw_wrapper_node.py`** key additions:
- `_cmd_watchdog_check` at 20 Hz, calls `_slam_brake` on timeout
- `send_velocity`: if `qpps == 0` → `DutyAccelM1/M2(brake_accel, 0)` (not plain `Duty`)
- `_slam_brake` = `DutyAccelM1/M2(brake_accel, 0)` on both motors
- `destroy_node` calls `_slam_brake` before teardown

**`joy_controller_node.py`** key additions:
- `_ZERO_WINDOW_S = 0.5` wall-clock zero-publish window after L1 release
- On L1 held→released, publishes `Joy()` (all defaults = 0) on every iteration for 0.5 s, then silent (so `/joy_web` takeover after `gamepad_timeout=0.5 s` still works)

## 6. Evidence — bag analysis (test_record2/, 34.65 s, 269 joy + 269 cmd msgs)

Captured topics: `/joy_drive_raw /joy_web /cmd_drive /robot/state /drive/battery_voltage_raw /drive/battery_percentage /control/light /control/enable_charging /tactical_mode_toggle /control/autonomous_operation /fixposition/odometry_llh /fixposition/speed /tactical/logging/{info,warn,error}`.

Bag location: `/tmp/test_record2/` inside `ros2_jetson` container — **tmpfs, will not survive restart**. Pull it before powering off if you need it:
```
docker cp ros2_jetson:/tmp/test_record2 ~/gits/RCA_2026-05-16_bag/
```

Key numbers:

| Metric | Value |
|---|---|
| /joy_drive_raw rate | 7.8 Hz (lower than 10 Hz target; bounded drain + readline(0.1) blocks) |
| joy → cmd latency p50 | **2.2 ms** |
| joy → cmd latency p95 | 70 ms |
| joy → cmd latency max | 199 ms |
| First zero after L1 release | 1.0–11 ms |
| Zero count per release | 3–11 |
| Last zero (window close) | 233–594 ms (matches 500 ms design) |
| /joy_web messages | 0 (web is not interfering) |

**Conclusion: the software-side stop is fast.** From operator's finger leaving L1 to `Duty=0` command arriving at Roboclaw takes ~2 ms typical.

The remaining perceived delay must come from one of:
1. **Roboclaw firmware brake ramp** (current 0.17 s at brake_acceleration_factor=20.0)
2. **Roboclaw firmware brake-vs-coast mode** at duty=0 (need to verify — see §7)
3. **Chassis inertia after motors stop driving** (if coast mode, this dominates)

## 7. Open question — wheel-stop-vs-coast (unanswered)

When the motors are commanded `Duty=0`, the Roboclaw firmware **may** be configured for either:
- **Brake mode** — both low-side FETs on, motor leads shorted → active electromagnetic brake → wheels stop sharply
- **Coast mode** — H-bridge floating → motor freewheels → wheels keep spinning, robot decelerates only by ground friction

Default for Roboclaw 2x15 in duty mode is usually brake, but the operator may have changed it via Motion Studio. **Operator needs to observe wheel behavior at the moment of release:**
- **(A) Wheels stop spinning sharply** → it's the ramp; tune `brake_acceleration_factor` higher (try 30, 50). Software path is the lever.
- **(B) Wheels keep spinning freely while chassis coasts** → it's a Roboclaw firmware config issue. ROS-side ramping does nothing. Need to either: (i) configure Roboclaw via Motion Studio (USB cable, Basicmicro tool), (ii) send a momentary reverse-duty pulse before Duty=0 to force electrical brake, or (iii) accept the inertia.

This was the last question posed to the operator before the session paused.

## 8. Pre-existing issues NOT related to this bug (visible in logs but unrelated)

- `fixposition_driver_ros2` crashes in a loop — RTK at `192.168.10.107` unreachable. Blocks autonomous waypoint follower only; manual PS5 drive is unaffected. See `RCA_2026-05-15_robot_not_driving.md` §7.
- `person_detection_bridge` polls and gets 404. Harmless for drive.
- ESP debug stream sends `ly` / `lx` values up to ±508, well outside the documented ±100 range from `Joy.msg`. Kinematics clamps to ±100, so **only the bottom ~20 % of stick travel maps to the full speed range** — explains coarse-feeling fine joystick control. ESP firmware should be normalising. **Not blocking, not fixed in this RCA.**

## 9. How to test on robot (any future session)

Pre-flight (CLAUDE.md §3):
- Robot parked, e-stop in reach, manual mode active
- ESP power LED solid, PS5 paired (solid lightbar)

Build + restart in container (host shell):
```bash
docker exec ros2_jetson bash -lc "cd /app/ros2_ws && colcon build --packages-select drive control --symlink-install --build-base build_ros2 --install-base install_ros2"
docker restart ros2_jetson
# wait ~20 s for nodes to come up
docker logs --tail 200 ros2_jetson 2>&1 | grep -iE "joy_controller|roboclaw_drive|UART opened|wrapper init"
```

Record a diagnostic bag inside the container:
```bash
docker exec ros2_jetson bash -lc "source /opt/ros/jazzy/setup.bash && source /app/ros2_ws/install_ros2/setup.bash && cd /tmp && ros2 bag record -o test_record /joy_drive_raw /joy_web /cmd_drive /robot/state /drive/battery_voltage_raw /drive/battery_percentage /control/light /control/enable_charging /tactical_mode_toggle /control/autonomous_operation /fixposition/odometry_llh /fixposition/speed /tactical/logging/info /tactical/logging/warn /tactical/logging/error"
# Ctrl-C when done; bag is at /tmp/test_record/
```

Standard test sequence (with operator at PS5, e-stop in reach):
1. Idle baseline — hold L1, no stick, 3 s
2. Low speed creep + release — hold L1, 20 % forward 2 s, release stick
3. Medium speed + release — hold L1, 50 % forward 2 s, release stick
4. Reverse + release — hold L1, 50 % backward 2 s, release stick
5. L1 release while moving — hold L1 + 50 % forward, release L1 (not stick first)
6. Turn + release — hold L1, right stick to one side, release
7. Rapid stick wiggle + release

Analyze bag (rosbags pip package needed in container — `pip install --break-system-packages rosbags`):
```bash
docker exec ros2_jetson python3 - <<'PY'
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
ts = get_typestore(Stores.ROS2_JAZZY)
joy_def='int32 left_stick_forward\nint32 left_stick_right\nint32 right_stick_forward\nint32 right_stick_right\nbool select\nbool start\nbool x\nbool square\nbool triangle\nbool circle\nbool l1\nint32 l2\nbool r1\nint32 r2\nbool up\nbool down\nbool left\nbool right'
cmd_def='float64 left_vel\nfloat64 right_vel'
ts.register(get_types_from_msg(joy_def,'interfaces/msg/Joy'))
ts.register(get_types_from_msg(cmd_def,'interfaces/msg/CommandDrive'))
# ... then read /joy_drive_raw and /cmd_drive, compute L1 transitions and zero-cmd latencies
PY
```

Full analysis snippet is in conversation history (see "Comprehensive analysis: latency stats, per-release zero behavior" — search for `releases analyzed`).

## 10. How to revert (if any of this breaks something worse)

All changes are in three files. Revert per-file:
```bash
git restore tactical_mower/ros2_ws/src/control/control/joy_controller_node.py
git restore tactical_mower/ros2_ws/src/drive/config/params.yaml
git restore tactical_mower/ros2_ws/src/drive/src/drive/nodes/roboclaw_wrapper_node.py
docker exec ros2_jetson bash -lc "cd /app/ros2_ws && colcon build --packages-select drive control --symlink-install --build-base build_ros2 --install-base install_ros2"
docker restart ros2_jetson
```

This restores: original one-shot L1-release zero, `DutyAccel(drive_accel, qpps)` for all commands including zero, no watchdog, `velocity_timeout: 2.0` (dead config), `drive_acceleration_factor: 0.3`. **Note this restores the original "robot keeps moving for 1–3 s after release" symptom.**

If you want a partial revert (e.g. keep the watchdog but restore the original ramps for a test), edit `drive/config/params.yaml` and tune `drive_acceleration_factor` / `brake_acceleration_factor` directly — no rebuild needed, just `docker restart ros2_jetson`.

## 10b. Unrelated bug found and fixed in same session — DDS ROS_DOMAIN_ID mismatch

While the operator was testing joystick stops, separate symptom surfaced: "lidar is not publishing, rgb1 and cameras are not streaming."

**Root cause:** During Nano → Orin NX migration, `ROS_DOMAIN_ID` was bumped from 0 → 99 across `ros2_jetson`, `web_app`, and `theramal_camera_jazzy` via untracked `docker-compose.override.{yml,yaml}` files. The `livox_mid_360_obstacle_detection` and `RealsenseD_camera_Obstacle_avoidance` stacks were missed and stayed on domain 0.

**Symptom:** Topics like `/obstacles/lidar`, `/camera/camera/color/image_raw`, `/oak/rgb/image_raw`, `/livox/points` appeared in `ros2 topic list` (because local subscribers were bound) but `ros2 topic info` showed `Publisher count: 0` — publishers were on the other side of the DDS domain wall.

**Fix:** Created two new override files mirroring the existing pattern:

```yaml
# livox_mid_360_obstacle_detection/docker-compose.override.yaml
services:
  ros2_livox:
    environment:
      - ROS_DOMAIN_ID=99
```

```yaml
# RealsenseD_camera_Obstacle_avoidance/docker-compose.override.yml
services:
  realsense:
    environment:
      - ROS_DOMAIN_ID=99
```

Applied with `docker compose up -d` in each directory (env-only change, no rebuild needed).

**Verified post-fix** (from `ros2_jetson`):

| Topic | Publishers | Subscribers |
|---|---|---|
| /livox/points | 1 | 1 |
| /oak/rgb/image_raw | 1 | 2 |
| /oak/stereo/image_raw | 1 | 1 |
| /camera/camera/color/image_raw | 1 | 1 |
| /obstacles/lidar | 1 | 4 |
| /ip_camera/rgb_raw | 1 | 2 |
| /ip_camera/rgb2_raw | 1 | 1 |
| /ip_camera/thermal_raw | **0** | 1 |

**Still open after this fix:** `/ip_camera/thermal_raw` — not a DDS issue. The `theramal_camera_jazzy` container's launch file `eneo_ip_therm_camera dual_rtsp.launch.py` only spawns two RGB streams (rgb_camera_stream from `camera=1`, rgb2_camera_stream from `camera=2`, both off the same Axis Eneo at `192.168.10.193`). No thermal stream publisher is launched. Separate launch-config or eneo package issue, deferred.

**Service-name lookup table** (since overrides target service names, not container names):

| Compose file | Service | Container |
|---|---|---|
| `livox_mid_360_obstacle_detection/docker-compose.yaml` | `ros2_livox` | `livox_ros2_jazzy` |
| `RealsenseD_camera_Obstacle_avoidance/docker-compose.yml` | `realsense` | `intel_realsense_ros2` |
| `Tactical-Thermal-Stream/docker-compose.yml` | `eneo_INT-8SF0003M0A` | `theramal_camera_jazzy` |
| `tactical_mower/docker-compose.yaml` | `ros2`, `web_app` | `ros2_jetson`, `web_app` |

## 11. Memory entries written this session

Under `/home/quarero02/.claude/projects/-home-quarero02-gits/memory/`:
- `project_joystick_stop_delay_rca.md` — pointer to this doc + key params + open question
- `reference_diagnostic_bag_recipe.md` — how to record + analyze drive-cascade bags in container
- `feedback_brake_tuning.md` — params and trade-offs
- `project_dds_domain_id_fix.md` — the §10b DDS fix and the override files
- `reference_dds_domain_recipe.md` — inventory + diagnose ROS_DOMAIN_ID across containers

## 12. Nothing committed

Three files modified, none committed. Branch is still `agent/auto-dev`. Per CLAUDE.md §2 — operator approval required before commit.
