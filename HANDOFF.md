# Handoff — New Jetson Bring-up (.29 → .226)

**Created on the OLD Jetson (`quarero@192.168.10.226`) 2026-05-15 ~19:30, just before unplugging it.** The next Claude session runs on the NEW Jetson as user `quarero02`. Read this + `JETSON_TRANSFER.md` first.

## Current state on this Jetson (.29 → will become .226)

Per `JETSON_TRANSFER.md` §0 / §12 checks:

- **L4T R36.5.0 / JetPack 6.x** ✓
- **All groups correct** (`docker, dialout, gpio, i2c, video, render, plugdev`) ✓
- **nvgetty masked** ✓ → `/dev/ttyTHS1`, `/dev/ttyTHS2` available
- **Docker + nvidia runtime** ✓
- **All 4 images built** (`tactical_mower-ros2`, `-web_app`, `realsensed_…-realsense`, `livox_…-ros2_livox`) ✓
- **`~/gits/` cloned, `routen/` (routes + settings + security_events) present** ✓
- **Peripherals all enumerated**: `/dev/ttyACM0` (RoboClaw), Movidius MyriadX (OAK-D Lite), CP210x UART, Livox MID-360 (ping OK), AXIS F41 (ping OK) ✓

## Containers (5/5 created, but not all healthy)

| Container | Status when handoff written |
|---|---|
| `web_app` | Up ✓ |
| `theramal_camera_jazzy` | Up ✓ (both AXIS F41 RTSP streams connected) |
| `ros2_jetson` | Up — **first colcon build still in progress at 11/15** (building `fixposition_driver_msgs`). Entrypoint is `colcon build && ros2 launch system_bringup controller.launch.py`, so no control/drive/state-machine nodes exist yet. Just wait it out (~5–10 more min). |
| `intel_realsense_ros2` | Up |
| `livox_ros2_jazzy` | **Restart loop** — see "Open issue" below |

## Open issue: Livox HOST_IP

`HOST_IP="192.168.10.226"` is hardcoded in:
- `~/gits/livox_mid_360_obstacle_detection/docker/wait_for_lidar.sh:5` (baked into image — sha256 same as `/usr/local/bin/wait_for_lidar.sh` inside container)
- `~/gits/livox_mid_360_obstacle_detection/ros2_ws/src/livox_ros_driver2/config/MID360_config.json` lines 14/16/18/20 (`cmd_data_ip`, `push_msg_ip`, `point_data_ip`, `imu_data_ip`) — **bind-mounted live** via `ros2_ws -> /livox/ros2_ws`

Plan (operator chose option A from JETSON_TRANSFER.md §4): give the new Jetson the IP `.226` so all hardcoded refs Just Work — no code edits.

## What to do next on .29 (becomes .226)

**Pre-staged on .29 by the old-Jetson session:**
- Wired NetworkManager connection name confirmed: `Wired connection 1` (DEVICE=enP8p1s0, current ipv4.method=auto/DHCP)
- `~/gits/swap_ip.sh` — applies static .226. Refuses to run if .226 is still pingable. Will drop your SSH session when it brings the link up.
- `~/gits/restart_containers_after_ip_swap.sh` — restart all containers so they bind to .226. Run AFTER you reconnect via the new IP.

**Sequence:**
```bash
# On .29 (after operator unplugs old Jetson):
bash ~/gits/swap_ip.sh              # will prompt for sudo, then drop SSH

# Reconnect:
ssh quarero02@192.168.10.226

# Then:
bash ~/gits/restart_containers_after_ip_swap.sh
```

Verify per JETSON_TRANSFER.md §12 (with operator + e-stop):
1. PS5 deadman test
2. Web UI joystick
3. E-stop service
4. OAK-D Lite RGB in UI + obstacle debug
5. Person detection
6. Short autonomous run (single waypoint)

## Other items still pending from JETSON_TRANSFER.md

- **§11 NetBird** — not installed on .29. Needs setup key from operator's tenant.
- **§13 Rollback** — don't decommission the old Jetson until at least one successful patrol on .226.

## Useful state from this session

- SSH key from old Jetson (`quarero@.226`) is in `quarero02@.29:~/.ssh/authorized_keys` — won't matter once .226 is offline.
- No commits made anywhere this session.
- TLS / RTK / cameras have not been hardware-validated yet — only confirmed reachable.

## Gotchas saved to operator memory (older sessions)

- ESP UART deadman: Jetson only sees Serial1 frames while L1 is held; silent UART ≠ broken wiring
- PS5 fast-blink = ESP BT stack wedged; unplug/replug ESP
- Roboclaw M2 reads ~0.5 A at idle (zero-offset, not real load)
- Front depth camera is OAK-D Lite (named "realsense" in repo); depth topic `/oak/stereo/image_raw` (16UC1 mm); `pipeline_type=RGBD`, sync off
- Demo 2026-05-16 — see operator memory `customer_demo_2026_05_16.md` for demo-killer features
