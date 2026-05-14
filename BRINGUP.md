# Bring up a second Jetson at a new site

Companion checklist for `bringup-new-jetson.sh`. The script automates the host-side install + container start; this file covers the steps that can't or shouldn't be scripted.

Target: **Jetson Orin Nano**, flashed to **JetPack 6.x (L4T R36.x)**, on a **different site / LAN** than the source.

---

## Before you touch the new Jetson

1. **Push the current branch from the source Jetson** so the new one can clone the latest commit.
   ```
   cd /home/quarero/gits
   git push origin agent/auto-dev
   ```
   The commit `e151d41` is still local-only on the source as of writing — the new Jetson's clone will be out of date until you push.

2. **Confirm the target supports JetPack 6.** Orin Nano (incl. "Super" / 8 GB) does. Xavier NX/AGX do NOT.

3. **Get a NetBird setup key** from whoever owns the operator's NetBird tenant. One-time use, expires.

4. **Inventory the new-site peripherals and IPs** so you can edit config without guessing:
   - eneo camera IP (was `192.168.10.203` on source — almost certainly different here)
   - RTK base station setup (NTRIP caster URL, mountpoint, user/pass — check existing source config in `~/gits/tactical_mower/ros2_ws/src/<gnss>/` if RTK is used)
   - Home / charge waypoints (lat/lon/yaw at the new site)
   - Routes for the new site

---

## Flash the target

1. From a host laptop with NVIDIA SDK Manager: flash JetPack 6.x onto the Orin Nano's NVMe SSD. (Seeed's preinstalled JP5 image gets overwritten.)
2. First boot: complete the Ubuntu OOBE.
3. Create a regular user (suggested: `quarero`) and log in. **Do not run subsequent steps as root.**

---

## Run the bring-up script

```
scp /home/quarero/bringup-new-jetson.sh <new-jetson>:~/
ssh <new-jetson>
chmod +x ~/bringup-new-jetson.sh
~/bringup-new-jetson.sh
```

The script will:
- Verify L4T is R36.x.
- `apt install` base tools, docker, `nvidia-container-toolkit`.
- Add your user to `docker`, `dialout`, `gpio`, `i2c`, `video`, `render`, `plugdev`.
- Disable + mask `nvgetty.service` (frees `/dev/ttyTHS*` for the ESP32 link).
- Install NetBird.
- Clone `~/gits` from GitHub (HTTPS; you'll be prompted for a PAT).
- `docker compose up -d --build` for each of the four projects in order: livox, RealSense, thermal, then tactical_mower.

**Log out and log back in after the script finishes** so group membership takes effect.

---

## Manual steps after the script

### 1. Enroll NetBird
```
sudo netbird up --setup-key <KEY>
```
Verify with `netbird status`. The robot should now be reachable from the operator's network.

### 2. Edit site-specific config

The defaults shipped in git are for the source site. The new site needs different values.

| File | What to change |
|---|---|
| `~/gits/routen/settings/settings.yaml` | `home_point` lat/lon/yaw, `charge_point` lat/lon/yaw, `speed_factor`, `battery_threshold`, `enable_obstacle_avoidance` |
| `~/gits/routen/routes/*.yaml` | Delete source-site routes; write new ones for the new site (or copy templates and update waypoints) |
| `~/gits/routen/settings/schedules.yaml` | Rebuild — schedule entries reference route names that may no longer exist |
| `~/gits/tactical_mower/ros2_ws/src/eneo_event_publisher/eneo_event_publisher/eneo_parser.py` | Update `192.168.10.203` to the new site's eneo camera IP. **Better:** open a follow-up to make this configurable instead of code-coded. |

After editing `eneo_parser.py`, rebuild the affected container:
```
cd ~/gits/tactical_mower
docker compose build ros2_jetson
docker compose up -d ros2_jetson
```

### 3. Hardcoded credentials warning

Per `AUDIT.md` HIGH-4, this repo has hardcoded RTSP camera passwords. If the new site uses different cameras with different credentials, you will need to edit the camera launch files. Don't propagate the same passwords blindly — that's a regression of the existing audit finding.

### 4. Plug peripherals

| Device | Expected path |
|---|---|
| Roboclaw motor controller | `/dev/ttyACM0` (USB) |
| ESP32 (PS5 controller bridge) | `/dev/ttyTHS1` (UART1) |
| Spare UART | `/dev/ttyTHS2` |
| Livox Mid-360 LiDAR | Ethernet (configured in the livox compose project) |
| RealSense D-series | USB 3 |
| Thermal camera | per `Tactical-Thermal-Stream` compose |

Verify:
```
ls -l /dev/ttyACM* /dev/ttyTHS*
docker ps   # expect web_app, livox_ros2_jazzy, theramal_camera_jazzy, intel_realsense_ros2
```

### 5. Replicate the optional `person-detection` workload (if needed)

The source Jetson runs a separate `person-detection.service` under user `quarero-00` (YOLOv8 on an AXIS F41 camera). It is **independent of the tactical_mower stack** — only set it up if the new site also has an AXIS F41 and the customer wants the same person-detection layer.

Steps if needed:
1. Create `quarero-00` user.
2. `scp -r quarero-00@source:~/Desktop/camera-ai /home/quarero-00/Desktop/camera-ai` (includes YOLO weights — large).
3. Copy `/etc/systemd/system/person-detection.service` to the new Jetson and `systemctl enable --now person-detection`.
4. Confirm the AXIS F41 IP in `live_detect.py` matches the new site.

### 6. Env vars (optional)

The source does NOT set `SECURITY_EMAIL_USER` / `SECURITY_EMAIL_PASS` anywhere — email notifications fall back to whatever is in code/config. If the new deployment needs email alerts with a different SMTP account, set them as a docker-compose `env_file` (don't commit the file).

---

## Validation gates

Per `CLAUDE.md` sections 1 and 8 — no autonomous test without the operator physically present and able to hit e-stop.

1. **Containers up:** `docker ps` shows all four expected names, healthy.
2. **Web UI:** reachable at `http://<new-jetson>:8020` over NetBird.
3. **Manual drive only, no autonomous:** PS5 controller via ESP32 moves the wheels. Watch for the known issues already documented in `AUDIT.md` HIGH-1 (no `/cmd_drive` watchdog) and HIGH-6 (LPF rollback).
4. **E-stop reachable:** verify the e-stop service halts motors. Note `AUDIT.md` HIGH-6 caveat: e-stop only works when `/joy_drive_raw` is actively streaming.
5. **One short routed run** with operator on e-stop, only after manual-drive and e-stop are both verified.

---

## What deliberately did NOT come over

- `~/gits/routen/security_events/*.mp4` — captured customer footage, PII per `CLAUDE.md`. Stays on the source Jetson.
- NetBird machine key — each peer enrolls separately.
- SSH host keys, machine-id, hostname — regenerated per host.
- `~/uart_sniffer.py`, `~/uart_sweep.py` — debug tools, not part of the runtime stack.
