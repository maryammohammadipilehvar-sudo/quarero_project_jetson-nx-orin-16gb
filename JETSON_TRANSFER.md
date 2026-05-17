# Jetson Transfer Plan

End-to-end procedure for moving this robot's software stack from the **current Jetson Orin Nano** (`quarero-00`, `192.168.10.226`, JetPack R36.4.7 / L4T 5.15.148-tegra / CUDA 12.6) onto a **new Jetson** while preserving identical behaviour.

Audience: the robot operator. Assume the same LAN, same peripherals, same customer site. Mark every step as **SOURCE** (run on current Jetson) or **TARGET** (run on new Jetson).

> **Safety**
> - Don't kill running containers on SOURCE while the robot is moving or on a scheduled patrol (CLAUDE.md §3).
> - Don't test autonomous driving without operator physically present with e-stop (CLAUDE.md §1, §8).
> - `~/gits/routen/security_events/` contains PII MP4 clips — do not upload to any external tool.

---

## 0. Pre-flight inventory (do this before unplugging anything)

**SOURCE**, in any order:

```bash
# Confirm current IP, hostname, JetPack version
hostname
ip -4 addr show
cat /etc/nv_tegra_release | head -1

# Snapshot what's running so you can compare on TARGET
docker ps --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}' | tee /tmp/docker-ps-before.txt
ls -l /dev/ttyACM* /dev/ttyTHS* 2>/dev/null | tee /tmp/tty-before.txt

# Snapshot group membership
groups | tee /tmp/groups-before.txt
```

Expected (today's state):
- IP: `192.168.10.226` on `enP8p1s0`
- L4T R36.4.7 (JetPack 6.x)
- Containers: `web_app`, `livox_ros2_jazzy`, `theramal_camera_jazzy`, `intel_realsense_ros2`, `ros2_jetson`
- `/dev/ttyACM0` (RoboClaw), `/dev/ttyTHS1` (ESP32 PS5 bridge), `/dev/ttyTHS2` (spare)
- Groups: `docker, dialout, gpio, i2c, video, render, plugdev` (+ more)

---

## 1. Repair the corrupt git on SOURCE

**Status as of 2026-05-15:** `~/gits/.git` has **28 zero-byte loose objects** (all touched at 15:42), making `agent/auto-dev` un-pushable. `main` is fine and already in sync with origin.

Goal: get `agent/auto-dev` pushed to `origin` so TARGET can clone cleanly.

### 1a. Confirm the damage

**SOURCE**:
```bash
cd ~/gits
git fsck --full 2>&1 | tee /tmp/git-fsck.log | head -40
find .git/objects -type f -empty | tee /tmp/empty-objects.txt
wc -l /tmp/empty-objects.txt   # should be ~28
```

### 1b. Attempt least-destructive recovery first

If the corrupt objects already exist on `origin`, fetching with the empty files moved aside will repopulate them.

**SOURCE**:
```bash
cd ~/gits

# Move the empty objects out of the way (reversible — they're zero bytes)
mkdir -p /tmp/git-empty-bak
xargs -a /tmp/empty-objects.txt -I{} mv {} /tmp/git-empty-bak/

# Try fetching from origin
git fetch origin --prune --tags
git fsck --full 2>&1 | grep -c "is empty"   # how many still broken?

# If zero broken now, you're done. Push:
git push origin agent/auto-dev
git push origin main
```

If `git fsck` still reports broken objects, the missing data isn't on `origin` either. Move on to 1c.

### 1c. Fallback: salvage by re-clone + cherry-pick of working tree

If the agent/auto-dev commits cannot be recovered from origin:

**SOURCE**:
```bash
# 1. Move the corrupt repo aside (do NOT delete yet — the working tree is intact)
cd ~
mv gits gits.corrupt

# 2. Fresh clone from origin to /home/quarero/gits
git clone --branch agent/auto-dev https://github.com/maryammohammadipilehvar-sudo/quarero-projects.git gits

# 3. Overlay any uncommitted/unpushed local changes from the working tree
#    (skip .git/, build/, install/ — these are regenerated)
rsync -av --delete \
  --exclude='.git/' \
  --exclude='build/' \
  --exclude='install/' \
  --exclude='log/' \
  ~/gits.corrupt/ ~/gits/

# 4. Review what changed since origin/agent/auto-dev
cd ~/gits
git status
git diff

# 5. Commit anything genuinely new under a clear message, then push
#    DO NOT 'git add -A' — add explicitly (CLAUDE.md §2). Confirm each file.
git add <each file by name>
git commit -m "Recover unpushed changes after .git corruption on $(hostname) 2026-05-15"
git push origin agent/auto-dev

# 6. Once verified, you can delete the corrupt backup
#    rm -rf ~/gits.corrupt   # do this only after TARGET is fully validated
```

### 1d. Record what was lost (if anything)

Compare commit count before/after to know whether any commit messages or authorship were lost. The working-tree state is preserved either way.

---

## 2. Back up operator data on SOURCE

`~/gits/routen/` is **not in git** and contains routes, schedules, settings, and security event clips (PII). Always back up to a second medium first.

**SOURCE**:
```bash
# Back up to a USB drive mounted at /mnt/usb (or any external medium)
sudo mkdir -p /mnt/usb/routen-backup-$(date +%F)
sudo rsync -avh --progress ~/gits/routen/ /mnt/usb/routen-backup-$(date +%F)/

# Verify size + file count match
du -sh ~/gits/routen /mnt/usb/routen-backup-*
find ~/gits/routen -type f | wc -l
find /mnt/usb/routen-backup-* -type f | wc -l
```

Optionally also snapshot:
- `~/.bashrc` (only CUDA paths + nvm config — nothing site-specific)
- `~/.ssh/` (if you use SSH for git on this host)
- `/etc/systemd/system/person-detection.service` (only if AXIS F41 is in use on this site)

---

## 3. Flash the TARGET Jetson

Use **NVIDIA SDK Manager** on a host PC to flash the new Jetson Orin Nano with **JetPack 6.x (L4T R36.x)**.

> **Why this matters**: All Docker images were built against L4T R36 / CUDA 12.6. Any other JetPack series breaks the runtime / nvidia container toolkit / TensorRT compatibility.

After first boot:
1. Create user `quarero` (matches source — many docker-compose paths reference `/home/quarero/...`).
2. Set hostname (cosmetic, optional): `sudo hostnamectl set-hostname quarero-00`.
3. Connect Ethernet.

---

## 4. Assign the right IP to TARGET

The robot's services hardcode LAN IPs. TARGET must take **192.168.10.226** (or every camera/Jetson URL must be re-edited).

Preferred path: have your DHCP server reserve `192.168.10.226` for TARGET's MAC, then `nmcli`/reboot to pick it up.

Manual fallback (NetworkManager — current Jetson uses NM, not netplan):
```bash
# Show current connection
nmcli con show
nmcli dev status

# Replace <CON_NAME> with the wired connection's name
sudo nmcli con mod "<CON_NAME>" ipv4.addresses 192.168.10.226/24
sudo nmcli con mod "<CON_NAME>" ipv4.gateway 192.168.10.1
sudo nmcli con mod "<CON_NAME>" ipv4.dns "1.1.1.1 8.8.8.8"
sudo nmcli con mod "<CON_NAME>" ipv4.method manual
sudo nmcli con up "<CON_NAME>"
ip -4 addr show
```

Verify:
```bash
ping -c2 192.168.10.193   # AXIS F41
ping -c2 192.168.10.174   # AXIS bullet camera
ping -c2 192.168.10.3     # Livox MID-360
```

---

## 5. Run the bringup script on TARGET

The repo already ships `/home/quarero/gits/bringup-new-jetson.sh` which handles: L4T sanity check, apt deps, Docker, nvidia-container-toolkit, group membership, masking `nvgetty`, installing NetBird, cloning the repo, and `docker compose up -d --build` for all four projects.

**TARGET**:
```bash
# The script clones into ~/gits, so don't pre-create it.
# But we need the script itself. Two paths:

# OPTION A: get the script directly via curl from GitHub raw
curl -O https://raw.githubusercontent.com/maryammohammadipilehvar-sudo/quarero-projects/agent/auto-dev/bringup-new-jetson.sh
chmod +x bringup-new-jetson.sh

# OPTION B: rsync just the script from SOURCE first
# (run this from SOURCE side, replacing <TARGET_IP> with TARGET's temp IP)
# rsync -av ~/gits/bringup-new-jetson.sh quarero@<TARGET_IP>:~/

# Then on TARGET:
./bringup-new-jetson.sh
```

After it finishes, **log out and back in** so the `docker`/`dialout`/`gpio` group memberships take effect, then verify:
```bash
groups | grep -E 'docker|dialout|gpio|i2c|video'
docker info | grep -i 'Runtimes:.*nvidia'
```

---

## 6. Transfer operator data to TARGET

Once the repo is cloned on TARGET (the bringup script does this), drop the `routen/` payload on top.

**SOURCE → TARGET** (run from SOURCE):
```bash
# Use --delete only after you've verified the backup in section 2.
rsync -avh --progress \
  ~/gits/routen/ \
  quarero@192.168.10.226:/home/quarero/gits/routen/

# Verify on TARGET
ssh quarero@192.168.10.226 'find /home/quarero/gits/routen -type f | wc -l'
```

If TARGET is on a different LAN segment during setup, use a USB drive instead of network rsync.

---

## 7. Decide: rebuild Docker images vs. save/load

The bringup script already runs `docker compose up -d --build` for each project. That works but takes 30–60 min and needs reliable upstream connectivity (apt, pip).

If your network at the target site is slow or you want to minimise downtime: `docker save` on SOURCE, transfer the tarballs, `docker load` on TARGET, then run compose with the prebuilt tags.

**SOURCE**:
```bash
mkdir -p ~/docker-export
cd ~/docker-export
for img in \
  tactical_mower-ros2 \
  tactical_mower-web_app \
  livox_mid_360_obstacle_detection-ros2_livox \
  realsensed_camera_obstacle_avoidance-realsense; do
    docker save "$img:latest" | gzip > "${img}.tar.gz"
done

# Also save the thermal image (lookup the real name)
docker images | grep -i thermal
# docker save <thermal-image-name>:latest | gzip > thermal_image.tar.gz

du -sh ~/docker-export/*
```

Transfer the `.tar.gz` files to TARGET via USB or rsync, then:

**TARGET**:
```bash
cd ~/docker-export
for f in *.tar.gz; do docker load < "$f"; done
docker images
# Now docker compose up -d (no --build) will reuse the loaded images
```

---

## 8. Site-specific edits (already in the bringup script's final checklist)

Only needed if the **site** changed; for a same-site Jetson swap these stay the same. Edit on TARGET:

- `~/gits/routen/settings/settings.yaml`
  - `home_point.{latitude,longitude,yaw}`
  - `charge_point.{latitude,longitude,yaw}`
  - `speed_factor`, `battery_threshold`, `enable_obstacle_avoidance`
- `~/gits/routen/routes/<route>.yaml` — re-record waypoints for new site
- `~/gits/routen/settings/schedules.yaml` — point at the new routes
- `~/gits/tactical_mower/ros2_ws/src/eneo_event_publisher/eneo_event_publisher/eneo_parser.py` — only if Eneo camera moved off `192.168.10.203`

After any change inside `tactical_mower/`, rebuild the affected container:
```bash
cd ~/gits/tactical_mower
docker compose build ros2_jetson
docker compose up -d ros2_jetson
```

---

## 9. Hardcoded IPs to verify (no edits needed if LAN unchanged)

These are baked into code, not config. Grep before container rebuild:

```bash
grep -rn '192\.168\.10\.' ~/gits/ \
  --include='*.py' --include='*.cpp' --include='*.yaml' --include='*.yml' \
  --include='*.launch' --include='*.sh' \
  --exclude-dir=.git --exclude-dir=build --exclude-dir=install
```

Expected hits (same LAN, no edits needed):
| IP | Role | File(s) |
|---|---|---|
| `192.168.10.193` | AXIS F41 multi-channel | `Tactical-Thermal-Stream/.../thermal_camera.yml` |
| `192.168.10.174` | AXIS bullet (rgb1 person detection) | `Tactical-Thermal-Stream/.../live_detect.py` |
| `192.168.10.140` | Jetson 2 HTTP (person detection bridge) | `tactical_mower/.../person_detection_bridge.py` |
| `192.168.10.3`   | Livox MID-360 healthcheck | `Tactical-Thermal-Stream/docker-compose.yml` |
| `192.168.10.203` | Eneo camera (parser) | `tactical_mower/.../eneo_parser.py` |

---

## 10. Plug peripherals and verify devices on TARGET

Plug them in the **same order** as SOURCE so the kernel assigns the same `/dev/tty*` nodes:

1. RoboClaw via USB → expect `/dev/ttyACM0`
2. ESP32 over UART1 → expect `/dev/ttyTHS1`
3. (Optional) spare UART2 → `/dev/ttyTHS2`
4. Livox MID-360 over Ethernet → ping `192.168.10.3`
5. RealSense / OAK-D Lite over USB 3 → `lsusb | grep -iE 'intel|movidius|luxonis'`
6. Fixposition RTK (USB/serial) → check `dmesg | tail -30` after plugging

Verify:
```bash
ls -l /dev/ttyACM0 /dev/ttyTHS1 /dev/ttyTHS2
lsusb
ip -4 a
```

If `/dev/ttyTHS1` is missing, `nvgetty` is still holding it — re-run the `disable_nvgetty` step from the bringup script or `sudo systemctl mask nvgetty.service` and reboot.

---

## 11. NetBird VPN enrollment (if used)

NetBird machine identity does **not** transfer. Request a fresh setup key from the operator's NetBird tenant.

**TARGET**:
```bash
sudo netbird up --setup-key <KEY-FROM-OPERATOR>
netbird status
```

Verify the new Jetson's NetBird IP is in the operator's tenant.

---

## 12. Bring the stack up and validate

The bringup script already started the four compose projects. Verify state matches the SOURCE snapshot from §0.

**TARGET**:
```bash
docker ps --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'
# Compare against /tmp/docker-ps-before.txt from SOURCE

# Web UI
curl -sI http://localhost:8020 | head -3

# ROS topics visible from outside the container
docker exec -it ros2_jetson bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 topic list | head -40'
```

Then progress through validation **in this order**, with the operator physically present and e-stop in hand:

1. **PS5 deadman test** — connect controller, hold L1, push stick. Robot should crawl. Release L1 → motors stop.
2. **Web UI joystick** (`/joy_web`) — same test from the browser.
3. **E-stop service** — trigger from web UI or `ros2 service call`. Motors stop. **Note (AUDIT HIGH-6):** e-stop only fires while `/joy_drive_raw` is actively streaming.
4. **Camera feed** — OAK-D Lite RGB visible in web UI. Obstacle debug image populates.
5. **Person detection** — walk in front of camera; `/person_detected` should publish `True`.
6. **Short autonomous run** — a single waypoint route on clean ground. Operator on e-stop the entire time.

---

## 13. Rollback plan

If TARGET fails validation, SOURCE is still intact and untouched (other than the git repair in §1). To roll back:

```bash
# On SOURCE — bring its containers back up if you stopped them
cd ~/gits/livox_mid_360_obstacle_detection && docker compose up -d
cd ~/gits/RealsenseD_camera_Obstacle_avoidance && docker compose up -d
cd ~/gits/Tactical-Thermal-Stream && docker compose up -d
cd ~/gits/tactical_mower && docker compose up -d

# Unplug TARGET, plug peripherals back into SOURCE
```

Don't decommission SOURCE until TARGET has run **at least one successful patrol** under operator supervision.

---

## 14. Do-not-transfer list

- `~/gits/tactical_mower/tactical_mower/` (diverged inner copy — CLAUDE.md §7)
- Inner copies of `livox_obstacle/`, `realsense_obstacle/`, `thermal_stream/` inside `tactical_mower/`
- `~/gits_backup_20260424_*` and `~/gits_old` (≈15 GB combined; insurance only, not needed)
- `install/`, `build/`, `log/` directories under any `ros2_ws` (rebuilt automatically)
- `.git/objects/` zero-byte files (let the fresh clone supply them)
- Machine-id, SSH host keys, NetBird machine key (TARGET generates / enrols fresh)

---

## 15. Appendix A — what each container provides

| Container | Image | Role | Key host devices |
|---|---|---|---|
| `ros2_jetson` | `tactical_mower-ros2` | ROS 2 control stack (state machine, drive, RoboClaw, RTK, navigation) | `/dev/ttyACM0`, `/dev/ttyTHS1`, `/dev/ttyTHS2` |
| `web_app` | `tactical_mower-web_app` | FastAPI UI on `:8020` | `/dev/shm`, `routen/` bind |
| `intel_realsense_ros2` | `realsensed_camera_obstacle_avoidance-realsense` | OAK-D Lite (legacy name) + depth obstacle detection | `/dev` |
| `livox_ros2_jazzy` | `livox_mid_360_obstacle_detection-ros2_livox` | Livox MID-360 LiDAR driver + obstacle sectors | — |
| `theramal_camera_jazzy` | (thermal stream image) | Eneo/AXIS F41 RTSP → ROS topics | — |

All four use `network_mode: host`, `ipc: host`, `pid: host`, `runtime: nvidia`, `restart: unless-stopped`, and ship `ROS_DOMAIN_ID=0` / `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`.

---

## 16. Appendix B — estimated wall-clock

| Phase | Time | Can run in parallel? |
|---|---|---|
| Git repair on SOURCE (§1) | 5–30 min | No |
| Operator-data backup (§2) | 5–15 min | Yes (with §1) |
| Flash JetPack on TARGET (§3) | 30–45 min | Yes (with §1, §2) |
| Bringup script on TARGET (§5) | 20–60 min | — |
| Routen transfer (§6) | 5–15 min | Yes (with §5 second half) |
| Docker save+load (§7, if used) | 30–45 min | Yes (with §5) |
| Hardware verify (§10) | 10 min | — |
| Validation runs (§12) | 30–60 min | — |
| **Total** | **2–4 h** | Parallel optimised |

---

## 17. Step 2 result (recorded 2026-05-15)

- `origin/main` ↔ local `main`: already in sync. `git push origin main` is a no-op.
- `agent/auto-dev`: **push blocked** by 28 corrupt loose objects under `.git/objects/`, all created today at 15:42 (single failed write). `git log` and `git status` on this branch crash; `main` is unaffected. See §1 for recovery.
