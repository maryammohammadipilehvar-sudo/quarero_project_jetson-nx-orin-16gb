# AUDIT.md — issue inventory (as of 2026-04-24, read-only review)

Severity key:
- **HIGH** — safety, autonomous-mode correctness, data loss, credential exposure, crashes.
- **MEDIUM** — functional bugs, missing error handling, operational foot-guns.
- **LOW** — style, dead code, docs, cosmetics.

No code has been changed. Each item cites the canonical file path. Line numbers are from the outer `tactical_mower/ros2_ws/` tree (the one docker-compose actually builds), unless noted.

---

## Project classification

| Directory | Class | Notes |
|---|---|---|
| `tactical_mower/` | ACTIVE | Main on-robot stack. Outer `ros2_ws/` is canonical. |
| `livox_mid_360_obstacle_detection/` | ACTIVE | Standalone Livox LiDAR container. |
| `RealsenseD_camera_Obstacle_avoidance/` | ACTIVE | Standalone RealSense depth container. |
| `Tactical-Thermal-Stream/` | ACTIVE | Standalone Eneo RTSP→ROS bridge; also ships `live_detect.py` (YOLOv8 MJPEG). |
| `routen/` | ACTIVE | Runtime data (routes, schedules, settings, event clips). Not code. |
| `tactical_mower/tactical_mower/` | BACKUP | Partially diverged older copy of the outer workspace. See MED-1. |
| `tactical_mower/livox_obstacle/`, `…/realsense_obstacle/`, `…/thermal_stream/`, `…/routen/` | BACKUP | Mirrored copies of the top-level siblings; diverge from them. See MED-1. |
| `AI_camera` (1-byte file) | EXPERIMENT | Empty placeholder committed on 2026-04-07. Not used by anything. |

Source counts (canonical trees only, excluding `fixposition/` submodule, the nested BACKUP copies, and build artifacts):

| Project | Python files | Python LoC | C/C++ files | Other |
|---|---:|---:|---:|---|
| `tactical_mower/` (outer `ros2_ws/` + web + arduino) | ~185 | ~32,500 | 0 first-party | 34 JS/HTML/CSS, 2 `.ino` (ESP32) |
| `livox_mid_360_obstacle_detection/` (excl. `livox_ros_driver2` submodule) | 8 | ~900 | 0 | — |
| `RealsenseD_camera_Obstacle_avoidance/` | 7 | ~420 | 0 | — |
| `Tactical-Thermal-Stream/` | 8 | ~870 | 0 | — |
| **Total first-party ACTIVE LoC** | ~208 | **~34,700** | 0 | — |

---

## tactical_mower — HIGH

**HIGH-1. No `/cmd_drive` watchdog on the motor driver.**
`tactical_mower/ros2_ws/src/drive/src/drive/nodes/roboclaw_wrapper_node.py:148` subscribes to `/cmd_drive` and immediately relays each message to `rc.DutyAccelM1/M2`. There is no inactivity timeout. RoboClaw in duty mode holds the last command until a new one arrives. If any upstream publisher (`robot_controller`, `tactical_wp_follower`) crashes, hangs, or the topic stalls, **the robot continues at the last commanded velocity**. The ESP32-side L1 deadman covers manual driving, but the autonomous path has no equivalent floor. Add a timer that zeros both motors if no `/cmd_drive` has arrived in, say, 300 ms.

**HIGH-2. `video_ringbuffer` node is commented out of system_bringup.**
`tactical_mower/ros2_ws/src/system_bringup/launch/controller.launch.py:64-78, 98` — the `video_ringbuffer` `Node(...)` is commented out and not included in the returned LaunchDescription. `eneo_event_publisher` *is* launched and will publish `/security_alert` messages on person/fire events. The service `/capture_event_clips` that `robot_web_interface` calls on those alerts is served by `video_ringbuffer`. Result: **security events fire but no clips are recorded**. This is the stated security purpose of the robot — verify on-robot and re-enable (or document why it's launched elsewhere).

**HIGH-3. Web API has no authentication.**
`tactical_mower/ros2_ws/src/robot_web_interface/src/robot_web_interface/main.py` + `api/routes/*.py`. Port 8020 is exposed via `network_mode: host`, no login, no token, no CORS allowlist. The routes include control endpoints (drive, e-stop, lights, siren, charging, route start/stop, schedule edits). Anyone on the LAN (or any device that reaches the 4G router behind the robot) can drive the robot. If LAN-only is the real deployment model, document it explicitly and firewall port 8020; otherwise add auth before the next customer demo.

**HIGH-4. Hardcoded RTSP credentials committed to the repo.**
- `Tactical-Thermal-Stream/workspace/src/eneo_ip_therm_camera/launch/dual_rtsp.launch.py:20` — `rtsp://Quarero:quarero00@{ip}:{port}/axis-media/media.amp`.
- `Tactical-Thermal-Stream/workspace/src/eneo_ip_therm_camera/eneo_ip_therm_camera/live_detect.py:47` — `rtsp://root:axis@192.168.10.174:554/axis-media/…`.
- Duplicated in `tactical_mower/thermal_stream/workspace/src/eneo_ip_therm_camera/launch/dual_rtsp.launch.py:24`.
Git remote is a GitHub repo. Rotate the passwords, move to env vars (pattern already used for SMTP — `SECURITY_EMAIL_PASS`), and re-examine whether `root:axis` is a camera factory default (it is — AXIS default root password is often left as-is).

**HIGH-5. Alarm / siren services always return success but do nothing.**
`tactical_mower/ros2_ws/src/drive/src/drive/nodes/robot_controller_node.py:292-320`. `_alarm_control_service` and `_siren_control_service` set `response.success = True` but the comment admits "actual hardware control not yet implemented." Operator pressing "Alarm" in the UI will see a green success and believe the siren is active. For a security product this is a worst-case silent-fail. Either wire it up to the relay topic (same pattern `_light_control_service` uses at :265) or make the service return `success=False` with a clear message until it's wired.

**HIGH-6. Manual-steering LPF attempt regressed coast-on-release (rolled back 2026-04-24).**
`tactical_mower/ros2_ws/src/drive/src/drive/controllers/joystick_controller.py:56-64` + `…/drive/kinematics/differential_drive.py:45`. Tried fixing over-reactive PS5 steering with a first-order low-pass (`out = 0.25·in + 0.75·last`) on both axes plus widened dead-band (2→5%). On-robot test showed a 1–2 m coast after joystick release. Root cause: the ESP32 publishes `/joy_drive_raw` only on stick change, not continuously, so the history-dependent filter has no samples to decay against once the stick returns to center. The last `/cmd_drive` published is ~75% of the pre-release command, and roboclaw_wrapper holds it. Both edits reverted same day. Next attempt must be stateless input shaping (expo/cubic on the stick axis) + wider dead-band — never a history-dependent filter while the input stream is event-driven. Two pre-existing defects confirmed in the process:
- **HIGH-1 is actively reachable.** The no-`/cmd_drive`-watchdog issue bit us immediately once the "last message on stick release = 0" coincidence was broken by the filter. Priority raised — fix the watchdog in `roboclaw_wrapper_node.py` independent of any steering work.
- **Software e-stop depends on a live `/joy_drive_raw` stream to take effect.** `_process_joy_command` only substitutes `(0, 0)` when a Joy message arrives; during UART silence (the normal at-rest state) activating the e-stop service cannot publish zero on `/cmd_drive`. The e-stop is not an independent safety layer — it piggybacks on the same data stream the operator is using. Either have `robot_controller` publish on a timer with the latest-known (or zero-if-estop) state, or implement the HIGH-1 watchdog on the roboclaw side — ideally both.

---

## tactical_mower — MEDIUM

**MED-1. Three separate kinds of duplicated trees inside `tactical_mower/`.**
- A nested `tactical_mower/tactical_mower/` that is a partial older copy of the outer workspace. `diff -rq` shows divergence in `control/config/params.yaml`, `control/control/navigation/waypoint_follower.py`, `tactical_wp_follower_node.py`, `control/launch/control.launch.py`, `robot_web_interface/main.py`, several `ros_interface/*.py`, and `static/*`. The outer copy has `person_detection_bridge.py` and `fusion_node.py` the inner one lacks.
- `tactical_mower/livox_obstacle/`, `/realsense_obstacle/`, `/thermal_stream/`, `/routen/` mirror the top-level sibling projects. `realsense_obstacle` and `livox_obstacle` are identical to their siblings; `thermal_stream/` differs (`dual_rtsp.launch.py` uses a different RTSP URL shape; top-level has `live_detect.py` and nested doesn't); `routen/` differs (nested has `Tex.yaml`, top-level has `Mary.yaml`; settings/schedules differ).
- `tactical_mower/ros2_ws/src/control/launch/control.launch.py` launches `fusion_node`, `person_detection_bridge`, `sector_publisher`, `sector_viz`, `obstacle_viz` — nodes that don't exist in the nested copy.
Net effect: unclear which is canonical, and a naive developer syncing both trees would quietly break the deployed stack. Decide on one canonical tree and delete the rest in a single dedicated commit.

**MED-2. `send_velocity` has no try/except around serial calls.**
`tactical_mower/ros2_ws/src/drive/src/drive/nodes/roboclaw_wrapper_node.py:319-341`. The RoboClaw driver (`roboclaw_3.py`) talks packet-serial over USB; CRC mismatches and timeouts are common on a vibrating robot. An exception here crashes the subscription callback and effectively kills the node, which then ties into HIGH-1 (motors keep last duty). Battery-read already has a `try/except` at :186; mirror that here.

**MED-3. Docker-compose has a hand-edited device override.**
`tactical_mower/docker-compose.yaml:23` — `# TODO: Change first part back to ACM0` sits next to `/dev/ttyACM0:/dev/ttyACM0`. The TODO implies the host-side device is drifting (`ACM0` vs `ACM1` vs something else). Confirm with the operator whether the RoboClaw is reliably on `ACM0` at boot; if not, add a udev rule with a stable symlink so the mapping doesn't rot silently.

**MED-4. Settings-file path is hardcoded to `/routen/settings/settings.yaml`.**
`drive/src/drive/nodes/robot_controller_node.py:69`, `control/control/nodes/tactical_wp_follower_node.py:46`. Defaults point at `/routen/...` (the container bind-mount path), which works inside Docker but silently fails natively. The `tactical_wp_follower_node` (:68-73) does fall back through a candidate list (`/routen/routes`, `/data/routes`), but the settings file does not — a missing file becomes a hard-to-diagnose default-value startup. Use the same fallback pattern for settings.

**MED-5. Mixed-language user-facing log messages.**
`drive/src/drive/nodes/robot_controller_node.py:244` — `⚠️ Die manuelle Steuerung ist während des Ladevorgangs deaktiviert.`
`control/control/nodes/tactical_wp_follower_node.py:2466-2468, 2478` — German warning and info sent to the web app.
`RealsenseD_camera_Obstacle_avoidance/.../simple_obstacle_detector.py` — module docstring + logs in German.
Other files log in English. The web UI surfaces `/tactical/logging/*` directly to the operator. Decide on one language per surface (operator-facing = German looks like the intent here; developer logs = English) and be consistent, otherwise users see unpredictable mix.

**MED-6. `eneo_event_publisher` depends on `video_ringbuffer` for shared camera IDs.**
`tactical_mower/ros2_ws/src/eneo_event_publisher/eneo_event_publisher/eneo_event_node.py:20-24`. Tries to import `video_ringbuffer.camera_config`; falls back to a hardcoded `['eneo_rgb', 'eneo_thermal']`. With HIGH-2 (video_ringbuffer disabled) the fallback is the only path exercised — the two lists can drift silently. Move `EVENT_RECORDING_CAMERA_IDS` into the shared `interfaces` package or into a tiny config-only package both depend on.

**MED-7. `robot_controller_node._tactical_robot_state_callback` re-imports `json` inside the callback.**
`drive/src/drive/nodes/robot_controller_node.py:428` — `import json` inside the callback body while `import json` is already at :18. Harmless (Python caches), but signals nobody is reviewing this path — the callback parses JSON on every `/tactical/robot/state` message at 1 Hz, no schema validation, no try/except around the outer structure access. Parse failures will bubble into the executor.

**MED-8. GitHub App install failed for this repo.**
User invoked `/install-github-app` this session and got: `Failed to access repository maryammohammadipilehvar-sudo/quarero-projects`. The remote is reachable but the Claude GitHub App does not have access. Either grant it, move the remote to the operator's own GitHub account, or document the manual PR workflow. Unrelated to code but operationally blocking for PR-based review.

---

## tactical_mower — LOW

**LOW-1. `AI_camera` file in repo root is a 1-byte stub.**
Committed 2026-04-07. Added to the repo with no associated code or context. Either build the intended project under a real name or delete.

**LOW-2. Commented-out code blocks with no ticket reference.**
`system_bringup/launch/controller.launch.py:63-78` keeps the entire `video_ringbuffer_node = Node(...)` block commented out — see HIGH-2. Also `docker-compose.yaml:23` (see MED-3). Dead code without a "why" is rot.

**LOW-3. `autostart_drive.sh` at repo root has no header or docs.**
`tactical_mower/autostart_drive.sh` — 720 bytes, executable, no explanation in the README of when it runs vs the docker-compose path.

**LOW-4. Arduino firmware at 536 lines in one `.ino` file.**
`tactical_mower/arduino/PS5_ESP32/PS5_ESP32.ino`. Not a bug, just a size note — any safety-relevant change (deadman, UART framing) will be harder to review than it needs to be.

---

## livox_mid_360_obstacle_detection

**LOW-5. No units tests beyond stock `test_copyright/flake8/pep257`.** Stub tests only.

**LOW-6. README uses placeholder LIDAR_IP / host_ip guidance** without recording the actual values used in the deployed config. The MID-360 config file is shipped inside the submodule; the production values are not pinned in this repo.

---

## RealsenseD_camera_Obstacle_avoidance

**MED-9. `depth_callback` does no QoS mismatch guard and no try/except.**
`workspace/src/realsense_obstacle/realsense_obstacle/simple_obstacle_detector.py:87-95` — `cv_bridge.imgmsg_to_cv2` raises on encoding mismatch; if the RealSense driver ever switches encoding (e.g. `16UC1` vs `32FC1`) the node dies silently. Wrap the callback or assert encoding early with a clear fatal log.

**LOW-7. Module docstring and all log messages in German.** See MED-5 (cross-cutting).

**LOW-8. Compose uses `network_mode: bridge`** while the rest of the stack uses `network_mode: host`. The README flags this — either standardize or document per-component which mode is expected in production.

---

## Tactical-Thermal-Stream

**HIGH-4 applies here (hardcoded RTSP creds, listed above).**

**MED-10. `live_detect.py` exposes an unauthenticated MJPEG server on 0.0.0.0:8080.**
`Tactical-Thermal-Stream/workspace/src/eneo_ip_therm_camera/eneo_ip_therm_camera/live_detect.py` (per-module docstring). Same LAN-exposure class as HIGH-3 (web UI) — live video of the customer site on an open port. If this is meant to be embedded only inside the web app, firewall it; if it's a separate debug tool, document that it is debug-only.

**LOW-9. `pip install --break-system-packages`** instruction in the docstring (line 17) is a red flag to lift into the Dockerfile instead of operator instructions.

**LOW-10. `workspace/error.txt`** (3 lines, committed in the initial commit) — one-off error dump left in the repo. Remove.

---

## Cross-cutting / structural

**MED-11. No CI, no linting, no pre-commit.** The stock ROS test stubs (`test_flake8.py`, `test_pep257.py`) exist per package but there is no umbrella runner and they are not enforced anywhere. For a multi-author, safety-relevant codebase this is a meaningful gap — even a minimal GitHub Actions job that runs `ruff` + `ament_flake8` on PRs would catch the obvious stuff.

**MED-12. `requirements-ros2_jetson.txt` / `requirements-web.txt` are tiny and unpinned in places.** At `tactical_mower/requirements-*.txt`. Worth pinning exact versions for reproducibility since the Jetson image is the production artifact.

**LOW-11. `.gitignore` was updated 2026-04-24 to add `*.pt/.onnx/.engine` (ML weights) and `*.db3/.mcap` (ROS bags).** Good. Confirm no weights or bags were committed *before* that change: none appear in `git ls-files` today, so this looks clean.

---

## Items deliberately NOT in this audit
- Read-through of the `fixposition/` submodule (third-party SDK).
- Read-through of `livox_ros_driver2` (third-party submodule).
- Frontend JS/HTML/CSS beyond noting it exists (34 files).
- Performance / latency of the control loop.
- Deep review of the YOLOv8 model / TensorRT engine export path in `live_detect.py`.
- Kinematics correctness (`differential_drive.py`).
- Roboclaw driver vendor code.

These are flagged so the operator knows what was skipped.
