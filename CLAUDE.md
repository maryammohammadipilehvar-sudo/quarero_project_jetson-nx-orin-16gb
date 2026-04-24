# CLAUDE.md — operating rules for this robot codebase

This repo runs on a **live robot**. Code here moves motors, drives outdoors, actuates relays, and captures video of people. A bad change can crash the robot into something, drain the battery, lose security footage, or lock the operator out of the UI. Read these rules before every non-trivial task.

## 1. Before you touch anything autonomous

The **autonomous control loop** is concentrated in:
- `tactical_mower/ros2_ws/src/control/control/nodes/tactical_wp_follower_node.py`
- `tactical_mower/ros2_ws/src/control/control/robot_state_machine/`
- `tactical_mower/ros2_ws/src/control/control/navigation/`
- `tactical_mower/ros2_ws/src/control/control/obstacle_avoidance/`
- `tactical_mower/ros2_ws/src/drive/src/drive/` (robot_controller, roboclaw_wrapper)

Rules:
- **Design first, code second.** For any change in those files that touches motion, state-machine transitions, obstacle handling, RTK gating, docking, undocking, or home-return: write down the intended behavior and the failure modes *before* editing. Confirm with the operator if the change isn't strictly a bug fix.
- **Fail soft on hardware.** A missing LiDAR, a dropped GPS fix, a stale joystick message — the correct response is "stop the motors and log", never "assume last value is good". The existing code has watchdogs for LiDAR, RTK, GPS jump; match that pattern, never undercut it.
- **Never remove an e-stop or safety gate** (emergency stop service, charging-state lockout, autonomous-operation flag, RTK gating, LiDAR timeout) without the operator's explicit go-ahead.
- **Watch for missing watchdogs.** `roboclaw_wrapper_node.py` has no `/cmd_drive` timeout; motors hold the last duty until a new command arrives. If you add a new publisher of `/cmd_drive`, make sure it zeros on shutdown and on any error path.

## 2. Git / deployment

- **Never commit to `main` directly.** Work on a branch, push, open a PR. The only current remote is `origin` (GitHub `maryammohammadipilehvar-sudo/quarero-projects`); the `/install-github-app` attempt failed because that repo isn't accessible to the GitHub App — flag this to the operator, don't paper over it.
- **Never `git push --force` or rewrite history on `main`.**
- **No commits without the operator's explicit approval.** Every session: propose the diff, wait for "yes, commit".
- **Never `git add -A` / `git add .`** in `~/gits/`. Add files by name. `~/gits/routen/security_events/` holds captured MP4 clips of real people on the customer's site — treat as PII.

## 3. Filesystem & runtime on the Jetson

- **Don't edit systemd service files** (`/etc/systemd/**`) without approval. The robot autostarts on boot; a bad unit bricks it until someone drives out to it.
- **Don't `sudo`** without explicit approval each time. Ask, don't assume past approval.
- **Don't kill, restart, or reset containers** (`ros2_jetson`, `web_app`) while the robot is moving or on a scheduled patrol. If you need a restart, ask the operator to confirm the robot is parked and manual mode is active.
- **Don't `rm`** anything under `~/gits/routen/` — that's live operator data (routes, settings, event clips). If cleanup is needed, confirm each path.
- **Don't modify `/dev/tty*` permissions or udev rules** without approval; the UART/USB mappings are already wired into docker-compose.

## 4. Secrets & credentials

- **Never put secrets in code or commit messages.** SMTP creds already read from `SECURITY_EMAIL_USER` / `SECURITY_EMAIL_PASS` env vars — keep that pattern.
- **Hardcoded RTSP passwords exist** in this repo today (see AUDIT.md, HIGH-4). Don't propagate them. When editing camera launch files, move creds to env/config instead of hardcoding more copies.
- **Never paste customer data, camera footage, route YAMLs, or IPs into external tools** (pastebin, cloud LLMs the operator hasn't approved, diagram renderers). Operator is on a private LAN and intends to stay that way.

## 5. Logging & observability

- **Prefer structured, greppable log prefixes.** The existing code uses `[component] ...` informally; match what the file around your change does. If adding a new prefix, register it in a short comment.
- **Don't downgrade log levels** (WARN → DEBUG) to "clean up" output. If a warning is noisy, fix the cause.
- **Mixed-language logs.** Existing user-facing messages are partly German, partly English (see e.g. `robot_controller_node.py:244`). Don't "fix" this opportunistically — the web UI surfaces some of these directly to the operator; match the surrounding file's language.

## 6. Code style — minimum viable

- **Keep changes scoped.** Don't refactor surrounding code while fixing a bug; open a separate PR.
- **Don't add speculative abstractions** or "future-proofing". The codebase already has several parallel path-follower, state-machine, and docking abstractions — adding more makes it worse, not better.
- **Match existing patterns** in the file you're editing (dataclasses vs dicts, logger style, parameter declaration idiom). The packages aren't uniform across the workspace; be locally consistent.
- **Don't add comments that restate the code.** Comments exist for *why*, constraints, workarounds — not *what*.

## 7. The duplicate-tree trap

- `~/gits/tactical_mower/tactical_mower/` is an older, **partially diverged** copy of the outer workspace (see AUDIT.md, MED-1). **Edit only the outer `tactical_mower/ros2_ws/`** — that is what the docker-compose builds. Do not edit or "sync" the inner copy without a plan to delete it entirely.
- The top-level sibling directories (`livox_mid_360_obstacle_detection/`, `RealsenseD_camera_Obstacle_avoidance/`, `Tactical-Thermal-Stream/`, `routen/`) have mirrored copies inside `tactical_mower/` that have diverged in places. Treat the top-level directories as canonical unless you confirm otherwise with the operator.

## 8. Testing

- There are **no unit/integration tests worth the name** in this codebase beyond the stock ROS `test_copyright/flake8/pep257` stubs. "It built" ≠ "it works".
- **For drive / autonomy / obstacle-avoidance changes**, state a concrete on-robot test plan (what to set, what to observe, what the expected and failure behaviors are). Do not rely on a rebuild as validation.
- **Never test autonomous driving changes without the operator physically present** with the robot and able to hit e-stop.

## 9. When in doubt, ask

This robot operates around people and property for a paying customer. The cost of pausing to confirm with the operator is a few minutes; the cost of a bad autonomous run is much higher. Ask.
