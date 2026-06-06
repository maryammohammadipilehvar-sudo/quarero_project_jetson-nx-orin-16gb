# DESIGN — ArUco precision docking (2026-06-05)

**Status:** code complete, **shipped OFF by default**, NOT yet tested on-robot.
Marker-based final-approach refinement layered on top of the existing GPS
line-docking. When disabled (the default) docking behaviour is byte-for-byte
the current GPS path.

Operator decisions (2026-06-05): dock is **nose-first** (OAK camera faces the
marker), **teach-and-repeat** setpoint, detection runs in the **camera
container**, scope is **final-approach refinement only** with GPS fallback.

Companion: `CODEBASE_MENTAL_MODEL.md` §8 (docking), `RCA_2026-05-31_docking_reliability.md`
(the GPS docking this sits on top of).

---

## 1. Why

GPS docking confirms "docked" from RTK pose alone (no contact sensor). It parks
a consistent 10–18 cm short and the 3 cm lateral gate is at the edge of what the
fix can hit. A 150 mm ArUco marker at the station gives cm-accurate, drift-free
relative pose at close range — enough to seat precisely and to *confirm* the dock
geometrically instead of trusting GPS proximity.

Marker: **DICT_4X4_50, ID 0, 150 mm** (`docking_aruco/marker_main_id0.pdf`).

## 2. Architecture (three layers)

```
OAK-D Lite RGB  ─/oak/rgb/image_rect + /oak/rgb/camera_info─┐
                                                            ▼
  [camera container]  aruco_dock_detector  (cv2.aruco + solvePnP)
     • detects ID 0, estimates pose, gates on range + reprojection error
     • publishes /docking/aruco_pose (PoseStamped, camera optical frame)
     • publishes /docking/aruco_debug (annotated image, for rqt_image_view)
                                                            │
                          /docking/aruco_pose  ─────────────┘
                                                            ▼
  [ros2_jetson]  tactical_wp_follower
     • caches the pose, reduces to (lateral=x, range=z, bearing, age)
     • loads /routen/settings/aruco_dock.yaml every 2 s (off if absent)
     • passes aruco_marker + aruco_config into DOCKING via on_update kwargs
                                                            ▼
  DockingState  (docking_state.py)  — new VISUAL_SERVO sub-state
     FOLLOW_LINE ──(enabled + setpoint + fresh marker + within engage range)──► VISUAL_SERVO
     VISUAL_SERVO ──(marker lost / timeout)──► FOLLOW_LINE   (silent GPS fallback)
     VISUAL_SERVO ──(lateral & range match taught setpoint, N reads)──► COMPLETED → on_docked()
```

**Layer 3 is default-off twice over:** `aruco_dock.yaml` absent → `_aruco_config`
empty → `enabled` stays False; and even if present, `setpoint_valid` must be true.
Either gate false → `VISUAL_SERVO` is never entered → GPS docking is unchanged.

## 3. Control law (VISUAL_SERVO)

Pure marker-frame teach-and-repeat (no GPS, no camera-mount extrinsics needed):
- `lat_err = marker.x − setpoint.x`, `range_err = marker.z − setpoint.z`,
  `yaw_err = normalize(marker.bearing − setpoint.bearing)`.
- `steering = −(gain_lateral·lat_err + gain_yaw·yaw_err)` (clamped to `max_steering`,
  `invert_steering` flips sign if the bench test disagrees).
- `speed = creep_speed_ratio·100` while `range_err > 0`, else 0 (stop pushing once seated).
- **Docked** when `|lat_err| ≤ dock_lateral_tolerance` AND `|range_err| ≤ dock_range_tolerance`
  for `dock_confirm_readings` consecutive frames → `on_docked()` (same callback the GPS path
  uses → same DOCKED transition).

## 4. Safety / failure modes (CLAUDE.md §1)

| Failure | Response |
|---|---|
| ArUco docking disabled / no setpoint / detector not running | Never enters VISUAL_SERVO. GPS docking unchanged. |
| Marker never acquired in the final stretch | Stays in FOLLOW_LINE → GPS docks as today. |
| Marker lost/occluded mid-servo (age > `stale_timeout`) | Revert to FOLLOW_LINE; GPS takes over. No fake dock. |
| Servo can't converge (`servo_timeout`, default 25 s) | Revert to FOLLOW_LINE; GPS makes the final call. |
| False / wrong-ID / ambiguous detection | Detector gates on ID, range bounds, and reprojection error; control's tight tolerances + GPS fallback arbitrate. |
| Steering sign wrong on this mount | `invert_steering` flag; caught on the bench before `enabled: true`. |
| **GPS lateral gate would FAIL while servoing** | The GPS docked/FAILED gate is **skipped** while in VISUAL_SERVO (marker is the authority at close range). It resumes the instant we revert. |

**Untouched:** RTK gate, obstacle EMERGENCY_STOP, LiDAR timeout, the 0.3 s Roboclaw
`/cmd_drive` watchdog, MANUAL override, charging lockout. VISUAL_SERVO only *refines*
steering/speed inside the final approach; it cannot bypass any stop.

## 5. Files changed

Camera container (`RealsenseD_camera_Obstacle_avoidance/`):
- NEW `…/realsense_obstacle/aruco_dock_detector.py` — detector node.
- `…/setup.py` — register `aruco_dock_detector` console script.
- `…/launch/realsense_obstacle.launch.py` — launch the detector node.
- `…/config/params.yaml` — `aruco_dock_detector` params.
- `Dockerfile` — add `opencv-contrib-python-headless` (cv2.aruco; apt OpenCV 4.6 omits it).

Control container (`tactical_mower/ros2_ws/src/control/`):
- `…/robot_state_machine/states/docking_state.py` — `VISUAL_SERVO` sub-state, `ArucoDockConfig`,
  `_execute_visual_servo`, `_marker_is_fresh`, GPS-gate guard, FOLLOW_LINE→VISUAL_SERVO hand-off.
- `…/nodes/tactical_wp_follower_node.py` — `/docking/aruco_pose` subscription, `_aruco_pose_callback`,
  `_load_aruco_config`, `_marker_bearing_from_quat`, marker+config into `_build_state_update_kwargs`.

Tooling / config:
- NEW `docking_aruco/teach_dock_setpoint.py` — teach-and-repeat capture.
- NEW `docking_aruco/aruco_dock.example.yaml` — config schema (live file: `/routen/settings/aruco_dock.yaml`).

## 6. Bring-up (operator, robot parked + manual)

1. **Rebuild the camera container** (adds opencv-contrib + the new node):
   `cd RealsenseD_camera_Obstacle_avoidance && docker compose build && docker compose up -d`
   (confirm the robot is parked and in manual first — CLAUDE.md §3).
2. **Verify detection.** Place the marker; in the camera container run
   `ros2 topic echo /docking/aruco_pose --once` (should print a pose) and view
   `/docking/aruco_debug` in `rqt_image_view` (axes drawn on the marker).
   `position.z` ≈ true distance to the marker; `position.x` ≈ 0 when centred.
3. **Teach the setpoint.** Manually drive the robot to the *perfect* docked
   position (contacts mated), marker in view, then run
   `python3 docking_aruco/teach_dock_setpoint.py --samples 40` inside ros2_jetson.
   It writes `/routen/settings/aruco_dock.yaml` with `setpoint_valid: true`,
   `enabled: false`.
4. **Bench-verify steering sign** (wheels off ground / clear space, e-stop ready):
   temporarily `enabled: true`, trigger a dock, watch that as the marker is offset
   left/right the robot steers *toward* centre. If it steers away, set
   `invert_steering: true`. Then disable again.

## 7. On-robot test plan (operator present, e-stop in reach — CLAUDE.md §8)

- Set `enabled: true` in `/routen/settings/aruco_dock.yaml` (picked up within 2 s).
- Trigger a dock (Run-Now a short route then return, or go-to-charge). Watch
  `/tactical/robot/charging_status` and `docker logs ros2_jetson | grep -iE "FOLLOW_LINE|VISUAL"`.
- **Expect:** GPS line-follow to ~1.5 m, then `FOLLOW_LINE → VISUAL_SERVO`, a smooth
  centre-and-creep, then `VISUAL dock complete` with lateral/range errors within tolerance.
- **Failure rehearsal:** cover the marker mid-servo → expect `marker lost/stale →
  reverting to FOLLOW_LINE` and GPS docking resumes. Confirm e-stop halts instantly.
- After each dock, run `ros2_ws/pose_diff_measure.py` to log the GPS residual for comparison.

**Tuning knobs** (`aruco_dock.yaml`): `gain_lateral` / `gain_yaw` (raise if sluggish,
lower if oscillating), `creep_speed_ratio` (raise if it stalls short, lower if it shoves
the dock), `engage_distance`, `dock_*_tolerance`, `servo_timeout`.

## 8. Residual risks / open items

- **OpenCV ABI:** the pip `opencv-contrib-python-headless` wheel is a newer build than
  the system OpenCV cv_bridge links. Detector uses pip `cv2` only; verify both
  `simple_obstacle_detector` and `aruco_dock_detector` still import cv2 after the rebuild.
- **camera_info presence:** assumes depthai publishes `/oak/rgb/camera_info` (rectified
  intrinsics). The detector waits for it and warns if absent — confirm in step 2.
- **No camera→base TF used** (servo is marker-frame). If the mount changes, re-teach.
- **Branch hygiene:** these are autonomy/camera changes; they should NOT ride the
  `ui-customer-handoff` branch — split before committing (operator approval required).
- Still no contact sensor; VISUAL dock is a geometric proxy, but a much tighter one
  than GPS proximity.

## 9. 2026-06-05 — ArUco made PRIMARY, marker-owned hardcoded stop (operator request)

Behaviour inversion from the original design above. **ArUco is now the primary
final-approach controller; GPS is the assistant.**

- **GPS role:** coarse-positions the robot in front of the dock (drive-back + line
  align/follow) and **gates engagement** (`engage_distance`, 1.5 m). It no longer owns
  the stop.
- **ArUco role:** once a fresh marker is seen inside `engage_distance`, `FOLLOW_LINE →
  VISUAL_SERVO`. The marker is **centred** in the camera (pure lateral centring, no
  bearing term in markerless mode) and the robot drives straight in. The marker's
  **RANGE owns the longitudinal stop**.
- **Stop distance hardcoded:** `MARKER_STOP_RANGE_M = 0.85` m (camera→marker = →wall),
  in `docking_state.py` (constant, not the yaml). `enabled` default is now `True`.
  Docked is declared after `docked_confirm_readings` (3) consecutive in-range,
  centred frames — no GPS-overshoot instant-accept in marker mode.
- **Anti-wall (RCA 2026-06-05 wall-crash) preserved, now relative to the stop:** creep
  holds at 0.85 m; a hard abort floor at `0.85 − MARKER_STOP_ABORT_MARGIN(0.15) = 0.70`
  m → FAILED if breached; range-jump reject; creep only on a FRESH frame.
- **Fail-soft unchanged:** marker lost / `servo_timeout` → revert to GPS line-follow
  (its own charge-point stop + overshoot guard); off-centre at the stop → FAILED, never
  a fake dock.
- **Note:** 0.85 m is ~17 cm closer to the wall than the old taught docked range
  (1.02 m). Operator measured & confirmed 0.85 m as the intended standoff on 2026-06-05.
  The live `aruco_dock.yaml min_range_m: 0.85` is now **ignored in markerless mode**
  (the floor is the hardcoded 0.70 m); leave or update it, it no longer gates the stop.
- **Not yet on-robot tested** with these changes. Verify `invert_steering` direction on
  the bench first (centring should drive the marker TOWARD centre, not away).

## 10. 2026-06-06 — staged single-authority rework (supersedes §9 stop logic)

§9 left GPS and the marker both touching the final stop (GPS `gps_stop_tolerance` from
the charge point owned the stop while the marker servoed lateral). With the marker
min-range floor and the GPS charge point at different distances, that produced a
`VISUAL_SERVO ⇄ FOLLOW_LINE` oscillation that never seated, and the GPS docked-check
also hard-FAILed on a 3 cm lateral that GPS can't hit on grass. Reworked to a clean
staged model:

- **One authority per regime, monotonic handoff.** GPS coarse-approaches (APPROACH/
  ALIGN/FOLLOW_LINE) and *gates engagement* (`engage_distance`). Once a fresh marker is
  acquired inside that range, the **marker owns BOTH steering AND the longitudinal stop
  + the "docked" decision.** GPS no longer participates in the final stop — it is purely
  the fallback. This removes the dual-authority livelock.
- **Stop = camera→marker range vs a target standoff.** `stop_range_m` (markerless) or
  `setpoint_range` (taught). Docked when the marker is centred (`dock_lateral_tolerance`)
  AND at the standoff (`dock_range_tolerance`) for `dock_confirm_readings` consecutive
  **fresh** frames → `on_docked()` (the same DOCKED transition the GPS path uses).
- **`MARKER_STOP_RANGE_M` constant from §9 was never in the code.** The standoff is the
  `stop_range_m` config field (default 0.85 m); `min_range_m` is now the hard anti-wall
  abort floor only (default 0.70 m, **must be below** `stop_range_m`). Older yamls that
  set `min_range_m: 0.85` should move that value to `stop_range_m` and lower the floor.
- **Anti-oscillation.** A brief marker dropout mid-servo HOLDS position for
  `reacquire_grace` (2 s) before falling back to GPS; after any servo→GPS revert,
  re-engagement is blocked for `reengage_cooldown` (6 s) so the two controllers can't
  ping-pong. Hard floor breach → FAILED; range-jump frame → steer but don't creep;
  creep only on a fresh trustworthy frame (all retained from §9).
- **GPS fallback restored to working.** In GPS-only FOLLOW_LINE the lateral gate is now
  GENEROUS (`charge_lateral_tolerance` 0.15 m) — complete at charge distance, FAIL only
  if grossly sideways. cm-precision is the marker's job, not GPS's.
- **Navigation regression fixed (same branch).** The Nav2 follower spin-guard previously
  called `on_route_completed()` on a 15 s in-place-rotation timeout — falsely reporting
  the route SUCCESSFUL and dropping the remaining waypoints. It now **skips the waypoint**
  (`_advance_to_next_waypoint`, mirroring the linear follower's rotation timeout); real
  completion is still only signalled when it was the final ONCE waypoint.
- **`enabled` reset to FALSE** in the live yaml: the steering sign and marker visibility
  from engage range down to the standoff are physical facts that MUST be bench-verified
  with the operator present (CLAUDE.md §1/§8) before arming. Not remotely verifiable.
- **Still no contact sensor.** Dock confirmation is a tight geometric proxy. The single
  highest-value follow-up is confirming actual charge current (or a dock limit switch):
  the RoboClaw `ReadCurrents` only reads MOTOR current, and charging runs through a
  separate ESP relay, so no charge-current signal exists today.

Files touched 2026-06-06: `docking_state.py` (VISUAL_SERVO rework, GPS docked-check
scoped to FOLLOW_LINE, `_revert_to_follow_line`, config fields), `waypoint_follower_nav2.py`
(spin-guard fail-soft), `aruco_dock.yaml` + `aruco_dock.example.yaml` + `teach_dock_setpoint.py`
(new schema).
