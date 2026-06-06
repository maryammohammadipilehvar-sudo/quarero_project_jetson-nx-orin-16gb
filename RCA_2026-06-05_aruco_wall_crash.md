# RCA 2026-06-05 — ArUco precision docking drives into the wall

## Symptom
On "go back to charge" with ArUco precision docking enabled, the robot servos on the
dock marker but drives **too close and crashes into the wall the marker is mounted on**.

## Root cause — the marker range was the sole, blindly-trusted stop
In `docking_state.py` the visual servo (`_execute_visual_servo`) drove forward purely
on the marker, with the GPS charge-point stop switched off:

1. **GPS stop disabled during VISUAL_SERVO.** The docked / overshoot check at the top
   of `on_update` was skipped whenever the sub-state was `VISUAL_SERVO` (`if ... !=
   VISUAL_SERVO`). So once the servo engaged, the *only* thing that could stop forward
   motion was `range_err <= 0` (marker range ≤ taught `setpoint_range`). Nothing else.
2. **Blind coasting on a stale marker.** `_marker_is_fresh()` accepted poses up to
   `stale_timeout = 1.0 s` old. As the camera neared the wall-mounted marker, the
   marker left the OAK FOV / the detector dropped out, and the servo kept creeping on
   the last frozen range for up to a second (~0.3 m) before reverting to GPS.
3. **No obstacle floor during docking.** The LiDAR gate runs only in NAVIGATING / RTH
   (`tactical_wp_follower_node.py:2544`), so nothing physical stopped the robot.
4. **Suspect setpoint.** Taught `setpoint_yaw = 2.65 rad ≈ 152°` (a head-on dock should
   be ≈ 0°), so the teach pose was likely off and `setpoint_range` not trustworthy.

Any of 1–3 turned a marker glitch / FOV loss / bad setpoint into a wall collision
instead of a safe stop.

## Fix — GPS owns the stop, marker owns alignment, with independent anti-wall floors
The marker is mounted ON the wall, so its range *is* the wall distance, and the saved
web-app charge point is the correct place to stop. So:

`docking_state.py`:
- **The saved GPS charge point now owns the stop + "docked" decision during the visual
  servo too** (the docked-check runs for FOLLOW_LINE *and* VISUAL_SERVO). The robot
  always ends at the operator-saved charge position and can never be driven past it.
  During the servo the gate uses the **marker's** lateral error (more precise at close
  range) so re-enabling it keeps the dock tight instead of false-failing.
- The overshoot/`past_charge` guard is now active during the servo as well.
- `_execute_visual_servo` now only **steers (marker) and creeps**; it never declares
  docked. Forward motion is gated by INDEPENDENT stops:
  - **Hard min-range floor** `max(min_range_m, setpoint_range − range_floor_margin)` —
    breaching it aborts the dock (FAILED + operator notice), never pushes closer.
  - **Range-jump rejection** (`max_range_jump`) — don't creep on an untrustworthy frame.
  - **Fresh-marker-only creep** — a stale/lost marker stops forward motion immediately
    (no 1 s blind coast) and hands back to the GPS line-follow.
  - Still stops at the taught range.
- New config (`ArucoDockConfig`): `min_range_m=0.4`, `range_floor_margin=0.05`,
  `max_range_jump=0.3`.

The GPS-only docking path (ArUco disabled) is unchanged — verified.

## Status
- Deployed (ros2_jetson rebuilt + restarted). Robot in MANUAL, schedule off.
- **ArUco docking is still `enabled: false`** — keep it off until the setpoint is
  re-taught and the steering direction bench-verified.

## Follow-up enhancements (operator request 2026-06-05)
- **Centre-the-marker servo.** `ArucoDockConfig.center_marker` (default **True**): the
  visual servo now keeps the marker in the MIDDLE of the camera (lateral target =
  optical centre, 0) and drives that centred line to the charge point, instead of
  servoing to the taught `setpoint_lateral`. Pure-centring also drops the bearing term
  so the single steering DOF is spent only on centring. This greatly reduces reliance
  on the (previously suspect) taught lateral/yaw — the GPS charge point still owns the
  stop, so the robot ends at the saved position.
- **Auto headlight for night docking.** wp_follower turns the headlight **ON** when it
  enters RETURNING_TO_HOME / DOCKING (marker visibility at night) and **OFF** once the
  flow ends (docked, or aborted). Edge-triggered + self-scoped (`_auto_light_on`) so it
  never overrides manual light control in other states. Publishes `/control/light`
  (joy_controller → ESP relay); robot_controller now subscribes to `/control/light` so
  `/robot/state.light_state` stays truthful for the UI. **Verified end-to-end on-robot.**

## Before re-enabling ArUco (operator, present at robot)
1. Physically dock the robot correctly on the charge contacts.
2. Re-run `docking_aruco/teach_dock_setpoint.py`; sanity-check `setpoint_yaw ≈ 0` and
   `setpoint_range` ≥ the true camera-to-marker standoff.
3. Set `min_range_m` in `aruco_dock.yaml` to just below that true standoff.
4. Bench-verify steering direction (`invert_steering`) per `aruco_dock.example.yaml`.
5. Set `enabled: true`, then test go-to-charge with a hand on the e-stop. Expect: the
   marker refines alignment, the robot stops at the saved charge point, and a marker
   loss / bad frame stops it (never advances toward the wall).
